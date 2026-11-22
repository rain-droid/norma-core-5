"""Drive the arm one action at a time, straight through the Station (no daemon needed).

    uv run python scripts/arm.py pose
    uv run python scripts/arm.py to 0.0 0.20 0.10 --pitch 90      # tool point, metres
    uv run python scripts/arm.py rel 0 0 0.03                      # straight-line shift
    uv run python scripts/arm.py joint 2 10                        # nudge joint 2 by +10 deg
    uv run python scripts/arm.py joints -2 -60 20 90 0 10 -12      # absolute, degrees
    uv run python scripts/arm.py grip 1.0 | grasp | release | hold | sit

Every action prints the resulting pose and saves the camera frames to logs/snap/.
Torque stays on after the command returns (the arm holds its pose) unless you `release`.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import sys
import time
from pathlib import Path

import numpy as np

from elrobot.config import REPO_ROOT, Config
from elrobot.link import StationLink
from elrobot.robot import Robot, RobotError

from elrobot.profile import ACTIVE as PROFILE  # noqa: E402
from elrobot.servo import ARM_IDS, GRIPPER_ID  # noqa: E402

SIT_FILE = REPO_ROOT / "data" / ("sit_pose.json" if PROFILE.name == "elrobot" else f"sit_pose_{PROFILE.name}.json")


def show(robot: Robot) -> None:
    q = robot.q()
    T = robot.kin.fk(q)
    m = robot.link.motors
    print("q deg     :", " ".join(f"{math.degrees(v):7.1f}" for v in q))
    print("load      :", " ".join(f"{m[i].load:7d}" for i in ARM_IDS), f"| grip {m[GRIPPER_ID].load}")
    print("temp      :", " ".join(f"{m[i].temp:7d}" for i in ARM_IDS), f"| grip {m[GRIPPER_ID].temp}")
    print(f"tcp       : x={T[0, 3]:.3f} y={T[1, 3]:.3f} z={T[2, 3]:.3f}  approach=({T[0, 2]:.2f}, {T[1, 2]:.2f}, {T[2, 2]:.2f})"
          f"  pitch={math.degrees(math.asin(-T[2, 2])):.0f}deg")
    print(f"gripper   : open={robot.gripper_open():.2f} holding={robot.holding}  torque={robot.torque_on}")


async def main(a) -> int:
    cfg = Config.load()
    link = StationLink(cfg.station_host, cfg.bus_serial, cfg.cameras)
    await link.start()
    try:
        if not await link.wait_connected(8):
            print("not connected")
            return 2
        robot = Robot(link, cfg)
        why = robot.why_not_ready()
        if why:
            print(why)
            return 2
        t0 = time.monotonic()
        try:
            if a.cmd == "pose":
                pass
            elif a.cmd == "hold":
                await robot.enable()
            elif a.cmd == "release":
                await robot.release()
            elif a.cmd == "to":
                r = await robot.move_to(a.xyz, pitch=a.pitch, yaw=a.yaw, speed=a.speed, linear=a.linear, allow_low=a.low)
                print("result:", r)
            elif a.cmd == "rel":
                r = await robot.move_relative(a.xyz, speed=a.speed, allow_low=a.low)
                print("result:", r)
            elif a.cmd == "joint":
                q = robot._plan_start()
                q[a.id - 1] += math.radians(a.deg)
                r = await robot.move_joints(q, speed=a.speed, check=not a.nocheck, allow_low=a.low)
                print("result:", r)
            elif a.cmd == "joints":
                r = await robot.move_joints(np.radians(a.deg), speed=a.speed, check=not a.nocheck, allow_low=a.low)
                print("result:", r)
            elif a.cmd == "grip":
                print("reached:", round(await robot.set_gripper(a.open), 3))
            elif a.cmd == "grasp":
                print("grasp:", await robot.grasp())
            elif a.cmd == "savesit":
                SIT_FILE.parent.mkdir(parents=True, exist_ok=True)
                SIT_FILE.write_text(json.dumps({"q": [round(float(v), 4) for v in robot.q()]}))
                print("saved", SIT_FILE)
            elif a.cmd == "sit":
                q_sit = np.array(json.loads(SIT_FILE.read_text())["q"])
                # come in from above: shoulder raised first, then lowered onto the rest
                above = q_sit.copy()
                above[1] += math.radians(22)
                await robot.move_joints(above, speed=a.speed, check=False)
                await robot.move_joints(q_sit, speed=0.3, check=False)
                await robot.release()
        except RobotError as e:
            print(f"ROBOT ERROR: {e}")
            show(robot)
            return 1
        print(f"({time.monotonic() - t0:.1f}s)")
        await asyncio.sleep(0.4)
        show(robot)
        out = Path("logs/snap")
        out.mkdir(parents=True, exist_ok=True)
        for name, fr in link.frames.items():
            (out / f"{name}.jpg").write_bytes(fr.jpeg)
        return 0
    finally:
        await link.stop()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--speed", type=float, default=0.5)
        p.add_argument("--low", action="store_true", help="allow the fingertips down to the table surface")
        p.add_argument("--nocheck", action="store_true", help="skip clearance checks (joint moves only)")

    for name in ("pose", "hold", "release", "grasp", "savesit"):
        sub.add_parser(name)
    p = sub.add_parser("sit"); common(p)
    p = sub.add_parser("to"); p.add_argument("xyz", type=float, nargs=3); p.add_argument("--pitch", type=float)
    p.add_argument("--yaw", type=float); p.add_argument("--linear", action="store_true"); common(p)
    p = sub.add_parser("rel"); p.add_argument("xyz", type=float, nargs=3); common(p)
    p = sub.add_parser("joint"); p.add_argument("id", type=int); p.add_argument("deg", type=float); common(p)
    p = sub.add_parser("joints"); p.add_argument("deg", type=float, nargs=7); common(p)
    p = sub.add_parser("grip"); p.add_argument("open", type=float)
    logging.basicConfig(level=logging.ERROR)
    sys.exit(asyncio.run(main(ap.parse_args())))
