"""Calibrate the arm's joint ranges through the Station.

    uv run python scripts/calibrate.py --manual   # you move each joint to both end stops (reliable)
    uv run python scripts/calibrate.py --auto     # the arm sweeps itself; needs ~45 cm clear
    uv run python scripts/calibrate.py --stop     # abort a running auto-calibration
    uv run python scripts/calibrate.py --status

Manual mode speaks progress through the Mac's speakers, so you can keep both hands on the arm.
Auto mode is NormaCore's stock routine. On a 7.5 V supply the shoulder cannot lift the arm at
the routine's torque limit, and the result is wrong; prefer --manual unless you run 12 V.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import time
from pathlib import Path

from elrobot.config import Config
from elrobot.kinematics import Kinematics
from elrobot.link import StationLink

JOINT_WORDS = {1: "base", 2: "shoulder", 3: "lower elbow", 4: "upper elbow", 5: "forearm roll",
               6: "wrist bend", 7: "wrist roll", 8: "gripper"}


def say(text: str) -> None:
    """Speak without blocking; drop the line if the previous one is still playing."""
    import shutil
    import subprocess

    print(f"  [say] {text}", flush=True)
    if shutil.which("say"):
        subprocess.Popen(["say", "-r", "190", text])


def expected_spans(cfg: Config) -> dict[int, int]:
    """Design range of each joint in ticks (4096 per turn), from the URDF limits."""
    import math

    kin = Kinematics(cfg.urdf)
    spans = {j.id: int((j.upper - j.lower) * 4096 / (2 * math.pi)) for j in kin.joints}
    spans[8] = int((kin.gripper_upper - kin.gripper_lower) * 4096 / (2 * math.pi))
    return spans


async def manual(link: StationLink, cfg: Config, args) -> int:
    want = expected_spans(cfg)
    need = {i: int(want[i] * args.fraction) for i in want}
    # The Station rejects the reset if any servo misses a reply (it happens: one timeout desyncs
    # the half-duplex bus for the next few servos), and then silently keeps the old ranges. So
    # confirm the ranges really went back to "unfrozen" and retry if not.
    for attempt in range(1, 9):
        await link.reset_calibration()
        for _ in range(12):
            await asyncio.sleep(0.25)
            if not any(m.range_frozen for m in link.motors.values()):
                break
        if not any(m.range_frozen for m in link.motors.values()):
            print(f"calibration reset accepted (attempt {attempt})", flush=True)
            break
        print(f"reset attempt {attempt} was rejected, retrying", flush=True)
        await asyncio.sleep(0.5)
    else:
        say("Could not reset the calibration.")
        return 6
    await asyncio.sleep(0.5)
    say("Calibration started. Move each joint slowly to both of its end stops.")
    print("expected spans:", want)
    done: set[int] = set()
    t0 = time.monotonic()
    last_change = time.monotonic()
    last_spans: dict[int, int] = {}
    last_remind = time.monotonic()
    all_done_at = None
    while True:
        await asyncio.sleep(0.25)
        spans = {i: (m.range_max - m.range_min) % 4096 for i, m in link.motors.items()}
        if spans != last_spans:
            last_spans = spans
            last_change = time.monotonic()
        for i in sorted(spans):
            if i not in done and spans[i] >= need[i]:
                done.add(i)
                say(f"{JOINT_WORDS[i]} done")
                print(f"[{time.monotonic() - t0:6.1f}s] joint {i} complete: span {spans[i]} (design {want[i]})", flush=True)
        missing = [i for i in sorted(want) if i not in done]
        now = time.monotonic()
        if not missing:
            all_done_at = all_done_at or now
            # keep listening briefly so the last joint can reach its true stop
            if now - last_change > 2.5 or now - all_done_at > 12:
                break
        elif now - last_remind > 14:
            last_remind = now
            say("Still need " + ", ".join(JOINT_WORDS[i] for i in missing))
            print(f"[{now - t0:6.1f}s] spans {spans}", flush=True)
        if now - t0 > args.timeout:
            say("Calibration timed out. Nothing was saved.")
            print(f"timeout; spans {spans}, missing {missing}")
            return 5
    await link.freeze_calibration()
    for _ in range(40):
        await asyncio.sleep(0.25)
        if all(m.range_frozen for m in link.motors.values()):
            break
    ok = link.calibrated and all(m.range_frozen for m in link.motors.values())
    say("Calibration saved." if ok else "Calibration could not be saved.")
    print(f"frozen={ok}")
    dump(link)
    print("span vs design:", {i: f"{(m.range_max - m.range_min) % 4096}/{want[i]}" for i, m in sorted(link.motors.items())})
    return 0 if ok else 1


def dump(link: StationLink) -> None:
    for i in sorted(link.motors):
        m = link.motors[i]
        print(f"  {i}: tick={m.tick:4d} torque={m.torque!s:5} cur={m.current:3d} T={m.temp} status={m.status} "
              f"range=[{m.range_min},{m.range_max}] span={(m.range_max - m.range_min) % 4096} frozen={m.range_frozen}")


async def return_to(link: StationLink, target: dict[int, int], lift_ticks: int = 340) -> bool:
    """Drive the arm back to a pose given in servo ticks, then let it rest.

    Works purely in ticks so it does not depend on the calibration being right. The shoulder is
    kept raised until every other joint is in place, then lowered slowly, so the arm settles onto
    the table from above instead of sweeping across it.
    """
    from elrobot.servo import ALL_IDS, ARM_IDS, REG_TORQUE_LIMIT

    m = link.motors
    start = {i: m[i].tick for i in ALL_IDS}
    # "up" for the shoulder is away from where it rests and towards where it is now
    up = 1 if start[2] > target[2] else -1
    above = dict(target)
    above[2] = target[2] + up * min(lift_ticks, abs(start[2] - target[2]))

    await link.write_goals(start)
    await link.write_speed({i: 0 for i in ALL_IDS})
    await link.write_acc({i: 40 for i in ALL_IDS})
    await link.sync_write(REG_TORQUE_LIMIT, {i: (1000).to_bytes(2, "little") for i in ARM_IDS})
    await asyncio.sleep(0.05)
    await link.set_torque(ALL_IDS, True)
    await asyncio.sleep(0.3)

    async def glide(a: dict[int, int], b: dict[int, int], ticks_per_s: float) -> bool:
        dist = max(abs(b[i] - a[i]) for i in a)
        T = max(0.6, dist / ticks_per_s * 1.875)
        t0 = time.monotonic()
        behind_since = None
        while True:
            t = time.monotonic() - t0
            tau = min(1.0, t / T)
            s_ = tau ** 3 * (10 - 15 * tau + 6 * tau * tau)
            goal = {i: int(round(a[i] + (b[i] - a[i]) * s_)) for i in a}
            await link.write_goals(goal)
            lag = max(abs(goal[i] - m[i].tick) for i in ARM_IDS)
            if lag > 260:
                behind_since = behind_since or time.monotonic()
                if time.monotonic() - behind_since > 0.6:
                    await link.write_goals({i: m[i].tick for i in a})
                    print(f"  blocked while returning (lag {lag} ticks); holding here", flush=True)
                    return False
            else:
                behind_since = None
            if tau >= 1.0:
                break
            await asyncio.sleep(0.02)
        await asyncio.sleep(0.4)
        return True

    ok = await glide(start, above, 500.0) and await glide(above, target, 220.0)
    await asyncio.sleep(0.3)
    if ok:
        await link.set_torque(ALL_IDS, False)
    return ok


async def auto(link: StationLink, cfg: Config, args) -> int:
    from elrobot.servo import ALL_IDS

    want = expected_spans(cfg)
    out = Path("logs/calib")
    out.mkdir(parents=True, exist_ok=True)
    for f in out.glob("*.jpg"):
        f.unlink()
    # Raw encoder positions of the pose the arm is sitting in. Calibration rewrites each servo's
    # offset, so ticks change meaning; encoder positions do not.
    home_enc = {i: link.motors[i].encoder for i in ALL_IDS}
    print("sitting pose (encoder):", home_enc, flush=True)

    await link.auto_calibrate()
    print("auto-calibration requested", flush=True)
    t0 = time.monotonic()
    last = None
    last_frame = 0.0
    seen_running = False
    csv = open(out / "auto_log.csv", "w")
    csv.write("t,step,phase," + ",".join(f"enc{i},load{i},cur{i},tq{i}" for i in ALL_IDS) + "\n")
    code = 0
    while True:
        await asyncio.sleep(0.1)
        c = link.calibration
        now = time.monotonic() - t0
        key = (c.status, c.step, c.phase)
        if key != last:
            print(f"[{now:6.1f}s] {c.status} step {c.step}/{c.total}: {c.phase} {c.error}", flush=True)
            last = key
        mm = link.motors
        csv.write(f"{now:.2f},{c.step},{c.phase}," + ",".join(f"{mm[i].encoder},{mm[i].load},{mm[i].current},{int(mm[i].torque)}" for i in ALL_IDS if i in mm) + "\n")
        if c.status == "in_progress":
            seen_running = True
        if time.monotonic() - last_frame > 4.0:
            last_frame = time.monotonic()
            for name, fr in link.frames.items():
                (out / f"{name}_{int(now):04d}.jpg").write_bytes(fr.jpeg)
        hot = [i for i, m in link.motors.items() if m.temp >= 65]
        if hot:
            print(f"servo(s) {hot} over 65C, stopping")
            await link.stop_auto_calibrate()
            code = 3
            break
        if seen_running and c.status in ("done", "failed", "stopped"):
            break
        if not seen_running and now > 15:
            print("calibration never started")
            return 4
        if now > args.timeout:
            print("timeout, stopping")
            await link.stop_auto_calibrate()
            code = 5
            break
    csv.close()
    # wait for the freeze to land (ranges become valid and frozen)
    for _ in range(40):
        await asyncio.sleep(0.25)
        if link.calibrated and all(m.range_frozen for m in link.motors.values()):
            break
    status = link.calibration.status
    print(f"result: {status} {link.calibration.error}  calibrated={link.calibrated}")
    dump(link)
    ratios = {i: ((m.range_max - m.range_min) % 4096) / want[i] for i, m in sorted(link.motors.items())}
    print("span / design:", {i: round(r, 2) for i, r in ratios.items()})
    good = code == 0 and link.calibrated and all(0.90 <= r <= 1.15 for r in ratios.values())
    print("CALIBRATION", "OK" if good else "SUSPECT: " + str([i for i, r in ratios.items() if not 0.90 <= r <= 1.15]))

    if not args.no_return:
        target = {i: (home_enc[i] - link.motors[i].offset) % 4096 for i in ALL_IDS}
        print("returning to the sitting pose:", target, flush=True)
        back = await return_to(link, target)
        print("returned and released" if back else "did not get all the way back; holding with torque on")
    return 0 if good else 1


async def main(args) -> int:
    cfg = Config.load()
    link = StationLink(cfg.station_host, cfg.bus_serial, cfg.cameras)
    await link.start()
    if not await link.wait_connected(8):
        print("not connected to the servo bus (is the Station running and the arm powered?)")
        return 2
    try:
        if args.status:
            print(f"calibrated={link.calibrated} {link.calibration}")
            dump(link)
            return 0
        if args.stop:
            await link.stop_auto_calibrate()
            print("stop sent")
            return 0
        if args.manual:
            return await manual(link, cfg, args)
        if not args.auto:
            print("choose --manual, --auto, --stop or --status")
            return 2

        return await auto(link, cfg, args)
    finally:
        await link.stop()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--manual", action="store_true")
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--fraction", type=float, default=0.93, help="manual: share of the design range that counts as complete")
    ap.add_argument("--no-return", action="store_true", help="auto: leave the arm where calibration ends")
    ap.add_argument("--stop", action="store_true")
    ap.add_argument("--status", action="store_true")
    ap.add_argument("--timeout", type=float, default=600)
    logging.basicConfig(level=logging.WARNING)
    raise SystemExit(asyncio.run(main(ap.parse_args())))
