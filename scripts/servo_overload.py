"""Show or set the overload-protection threshold stored in the arm servos.

    uv run python scripts/servo_overload.py        # show
    uv run python scripts/servo_overload.py 50     # set servos 1..7 to 50 %

A servo whose output stays above this percentage for 2 s drops to 20 % torque and reports status
bit 32 until its torque is switched off and on again. NormaCore's calibration leaves it at 25
(the servo's factory value is 80). On a 7.5 V supply the shoulder passes 25 % on any lift from an
extended pose, so the arm sags mid-task.

The value lives in the servo's EEPROM: it survives power cycles, and running the Station's
calibration again puts it back to 25. Run this with the Station up, the arm resting and torque off.
"""

from __future__ import annotations

import asyncio
import sys

from elrobot.config import Config
from elrobot.link import StationLink
from elrobot.servo import ARM_IDS, REG_OVERLOAD_TORQUE


def show(link: StationLink) -> None:
    for i in sorted(link.motors):
        m = link.motors[i]
        print(f"  servo {i}: overload threshold {m.overload_torque:3d} %   status {m.status}   torque {'on' if m.torque else 'off'}")


async def main(value: int | None) -> None:
    cfg = Config.load()
    link = StationLink(cfg.station_host, cfg.bus_serial)
    await link.start()
    try:
        if not await link.wait_connected(10):
            sys.exit("no servo state from the Station")
        show(link)
        if value is None:
            return
        if any(m.torque for m in link.motors.values()):
            sys.exit("torque is on: rest the arm first (workspace/tools/rest.py)")
        for attempt in range(3):
            wrong = [i for i in ARM_IDS if link.motors[i].overload_torque != value]
            if not wrong:
                break
            for i in wrong:
                await link.write_eeprom(i, REG_OVERLOAD_TORQUE, bytes([value]))
            await asyncio.sleep(1.0)
        print("after:")
        show(link)
        wrong = [i for i in ARM_IDS if link.motors[i].overload_torque != value]
        if wrong:
            sys.exit(f"servo(s) {wrong} did not take the value")
    finally:
        await link.stop()


if __name__ == "__main__":
    arg = int(sys.argv[1]) if len(sys.argv) > 1 else None
    if arg is not None and not 20 <= arg <= 80:
        sys.exit("threshold must be between 20 and 80 percent")
    asyncio.run(main(arg))
