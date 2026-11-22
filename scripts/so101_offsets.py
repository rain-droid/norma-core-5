"""Show the SO-101 servos and, with --write, restore the calibration from data/so101_calibration.yaml.

    uv run python scripts/so101_offsets.py            # read only
    uv run python scripts/so101_offsets.py --write    # torque off, then write offsets + angle limits

The position offset (0x1F) and min/max angle limits (0x09/0x0B) live in servo EEPROM. A Station
auto-calibration rewrites them, after which the calibration file no longer matches the arm.
"""

from __future__ import annotations

import argparse
import asyncio
import struct

from elrobot.config import Config
from elrobot.link import StationLink
from elrobot.profile import ACTIVE as PROFILE
from elrobot.servo import TICKS

REG_MIN, REG_MAX, REG_OFFSET, REG_LOCK, REG_TORQUE = 0x09, 0x0B, 0x1F, 0x37, 0x28


def enc_offset(v: int) -> bytes:
    return struct.pack("<H", (abs(v) & 0x7FF) | (0x800 if v < 0 else 0))


async def wait_motors(link) -> bool:
    for _ in range(40):
        await asyncio.sleep(0.25)
        if all(i in link.motors and not link.motors[i].has_error for i in PROFILE.all_ids):
            return True
    return False


def show(link, cal) -> None:
    for i in PROFILE.all_ids:
        m, c = link.motors[i], cal[i]
        raw = link.raw.get(i, b"")
        lim = struct.unpack_from("<HH", raw, REG_MIN) if len(raw) > REG_MAX + 1 else ("?", "?")
        print(f"  {i} {c['name']:14s} tick {m.tick:4d} offset {m.offset:5d} (want {c['homing_offset']:5d}) limits {lim} "
              f"(want {c['range_min']}..{c['range_max']}) torque {int(m.torque)} {m.volt:.1f} V {m.temp} C status {m.status}")


async def main(write: bool) -> int:
    cfg = Config.load()
    cal = PROFILE.load_calibration()
    link = StationLink(cfg.station_host, cfg.bus_serial, cfg.cameras)
    await link.start()
    try:
        if not await wait_motors(link):
            print("servos not reporting (is the arm powered?) have:", sorted(link.motors))
            return 2
        print("before:")
        show(link, cal)
        if not write:
            return 0
        ids = list(PROFILE.all_ids)
        before = {i: link.motors[i].tick for i in ids}
        old = {i: link.motors[i].offset for i in ids}
        await link.set_torque(ids, False)
        await asyncio.sleep(0.3)
        await link.sync_write(REG_LOCK, {i: b"\x00" for i in ids})
        await asyncio.sleep(0.1)
        await link.sync_write(REG_OFFSET, {i: enc_offset(int(cal[i]["homing_offset"])) for i in ids})
        await asyncio.sleep(0.1)
        await link.sync_write(REG_MIN, {i: struct.pack("<H", int(cal[i]["range_min"])) for i in ids})
        await asyncio.sleep(0.1)
        await link.sync_write(REG_MAX, {i: struct.pack("<H", int(cal[i]["range_max"])) for i in ids})
        await asyncio.sleep(0.1)
        await link.sync_write(REG_LOCK, {i: b"\x01" for i in ids})
        await asyncio.sleep(1.0)
        print("after:")
        show(link, cal)
        ok = True
        for i in ids:
            want = (before[i] + old[i] - int(cal[i]["homing_offset"])) % TICKS
            got = link.motors[i].tick
            d = (got - want + TICKS // 2) % TICKS - TICKS // 2
            if abs(d) > 12:
                ok = False
                print(f"  servo {i}: position reads {got}, expected {want} after the offset change")
        print("offsets applied" if ok else "NOT all applied; see above")
        return 0 if ok else 1
    finally:
        await link.stop()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    raise SystemExit(asyncio.run(main(ap.parse_args().write)))
