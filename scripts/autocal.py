"""Run the Station's SO-101 auto-calibration and print what it measured."""
import asyncio, json, struct, time
from elrobot.config import Config
from elrobot.link import StationLink
async def main():
    cfg = Config.load(); l = StationLink(cfg.station_host, cfg.bus_serial, cfg.cameras); await l.start()
    for _ in range(40):
        await asyncio.sleep(0.25)
        if len(l.motors) >= 6: break
    print("before:", {i: (m.tick, m.offset, m.range_min, m.range_max) for i, m in sorted(l.motors.items())}, flush=True)
    await l.auto_calibrate()
    t0, last, seen = time.monotonic(), None, False
    while time.monotonic() - t0 < 300:
        await asyncio.sleep(0.5)
        c = l.calibration
        if c is None: continue
        cur = (c.status, c.step, c.phase)
        if cur != last: print(f"{time.monotonic()-t0:5.1f}s", c.status, f"{c.step}/{c.total}", c.phase, c.error, flush=True); last = cur
        if c.status == "in_progress": seen = True
        if seen and c.status in ("done", "failed", "stopped"): break
    else:
        print("TIMEOUT: stopping"); await l.stop_auto_calibrate()
    await asyncio.sleep(3)
    out = {}
    for i, m in sorted(l.motors.items()):
        raw = l.raw.get(i, b""); lim = struct.unpack_from("<HH", raw, 0x09) if len(raw) > 0x0C else None
        out[i] = dict(tick=m.tick, offset=m.offset, range_min=m.range_min, range_max=m.range_max, frozen=m.range_frozen, eeprom_limits=lim, torque=int(m.torque))
        print(i, out[i])
    json.dump(out, open("data/autocal_result.json", "w"), indent=1)
    await l.stop()
asyncio.run(main())
