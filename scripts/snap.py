"""Save the latest frame from each camera (named per config.yaml) and print servo state.

    uv run python scripts/snap.py [out_dir]
"""

from __future__ import annotations

import asyncio
import logging
import sys
from pathlib import Path

from elrobot.config import Config
from elrobot.link import StationLink


async def main(out: Path) -> None:
    cfg = Config.load()
    link = StationLink(cfg.station_host, cfg.bus_serial, cfg.cameras)
    await link.start()
    ok = await link.wait_connected(8)
    await asyncio.sleep(1.5)  # let camera queues get discovered and deliver a frame
    print(f"connected={ok} calibrated={link.calibrated} bus={link.bus_serial} calibration={link.calibration}")
    for i in sorted(link.motors):
        m = link.motors[i]
        print(f"  {i}: tick={m.tick:4d} goal={m.goal_tick:4d} torque={m.torque!s:5} cur={m.current:3d} load={m.load:4d} "
              f"V={m.volt:4.1f} T={m.temp} status={m.status} range=[{m.range_min},{m.range_max}] frozen={m.range_frozen}")
    out.mkdir(parents=True, exist_ok=True)
    for name, fr in link.frames.items():
        p = out / f"{name}.jpg"
        p.write_bytes(fr.jpeg)
        print(f"  camera {name}: {fr.width}x{fr.height} {link.cameras[name].fps:.1f}fps -> {p}")
    await link.stop()


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    asyncio.run(main(Path(sys.argv[1] if len(sys.argv) > 1 else "logs/snap")))
