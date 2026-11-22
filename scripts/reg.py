"""Write one servo register through the Station: reg.py ADDR VALUE SIZE ID [ID...] [--eeprom]."""
import asyncio, sys
from elrobot.config import Config
from elrobot.link import StationLink

async def main():
    args = [a for a in sys.argv[1:] if a != "--eeprom"]
    addr, val, size, ids = int(args[0], 0), int(args[1]), int(args[2]), [int(i) for i in args[3:]]
    cfg = Config.load(); l = StationLink(cfg.station_host, cfg.bus_serial, cfg.cameras); await l.start(); await asyncio.sleep(2.5)
    if "--eeprom" in sys.argv:
        await l.sync_write(0x37, {i: b"\x00" for i in ids}); await asyncio.sleep(0.1)
    await l.sync_write(addr, {i: val.to_bytes(size, "little") for i in ids}); await asyncio.sleep(0.1)
    if "--eeprom" in sys.argv:
        await l.sync_write(0x37, {i: b"\x01" for i in ids}); await asyncio.sleep(0.1)
    await asyncio.sleep(0.5); await l.stop(); print("wrote", hex(addr), val, ids)
asyncio.run(main())
