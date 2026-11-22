"""Print selected registers of every servo."""
import asyncio
from elrobot.config import Config
from elrobot.link import StationLink
NAMES = {0x0E: ("max_temp",1), 0x10: ("max_torque",2), 0x15: ("P",1), 0x16: ("D",1), 0x17: ("I",1), 0x18: ("min_start",2), 0x1A:("cw_dead",1),0x1B:("ccw_dead",1),0x1C:("prot_cur",2),0x21:("mode",1),0x22:("prot_tq",1),0x23:("prot_time",1),0x24:("ovl_tq",1),0x25:("speed_P",1),0x26:("oc_time",1),0x27:("speed_I",1),0x29:("acc",1),0x2E:("speed",2),0x30:("tq_lim",2),0x55:("max_speed?",2)}
async def main():
    cfg = Config.load(); l = StationLink(cfg.station_host, cfg.bus_serial, cfg.cameras); await l.start(); await asyncio.sleep(2.5)
    for i, d in sorted(l.raw.items()):
        print(i, len(d), {n: int.from_bytes(d[a:a+s], "little") for a, (n, s) in NAMES.items() if a + s <= len(d)})
    await l.stop()
asyncio.run(main())
