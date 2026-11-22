"""Safety check: switch on ONE servo, hold, nudge it a few degrees and back, switch it off."""
import asyncio, sys
from elrobot.config import Config
from elrobot.link import StationLink
async def main(i=int(sys.argv[1]), step=int(sys.argv[2])):
    cfg = Config.load(); l = StationLink(cfg.station_host, cfg.bus_serial, cfg.cameras); await l.start(); await asyncio.sleep(3)
    t0 = l.motors[i].tick; print("start tick", t0)
    await l.write_goals({i: t0}); await l.write_acc({i: 30}); await l.write_speed({i: 400})
    await l.sync_write(0x30, {i: (400).to_bytes(2, "little")}); await asyncio.sleep(0.1)
    await l.set_torque([i], True)
    for k in range(6):
        await asyncio.sleep(0.25); print(" hold", l.motors[i].tick, "goal", l.motors[i].goal_tick, "load", l.motors[i].load)
        if abs(l.motors[i].tick - t0) > 40: print("RUNAWAY, torque off"); await l.set_torque([i], False); await l.stop(); return
    for g in (t0 + step, t0):
        await l.write_goals({i: g})
        for k in range(6): await asyncio.sleep(0.25); print(" goal", g, "tick", l.motors[i].tick, "load", l.motors[i].load)
    await l.set_torque([i], False); await asyncio.sleep(0.3); await l.stop(); print("done, torque off")
asyncio.run(main())
