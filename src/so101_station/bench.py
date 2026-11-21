"""One-degree, single-servo wrist test. No table map or other motor commands."""
from __future__ import annotations

import argparse
import asyncio
import fcntl
import hashlib
import json
import math
import signal
import time
from pathlib import Path

from .calibration import load_calibration
from .cleanup.backend import StationBackend
from .cleanup.geometry import ArmModel, CleanupError
from .station import _buses, inspect_state


async def wrist_test(backend, *, execute=False, expected_tick=None, delta_deg=-1.0,
                     stopped=None, pause=asyncio.sleep):
    """Enable only motor 5, move <=1 degree and return; retain wrist torque."""
    if not math.isfinite(delta_deg) or not .5 <= abs(delta_deg) <= 1:
        raise CleanupError("wrist test requires 0.5 to 1 degree")
    stopped = stopped or asyncio.Event()
    joint = backend.model.calibration.joint("wrist_roll")
    initial = backend.state()
    start = initial["ticks"][5]
    target = joint.nudge(start, delta_deg)
    report = {"bus": backend.bus, "motor_id": 5, "start_tick": start, "target_tick": target,
              "delta_deg": (target-start)*360/4096, "execute": execute}
    if not execute:
        return report
    if expected_tick is None or abs(start-expected_tick) > 3:
        raise CleanupError("wrist moved since inspection; supply a fresh --expected-tick")
    rows = {r["motor_id"]: r for r in initial["report"]["joints"]}
    if any(rows[i]["torque"] for i in range(1, 7) if i != 5):
        raise CleanupError("first wrist test requires the other five motors to remain unpowered and supported")
    if min(start, target) < joint.range_min+12 or max(start, target) > joint.range_max-12:
        raise CleanupError("wrist too close to an endpoint for the bench test")

    samples = []
    active = False

    def checked():
        if stopped.is_set():
            raise CleanupError("wrist test stopped")
        state = backend.state()
        for row in state["report"]["joints"]:
            i = row["motor_id"]
            if i != 5 and (row["torque"] or abs(state["ticks"][i]-initial["ticks"][i]) > 15):
                raise CleanupError("another joint moved or enabled; stop and support the arm")
            if row["temperature_c"] >= 60 or not 6 <= row["voltage_v"] <= 13:
                raise CleanupError("servo temperature/voltage unsuitable for bench test")
        tick = state["ticks"][5]
        if not min(start,target)-5 <= tick <= max(start,target)+5:
            raise CleanupError("wrist moved outside the small test interval")
        if active and not state["report"]["joints"][4]["torque"]:
            raise CleanupError("wrist torque unexpectedly disabled")
        samples.append(tick)
        return state

    async def write(address, value, size):
        checked()
        await backend.write(address, {5: int(value).to_bytes(size,"little")}, stopped)

    async def wait_for(predicate, message, seconds=2):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            state = checked()
            if predicate(state):
                return state
            await pause(.05)
        raise CleanupError(message)

    def ram():
        bus = next(b for b in _buses(backend.latest) if b.get_bus().get_serial_number() == backend.bus)
        return bytes(next(m for m in bus.get_motors() if m.get_id() == 5).get_state())

    async def move(goal):
        current = checked()["ticks"][5]
        step = 1 if goal > current else -1
        for tick in range(current+step, goal+step, step):
            joint.validate_tick(tick)
            await write(0x2A,tick,2)
            await pause(.1)  # <=10 ticks/s, approximately 0.88 degrees/s.
        settled = await wait_for(lambda s: abs(s["ticks"][5]-goal)<=2,
                                 "wrist did not reach the target; no further movement")
        return settled["ticks"][5]

    try:
        async with asyncio.timeout(15):
            checked()
            # Seed current position before enabling ONLY the wrist. RAM only.
            await write(0x2A,start,2)
            await write(0x2E,20,2)
            await write(0x29,5,1)
            original_cap = int.from_bytes(ram()[0x30:0x32],"little")
            if original_cap <= 0:
                raise CleanupError("wrist RAM torque limit is zero")
            cap = min(original_cap,200)
            await write(0x30,cap,2)
            def configured(_):
                data=ram()
                return (int.from_bytes(data[0x2A:0x2C],"little")==start
                        and int.from_bytes(data[0x2E:0x30],"little")==20
                        and int.from_bytes(data[0x30:0x32],"little")==cap and data[0x29]==5)
            await wait_for(configured,"wrist configuration not confirmed")
            if abs(checked()["ticks"][5]-start)>3:
                raise CleanupError("wrist moved before torque enable")
            await write(0x28,1,1)
            await wait_for(lambda s:s["report"]["joints"][4]["torque"],"wrist torque enable not confirmed")
            active=True
            reached=await move(target)
            await pause(.4)
            returned=await move(start)
            report.update(status="moved_and_returned", reached_tick=reached, returned_tick=returned,
                          measured_delta_deg=(reached-start)*360/4096, torque_limit=cap,
                          wrist_torque_left_on=True, other_motors="unchanged/off", samples=samples)
            return report
    except BaseException as exc:
        # Hold only the wrist, even if a passive shoulder drifted out of bounds.
        hold_error = None
        try:
            r=inspect_state(backend.model.calibration,backend.latest,backend.bus)
            wrist=next(j for j in r["joints"] if j["motor_id"]==5)
            if wrist["issues"]:
                raise CleanupError("cannot hold wrist without valid fresh telemetry")
            joint.validate_tick(wrist["tick"])
            await backend.write(0x2A,{5:wrist["tick"].to_bytes(2,"little")})
        except Exception as err:
            hold_error=str(err)
        if isinstance(exc,asyncio.CancelledError):
            raise
        raise CleanupError(f"{exc or 'test timed out'}; wrist hold error={hold_error}") from exc


async def run(args):
    identity=f"{args.host}/{args.bus_serial}".encode()
    name=hashlib.sha256(identity).hexdigest()[:16]
    with open(Path('/tmp')/f"so101-cleanup-{name}.lock",'a') as lease:
        fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
        model=ArmModel(load_calibration())
        backend=StationBackend(args.host,args.bus_serial,None,model,args.execute)
        stopped=asyncio.Event()
        loop=asyncio.get_running_loop()
        for sig in (signal.SIGINT,signal.SIGTERM):
            loop.add_signal_handler(sig,stopped.set)
        try:
            await backend.start()
            return await wrist_test(backend,execute=args.execute,expected_tick=args.expected_tick,
                                    delta_deg=args.delta_deg,stopped=stopped)
        finally:
            await backend.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='127.0.0.1:8888')
    parser.add_argument('--bus-serial',required=True)
    parser.add_argument('--delta-deg',type=float,default=-1)
    parser.add_argument('--expected-tick',type=int)
    parser.add_argument('--execute',action='store_true',help='operator has supported the arm and cleared the wrist')
    parser.add_argument('--out',type=Path,default=Path('logs/so101-first-control/wrist-test.json'))
    args=parser.parse_args()
    try:
        result=asyncio.run(run(args))
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))
        return 0
    except (CleanupError,OSError,ValueError,TimeoutError) as exc:
        print(json.dumps({'error':str(exc)}))
        return 1


if __name__=='__main__':
    raise SystemExit(main())
