"""Diagnostic: dump what the Station sees (servos, calibration, cameras) and save one frame per camera.

    uv run python scripts/probe.py [--out logs/probe] [--host 127.0.0.1:8888]
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import time
from pathlib import Path

from elrobot.norma import drivers, inference, new_station_client, st3215, usbvideo
from elrobot.servo import MotorState


async def main(host: str, out: Path) -> None:
    log = logging.getLogger("probe")
    client = await new_station_client(host, log)

    q: asyncio.Queue = asyncio.Queue()
    client.follow("st3215/inference", q)
    entry = await asyncio.wait_for(q.get(), timeout=5)
    # count how fast snapshots arrive
    t0 = time.monotonic()
    n = 1
    while time.monotonic() - t0 < 1.0:
        try:
            entry = await asyncio.wait_for(q.get(), timeout=0.2)
            n += 1
        except asyncio.TimeoutError:
            break
    print(f"st3215/inference: {n} snapshots/s, {len(bytes(entry.Data))} bytes each")

    state = st3215.InferenceStateReader(memoryview(bytes(entry.Data)))
    for bus in state.get_buses():
        info = bus.get_bus()
        print(f"\nbus serial={info.get_serial_number()!r} port={info.get_port_name()} "
              f"vid:pid={info.get_vid():04x}:{info.get_pid():04x} baud={info.get_port_baud_rate()}")
        ac = bus.get_auto_calibration()
        if ac is not None:
            print(f"  auto-calibration: status={ac.get_status().name} step {ac.get_current_step()}/{ac.get_total_steps()} "
                  f"phase={ac.get_current_phase()!r} err={ac.get_error_message()!r}")
        print("  id  tick  goal torque  cur load  volt temp status  range_min range_max frozen  span")
        for m in bus.get_motors():
            s = MotorState.from_dump(m.get_id(), bytes(m.get_state()))
            rmin, rmax = m.get_range_min(), m.get_range_max()
            err = m.get_error()
            print(f"  {s.id:2d} {s.tick:5d} {s.goal_tick:5d} {str(s.torque):5s} {s.current:4d} {s.load:4d} "
                  f"{s.volt:5.1f} {s.temp:4d} {s.status:6d}  {rmin:9d} {rmax:9d} {str(m.get_range_freezed()):6s} "
                  f"{(rmax - rmin) % 4096:5d}" + (f"  ERROR {err.get_kind().name}" if err is not None and len(bytes(m.get_state())) == 0 else ""))

    # cameras: the inference-states queue lists every live queue with its type
    qr = client.read_from_tail("inference-states", offset=b"\x00", limit=1, step=1, buf_size=1)
    e = await asyncio.wait_for(qr.data.get(), timeout=5)
    rx = inference.InferenceRxReader(memoryview(bytes(e.Data)))
    cam_queues = [x.get_queue() for x in rx.get_entries() if x.get_type() == drivers.QueueDataType.QDT_USB_VIDEO_FRAMES]
    print(f"\nqueues: {[(x.get_queue(), x.get_type().name) for x in rx.get_entries()]}")
    out.mkdir(parents=True, exist_ok=True)
    for cq in cam_queues:
        fq: asyncio.Queue = asyncio.Queue()
        client.follow(cq, fq)
        t0 = time.monotonic()
        frames = 0
        last = None
        cam = None
        while time.monotonic() - t0 < 2.0:
            try:
                fe = await asyncio.wait_for(fq.get(), timeout=1.0)
            except asyncio.TimeoutError:
                break
            env = usbvideo.RxEnvelopeReader(memoryview(bytes(fe.Data)))
            if env.get_type() != usbvideo.RxEnvelopeType.ET_FRAMES:
                print(f"  {cq}: envelope {env.get_type().name} {env.get_error()!r}")
                continue
            pack = env.get_frames()
            data = pack.get_frames_data()
            frames += len(data)
            last = (bytes(data[-1]), pack.get_format().get_width(), pack.get_format().get_height())
            cam = env.get_camera()
        if last:
            name = cq.split("/")[-1][:8]
            path = out / f"cam_{name}.jpg"
            path.write_bytes(last[0])
            desc = f"unique_id={cam.get_unique_id()} product={cam.get_product()!r}" if cam is not None else ""
            print(f"  {cq}: {frames / 2:.1f} fps, {last[1]}x{last[2]}, {len(last[0]) // 1024} KB -> {path}  {desc}")
        else:
            print(f"  {cq}: no frames")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1:8888")
    ap.add_argument("--out", default="logs/probe")
    a = ap.parse_args()
    logging.basicConfig(level=logging.WARNING)
    asyncio.run(main(a.host, Path(a.out)))
