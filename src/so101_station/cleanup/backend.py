"""Persistent Station transport and a hardware-free tabletop fixture."""
from __future__ import annotations

import asyncio
import io
import math
import time
from dataclasses import dataclass

import numpy as np
from PIL import Image, ImageDraw

from ..station import _client, _protocol, inspect_state
from .geometry import ArmModel, CleanupError


@dataclass
class CameraFrame:
    jpeg: bytes
    width: int
    height: int
    camera_id: str
    captured_ns: int


class StationBackend:
    mode = "real"

    def __init__(self, host, bus_serial, camera_queue, model: ArmModel, enable_motion=False):
        if not bus_serial:
            raise CleanupError("real backend requires an explicit bus serial")
        self.host, self.bus, self.camera_queue = host, bus_serial, camera_queue
        self.model, self.enabled = model, enable_motion
        self.client = None
        self.latest = None
        self.task = None
        self.lock = asyncio.Lock()

    async def start(self):
        self.client = _client(self.host, allow_commands=self.enabled)
        await self.client.wait_ready(timeout=5)
        stream = self.client.read_from_tail("st3215/inference", offset=b"\x00", limit=0, step=1, buf_size=16)
        async def pump():
            while True:
                entry = await stream.data.get()
                if entry is None:
                    self.latest = None
                    return
                self.latest = bytes(entry.Data)
        self.task = asyncio.create_task(pump())
        async with asyncio.timeout(5):
            while self.latest is None:
                await asyncio.sleep(0.02)
        self.state()

    async def close(self):
        if self.task:
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
        if self.client:
            await self.client.close()

    def state(self):
        if self.latest is None:
            raise CleanupError("Station has no motor state")
        report = inspect_state(self.model.calibration, self.latest, self.bus)
        if not report["profile_matches_live_state"]:
            raise CleanupError("; ".join(report["issues"]))
        return {"mode": self.mode, "q": [j["angle_rad"] for j in report["joints"][:5]],
                "ticks": {j["motor_id"]: j["tick"] for j in report["joints"]},
                "torque": all(j["torque"] for j in report["joints"]), "report": report}

    async def look(self, queue=None):
        queue = queue or self.camera_queue
        if not queue:
            raise CleanupError("camera capture requires an explicit camera queue")
        stream = self.client.read_from_tail(queue, offset=b"\x00", limit=1, step=1, buf_size=2)
        entry = await asyncio.wait_for(stream.data.get(), 3)
        if entry is None:
            raise CleanupError("camera queue ended without a frame")
        env = _protocol().usbvideo.RxEnvelopeReader(memoryview(bytes(entry.Data)))
        if env.get_type() != _protocol().usbvideo.RxEnvelopeType.ET_FRAMES:
            raise CleanupError("camera returned an error/non-frame envelope")
        pack = env.get_frames()
        frames, stamps = pack.get_frames_data(), pack.get_stamps()
        if not frames or len(stamps) != len(frames):
            raise CleanupError("camera frame/timestamp mismatch")
        stamp = stamps[-1].get_local_stamp_ns()
        age = (time.time_ns() - stamp) / 1e9
        if not -0.1 <= age <= 0.75:
            raise CleanupError("camera sample is stale or clocks disagree")
        jpeg = bytes(frames[-1])
        try:
            image = Image.open(io.BytesIO(jpeg))
            image.load()
        except Exception as exc:
            raise CleanupError("Station camera must provide a decodable JPEG") from exc
        if image.format != "JPEG":
            raise CleanupError("Station camera did not provide JPEG")
        return CameraFrame(jpeg, *image.size, env.get_camera().get_unique_id(), stamp)

    async def discover_cameras(self, identities):
        stream = self.client.read_from_tail("inference-states", offset=b"\x00", limit=1, step=1, buf_size=2)
        entry = await asyncio.wait_for(stream.data.get(), 3)
        if entry is None:
            raise CleanupError("camera directory unavailable")
        n = _protocol()
        directory = n.inference.InferenceRxReader(memoryview(bytes(entry.Data)))
        queues = [e.get_queue() for e in directory.get_entries()
                  if e.get_type() == n.drivers.QueueDataType.QDT_USB_VIDEO_FRAMES]
        found = {}
        for queue in queues:
            try:
                frame = await self.look(queue)
            except (ValueError, TimeoutError):
                continue
            for name, identity in identities.items():
                if frame.camera_id == identity:
                    if name in found:
                        raise CleanupError(f"ambiguous camera identity: {identity}")
                    found[name] = queue
        if set(found) != set(identities):
            raise CleanupError(f"missing cameras: {sorted(set(identities) - set(found))}")
        return found

    async def disable(self):
        await self.write(0x28, {i: b"\x00" for i in range(1, 7)})
        async with asyncio.timeout(2):
            while any(j["torque"] for j in self.state()["report"]["joints"]):
                await asyncio.sleep(.05)

    async def write(self, address, values, stop=None):
        if not self.enabled:
            raise CleanupError("real motion is disabled; start with --enable-motion after setup")
        if address not in {0x28, 0x29, 0x2A, 0x2E, 0x30} or not set(values) <= {1, 2, 3, 4, 5, 6}:
            raise CleanupError("cleanup only permits the selected servo RAM control registers")
        expected_size = 1 if address in {0x28, 0x29} else 2
        if not values or any(len(v) != expected_size for v in values.values()):
            raise CleanupError("invalid servo register write size")
        async with self.lock:
            if stop is not None and stop.is_set():
                raise CleanupError("stopped")
            n = _protocol()
            command = n.st3215.Command(target_bus_serial=self.bus, sync_write=n.st3215.ST3215SyncWriteCommand(
                address=address, motors=[n.st3215.ST3215SyncWriteCommand_MotorWrite(motor_id=i, value=v) for i, v in values.items()]))
            driver = n.commands.DriverCommand(type=n.drivers.StationCommandType.STC_ST3215_COMMAND, body=command.encode())
            await asyncio.wait_for(n.send_commands(self.client, [driver]), 1)

    async def goals(self, ticks, stop=None):
        if set(ticks) != {1, 2, 3, 4, 5, 6}:
            raise CleanupError("every servo goal must include all six motors")
        for j in self.model.calibration.joints:
            j.validate_tick(ticks[j.motor_id])
        await self.write(0x2A, {i: int(t).to_bytes(2, "little") for i, t in ticks.items()}, stop)

    async def enable(self, stop, speed_rad_s=.15):
        if not math.isfinite(speed_rad_s) or not 0 < speed_rad_s <= .45:
            raise CleanupError("speed must be in (0, 0.45] rad/s")
        state = self.state()
        await self.goals(state["ticks"], stop)
        speed = round(100 * speed_rad_s / .15)
        await self.write(0x2E, {i: (speed if i < 6 else 200).to_bytes(2, "little") for i in range(1, 7)}, stop)
        await self.write(0x29, {i: bytes([min(30, max(1, round(speed / 10)))]) for i in range(1, 7)}, stop)
        await self.write(0x30, {6: (500).to_bytes(2, "little")}, stop)
        await self.write(0x28, {i: b"\x01" for i in range(1, 7)}, stop)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if stop.is_set():
                raise CleanupError("stopped")
            if self.state()["torque"]:
                return
            await asyncio.sleep(0.05)
        raise CleanupError("torque enable not confirmed by telemetry")

    async def hold(self):
        # No torque-off on stop: the arm and a held object must not be dropped.
        await self.goals(self.state()["ticks"])


def mock_workspace():
    # Synthetic camera mapping and small reachable region; NEVER a real calibration.
    world = [[0.00, -0.22], [0.12, -0.22], [0.12, -0.12], [0.00, -0.12], [0.06, -0.22], [0.06, -0.12]]
    pixels = [[80 + x * 4000, 60 + (y + .22) * 3000] for x, y in world]
    return {"sim_only": True, "camera_fixed": True, "camera_id": "mock-overhead", "image_size": [640, 480],
            "pixel_points": pixels, "table_points_m": world, "workspace_polygon_m": world[:4],
            "drop_xy_m": [.10, -.14], "table_z_m": 0, "grasp_tip_z_m": .008,
            "travel_z_m": .075, "drop_tip_z_m": .04}


class MockBackend:
    mode = "sim"
    enabled = True

    def __init__(self, model):
        self.model = model
        self.q = model.solve([.04, -.18, .075], np.zeros(5))
        self.grip = 3484
        self.torque = False
        self.holding = None
        self.objects = [{"label": "crumpled_paper", "xy": [.025, -.19]}, {"label": "snack_wrapper", "xy": [.075, -.205]}]
        self.commands = []
        self.stale = False

    async def start(self):
        pass

    async def close(self):
        pass

    def state(self):
        if self.stale:
            raise CleanupError("synthetic stale motor state")
        return {"mode": "sim", "q": self.q.tolist(), "ticks": self.model.ticks(self.q, self.grip), "torque": self.torque}

    async def enable(self, stop, speed_rad_s=.15):
        if stop.is_set():
            raise CleanupError("stopped")
        self.torque = True

    async def goals(self, ticks, stop=None):
        if stop is not None and stop.is_set():
            raise CleanupError("stopped")
        self.commands.append(dict(ticks))
        self.q = np.array([j.ticks_to_radians(ticks[j.motor_id]) for j in self.model.joints])
        pos = self.model.fk(self.q)[:3, 3]
        if ticks[6] < self.grip and pos[2] < .035 and self.holding is None:
            self.holding = next((o for o in self.objects if np.linalg.norm(np.array(o["xy"]) - pos[:2]) < .02), None)
        if self.holding is not None:
            self.holding["xy"] = pos[:2].tolist()
            self.grip = max(2700, ticks[6])
            if ticks[6] > 3300:
                self.holding = None
                self.grip = ticks[6]
        else:
            self.grip = ticks[6]
        await asyncio.sleep(0)

    async def hold(self):
        self.commands.append({"hold": True})

    async def disable(self):
        self.torque = False

    async def discover_cameras(self, identities):
        return dict(identities)

    async def look(self, queue=None):
        image = Image.new("RGB", (640, 480), "#c5b59a")
        draw = ImageDraw.Draw(image)
        draw.rectangle((400, 250, 600, 430), fill="#53686c", outline="black", width=3)
        draw.text((460, 405), "TRASH TRAY", fill="white")
        for obj in self.objects:
            x, y = obj["xy"]
            u, v = 80 + x * 4000, 60 + (y + .22) * 3000
            if obj["label"] == "crumpled_paper":
                draw.polygon([(u-20,v-10),(u-6,v-22),(u+18,v-12),(u+22,v+10),(u,v+20),(u-22,v+5)], fill="white", outline="#777777")
                draw.line((u-15,v-8,u+12,v+12,u-4,v-14), fill="#999999", width=2)
            else:
                draw.polygon([(u-27,v-12),(u+24,v-16),(u+28,v+12),(u-23,v+14)], fill="#df3c26", outline="white")
                draw.text((u-19,v-6), "SNACK", fill="#fff0c0")
        draw.text((12, 450), "SIMULATED TABLE — NO HARDWARE", fill="black")
        buf = io.BytesIO(); image.save(buf, format="JPEG")
        return CameraFrame(buf.getvalue(), 640, 480, queue or "mock-overhead", time.time_ns())
