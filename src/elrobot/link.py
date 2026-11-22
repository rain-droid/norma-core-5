"""Hardware link: the one place that talks to the servo bus and cameras.

Two implementations with the same surface:

* `StationLink` talks to a running NormaCore Station over its TCP queue protocol.
* `SimLink` simulates eight servos in-process so the rest of the stack runs with no hardware.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Callable

from .servo import (
    ALL_IDS,
    GRIPPER_ID,
    REG_ACC,
    REG_GOAL_POSITION,
    REG_GOAL_SPEED,
    REG_TORQUE_ENABLE,
    TICKS,
    MotorState,
)

log = logging.getLogger("elrobot.link")

STATE_FRESH_S = 0.5


@dataclass
class Frame:
    jpeg: bytes
    width: int
    height: int
    stamp: float  # time.monotonic() at receipt
    seq: int = 0


@dataclass
class CalibrationProgress:
    status: str = "idle"  # idle | in_progress | done | failed | stopped
    step: int = 0
    total: int = 0
    phase: str = ""
    error: str = ""


@dataclass
class CameraInfo:
    name: str
    unique_id: str = ""
    queue: str = ""
    fps: float = 0.0
    _count: int = 0
    _t0: float = field(default_factory=time.monotonic)


class Link:
    mode = "none"

    def __init__(self) -> None:
        self.motors: dict[int, MotorState] = {}
        self.state_stamp: float = 0.0
        self.bus_serial: str | None = None
        self.frames: dict[str, Frame] = {}
        self.cameras: dict[str, CameraInfo] = {}
        self.raw: dict[int, bytes] = {}  # last register dump per servo
        self.calibration = CalibrationProgress()

    # ---- state

    @property
    def state_age(self) -> float:
        return time.monotonic() - self.state_stamp if self.state_stamp else float("inf")

    @property
    def connected(self) -> bool:
        return self.state_age < STATE_FRESH_S and all(i in self.motors for i in ALL_IDS)

    @property
    def calibrated(self) -> bool:
        return all(i in self.motors and self.motors[i].calibrated for i in ALL_IDS)

    async def start(self) -> None:
        raise NotImplementedError

    async def stop(self) -> None:
        raise NotImplementedError

    async def wait_connected(self, timeout: float = 10.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.connected:
                return True
            await asyncio.sleep(0.05)
        return self.connected

    # ---- writes

    async def sync_write(self, address: int, values: dict[int, bytes]) -> None:
        raise NotImplementedError

    async def write_goals(self, ticks: dict[int, int]) -> None:
        await self.sync_write(REG_GOAL_POSITION, {i: int(t).to_bytes(2, "little") for i, t in ticks.items()})

    async def write_speed(self, speeds: dict[int, int]) -> None:
        """Goal speed in ticks/s (0 = servo maximum)."""
        await self.sync_write(REG_GOAL_SPEED, {i: int(max(0, min(4000, s))).to_bytes(2, "little") for i, s in speeds.items()})

    async def write_acc(self, accs: dict[int, int]) -> None:
        """Acceleration in units of 100 ticks/s^2 (0 = servo maximum)."""
        await self.sync_write(REG_ACC, {i: bytes([max(0, min(254, int(a)))]) for i, a in accs.items()})

    async def set_torque(self, ids, enable: bool) -> None:
        await self.sync_write(REG_TORQUE_ENABLE, {i: (b"\x01" if enable else b"\x00") for i in ids})

    # ---- calibration (real hardware only)

    async def auto_calibrate(self) -> None:
        raise NotImplementedError

    async def stop_auto_calibrate(self) -> None:
        raise NotImplementedError


class StationLink(Link):
    """Follows the Station's `st3215/inference` and camera queues and enqueues servo commands.

    The upstream client applies backpressure all the way to the socket when a consumer is slow,
    which shows up as stale state. So every followed queue is drained by its own task that only
    keeps the newest entry, and the whole connection is rebuilt if state stops arriving.
    """

    mode = "real"

    def __init__(self, host: str = "127.0.0.1:8888", bus_serial: str | None = None, camera_names: dict[str, str] | None = None):
        super().__init__()
        self.host = host
        self._want_serial = bus_serial
        # unique_id (or a substring of it) -> name. Unlisted cameras get cam1, cam2, ...
        self._camera_names = camera_names or {}
        self._client = None
        self._tasks: list[asyncio.Task] = []
        self._supervisor: asyncio.Task | None = None
        self._stopping = False
        self.station_ok = False

    async def start(self) -> None:
        self._stopping = False
        self._supervisor = asyncio.create_task(self._supervise(), name="station-supervisor")

    async def stop(self) -> None:
        self._stopping = True
        for t in [self._supervisor, *self._tasks]:
            if t:
                t.cancel()
        await asyncio.gather(*[t for t in [self._supervisor, *self._tasks] if t], return_exceptions=True)
        self._tasks.clear()

    async def _supervise(self) -> None:
        from . import norma

        while not self._stopping:
            try:
                self._client = await norma.new_station_client(self.host, logging.getLogger("elrobot.station"))
                self.station_ok = True
                log.info("connected to Station at %s", self.host)
                self._tasks = [
                    asyncio.create_task(self._state_loop(), name="station-state"),
                    asyncio.create_task(self._camera_loop(), name="station-cameras"),
                ]
                # Watchdog: if servo state goes quiet, tear everything down and reconnect.
                quiet_since = None
                while not self._stopping:
                    await asyncio.sleep(0.5)
                    if any(t.done() for t in self._tasks):
                        break
                    if self.state_stamp and self.state_age > 3.0:
                        quiet_since = quiet_since or time.monotonic()
                        if time.monotonic() - quiet_since > 3.0:
                            log.warning("no servo state for %.1fs, reconnecting to Station", self.state_age)
                            # One reconnect per outage: with the stamp cleared this does not fire
                            # again until servo state has been seen once more.
                            self.state_stamp = 0.0
                            self.motors = {}
                            break
                    else:
                        quiet_since = None
            except asyncio.CancelledError:
                raise
            except Exception as e:  # connection refused, setup timeout, ...
                log.warning("Station link error: %s", e)
            finally:
                self.station_ok = False
                for t in self._tasks:
                    t.cancel()
                await asyncio.gather(*self._tasks, return_exceptions=True)
                self._tasks = []
                if self._client is not None:
                    try:
                        self._client.close_connection()
                    except Exception:
                        pass
                    self._client = None
            if not self._stopping:
                await asyncio.sleep(1.0)

    @staticmethod
    async def _latest(q: asyncio.Queue):
        """Wait for one entry, then skip ahead to the newest one already queued."""
        entry = await q.get()
        while not q.empty():
            entry = q.get_nowait()
        return entry

    async def _state_loop(self) -> None:
        from .norma import st3215

        q: asyncio.Queue = asyncio.Queue()
        self._client.follow("st3215/inference", q)
        status_names = {0: "idle", 1: "in_progress", 2: "done", 3: "failed", 4: "stopped"}
        while True:
            entry = await self._latest(q)
            if entry is None:
                continue
            state = st3215.InferenceStateReader(memoryview(bytes(entry.Data)))
            bus = self._pick_bus(state.get_buses() or [])
            if bus is None:
                continue
            self.bus_serial = bus.get_bus().get_serial_number()
            motors = {}
            for m in bus.get_motors() or []:
                dump = bytes(m.get_state())
                self.raw[m.get_id()] = dump
                motors[m.get_id()] = MotorState.from_dump(
                    m.get_id(),
                    dump,
                    range_min=m.get_range_min(),
                    range_max=m.get_range_max(),
                    range_frozen=m.get_range_freezed(),
                    has_error=len(dump) == 0,
                    stamp_ns=int(m.get_monotonic_stamp_ns() or 0),
                )
            self.motors = motors
            self.state_stamp = time.monotonic()
            ac = bus.get_auto_calibration()
            if ac is not None:
                self.calibration = CalibrationProgress(
                    status=status_names.get(int(ac.get_status()), "idle"),
                    step=ac.get_current_step(),
                    total=ac.get_total_steps(),
                    phase=ac.get_current_phase() or "",
                    error=ac.get_error_message() or "",
                )

    def _pick_bus(self, buses):
        best = None
        for b in buses:
            info = b.get_bus()
            if info is None:
                continue
            if self._want_serial:
                if info.get_serial_number() == self._want_serial:
                    return b
                continue
            n = len(b.get_motors() or [])
            if best is None or n > len(best.get_motors() or []):
                best = b
        return best

    async def _camera_loop(self) -> None:
        """Discover camera queues from `inference-states` and follow each one."""
        from .norma import drivers, inference

        followed: dict[str, asyncio.Task] = {}
        try:
            while True:
                try:
                    qr = self._client.read_from_tail("inference-states", offset=b"\x00", limit=1, step=1, buf_size=1)
                    e = await asyncio.wait_for(qr.data.get(), timeout=3.0)
                    if e is not None:
                        rx = inference.InferenceRxReader(memoryview(bytes(e.Data)))
                        for x in rx.get_entries() or []:
                            if x.get_type() == drivers.QueueDataType.QDT_USB_VIDEO_FRAMES and x.get_queue() not in followed:
                                followed[x.get_queue()] = asyncio.create_task(self._follow_camera(x.get_queue()))
                except asyncio.TimeoutError:
                    pass
                await asyncio.sleep(3.0)
        finally:
            for t in followed.values():
                t.cancel()

    def _name_camera(self, unique_id: str) -> str:
        for info in self.cameras.values():  # seen before this reconnect: keep its name
            if unique_id and info.unique_id == unique_id:
                return info.name
        for key, name in self._camera_names.items():
            if key and key in unique_id:
                return name
        n = 1
        while f"cam{n}" in self.cameras:
            n += 1
        return f"cam{n}"

    async def _follow_camera(self, queue: str) -> None:
        from .norma import usbvideo

        short = "usbvideo/" + queue.rsplit("/", 1)[-1]
        q: asyncio.Queue = asyncio.Queue()
        self._client.follow(queue, q)
        info: CameraInfo | None = None
        seq = 0
        while True:
            entry = await self._latest(q)
            if entry is None:
                continue
            env = usbvideo.RxEnvelopeReader(memoryview(bytes(entry.Data)))
            if env.get_type() != usbvideo.RxEnvelopeType.ET_FRAMES:
                continue
            pack = env.get_frames()
            data = pack.get_frames_data()
            if not data:
                continue
            jpeg = bytes(data[-1])
            if jpeg[:2] != b"\xff\xd8":
                continue
            if info is None:
                cam = env.get_camera()
                uid = cam.get_unique_id() if cam is not None else short
                info = CameraInfo(name=self._name_camera(uid or short), unique_id=uid or "", queue=short)
                self.cameras[info.name] = info
                log.info("camera %s -> %s (%s)", info.name, uid, short)
            fmt = pack.get_format()
            seq += 1
            now = time.monotonic()
            self.frames[info.name] = Frame(jpeg=jpeg, width=fmt.get_width(), height=fmt.get_height(), stamp=now, seq=seq)
            info._count += 1
            if now - info._t0 >= 2.0:
                info.fps = info._count / (now - info._t0)
                info._count, info._t0 = 0, now

    # ---- writes

    async def _send(self, st_cmd) -> None:
        from .norma import commands, drivers, send_commands

        if self._client is None or not self._client.setup_done:
            raise ConnectionError("not connected to Station")
        drv = commands.DriverCommand(type=drivers.StationCommandType.STC_ST3215_COMMAND, body=st_cmd.encode())
        t0 = time.monotonic()
        await asyncio.wait_for(send_commands(self._client, [drv]), timeout=1.0)
        dt = time.monotonic() - t0
        if dt > 0.08:
            log.warning("slow command enqueue: %.0f ms", dt * 1000)

    async def sync_write(self, address: int, values: dict[int, bytes]) -> None:
        from .norma import st3215

        if not values:
            return
        if not self.bus_serial:
            raise ConnectionError("no servo bus")
        await self._send(
            st3215.Command(
                target_bus_serial=self.bus_serial,
                sync_write=st3215.ST3215SyncWriteCommand(
                    address=address,
                    motors=[st3215.ST3215SyncWriteCommand_MotorWrite(motor_id=i, value=v) for i, v in values.items()],
                ),
            )
        )

    async def auto_calibrate(self) -> None:
        from .norma import st3215

        await self._send(st3215.Command(target_bus_serial=self.bus_serial, auto_calibrate=st3215.AutoCalibrateCommand(calibrate=True)))

    async def stop_auto_calibrate(self) -> None:
        from .norma import st3215

        await self._send(st3215.Command(target_bus_serial=self.bus_serial, stop_auto_calibrate=st3215.StopAutoCalibrateCommand(stop=True)))

    async def reset_calibration(self) -> None:
        """Forget the stored ranges; the Station then learns them from every position it sees."""
        from .norma import st3215

        await self._send(st3215.Command(target_bus_serial=self.bus_serial, reset_calibration=st3215.ResetCalibrationCommand(reset=True)))

    async def freeze_calibration(self) -> None:
        """Save the learned ranges and re-zero each servo so its arc is centred on tick 2048."""
        from .norma import st3215

        await self._send(st3215.Command(target_bus_serial=self.bus_serial, freeze_calibration=st3215.FreezeCalibrationCommand(freeze=True)))


class SimLink(Link):
    """Eight simulated servos. Each tracks its goal with a speed and acceleration limit.

    Ranges are laid out the way a frozen calibration would leave them: centred on 2048 with a
    span equal to the URDF joint range at 4096 ticks per turn.
    """

    mode = "sim"
    MAX_SPEED = 3400.0  # ticks/s

    def __init__(self, joint_limits: dict[int, tuple[float, float]], start_ticks: dict[int, int] | None = None):
        super().__init__()
        self.bus_serial = "SIM"
        self._pos: dict[int, float] = {}
        self._vel: dict[int, float] = {}
        self._speed_limit: dict[int, float] = {}
        self._acc: dict[int, float] = {}
        self._task: asyncio.Task | None = None
        self.frame_provider: Callable[[], dict[str, Frame]] | None = None
        # Optional hook: (gripper_tick) -> tick the jaws stall at, or None. Lets a simulated
        # object block the gripper so grasp verification has something to detect.
        self.gripper_block: Callable[[float], float | None] | None = None
        import math

        for i in ALL_IDS:
            lo, hi = joint_limits[i]
            span = int(round((hi - lo) * TICKS / (2 * math.pi)))
            rmin = 2048 - span // 2
            m = MotorState(id=i, range_min=rmin, range_max=rmin + span, range_frozen=True, volt=12.0, temp=30)
            start = (start_ticks or {}).get(i, 2048)
            m.tick = m.goal_tick = start
            self.motors[i] = m
            self._pos[i] = float(start)
            self._vel[i] = 0.0
            self._speed_limit[i] = self.MAX_SPEED
            self._acc[i] = 0.0

    async def start(self) -> None:
        self.state_stamp = time.monotonic()
        self._task = asyncio.create_task(self._run(), name="sim-servos")

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)

    async def _run(self) -> None:
        dt = 0.01
        last = time.monotonic()
        frame_t = 0.0
        while True:
            await asyncio.sleep(dt)
            now = time.monotonic()
            step = min(0.05, now - last)
            last = now
            for i, m in self.motors.items():
                load = 0
                if m.torque:
                    err = m.goal_tick - self._pos[i]
                    vmax = self._speed_limit[i]
                    # a servo's position loop is roughly proportional with a speed ceiling
                    v_des = max(-vmax, min(vmax, err * 25.0))
                    acc = self._acc[i]
                    if acc > 0:
                        dv = acc * 100.0 * step
                        v_des = max(self._vel[i] - dv, min(self._vel[i] + dv, v_des))
                    self._vel[i] = v_des
                    new = self._pos[i] + v_des * step
                    if i == GRIPPER_ID and self.gripper_block is not None:
                        block = self.gripper_block(new)
                        if block is not None and new < block:  # closing onto an object
                            new = block
                            self._vel[i] = 0.0
                            load = 400
                    self._pos[i] = new
                else:
                    self._vel[i] = 0.0
                m.tick = int(round(self._pos[i])) % TICKS
                m.speed = int(self._vel[i])
                m.moving = abs(self._vel[i]) > 5
                m.load = load
                m.current = int(min(500, abs(self._vel[i]) * 0.02 + load * 0.3 + (3 if m.torque else 0)))
            self.state_stamp = now
            if self.frame_provider is not None and now - frame_t > 0.1:
                frame_t = now
                try:
                    for name, fr in self.frame_provider().items():
                        self.frames[name] = fr
                        self.cameras.setdefault(name, CameraInfo(name=name, fps=10.0))
                except Exception:
                    log.exception("sim frame provider failed")

    async def sync_write(self, address: int, values: dict[int, bytes]) -> None:
        for i, v in values.items():
            m = self.motors.get(i)
            if m is None:
                continue
            if address == REG_GOAL_POSITION:
                m.goal_tick = int.from_bytes(v[:2], "little") % TICKS
            elif address == REG_TORQUE_ENABLE:
                m.torque = v[0] != 0
                if m.torque:
                    m.goal_tick = m.tick
            elif address == REG_GOAL_SPEED:
                s = int.from_bytes(v[:2], "little")
                self._speed_limit[i] = float(s) if s > 0 else self.MAX_SPEED
            elif address == REG_ACC:
                self._acc[i] = float(v[0])

    async def auto_calibrate(self) -> None:
        self.calibration = CalibrationProgress(status="done", step=32, total=32, phase="simulated")

    async def stop_auto_calibrate(self) -> None:
        pass
