"""Read Station telemetry and compare EEPROM to the imported SO-101 profile.

Snapshot commands remain read-only. The cleanup executor can explicitly request
a command-enabled transport; it restricts writes to servo RAM registers. This
module never constructs an ElRobot Robot or StationLink.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from .calibration import Calibration, CalibrationError


def _protocol():
    from elrobot import norma

    return norma


def _client(host: str, *, allow_commands: bool = False):
    _protocol()  # Make the vendored client/generated modules importable.
    from software.station.shared.station_py.client import Client

    class SnapshotClient(Client):
        def __init__(self, address):
            self.owned_tasks = set()
            super().__init__(address, logging.getLogger("so101.station"))

        async def manage_connection(self):
            task = asyncio.current_task()
            self.owned_tasks.add(task)
            try:
                await super().manage_connection()
            finally:
                self.owned_tasks.discard(task)

        async def _read_async(self, *args, **kwargs):
            task = asyncio.current_task()
            self.owned_tasks.add(task)
            try:
                await super()._read_async(*args, **kwargs)
            finally:
                self.owned_tasks.discard(task)

        async def _enqueue(self, *args, **kwargs):
            if not allow_commands:
                raise PermissionError("SO-101 snapshot client is read-only")
            return await super()._enqueue(*args, **kwargs)

        async def close(self):
            self.close_connection()
            tasks = tuple(self.owned_tasks)
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    return SnapshotClient(host if ":" in host else f"{host}:8888")


async def read_state(host: str, timeout: float = 5.0, *, client_factory=None) -> bytes:
    """Fetch the newest stored inference entry; callers must check its age."""
    if not math.isfinite(timeout) or not 0 < timeout <= 60:
        raise CalibrationError("timeout must be in (0, 60] seconds")
    client = (client_factory or _client)(host)
    try:
        async with asyncio.timeout(timeout):
            await client.wait_ready(timeout=timeout)
            stream = client.read_from_tail("st3215/inference", offset=b"\x00", limit=1, step=1, buf_size=2)
            entry = await stream.data.get()
            if entry is None:
                raise CalibrationError(f"Station motor stream ended: {stream.err or 'no state'}")
            return bytes(entry.Data)
    finally:
        await client.close()


def _buses(payload: bytes):
    return _protocol().st3215.InferenceStateReader(memoryview(payload)).get_buses()


def discover(payload: bytes) -> dict:
    return {"read_only": True, "buses": [
        {"bus_serial": b.get_bus().get_serial_number(), "port": b.get_bus().get_port_name(),
         "motor_ids": [m.get_id() for m in b.get_motors()]}
        for b in _buses(payload)
    ]}


def _u16(data: bytes, address: int) -> int:
    return int.from_bytes(data[address:address + 2], "little")


def decode_homing_offset(raw: int) -> int:
    # STS uses sign-magnitude with bit 11 as the sign, not signed int16.
    if type(raw) is not int or not 0 <= raw < 4096:
        raise CalibrationError("invalid STS homing-offset encoding")
    return -(raw & 0x7FF) if raw & 0x800 else raw


def inspect_state(calibration: Calibration, payload: bytes, bus_serial: str, *, now_ns: int | None = None) -> dict:
    if not bus_serial or not bus_serial.strip():
        raise CalibrationError("select an explicit SO-101 bus serial")
    buses = _buses(payload)
    matches = [b for b in buses if b.get_bus().get_serial_number() == bus_serial]
    if len(matches) != 1:
        available = [b.get_bus().get_serial_number() for b in buses]
        raise CalibrationError(f"expected one bus {bus_serial!r}, found {len(matches)}; available: {available}")
    motors = matches[0].get_motors()
    ids = [m.get_id() for m in motors]
    expected_ids = {j.motor_id for j in calibration.joints}
    issues = []
    if len(ids) != len(set(ids)) or set(ids) != expected_ids:
        issues.append(f"expected exactly motor IDs {sorted(expected_ids)}, found {ids}")
    by_id = {m.get_id(): m for m in motors}
    now = time.time_ns() if now_ns is None else now_ns
    readings = []
    for joint in calibration.joints:
        motor = by_id.get(joint.motor_id)
        if motor is None:
            continue
        row = {"name": joint.name, "motor_id": joint.motor_id}
        problems = []
        stamp = motor.get_system_stamp_ns()
        age_s = (now - stamp) / 1e9
        row["age_ms"] = round(age_s * 1000, 1) if stamp else None
        if stamp <= 0 or not -0.1 <= age_s <= 0.5:
            problems.append("stale/missing timestamp or Station clock differs from this host")
        data = bytes(motor.get_state())
        if len(data) < 0x47:
            problems.append(f"incomplete servo register dump ({len(data)} bytes)")
        else:
            tick = _u16(data, 0x38)
            offset_raw = _u16(data, 0x1F)
            try:
                offset = decode_homing_offset(offset_raw)
            except CalibrationError as exc:
                offset = None
                problems.append(str(exc))
            eeprom_range = [_u16(data, 0x09), _u16(data, 0x0B)]
            row.update({
                "tick": tick, "homing_offset": offset, "expected_homing_offset": joint.homing_offset,
                "eeprom_tick_limits": eeprom_range, "expected_tick_limits": [joint.range_min, joint.range_max],
                "station_tick_limits": [motor.get_range_min(), motor.get_range_max()],
                "station_range_frozen": motor.get_range_freezed(),
                "torque": bool(data[0x28]), "temperature_c": data[0x3F], "voltage_v": data[0x3E] / 10,
                "servo_status": data[0x41],
                "angle_rad": joint.ticks_to_radians(tick) if 0 <= tick <= 4095 else None,
            })
            if data[0x05] != joint.motor_id:
                problems.append("register motor ID differs from Station motor ID")
            if offset != joint.homing_offset:
                problems.append("EEPROM homing offset differs from profile")
            if eeprom_range != [joint.range_min, joint.range_max]:
                problems.append("EEPROM limits differ from profile")
            if data[0x21] != 0:
                problems.append("servo is not in position mode")
            # The generated reader has no has_error(). Kind 0 is an I/O error,
            # so test field presence rather than treating zero as success.
            if data[0x41] != 0 or motor._error_buf is not None:
                problems.append("servo/Station reports an error")
            try:
                joint.validate_tick(tick)
            except CalibrationError as exc:
                problems.append(str(exc))
        row["issues"] = problems
        issues.extend(f"{joint.name}: {problem}" for problem in problems)
        readings.append(row)
    return {"read_only": True, "bus_serial": bus_serial, "motion_enabled": False,
            "profile_matches_live_state": not issues, "issues": issues, "joints": readings}
