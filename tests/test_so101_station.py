"""Exercise actual protobuf/register decoding with synthetic Station telemetry."""

import asyncio
import struct
import unittest
from types import SimpleNamespace

from elrobot.norma import st3215
from so101_station.calibration import CalibrationError, load_calibration
from so101_station.station import _client, decode_homing_offset, discover, inspect_state, read_state

NOW = 1_795_000_000_000_000_000


def packet(*, change=None, extra=False, stamp=NOW):
    motors = []
    for j in load_calibration().joints:
        data = bytearray(0x47)
        data[0x05] = j.motor_id
        struct.pack_into("<HH", data, 0x09, j.range_min, j.range_max)
        offset = abs(j.homing_offset) | (0x800 if j.homing_offset < 0 else 0)
        struct.pack_into("<H", data, 0x1F, offset)
        data[0x28] = 1
        data[0x3E] = 75
        data[0x3F] = 30
        struct.pack_into("<H", data, 0x38, j.center_tick)
        motor = st3215.InferenceState_MotorState(id=j.motor_id, system_stamp_ns=stamp, state=bytes(data),
                                                range_min=j.range_min, range_max=j.range_max, range_freezed=True)
        motors.append(motor)
    if change:
        change(motors)
    if extra:
        motors.append(st3215.InferenceState_MotorState(id=8))
    bus = st3215.InferenceState_BusState(bus=st3215.ST3215Bus(serial_number="SO101-TEST"), motors=motors)
    return st3215.InferenceState(buses=[bus]).encode()


class StationTests(unittest.TestCase):
    def inspect(self, payload):
        return inspect_state(load_calibration(), payload, "SO101-TEST", now_ns=NOW)

    def test_station_decode_matches_profile_with_negative_eeprom_offsets(self):
        result = self.inspect(packet())
        self.assertTrue(result["profile_matches_live_state"], result["issues"])
        self.assertFalse(result["motion_enabled"])
        shoulder = result["joints"][1]
        self.assertEqual(shoulder["homing_offset"], -573)
        self.assertEqual(shoulder["angle_rad"], 0)
        self.assertEqual(result["joints"][5]["motor_id"], 6)
        self.assertEqual(discover(packet())["buses"][0]["motor_ids"], [1, 2, 3, 4, 5, 6])

    def test_wrong_missing_or_extra_bus_motors_fail(self):
        for payload in (packet(extra=True), packet(change=lambda ms: ms.pop()),
                        packet(change=lambda ms: ms.append(ms[0]))):
            self.assertFalse(self.inspect(payload)["profile_matches_live_state"])
        for serial in ("", "OTHER-ROBOT"):
            with self.assertRaises(CalibrationError):
                inspect_state(load_calibration(), packet(), serial, now_ns=NOW)

    def test_changed_eeprom_and_out_of_range_positions_fail(self):
        for address, value in ((0x1F, 0), (0x09, 0), (0x38, 0), (0x38, 0xFFFF)):
            def change(motors):
                data = bytearray(motors[0].state)
                struct.pack_into("<H", data, address, value)
                motors[0].state = bytes(data)
            with self.subTest(address=address, value=value):
                self.assertFalse(self.inspect(packet(change=change))["profile_matches_live_state"])

    def test_stale_future_and_missing_timestamps_fail(self):
        for stamp in (0, NOW - 1_000_000_000, NOW + 1_000_000_000):
            self.assertFalse(self.inspect(packet(stamp=stamp))["profile_matches_live_state"])

    def test_partial_registers_and_error_kind_zero_fail(self):
        def short(motors):
            motors[0].state = b"\x00" * 10
        def io_error(motors):
            motors[0].error = st3215.ST3215Error(kind=st3215.ST3215Error_ST3215ErrorKind.SEK_IO,
                                               description="synthetic I/O error")
        for change in (short, io_error):
            self.assertFalse(self.inspect(packet(change=change))["profile_matches_live_state"])

    def test_sts_offset_is_sign_magnitude(self):
        self.assertEqual(decode_homing_offset(0x800 | 573), -573)
        self.assertEqual(decode_homing_offset(242), 242)
        with self.assertRaises(CalibrationError):
            decode_homing_offset(0xFFFF)

    def test_read_is_bounded_and_always_closes(self):
        class FakeClient:
            def __init__(self, host):
                self.closed = False
                self.stream = SimpleNamespace(data=asyncio.Queue(), err=None)
            async def wait_ready(self, timeout):
                pass
            def read_from_tail(self, queue, **kwargs):
                self.request = (queue, kwargs)
                return self.stream
            async def close(self):
                self.closed = True

        async def run():
            client = FakeClient("test")
            client.stream.data.put_nowait(SimpleNamespace(Data=packet()))
            result = await read_state("test", client_factory=lambda _: client)
            self.assertEqual(result, packet())
            self.assertTrue(client.closed)
            self.assertEqual(client.request[0], "st3215/inference")
            self.assertEqual(client.request[1]["limit"], 1)
            client = FakeClient("test")
            with self.assertRaises(TimeoutError):
                await read_state("test", timeout=0.01, client_factory=lambda _: client)
            self.assertTrue(client.closed)
            client = FakeClient("test")
            client.stream.data.put_nowait(None)
            with self.assertRaises(CalibrationError):
                await read_state("test", client_factory=lambda _: client)
            self.assertTrue(client.closed)
        asyncio.run(run())

    def test_real_snapshot_client_rejects_enqueue_without_connecting(self):
        # Replace connection management, but exercise the actual read-only class.
        from unittest.mock import patch
        from software.station.shared.station_py.client import Client

        async def no_connection(self):
            pass
        async def run():
            with patch.object(Client, "manage_connection", no_connection):
                client = _client("unused")
                await asyncio.sleep(0)
                try:
                    with self.assertRaises(PermissionError):
                        await client.enqueue("commands", b"not a real command")
                finally:
                    await client.close()
        asyncio.run(run())


if __name__ == "__main__":
    unittest.main()
