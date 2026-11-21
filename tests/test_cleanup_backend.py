import asyncio
import io
import time
from types import SimpleNamespace
from unittest.mock import patch

import httpx
import pytest
from PIL import Image

from elrobot.norma import st3215, usbvideo
from target.gen_python.protobuf.drivers.usbvideo import frame
from so101_station.cleanup.backend import StationBackend, MockBackend, mock_workspace
from so101_station.cleanup.geometry import ArmModel, CleanupError, TableMap
from so101_station.cleanup.harness import Harness
from so101_station.cleanup.service import create_app


def test_station_writes_target_exact_bus_and_all_six_motors_only():
    async def run():
        model = ArmModel()
        backend = StationBackend("unused", "SO101-ONLY", "camera", model, True)
        backend.client = object()
        captured = []
        async def send(client, commands):
            captured.extend(commands)
        with patch("elrobot.norma.send_commands", send):
            await backend.goals(model.ticks([0]*5,3484))
            cmd = st3215.CommandReader(memoryview(captured[0].body))
            assert cmd.get_target_bus_serial() == "SO101-ONLY"
            write = cmd.get_sync_write()
            assert write.get_address() == 0x2A
            assert {m.get_motor_id() for m in write.get_motors()} == {1,2,3,4,5,6}
            with pytest.raises(CleanupError):
                await backend.write(0x1F,{1:b"\x00\x00"})
            with pytest.raises(CleanupError):
                await backend.goals({1:2048})
            stop=asyncio.Event(); stop.set()
            with pytest.raises(CleanupError,match="stopped"):
                await backend.goals(model.ticks([0]*5,3484),stop)
            backend.enabled=False
            with pytest.raises(CleanupError,match="disabled"):
                await backend.goals(model.ticks([0]*5,3484))
            assert len(captured) == 1
    asyncio.run(run())


def test_camera_uses_source_timestamp_and_decodes_real_protobuf():
    async def run():
        model=ArmModel(); backend=StationBackend("unused","SO101","fixed-camera",model)
        buffer=io.BytesIO(); Image.new("RGB",(640,480),"white").save(buffer,format="JPEG")
        async def read(stamp):
            env=usbvideo.RxEnvelope(type=usbvideo.RxEnvelopeType.ET_FRAMES,
                camera=usbvideo.Camera(unique_id="TEST-CAMERA"), frames=frame.FramesPack(
                    format=frame.FrameFormat(width=640,height=480),
                    stamps=[frame.FrameStamp(local_stamp_ns=stamp)],frames_data=[buffer.getvalue()]))
            q=asyncio.Queue();q.put_nowait(SimpleNamespace(Data=env.encode()))
            backend.client=SimpleNamespace(read_from_tail=lambda *a,**kw:SimpleNamespace(data=q))
            return await backend.look()
        result=await read(time.time_ns())
        assert result.camera_id == "TEST-CAMERA" and result.width == 640
        with pytest.raises(CleanupError,match="stale"):
            await read(time.time_ns()-2_000_000_000)
    asyncio.run(run())


def test_http_tools_are_usable_without_hardware_and_reject_arbitrary_methods(tmp_path):
    async def run():
        model=ArmModel(); backend=MockBackend(model)
        h=Harness(backend,model,TableMap(mock_workspace()),tmp_path)
        app=create_app(h)
        async with app.router.lifespan_context(app):
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app),base_url="http://test") as client:
                response=await client.post("/tools/get_observation",json={"args":{}})
                result=response.json()
                assert result["ok"] and result["result"]["images"][0]["mime"] == "image/jpeg"
                response=await client.post("/tools/__dict__",json={"args":{}})
                assert not response.json()["ok"]
                response=await client.post("/tools/stop",json={"args":{}})
                assert response.json()["result"]["holding_position"]
    asyncio.run(run())


def test_hardware_stale_state_cannot_start_an_action(tmp_path):
    async def run():
        model=ArmModel();backend=MockBackend(model);h=Harness(backend,model,TableMap(mock_workspace()),tmp_path)
        obs=await h.get_observation();p=await h.plan_pick(obs["observation_id"],"crumpled_paper",.99,[180,150],True)
        backend.stale=True
        with pytest.raises(CleanupError,match="stale"):
            await h.execute_plan(p["plan_id"])
        assert not backend.commands and not backend.torque
    asyncio.run(run())


def test_threefold_speed_updates_arm_ram_without_speeding_gripper():
    async def run():
        model = ArmModel()
        backend = StationBackend('unused', 'SO101', None, model, True)
        backend.state = lambda: {'ticks': model.ticks([0]*5, 3300), 'torque': True}
        writes = []
        async def write(address, values, stop=None):
            writes.append((address, values))
        backend.write = write
        await backend.enable(asyncio.Event(), .45)
        settings = dict(writes)
        assert all(int.from_bytes(settings[0x2E][i], 'little') == 300 for i in range(1, 6))
        assert int.from_bytes(settings[0x2E][6], 'little') == 200
        assert settings[0x29][1] == bytes([30])
        assert writes[0][0] == 0x2A and writes[-1][0] == 0x28
        count = len(writes)
        with pytest.raises(CleanupError):
            await backend.enable(asyncio.Event(), float('nan'))
        assert len(writes) == count
    asyncio.run(run())
