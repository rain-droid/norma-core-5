import asyncio
import json
import time

import numpy as np
import pytest

from so101_station.cleanup.backend import MockBackend, mock_workspace
from so101_station.cleanup.geometry import ArmModel, CleanupError, TableMap
from so101_station.cleanup.harness import Harness
from so101_station.cleanup.mcp_server import content_from_result, on_list_tools


def make(tmp_path):
    model = ArmModel(); backend = MockBackend(model)
    return Harness(backend, model, TableMap(mock_workspace()), tmp_path)


async def plan(h):
    obs = await h.get_observation()
    result = await h.plan_pick(obs["observation_id"], "crumpled_paper", .99, [180,150], True)
    return obs, result


def test_complete_pick_drop_requires_explicit_visual_verification(tmp_path):
    async def run():
        h = make(tmp_path); before, p = await plan(h)
        result = await h.execute_plan(p["plan_id"])
        assert result["status"] == "awaiting_visual_verification"
        assert h.verified == 0
        assert np.linalg.norm(np.array(h.backend.objects[0]["xy"]) - h.table.drop) < .025
        with pytest.raises(CleanupError):
            await h.execute_plan(p["plan_id"])
        verified = h.verify(result["run_id"], result["after_observation_id"], True, True, "Fixture confirms paper is in the tray and source is clear.")
        assert verified["status"] == "verified_cleaned" and h.verified == 1
        with pytest.raises(CleanupError):
            h.verify(result["run_id"], result["after_observation_id"], True, True, "duplicate")
        assert result["after"]["captured_ns"] >= h.not_before_ns
        assert not h.busy
    asyncio.run(run())


def test_rejects_wrong_class_uncertainty_unmapped_pixels_and_missing_clearance(tmp_path):
    async def run():
        h = make(tmp_path); obs = await h.get_observation()
        for label, confidence, pixel, clear in [("phone", .99, [180,150], True), ("crumpled_paper",.6,[180,150],True),
                                               ("snack_wrapper",float("nan"),[180,150],True), ("crumpled_paper",.99,[0,0],True),
                                               ("crumpled_paper",.99,[180,150],False)]:
            with pytest.raises(CleanupError):
                await h.plan_pick(obs["observation_id"],label,confidence,pixel,clear)
        assert h.backend.commands == []
    asyncio.run(run())


def test_scene_change_invalidates_execution_before_enable(tmp_path):
    async def run():
        h=make(tmp_path); _, p=await plan(h)
        h.backend.objects[0]["xy"] = [.08,-.17]
        with pytest.raises(CleanupError,match="scene changed"):
            await h.execute_plan(p["plan_id"])
        assert not h.backend.torque
        assert all("hold" in c for c in h.backend.commands)
    asyncio.run(run())


def test_stop_between_stages_latches_and_does_not_resume(tmp_path):
    async def run():
        h=make(tmp_path); _, p=await plan(h)
        reached, release = asyncio.Event(), asyncio.Event()
        original = h.executor.path
        calls = 0
        async def pause(path):
            nonlocal calls
            await original(path); calls += 1
            if calls == 3:
                reached.set(); await release.wait()
        h.executor.path = pause
        task=asyncio.create_task(h.execute_plan(p["plan_id"]))
        await asyncio.wait_for(reached.wait(),10)
        await h.stop(); count=len(h.backend.commands); release.set()
        with pytest.raises(CleanupError,match="stopped"):
            await task
        assert all("hold" in c for c in h.backend.commands[count:])
        assert h.verified == 0 and h.executor.stopped.is_set()
    asyncio.run(run())


def test_stop_during_planning_discards_result(tmp_path):
    async def run():
        h=make(tmp_path); obs=await h.get_observation()
        def slow(*args):
            time.sleep(.15); return {}
        h.build_plan=slow
        task=asyncio.create_task(h.plan_pick(obs["observation_id"],"crumpled_paper",.99,[180,150],True))
        await asyncio.sleep(.03); await h.stop()
        with pytest.raises(CleanupError,match="during planning"):
            await task
        assert not h.plans and not h.planning
    asyncio.run(run())


def test_expired_observation_and_disabled_motion(tmp_path):
    async def run():
        h=make(tmp_path); obs,p=await plan(h)
        h.backend.enabled=False
        with pytest.raises(CleanupError,match="disabled"):
            await h.execute_plan(p["plan_id"])
        h.observations[obs["observation_id"]]["received"]-=61
        with pytest.raises(CleanupError,match="expired"):
            await h.plan_pick(obs["observation_id"],"crumpled_paper",.99,[180,150],True)
    asyncio.run(run())


def test_mcp_propagates_images_and_exposes_only_bounded_tools(tmp_path):
    async def run():
        tools=await on_list_tools(None,None)
        assert {t.name for t in tools.tools} == {"status","get_observation","plan_pick","execute_plan","verify","stop"}
        h=make(tmp_path); obs=await h.get_observation()
        content=content_from_result({"ok":True,"result":{"after":obs}})
        assert content[1].type == "image" and content[1].mime_type == "image/jpeg"
        assert "b64" not in content[0].text
    asyncio.run(run())
