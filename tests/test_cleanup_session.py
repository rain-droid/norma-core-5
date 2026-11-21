import asyncio
import json
import time

import httpx
import numpy as np
import pytest

from so101_station.cleanup.backend import MockBackend
from so101_station.cleanup.geometry import ArmModel, CleanupError
from so101_station.cleanup.session import SessionHarness, mock_session
from so101_station.cleanup.service import create_app
from so101_station.cleanup.mcp_server import content_from_result


def make(tmp_path):
    model = ArmModel()
    return SessionHarness(MockBackend(model), model, mock_session(model), tmp_path)


async def call(h, action, params=None):
    obs = await h.observe()
    return await h.act(str(len(h.actions)), obs['observation_id'], action, params, True)


async def ready(h):
    obs = await h.observe()
    result = await h.set_torque('enable', obs['observation_id'], 'hold')
    assert result['execution'] == 'completed'
    assert (await call(h, 'begin', {'target_id': 'paper-1'}))['execution'] == 'completed'


async def pick(h):
    await ready(h)
    assert (await call(h, 'approach'))['execution'] == 'completed'
    for _ in range(6):
        result = await call(h, 'nudge', {'delta_mm': [0, 0, -10]})
        assert result['execution'] == 'completed', result.get('reason')
    assert (await call(h, 'grasp'))['stage'] == 'grasped'
    result = await call(h, 'lift')
    assert result['stage'] == 'awaiting_lift', result.get('reason')
    return result


def test_pick_requires_fresh_lift_and_placement_assessments(tmp_path):
    async def run():
        h = make(tmp_path)
        lift = await pick(h)
        assert len(content_from_result(lift)) == 3
        count = len(h.backend.commands)
        denied = await call(h, 'carry')
        assert denied['execution'] == 'rejected' and len(h.backend.commands) == count
        with pytest.raises(CleanupError, match='stale_observation'):
            await h.assess('stale', lift['observation']['observation_id'], 'held', 'looks held', True)
        obs = await h.observe()
        uncertain = await h.assess('uncertain', obs['observation_id'], 'uncertain', 'occluded')
        assert uncertain['stage'] == 'awaiting_lift'
        held = await h.assess('held', uncertain['observation']['observation_id'], 'held', 'paper at tips; source clear', True)
        assert held['stage'] == 'holding'
        assert (await call(h, 'carry'))['stage'] == 'at_drop'
        assert (await call(h, 'release'))['stage'] == 'awaiting_placement'
        retreat = await call(h, 'retreat')
        obs = retreat['observation']['observation_id']
        delivered = await h.assess('delivered', obs, 'delivered', 'paper on cover and source clear', True, True)
        assert delivered['stage'] == 'complete' and h.verified == 1
        again = await h.assess('delivered', obs, 'delivered', 'paper on cover and source clear', True, True)
        assert again == delivered and h.verified == 1
        assert np.linalg.norm(np.array(h.backend.objects[0]['xy']) - [.1, -.14]) < .02
        events = [json.loads(line) for line in (h.out/'events.jsonl').read_text().splitlines()]
        assert any(e['event'] == 'observation' and len(e['views']) == 2 for e in events)
    asyncio.run(run())


def test_duplicate_busy_stop_and_no_resume(tmp_path):
    async def run():
        h = make(tmp_path)
        await ready(h)
        entered, release = asyncio.Event(), asyncio.Event()
        original = h.executor.path
        async def paused(path):
            entered.set()
            await release.wait()
            await original(path)
        h.executor.path = paused
        obs = await h.observe()
        args = ('motion', obs['observation_id'], 'nudge', {'delta_mm': [1, 0, 0]}, True)
        task = asyncio.create_task(h.act(*args))
        await asyncio.wait_for(entered.wait(), 3)
        assert (await h.act(*args))['execution'] == 'in_progress'
        with pytest.raises(CleanupError, match='conflict'):
            await h.act('motion', obs['observation_id'], 'open')
        with pytest.raises(CleanupError, match='busy'):
            await h.act('another', obs['observation_id'], 'open')
        await h.stop()
        count = len(h.backend.commands)
        release.set()
        result = await task
        assert result['stage'] == 'stopped'
        assert all('hold' in cmd for cmd in h.backend.commands[count:])
        assert await h.act(*args) == result
        assert (await call(h, 'open'))['execution'] == 'stopped'
    asyncio.run(run())


def test_empty_grasp_retry_budget_and_restart(tmp_path):
    async def run():
        h = make(tmp_path)
        await ready(h)
        h.backend.objects = []
        for _ in range(2):
            await call(h, 'open')
            result = await call(h, 'grasp')
            assert result['execution'] == 'empty_grasp'
            assert result['stage'] == 'retry_required'
        await call(h, 'open')
        count = len(h.backend.commands)
        result = await call(h, 'grasp')
        assert result['execution'] == 'rejected' and 'exhausted' in result['reason']
        assert len(h.backend.commands) == count
        restarted = SessionHarness(h.backend, h.model, h.config, tmp_path)
        assert restarted.stage == 'idle' and not restarted.actions and not restarted.observations
        assert len(h.backend.commands) == count
        with pytest.raises(CleanupError, match='stale_observation'):
            await restarted.act('old', result['observation']['observation_id'], 'begin', {'target_id': 'paper'})
    asyncio.run(run())


def test_stale_or_changed_state_and_motion_disabled(tmp_path):
    async def run():
        h = make(tmp_path)
        obs = await h.observe()
        h.observations[obs['observation_id']]['created'] -= 31
        with pytest.raises(CleanupError, match='stale_observation'):
            await h.set_torque('x', obs['observation_id'], 'hold')
        obs = await h.observe()
        h.backend.q[0] += .1
        with pytest.raises(CleanupError, match='state_changed'):
            await h.set_torque('y', obs['observation_id'], 'hold')
        h.backend.enabled = False
        obs = await h.observe()
        denied = await h.set_torque('z', obs['observation_id'], 'hold')
        assert denied['execution'] == 'rejected' and not h.backend.torque
        assert not h.backend.commands
    asyncio.run(run())


def test_cameras_fail_closed_and_motion_result_survives_capture_failure(tmp_path):
    async def run():
        h = make(tmp_path)
        await ready(h)
        original = h.backend.look
        async def wrong(queue=None):
            frame = await original(queue)
            frame.camera_id = 'different'
            return frame
        h.backend.look = wrong
        with pytest.raises(CleanupError, match='identity'):
            await h.observe()
        h.backend.look = original
        obs = await h.observe()
        async def broken(queue=None):
            raise CleanupError('camera disconnected')
        h.backend.look = broken
        result = await h.act('open-once', obs['observation_id'], 'open', scene_clear=True)
        assert result['execution'] == 'completed_without_observation' and 'observation' not in result
        count = len(h.backend.commands)
        assert await h.act('open-once', obs['observation_id'], 'open', scene_clear=True) == result
        assert len(h.backend.commands) == count
    asyncio.run(run())


def test_http_registry_and_dual_images(tmp_path):
    async def run():
        h = make(tmp_path)
        app = create_app(h)
        async with app.router.lifespan_context(app):
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://test') as client:
                assert set((await client.get('/tools')).json()) == {'status', 'observe', 'act', 'assess', 'set_torque', 'stop', 'probe_alignment', 'align_target', 'prepare_alignment', 'select_target', 'locate_target', 'park'}
                body = (await client.post('/tools/observe', json={'args': {}})).json()
                assert body['ok'] and len(body['result']['images']) == 2
                assert not (await client.post('/tools/execute_plan', json={'args': {}})).json()['ok']
                assert not h.backend.torque
    asyncio.run(run())


def test_limp_requires_support_and_never_resumes(tmp_path):
    async def run():
        h = make(tmp_path)
        await ready(h)
        obs = await h.observe()
        denied = await h.set_torque('no-support', obs['observation_id'], 'limp')
        assert denied['execution'] == 'rejected' and h.backend.torque
        result = await h.set_torque('supported', denied['observation']['observation_id'], 'limp', True)
        assert result['stage'] == 'stopped' and not h.backend.torque
        result = await h.set_torque('no-resume', result['observation']['observation_id'], 'hold')
        assert result['execution'] == 'stopped' and not h.backend.torque
    asyncio.run(run())


def test_no_write_for_oversize_nudge_or_planning_fault(tmp_path):
    async def run():
        h = make(tmp_path)
        await ready(h)
        count = len(h.backend.commands)
        result = await call(h, 'nudge', {'delta_mm': [11, 0, 0]})
        assert result['execution'] == 'rejected' and len(h.backend.commands) == count
        def bad(*args):
            raise CleanupError('no checked downward grasp pose')
        h.plan = bad
        result = await call(h, 'nudge', {'delta_mm': [1, 0, 0]})
        assert result['execution'] == 'rejected' and len(h.backend.commands) == count
    asyncio.run(run())


def test_real_bus_lease_excludes_second_service(tmp_path):
    async def run():
        h = make(tmp_path)
        h.backend.mode, h.backend.host, h.backend.bus = 'real', str(tmp_path), 'test-bus'
        app = create_app(h)
        second = create_app(h)
        async with app.router.lifespan_context(app):
            with pytest.raises(BlockingIOError):
                async with second.router.lifespan_context(second):
                    pass
    asyncio.run(run())


def test_operator_recovery_requires_confirmation_and_new_held_assessment(tmp_path):
    async def run():
        h = make(tmp_path)
        await pick(h)
        await h.stop()
        commands = len(h.backend.commands)
        denied = await call(h, 'resume_holding', {'operator_confirmed': False, 'evidence': 'held'})
        assert denied['execution'] == 'stopped' and h.executor.stopped.is_set()
        recovered = await call(h, 'resume_holding', {'operator_confirmed': True, 'evidence': 'operator requested resume; fresh views show held paper'})
        assert recovered['stage'] == 'awaiting_lift' and not h.executor.stopped.is_set()
        assert len(h.backend.commands) == commands
        assert (await call(h, 'carry'))['execution'] == 'rejected'
        assert h.attempt == 1
        await h.stop()
        assert (await call(h, 'resume_holding', {'operator_confirmed': True, 'evidence': 'again'}))['execution'] == 'stopped'
    asyncio.run(run())


def test_recovery_after_restart_does_not_grant_more_grasp_attempts(tmp_path):
    async def run():
        h = make(tmp_path)
        h.backend.torque = True
        recovered = await call(h, 'resume_holding', {'operator_confirmed': True, 'target_id': 'held-before-restart', 'evidence': 'operator requested placement; held object visible'})
        assert recovered['stage'] == 'awaiting_lift'
        assert h.attempt == 2 and h.target == 'held-before-restart'
        assert not h.backend.commands
    asyncio.run(run())


def test_new_stop_during_recovery_cannot_be_cleared(tmp_path):
    async def run():
        h = make(tmp_path)
        await ready(h)
        await h.stop()
        task = asyncio.create_task(call(h, 'resume_holding', {'operator_confirmed': True, 'evidence': 'operator requested resume'}))
        await asyncio.sleep(.15)
        await h.stop()
        result = await task
        assert result['stage'] == 'stopped' and h.executor.stopped.is_set()
        assert h.recoveries == 0
    asyncio.run(run())
