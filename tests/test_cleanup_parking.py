import asyncio
import numpy as np
from test_cleanup_visual import make

REST=[.0030679615757712823,-1.7165245016440325,1.5753982691585535,-.23930100291016002,.9188544919434991]


def configured(tmp_path):
    h=make(tmp_path);h.poses['rest']=REST
    # Proven setup's pickup hover; keeps both transitions in the checked corridor.
    h.poses['pickup_hover']=[-.4740007575,-.0246145397,.0240617227,1.5713539892,-.6734175659]
    return h


def test_park_then_prepare_passes_rest_and_finish_returns_there(tmp_path):
    async def run():
        h=configured(tmp_path);obs=await h.observe()
        parked=await h.park('park',obs['observation_id'],True,True)
        assert parked['execution']=='completed',parked.get('reason')
        assert h.at_rest(h.backend.q) and h.backend.torque
        ready=await h.prepare_alignment('prepare',parked['observation']['observation_id'],True,True)
        assert ready['execution']=='completed',ready.get('reason')
        assert ready['detail']['started_from_rest'] and h.alignment_prepared
        assert h.return_to_rest_pending and not h.at_rest(h.backend.q)
        end=await h.park('finish',ready['observation']['observation_id'],True,True)
        assert end['execution']=='completed' and h.at_rest(h.backend.q)
        assert not h.return_to_rest_pending and not h.alignment_prepared
    asyncio.run(run())


def test_rest_required_for_probe_and_fault_needs_explicit_parking_direction(tmp_path):
    async def run():
        h=configured(tmp_path);obs=await h.observe()
        result=await h.probe_alignment('no-prepare',obs['observation_id'],[280,180,64,64],True)
        assert result['execution']=='rejected' and not h.backend.commands
        await h.stop();obs=await h.observe();count=len(h.backend.commands)
        denied=await h.park('no-reset',obs['observation_id'],True,True)
        assert denied['execution']=='stopped' and len(h.backend.commands)==count
        done=await h.park('directed',denied['observation']['observation_id'],True,True,True)
        assert done['execution']=='completed' and h.at_rest(h.backend.q)
    asyncio.run(run())


def test_fast_transfer_keeps_dense_validation_and_bounds_segments(tmp_path):
    h=configured(tmp_path);start=h.backend.q.copy();target=np.array(REST)
    original=h.model.validate;validated=[]
    def count(q,floor):
        validated.append(q);return original(q,floor)
    h.model.validate=count
    fast=h.joint_path(start,target,step_rad=.20)
    assert len(validated)>len(fast)*4
    assert np.max(np.abs(np.diff(np.array([start,*fast]),axis=0)))<=.200001
    assert np.allclose(fast[-1],target)


def test_fast_move_paces_against_lag_without_exceeding_following_bound(tmp_path):
    async def run():
        h=make(tmp_path);h.executor.speed=.45;h.executor.command_hz=10
        original=h.backend.goals;tasks=[];leads=[]
        async def delayed(ticks,stop):
            q=np.array([j.ticks_to_radians(ticks[j.motor_id]) for j in h.model.joints])
            leads.append(float(np.max(np.abs(q-h.backend.q))))
            async def apply():
                await asyncio.sleep(.04);await original(ticks,stop)
            tasks.append(asyncio.create_task(apply()))
        h.backend.goals=delayed
        target=h.backend.q.copy();target[0]+=.20
        await h.executor.move(target)
        await asyncio.gather(*tasks)
        assert max(leads)<=.081  # one encoder tick of command quantization
        assert np.max(np.abs(h.backend.q-target))<.012
    asyncio.run(run())


def test_stalled_step_stops_sending_goals_and_new_stop_cancels_wait(tmp_path):
    async def run():
        import pytest
        from so101_station.cleanup.geometry import CleanupError
        h=make(tmp_path);h.executor.speed=.45;h.executor.command_hz=10;h.executor.step_wait_s=.08
        goals=[]
        async def stalled(ticks,stop):goals.append(ticks)
        h.backend.goals=stalled;target=h.backend.q.copy();target[0]+=.20
        with pytest.raises(CleanupError,match='catch up'):
            await h.executor.move(target)
        assert len(goals)==2
        goals.clear();h.executor.step_wait_s=1
        task=asyncio.create_task(h.executor.move(target))
        await asyncio.sleep(.025);h.executor.stopped.set()
        with pytest.raises(CleanupError,match='stopped'):await task
        assert len(goals)==2
    asyncio.run(run())
