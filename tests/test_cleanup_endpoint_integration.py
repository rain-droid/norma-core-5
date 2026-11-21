"""Endpoint control under simulated lag/bias; never connects to hardware."""
import asyncio
import numpy as np
import pytest
from test_cleanup_visual import make
from so101_station.cleanup.geometry import CleanupError


def set_ticks(h, ticks):
    h.backend.q = np.array([j.ticks_to_radians(t) for j, t in zip(h.model.joints, ticks)])


def arm_ticks(h, q):
    return [h.model.ticks(q, h.backend.grip)[i] for i in range(1, 6)]


def bias_goals(h, bias):
    original = h.backend.goals
    async def biased(ticks, stop=None):
        await original(ticks, stop)
        set_ticks(h, [ticks[i] + bias[i-1] for i in range(1, 6)])
    h.backend.goals = biased


def test_bias_correction_converges_across_direction_reversal(tmp_path):
    async def run():
        h=make(tmp_path);home=h.model.fk(h.backend.q)[:3,3];q=h.backend.q.copy()
        bias_goals(h, [3,-3,3,-2,0])
        for offset in ([5,0],[0,0],[-5,0],[0,0]):
            await h.visual.move(home,q,offset)
            d=h.visual.endpoint_result
            assert d['converged'] and 1<=d['trim_commands']<=5
            last=d['samples'][-1]
            assert max(map(abs,last['residual_ticks']))<=3
            assert last['position_error_mm']<=.5
            assert max(abs(a-b) for a,b in zip(last['desired_ticks'],last['command_ticks']))<=6
        assert all(c[6]==h.backend.grip for c in h.backend.commands)
    asyncio.run(run())


def test_measured_lag_is_allowed_to_settle_before_correction(tmp_path):
    async def run():
        h=make(tmp_path);q=h.backend.q.copy();home=h.model.fk(q)[:3,3];target=arm_ticks(h,q)
        set_ticks(h,[target[0]+6,*target[1:]])
        async def catching_up():
            for delta in range(5,-1,-1):
                await asyncio.sleep(.04)
                set_ticks(h,[target[0]+delta,*target[1:]])
        motion=asyncio.create_task(catching_up())
        await h.visual.refine_endpoint(q,home,h.backend.grip)
        await motion
        assert h.visual.endpoint_result['trim_commands']==0
        assert not h.backend.commands
    asyncio.run(run())


def test_stall_has_bounded_commands_and_diagnostic(tmp_path):
    async def run():
        h=make(tmp_path);q=h.backend.q.copy();home=h.model.fk(q)[:3,3];target=arm_ticks(h,q)
        set_ticks(h,[target[0]+3,*target[1:]]);stuck=h.backend.q.copy()
        original=h.backend.goals
        async def stalled(ticks,stop=None):
            await original(ticks,stop);h.backend.q=stuck.copy()
        h.backend.goals=stalled
        with pytest.raises(CleanupError,match='stalled|did not settle|deadline exceeded') as exc:
            await h.visual.refine_endpoint(q,home,h.backend.grip)
        assert exc.value.diagnostic['phase']=='endpoint'
        assert not h.visual.endpoint_result
        # Each correction has two checked path segments and two 20 Hz writes.
        assert len(h.backend.commands)<=12
        assert len(exc.value.diagnostic['samples'])<=4
        assert all(abs(c[1]-target[0])<=6 for c in h.backend.commands)
    asyncio.run(run())


def test_stop_during_settling_never_issues_trim(tmp_path):
    async def run():
        h=make(tmp_path);q=h.backend.q.copy();home=h.model.fk(q)[:3,3]
        task=asyncio.create_task(h.visual.refine_endpoint(q,home,h.backend.grip))
        await asyncio.sleep(.04);h.executor.stopped.set()
        with pytest.raises(CleanupError,match='stopped'):await task
        assert not h.backend.commands
    asyncio.run(run())


def test_compensated_path_is_validated_before_write(tmp_path):
    async def run():
        h=make(tmp_path);q=h.backend.q.copy();home=h.model.fk(q)[:3,3];target=arm_ticks(h,q)
        high=h.model.solve(home+[0,0,.01],q)
        set_ticks(h,[target[0]+3,*target[1:]])
        h.joint_path=lambda a,b:[high.tolist(),b.tolist()]
        with pytest.raises(CleanupError,match='corridor'):
            await h.visual.refine_endpoint(q,home,h.backend.grip)
        assert not h.backend.commands
    asyncio.run(run())


def test_leaving_measured_corridor_aborts_before_next_trim_waypoint(tmp_path):
    async def run():
        h=make(tmp_path);q=h.backend.q.copy();home=h.model.fk(q)[:3,3];target=arm_ticks(h,q)
        set_ticks(h,[target[0]+3,*target[1:]])
        high=h.model.solve(home+[0,0,.01],q)
        calls=[]
        async def deflected(waypoint):
            calls.append(waypoint);h.backend.q=high.copy()
        h.executor.move=deflected
        with pytest.raises(CleanupError,match='corridor'):
            await h.visual.refine_endpoint(q,home,h.backend.grip)
        assert len(calls)==1
    asyncio.run(run())


def test_stale_or_gripper_change_cannot_trigger_trim(tmp_path):
    async def run():
        h=make(tmp_path);q=h.backend.q.copy();home=h.model.fk(q)[:3,3];state=h.backend.state
        def stale():
            s=state();s['report']={'joints':[{'age_ms':130}]};return s
        h.backend.state=stale
        with pytest.raises(TimeoutError):
            await asyncio.wait_for(h.visual.refine_endpoint(q,home,h.backend.grip),.25)
        assert not h.backend.commands
        h.backend.state=state
        with pytest.raises(CleanupError,match='gripper changed'):
            await h.visual.refine_endpoint(q,home,h.backend.grip+3)
        assert not h.backend.commands
    asyncio.run(run())


def test_capture_rejects_drift_after_successful_refinement(tmp_path):
    async def run():
        h=make(tmp_path);q=h.backend.q.copy();home=h.model.fk(q)[:3,3]
        await h.visual.move(home,q,[0,0]);target=arm_ticks(h,q)
        set_ticks(h,[target[0]+3,*target[1:]])
        code,metrics=h.visual.state_problem([h.backend.state()])
        assert code=='precise_endpoint_drift'
        assert metrics['max_target_error_rad']<.0047
    asyncio.run(run())


def test_biased_encoder_backend_yields_repeatable_image_probe(tmp_path):
    async def run():
        h=make(tmp_path);bias_goals(h,[3,-3,3,-2,0]);obs=await h.observe()
        r=await h.probe_alignment('biased-x',obs['observation_id'],[280,180,64,64],True,axis='x')
        assert r['execution']=='completed',r.get('reason')
        assert r['detail']['repeatable']
        assert not r['detail']['validated_2d_map']
        assert all(row['sampling']['endpoint']['converged'] for row in r['detail']['samples'])
    asyncio.run(run())


def test_recorded_canceling_joint_errors_do_not_trigger_unnecessary_trim(tmp_path):
    async def run():
        h=make(tmp_path)
        desired=[1747,2058,2033,3077,1609]
        q=np.array([j.ticks_to_radians(t) for j,t in zip(h.model.joints,desired)])
        home=h.model.fk(q)[:3,3]
        # Exact first endpoint from real run d2306247774b4608b7dec746f280bd11:
        # shoulder/elbow errors cancel, producing only 0.356 mm tool error.
        set_ticks(h,[1747,2056,2035,3077,1609])
        await h.visual.refine_endpoint(q,home,h.backend.grip)
        assert not h.backend.commands
        assert h.visual.endpoint_result['trim_commands']==0
        assert h.visual.endpoint_result['samples'][0]['position_error_mm']<.5
        h.visual.target_q=q
        assert h.visual.state_problem([h.backend.state()])[0] is None
    asyncio.run(run())


def test_endpoint_diagnostic_survives_session_fault(tmp_path):
    async def run():
        h=make(tmp_path)
        async def fail(*args):
            error=CleanupError('endpoint refinement failed: simulated stall')
            error.diagnostic={'code':'endpoint_refinement_failed','phase':'endpoint','metrics':{'iteration':3}}
            raise error
        h.visual.refine_endpoint=fail
        obs=await h.observe()
        r=await h.probe_alignment('endpoint-fault',obs['observation_id'],[280,180,64,64],True,axis='x')
        assert r['execution']=='fault' and h.executor.stopped.is_set()
        assert r['diagnostic']['phase']=='endpoint'
        assert r['diagnostic']['metrics']['iteration']==3
    asyncio.run(run())


def test_fractional_ik_target_uses_same_joint_bound_as_capture(tmp_path):
    async def run():
        h=make(tmp_path)
        desired=[1747,2058,2033,3077,1609]
        q=np.array([j.ticks_to_radians(t) for j,t in zip(h.model.joints,desired)])
        q[1]+=.4*2*np.pi/4096
        home=h.model.fk(q)[:3,3]
        set_ticks(h,[1746,2055,2035,3076,1607])
        await h.visual.refine_endpoint(q,home,h.backend.grip)
        d=h.visual.endpoint_result
        assert d['trim_commands']>0  # Rounded goal alone would have accepted.
        assert d['samples'][0]['requested_joint_error_rad']>.0047
        assert d['samples'][-1]['requested_joint_error_rad']<=.0047
        h.visual.target_q=q
        assert h.visual.state_problem([h.backend.state()])[0] is None
    asyncio.run(run())
