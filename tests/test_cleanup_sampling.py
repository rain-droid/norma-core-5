import asyncio
import numpy as np
import pytest
from so101_station.cleanup.visual import PatchTracker, SamplingError
from test_cleanup_visual import make


def test_late_motor_settling_gets_separate_image_budget(tmp_path):
    async def run():
        h=make(tmp_path);v=h.visual;v.settle_window_s=.06;v.settle_timeout_s=.4;v.image_timeout_s=.4
        target=h.backend.q.copy();v.target_q=target.copy();h.backend.q[0]+=.008
        async def settle():
            await asyncio.sleep(.20);h.backend.q=target.copy()
        task=asyncio.create_task(settle());original=h._observe
        async def delayed_frame():
            await asyncio.sleep(.075);return await original()
        h._observe=delayed_frame
        obs,_,_=await v.capture(PatchTracker(h.backend.image,[280,180,64,64]))
        await task
        data=obs['visual_sample']
        assert data['settle_elapsed_s']>.2 and data['image_elapsed_s']>.2
        assert data['settle_elapsed_s']+data['image_elapsed_s']>.4
        assert data['stable_frames']==3 and data['frame_drift_px']<=1.5
        assert len(set(data['frame_observation_ids']))==3 and not h.backend.commands
    asyncio.run(run())


def test_joint_error_diagnostic_reports_phase_motor_values_and_budget(tmp_path):
    async def run():
        h=make(tmp_path);h.visual.settle_timeout_s=.08
        h.visual.target_q=h.backend.q.copy();h.visual.target_q[2]+=.01
        with pytest.raises(SamplingError) as exc:
            await h.visual.capture(PatchTracker(h.backend.image,[280,180,64,64]))
        d=exc.value.diagnostic
        assert d['code']=='joint_target_error' and d['phase']=='settle'
        assert d['metrics']['joint_errors_rad'][2]==pytest.approx(.01)
        assert d['timing']['image_s']==0 and not h.backend.commands
    asyncio.run(run())


def test_three_frame_burst_rejects_cumulative_drift(tmp_path):
    async def run():
        h=make(tmp_path);h.visual.image_timeout_s=.15
        tracker=PatchTracker(h.backend.image,[280,180,64,64]);original=tracker.locate;count=0
        def drift(image):
            nonlocal count
            count+=1;p,c=original(image);return p+[count,0],c
        tracker.locate=drift
        with pytest.raises(SamplingError) as exc:await h.visual.capture(tracker)
        assert exc.value.diagnostic['code']=='image_drift'
        assert exc.value.diagnostic['phase']=='image' and count>=3
        assert not h.backend.commands
    asyncio.run(run())


def test_diagnostic_survives_session_fault_result(tmp_path):
    async def run():
        h=make(tmp_path);original=h.visual.capture;calls=0
        async def fail_after_move(tracker):
            nonlocal calls
            calls+=1
            if calls>1:raise SamplingError('telemetry_not_fresh','settle',{'max_telemetry_age_ms':130},{})
            return await original(tracker)
        h.visual.capture=fail_after_move
        obs=await h.observe()
        r=await h.probe_alignment('sample-fault',obs['observation_id'],[280,180,64,64],True,axis='x')
        assert r['execution']=='fault' and r['stage']=='stopped'
        assert r['diagnostic']['code']=='telemetry_not_fresh'
        assert not h.visual.probes
    asyncio.run(run())


def test_one_axis_probe_returns_four_samples_without_alignment_map(tmp_path):
    async def run():
        h=make(tmp_path);obs=await h.observe()
        r=await h.probe_alignment('one-axis',obs['observation_id'],[280,180,64,64],True,axis='x')
        assert r['execution']=='completed',r.get('reason')
        d=r['detail'];assert d['repeatable']
        assert [row['offset_mm'] for row in d['samples']]==[[5,0],[0,0],[-5,0],[0,0]]
        assert 'probe_id' not in d and not d['validated_2d_map'] and not d['alignment_available']
        assert not h.visual.probes and not d['grasp_calibrated']
        assert np.allclose(d['response_px_per_commanded_mm'],h.backend.matrix[:,0],atol=.4)
        assert np.linalg.norm(h.backend.q-d['home_q'])<.003
    asyncio.run(run())


def test_burst_cannot_hide_motion_on_opposite_sides_of_anchor(tmp_path):
    async def run():
        h=make(tmp_path);original=h._observe;count=0;bad=[]
        async def moving_state():
            nonlocal count
            count+=1;obs=await original()
            if count<=2:
                obs['state']['ticks'][1]+=2 if count==1 else -2
                bad.append(obs['observation_id'])
            return obs
        h._observe=moving_state
        obs,_,_=await h.visual.capture(PatchTracker(h.backend.image,[280,180,64,64]))
        assert count>=5
        assert not set(bad)&set(obs['visual_sample']['frame_observation_ids'])
        assert obs['visual_sample']['motor_span_ticks']<=2
    asyncio.run(run())


def test_failed_camera_duration_is_included_in_diagnostic(tmp_path):
    async def run():
        from so101_station.cleanup.geometry import CleanupError
        h=make(tmp_path)
        async def broken_camera():
            await asyncio.sleep(.05);raise CleanupError('test camera missing')
        h._observe=broken_camera
        with pytest.raises(SamplingError) as exc:
            await h.visual.capture(PatchTracker(h.backend.image,[280,180,64,64]))
        d=exc.value.diagnostic
        assert d['code']=='camera_capture_failed' and d['timing']['image_s']>=.05
    asyncio.run(run())


def test_synchronous_tracker_cannot_accept_past_image_budget(tmp_path):
    async def run():
        import time
        h=make(tmp_path);h.visual.image_timeout_s=.02
        tracker=PatchTracker(h.backend.image,[280,180,64,64]);original=tracker.locate
        def slow(image):
            time.sleep(.03);return original(image)
        tracker.locate=slow
        with pytest.raises(SamplingError) as exc:await h.visual.capture(tracker)
        assert exc.value.diagnostic['code']=='image_processing_timeout'
        assert not h.backend.commands
    asyncio.run(run())
