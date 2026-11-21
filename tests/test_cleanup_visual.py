"""Synthetic image-motion physics tests; not evidence for real grasp accuracy."""
import asyncio
import time

import cv2
import numpy as np
import pytest

from so101_station.cleanup.backend import CameraFrame, MockBackend
from so101_station.cleanup.geometry import ArmModel, CleanupError
from so101_station.cleanup.session import SessionHarness, mock_session
from so101_station.cleanup.visual import PatchTracker, fit_response


class ImageMotionBackend(MockBackend):
    def __init__(self, model):
        super().__init__(model)
        self.torque = True
        self.home = model.fk(self.q)[:3,3].copy()
        self.matrix = np.array([[2., -3.], [4., 1.]])
        self.image = np.full((480,640,3), 110, np.uint8)
        self.image[180:244,280:344] = np.random.default_rng(9).integers(0,256,(64,64,3),dtype=np.uint8)
        self.blank = False

    async def look(self, queue=None):
        if queue != 'mock-wrist':
            return await super().look(queue)
        xy = (self.model.fk(self.q)[:2,3]-self.home[:2])*1000
        offset = self.matrix @ xy
        frame = cv2.warpAffine(self.image, np.float32([[1,0,offset[0]],[0,1,offset[1]]]), (640,480), borderValue=(110,110,110))
        if self.blank:
            frame[:] = 110
        _, data = cv2.imencode('.jpg', frame)
        return CameraFrame(data.tobytes(),640,480,'mock-wrist',time.time_ns())


def make(tmp_path):
    model = ArmModel(); backend = ImageMotionBackend(model)
    return SessionHarness(backend,model,mock_session(model),tmp_path)


async def probe(h):
    obs = await h.observe()
    result = await h.probe_alignment('probe',obs['observation_id'],[280,180,64,64],True)
    assert result['execution'] == 'completed', result.get('reason')
    return result


def test_fit_rejects_weak_motion_and_non_linear_response():
    commands = np.array([[5,0],[-5,0],[0,5],[0,-5],[0,0]])
    j = np.array([[2,-3],[4,1]])
    matrix, quality = fit_response(commands, commands@j.T+[100,200],np.array([100,200]))
    assert np.allclose(matrix,j) and quality['max_fit_error_px'] < 1e-10
    with pytest.raises(CleanupError,match='weak'):
        fit_response(commands,np.ones((5,2)),np.ones(2))
    pixels = commands@j.T+[100,200];pixels[-1] += 20
    with pytest.raises(CleanupError,match='drifting'):
        fit_response(commands,pixels,np.array([100,200]))


def test_patch_tracker_rejects_untextured_or_lost_target():
    image = np.full((200,200,3),110,np.uint8)
    with pytest.raises(CleanupError,match='texture'):
        PatchTracker(image,[50,50,40,40])
    image[50:90,50:90] = np.random.default_rng(2).integers(0,256,(40,40,3),dtype=np.uint8)
    tracker = PatchTracker(image,[50,50,40,40])
    moved = cv2.warpAffine(image,np.float32([[1,0,8],[0,1,-4]]),(200,200))
    point,confidence = tracker.locate(moved)
    assert np.allclose(point,[78,66],atol=.1) and confidence > .99
    with pytest.raises(CleanupError,match='lost'):
        tracker.locate(np.full_like(image,110))


def test_probe_and_local_alignment_with_no_model_per_step(tmp_path):
    async def run():
        h = make(tmp_path)
        result = await probe(h)
        data = result['detail']
        assert np.allclose(data['matrix_px_per_commanded_mm'],h.backend.matrix,atol=.4)
        assert data['quality']['held_out_error_px'] < 5
        assert data['grasp_calibrated'] is False
        assert np.linalg.norm(h.model.fk(h.backend.q)[:3,3]-h.backend.home) < .002
        # Give a nearby pixel objective requiring actual local closed-loop motion.
        goal = (np.array(data['reference_pixel'])+h.backend.matrix@[3,-2]).tolist()
        aligned = await h.align_target('align',result['observation']['observation_id'],data['probe_id'],goal,True)
        assert aligned['execution'] == 'completed', aligned.get('reason')
        assert aligned['detail']['aligned'] and aligned['detail']['final_error_px'] <= 4
        assert aligned['detail']['stable_fresh_frames'] >= 2
        assert not aligned['detail']['grasp_ready']
        assert 1 <= len(aligned['detail']['moves']) <= 8
        count = len(h.backend.commands)
        assert await h.align_target('align',result['observation']['observation_id'],data['probe_id'],goal,True) == aligned
        assert len(h.backend.commands) == count
    asyncio.run(run())


def test_expired_probe_and_height_change_reject_before_motion(tmp_path):
    async def run():
        h = make(tmp_path); result = await probe(h); ident = result['detail']['probe_id']
        count = len(h.backend.commands)
        h.visual.probes[ident]['created'] -= 301
        denied = await h.align_target('expired',result['observation']['observation_id'],ident,[312,212],True)
        assert denied['execution'] == 'rejected' and len(h.backend.commands) == count
        h.visual.probes[ident]['created'] = time.monotonic()
        xyz = h.model.fk(h.backend.q)[:3,3]
        h.backend.q = h.model.solve(xyz-[0,0,.02],h.backend.q)
        obs = await h.observe()
        denied = await h.align_target('height',obs['observation_id'],ident,[312,212],True)
        assert denied['execution'] == 'rejected' and len(h.backend.commands) == count
    asyncio.run(run())


def test_no_motion_for_bad_roi_or_closed_gripper(tmp_path):
    async def run():
        h = make(tmp_path); obs = await h.observe()
        denied = await h.probe_alignment('bad',obs['observation_id'],[0,0,64,64],True)
        assert denied['execution'] == 'rejected' and not h.backend.commands
        h.backend.grip = 2500
        obs = await h.observe()
        denied = await h.probe_alignment('closed',obs['observation_id'],[280,180,64,64],True)
        assert denied['execution'] == 'rejected' and not h.backend.commands
    asyncio.run(run())


def test_lost_tracking_after_motion_stops_and_invalidates(tmp_path):
    async def run():
        h = make(tmp_path); original = h.visual.move
        async def vanish(*args):
            result = await original(*args)
            h.backend.blank = True
            return result
        h.visual.move = vanish
        obs = await h.observe()
        result = await h.probe_alignment('lost',obs['observation_id'],[280,180,64,64],True)
        assert result['execution'] == 'fault' and h.stage == 'stopped'
        assert h.executor.stopped.is_set() and not h.visual.probes
        assert h.backend.commands[-1] == {'hold':True}
    asyncio.run(run())


def test_stop_interrupts_local_probe_and_duplicate_never_replays(tmp_path):
    async def run():
        h = make(tmp_path); entered, release = asyncio.Event(),asyncio.Event()
        original = h.visual.move
        async def paused(*args):
            entered.set(); await release.wait()
            return await original(*args)
        h.visual.move = paused
        obs = await h.observe(); args=('stop-probe',obs['observation_id'],[280,180,64,64],True)
        task = asyncio.create_task(h.probe_alignment(*args))
        await asyncio.wait_for(entered.wait(),5)
        assert (await h.probe_alignment(*args))['execution'] == 'in_progress'
        await h.stop(); count=len(h.backend.commands); release.set()
        result = await task
        assert result['stage'] == 'stopped' and not h.visual.probes
        assert all('hold' in c for c in h.backend.commands[count:])
    asyncio.run(run())


def test_visual_motion_preserves_open_gripper_and_checks_entire_path(tmp_path):
    async def run():
        h = make(tmp_path)
        home = h.model.fk(h.backend.q)[:3,3].copy(); home_q = h.backend.q.copy()
        h.executor.grip_goal = 2050  # Historical closed goal must not be replayed.
        await h.visual.move(home,home_q,[1,0])
        assert all(cmd[6] >= 3200 for cmd in h.backend.commands if 6 in cmd)
        count = len(h.backend.commands)
        high = h.model.solve(home+[0,0,.01],home_q)
        h.joint_path = lambda a,b: [high.tolist(), b.tolist()]
        with pytest.raises(CleanupError,match='corridor'):
            await h.visual.move(home,home_q,[0,0])
        assert len(h.backend.commands) == count
    asyncio.run(run())


def test_prepare_from_fault_requires_operator_and_preserves_empty_gripper(tmp_path):
    async def run():
        h = make(tmp_path)
        await h.stop(); obs = await h.observe(); count=len(h.backend.commands)
        denied = await h.prepare_alignment('no-confirm',obs['observation_id'],True,True)
        assert denied['execution']=='stopped' and len(h.backend.commands)==count
        ready = await h.prepare_alignment('prepare',denied['observation']['observation_id'],True,True,True)
        assert ready['execution']=='completed' and ready['detail']['ready']
        assert ready['stage']=='aligning' and not h.visual.probes
        assert all(cmd[6]>=3200 for cmd in h.backend.commands if 6 in cmd)
        assert np.max(np.abs(h.backend.q-h.poses['pickup_hover']))<.012
    asyncio.run(run())


def test_prepare_rejects_closed_gripper_and_new_stop_wins(tmp_path):
    async def run():
        h = make(tmp_path);h.backend.grip=2500;obs=await h.observe()
        denied=await h.prepare_alignment('closed-prep',obs['observation_id'],True,True,True)
        assert denied['execution']=='rejected' and not h.backend.commands
        h.backend.grip=3340;obs=await h.observe()
        task=asyncio.create_task(h.prepare_alignment('stop-prep',obs['observation_id'],True,True,True))
        await asyncio.sleep(.2);await h.stop();count=len(h.backend.commands)
        result=await task
        assert result['stage']=='stopped' and h.executor.stopped.is_set()
        assert len(h.backend.commands)==count
    asyncio.run(run())


def test_lower_command_cadence_preserves_target_with_fewer_writes(tmp_path):
    async def run():
        counts=[]
        for hz in [20,10]:
            h=make(tmp_path);h.executor.command_hz=hz;h.executor.speed=.45
            target=h.backend.q.copy();target[0]+=.025
            await h.executor.move(target)
            counts.append(len(h.backend.commands))
            assert np.max(np.abs(h.backend.q-target))<.012
        assert counts[1]<counts[0]
    asyncio.run(run())


def test_feature_tracker_is_opt_in_and_runs_through_probe_tool(tmp_path):
    async def run():
        h=make(tmp_path);obs=await h.observe()
        result=await h.probe_alignment('features',obs['observation_id'],[280,180,64,64],True,tracker='features')
        assert result['execution']=='completed',result.get('reason')
        assert result['detail']['tracker']=='features'
        data=result['detail']
        aligned=await h.align_target('feature-align',result['observation']['observation_id'],data['probe_id'],data['reference_pixel'],True)
        assert aligned['execution']=='completed' and aligned['detail']['aligned']
    asyncio.run(run())


def test_stable_capture_waits_for_lag_and_records_frame_time_pose(tmp_path):
    async def run():
        h=make(tmp_path); target=h.backend.q.copy()
        h.visual.target_q=target.copy(); h.backend.q[0]+=.008
        tracker=PatchTracker(h.backend.image,[280,180,64,64])
        async def settle():
            await asyncio.sleep(.12)
            h.backend.q=target.copy()
        task=asyncio.create_task(settle())
        obs,pixel,confidence=await h.visual.capture(tracker)
        await task
        sampling=obs['visual_sample']
        assert sampling['settle_elapsed_s'] >= .12+h.visual.settle_window_s
        assert sampling['stable_frames']==3 and sampling['motor_span_ticks']<=2
        assert sampling['state_read_ns']>=sampling['wrist_captured_ns']
        assert np.allclose(sampling['measured_xyz_m'],h.model.fk(target)[:3,3])
        assert not h.backend.commands  # Waiting never pushes the arm harder.
    asyncio.run(run())


def test_camera_capture_motion_is_discarded_even_if_it_returns(tmp_path):
    async def run():
        h=make(tmp_path); original=h._observe; calls=0
        async def moving_capture():
            nonlocal calls
            calls+=1
            if calls==1:
                q=h.backend.q.copy()
                h.backend.q[0]+=.01
                await asyncio.sleep(.06)
                h.backend.q=q
            return await original()
        h._observe=moving_capture
        obs,_,_=await h.visual.capture(PatchTracker(h.backend.image,[280,180,64,64]))
        assert calls>=3  # First frame rejected, then two quiet fresh frames.
        assert obs['visual_sample']['motor_span_ticks']<=2
        assert not h.backend.commands
    asyncio.run(run())


def test_unsettled_pose_times_out_and_stop_cancels_sampling(tmp_path):
    async def run():
        h=make(tmp_path); h.visual.settle_timeout_s=.25
        h.visual.target_q=h.backend.q.copy();h.visual.target_q[0]+=.01
        tracker=PatchTracker(h.backend.image,[280,180,64,64])
        with pytest.raises(CleanupError,match='unstable'):
            await h.visual.capture(tracker)
        assert not h.backend.commands
        h.visual.target_q=None;h.visual.settle_timeout_s=3
        task=asyncio.create_task(h.visual.capture(tracker))
        await asyncio.sleep(.04);await h.stop()
        with pytest.raises(CleanupError,match='stopped'):
            await task
        assert all('hold' in command for command in h.backend.commands)
    asyncio.run(run())


def test_stationary_motor_with_drifting_image_is_not_a_valid_sample(tmp_path):
    async def run():
        h=make(tmp_path);h.visual.image_timeout_s=.55
        tracker=PatchTracker(h.backend.image,[280,180,64,64]);original=tracker.locate
        calls=0
        def drifting(image):
            nonlocal calls
            point,confidence=original(image);calls+=1
            return point+[calls*4,0],confidence
        tracker.locate=drifting
        with pytest.raises(CleanupError,match='image_drift'):
            await h.visual.capture(tracker)
        assert calls>=2 and not h.backend.commands
    asyncio.run(run())


def test_zero_offset_restores_exact_anchor_without_resolving_ik(tmp_path):
    async def run():
        h=make(tmp_path);q=h.backend.q.copy();home=h.model.fk(q)[:3,3]
        h.backend.q[0]+=.005
        def should_not_solve(*args):
            raise AssertionError('Return must restore original joints')
        h.planner.solve=should_not_solve
        await h.visual.move(home,q,[0,0])
        expected=h.model.ticks(q,h.backend.grip)
        assert h.backend.commands[-1]==expected
        assert np.array_equal(h.visual.target_q,q)
    asyncio.run(run())


def test_old_telemetry_does_not_prove_stationary(tmp_path):
    h=make(tmp_path);state=h.backend.state()
    state['report']={'joints':[{'age_ms':150}]}
    assert not h.visual.quiet([state,state])


def test_capture_includes_observations_own_motor_sample(tmp_path):
    async def run():
        h=make(tmp_path);original=h._observe;calls=0
        async def transient_sample():
            nonlocal calls
            calls+=1
            obs=await original()
            if calls==1:
                obs['state']['ticks'][1]+=10
            return obs
        h._observe=transient_sample
        obs,_,_=await h.visual.capture(PatchTracker(h.backend.image,[280,180,64,64]))
        assert calls>=3
        assert obs['visual_sample']['motor_span_ticks']<=2
    asyncio.run(run())
