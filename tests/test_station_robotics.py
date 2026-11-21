"""Shared model/controller contracts for two different robot morphologies."""
import numpy as np
import pytest
mujoco = pytest.importorskip('mujoco', reason='install the simulation extra')
from fastapi.testclient import TestClient
from station_robotics.profiles import load_profile
from station_robotics.model import build_model
from station_robotics.servo import VisualServo
from station_robotics.simulation import Simulation
from station_robotics.app import create_app


@pytest.mark.parametrize('robot,count',[('so101',5),('elrobot',7)])
def test_cad_kinematics_match_existing_model_and_physics_holds(robot,count):
    s=Simulation(robot,render=False)
    try:
        assert len(s.qidx)==count and s.model.nu==count+1
        q=s.data.qpos[s.qidx].copy()
        mujoco.mj_forward(s.model,s.data)
        assert np.allclose(s.data.site('tool').xpos,s.profile.fk(q)[:3,3],atol=1e-8)
        assert np.allclose(s.data.site('tool').xmat.reshape(3,3),s.profile.fk(q)[:3,:3],atol=1e-8)
        start=s.data.site('tool').xpos.copy();paper=s.data.body('paper').xpos.copy()
        s.step(1.)
        assert np.linalg.norm(s.data.site('tool').xpos-start)<.003
        assert np.linalg.norm(s.data.body('paper').xpos-paper)<.001
        assert not s.state()['hardware_connected']
    finally:s.close()


def test_command_preserves_fixed_origin_and_limits():
    s=Simulation(render=False)
    try:
        anchor=s.anchor.copy();q=s.anchor_q.copy()
        s.command([4,0]);s.step(.5);s.command([4,4])
        assert np.array_equal(s.anchor,anchor) and np.array_equal(s.anchor_q,q)
        with pytest.raises(ValueError,match='neighborhood'):s.command([100,0])
    finally:s.close()


def test_image_feedback_uses_observation_and_enforces_budget():
    v=VisualServo();J=np.array([[2,-1],[1,3.]])
    errors=[np.array([15.,-12]),np.array([15.,-12])+J@[4,0],np.array([15.,-12])+J@[4,4]]
    assert np.allclose(v.fit(errors),J)
    error=np.array([20.,15.])
    for _ in range(12):
        delta=v.correction(error)
        if delta is None:continue
        assert np.linalg.norm(delta)<=5.00001
        error+=J@delta
    assert v.stable>=2 and np.linalg.norm(error)<=4
    v.moves=v.max_moves
    with pytest.raises(ValueError,match='exhausted'):v.correction([30,30])


def test_missing_or_ambiguous_visual_response_is_rejected():
    with pytest.raises(ValueError,match='weak'):VisualServo().fit([[1,1],[1,1],[1,1]])
    assert Simulation.detect(np.zeros((40,40,3),np.uint8)) is None
    with pytest.raises(ValueError,match='Invalid'):VisualServo().correction([np.nan,2])


def test_simulation_http_cannot_select_a_hardware_backend():
    client=TestClient(create_app())  # No lifespan: no renderer thread needed.
    data=client.get('/api/capabilities').json()
    assert not data['hardware_connected'] and set(data['profiles'])=={'so101','elrobot'}
    assert client.post('/api/profile/station').status_code==400
    assert client.post('/api/action',json={'command':'torque_on'}).status_code==400
    assert client.post('/api/settings',json={'mode':'vision','coarse_error_mm':99}).status_code==422


def test_paused_observation_can_resume_without_expired_motion_deadline():
    s=Simulation(render=False)
    try:
        s.stage='probe_x';s.running=False;anchor=s.anchor.copy();goals=s.data.ctrl.copy()
        s.step(4.)
        s.start(step=True)
        assert s.move_started==s.data.time
        assert np.array_equal(s.anchor,anchor) and np.array_equal(s.data.ctrl,goals)
        s.step(.5);assert not s.settled()
        s.step(.15);assert s.settled()
    finally:s.close()


def test_recorded_camera_scene_is_separate_and_uses_saved_resolution(tmp_path,monkeypatch):
    from station_robotics.scenes import load_scene, reference_path
    scene=load_scene('recorded_bench','so101')
    assert scene['status']=='estimated_not_calibrated'
    assert not scene['fit']['held_out_validation']
    import station_robotics.scenes as scenes
    monkeypatch.setattr(scenes,'ROOT',tmp_path)
    assert reference_path(scene,'overhead') is None
    recording=tmp_path/scene['source']['overhead'];recording.parent.mkdir(parents=True);recording.write_bytes(b'fixture')
    assert reference_path(scene,'overhead')==recording
    with pytest.raises(ValueError,match='different robot'):load_scene('recorded_bench','elrobot')
    with pytest.raises(ValueError,match='Unknown'):load_scene('../state','so101')
    s=Simulation(render=False,scene='recorded_bench')
    try:
        assert (s.width,s.height)==(853,480)
        assert s.model.camera('wrist').bodyid==s.model.body('camera_optical_frame').id
        point=np.array([.20,-.1,.022])
        assert np.allclose(s.unproject(s.project(point,'overhead'),'overhead'),point)
        assert s.state()['scene']=='recorded_bench'
        assert np.allclose(s.command_q,s.profile.hover)
        # Recorded joint pose is only used for a separate comparison data object.
        assert not np.allclose(s.command_q,list(scene['recorded_joints'].values())[:5])
    finally:s.close()


def test_recording_endpoints_do_not_allow_paths_or_hardware_actions():
    client=TestClient(create_app('elrobot'))
    assert client.post('/api/scene/recorded_bench').status_code==400
    assert client.post('/api/scene/baseline').status_code==200
    assert client.get('/api/recording/state.json').status_code==404
    assert client.post('/api/action',json={'command':'relocate','pixel':[300,500]}).status_code==400
