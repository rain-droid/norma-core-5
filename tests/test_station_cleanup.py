"""Policy, isolation and real upstream Station wire contracts for the cleanup demo."""
import asyncio
import numpy as np
import pytest
mujoco=pytest.importorskip('mujoco')
from station_robotics.cleanup_policy import validate_annotations
from station_robotics.cleanup_world import CleanupWorld
from station_robotics.station_protocol import JointBinding,SimStation,OwnedClient,StationActuators,st,usb
from station_robotics.profiles import load_profile


def sample():
    return [dict(label='crumpled paper',decision='trash',confidence=.96,bbox=[100,100,30,30],height_mm=15),dict(label='computer mouse',decision='keep',confidence=.99,bbox=[200,100,50,40],height_mm=15),dict(label='bin',decision='bin',confidence=.99,bbox=[300,100,90,90],height_mm=0)]


def test_semantic_selection_protects_belongings_unknown_and_overlap():
    assert len(validate_annotations(sample(),853,480))==3
    for label,decision,confidence in [('computer mouse','trash',.99),('pen','trash',.99),('crumpled paper','trash',.6),('something','trash',.99)]:
        a=sample();a[0].update(label=label,decision=decision,confidence=confidence)
        with pytest.raises(ValueError):validate_annotations(a,853,480)
    a=sample();a[1]['bbox']=[110,110,40,40]
    with pytest.raises(ValueError,match='overlaps'):validate_annotations(a,853,480)
    a=sample();a[0]['confidence']=float('nan')
    with pytest.raises(ValueError):validate_annotations(a,853,480)


@pytest.mark.parametrize('robot,count',[('so101',6),('elrobot',8)])
def test_station_binding_handles_both_morphologies(robot,count):
    binding=JointBinding.simulated(load_profile(robot));q=(binding.lower+binding.upper)/2
    ticks=binding.encode(q);assert len(ticks)==count
    assert np.max(abs(binding.decode(ticks)-q))<=np.pi/4096
    with pytest.raises(ValueError):binding.decode({1:2048})
    with pytest.raises(ValueError):binding.encode(binding.upper+1)


def test_world_does_not_publish_evaluation_labels_and_rejects_stale_motion():
    w=CleanupWorld(render=False)
    try:
        assert not any(n in str(w.state()) for n in ['item_a','item_b','item_c','item_d'])
        assert not w.running
        with pytest.raises(ValueError,match='classify'):w.start()
        with pytest.raises(ValueError,match='Stale'):w.apply_goals(np.r_[w.profile.hover,0],'old-generation')
        with pytest.raises(ValueError,match='expired'):w.submit('old',sample())
    finally:w.close()


@pytest.mark.parametrize('robot',['so101','elrobot'])
def test_upstream_station_client_command_state_and_camera_roundtrip(robot):
    async def trial():
        b=JointBinding.simulated(load_profile(robot));seen=[]
        snap={'binding':b,'q':(b.lower+b.upper)/2,'generation':'scene-1','frames':{'overhead':b'jpeg-fixture'},'resolution':[853,480]}
        server=SimStation(lambda:snap,lambda q,g:seen.append((q,g)));port=await server.start(0);client=OwnedClient(f'127.0.0.1:{port}')
        try:
            await client.wait_ready(timeout=3);actuator=StationActuators(client,'MUJOCO')
            await asyncio.wait_for(actuator.goals(b.encode(snap['q'])),2)
            assert len(seen)==1 and seen[0][1]=='scene-1'
            stream=client.read_from_tail('st3215/inference',offset=b'\x00',limit=1,step=1,buf_size=2)
            entry=await asyncio.wait_for(stream.data.get(),2);motors=st.InferenceStateReader(entry.Data).get_buses()[0].get_motors()
            assert len(motors)==len(b.ids)
            stream=client.read_from_tail('usbvideo/overhead',offset=b'\x00',limit=1,step=1,buf_size=2)
            entry=await asyncio.wait_for(stream.data.get(),2);pack=usb.RxEnvelopeReader(entry.Data).get_frames()
            assert pack.get_format().get_width()==853 and bytes(pack.get_frames_data()[0])==b'jpeg-fixture'
            with pytest.raises(Exception):await asyncio.wait_for(StationActuators(client,'REAL_BUS').goals(b.encode(snap['q'])),2)
            assert len(seen)==1
        finally:await client.close();await server.close()
    asyncio.run(trial())
