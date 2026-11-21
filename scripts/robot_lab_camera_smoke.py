"""Rendered camera-variation checks. Offline only; no physical camera or robot IO."""
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from station_robotics.simulation import Simulation

out=Path('logs/robot-lab/camera-match');out.mkdir(parents=True,exist_ok=True)
results=[]
for robot,scene in [('so101','recorded_bench'),('elrobot','baseline')]:
    for tilt,fov_scale in [(0,1.),(5,1.1),(-5,.9)]:
        sim=Simulation(robot,scene=scene)
        try:
            cid=sim.model.camera('wrist').id
            rotation=Rotation.from_quat(np.roll(sim.model.cam_quat[cid],-1))
            sim.model.cam_quat[cid]=np.roll((rotation*Rotation.from_euler('y',tilt,degrees=True)).as_quat(),1)
            sim.model.cam_fovy[cid]*=fov_scale
            mujoco.mj_forward(sim.model,sim.data)
            if tilt==0:
                for camera,data in sim.match_frames.items():(out/f'recorded-pose-{camera}.jpg').write_bytes(data)
            sim.observe();sim.start()
            for _ in range(600):
                sim.step(.05);sim.observe()
                if not sim.running:break
            state=sim.state()
            row={'robot':robot,'scene':scene,'wrist_tilt_deg':tilt,'wrist_fov_scale':fov_scale,'stage':sim.stage,'error_px':sim.error_px,'moves':sim.servo.moves,'last_error':sim.last_error}
            results.append(row);print(json.dumps(row),flush=True)
            (out/f'{robot}-tilt{tilt}.json').write_text(json.dumps(state,indent=2))
            (out/f'{robot}-tilt{tilt}-wrist.jpg').write_bytes(sim.frames['wrist'])
        finally:sim.close()
(out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
assert all(r['stage']=='complete' and r['error_px']<=4 for r in results),results
