"""Re-fit the *estimated* camera using saved, manually labelled CAD landmarks.

This only updates an offline simulation scene. It is not a camera/hand-eye calibration.
"""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from station_robotics.model import camera_rotation

path=Path(__file__).resolve().parents[1]/'src/station_robotics/scenes/recorded_bench.json'
scene=json.loads(path.read_text());landmarks=scene['fit']['landmarks']
xyz=np.array([row['model_world_m'] for row in landmarks]);uv=np.array([row['pixel'] for row in landmarks])
width,height=scene['resolution']
position=np.array([.6,0,.42]);rotation=camera_rotation(position,np.array([.14,0,.06]))
initial=np.r_[position,Rotation.from_matrix(rotation).as_rotvec(),60.]

def project(parameters):
    rotation=Rotation.from_rotvec(parameters[3:6]).as_matrix()
    points=(xyz-parameters[:3])@rotation
    focal=height/(2*np.tan(np.deg2rad(parameters[6]/2)))
    return np.column_stack([width/2+focal*points[:,0]/-points[:,2],height/2-focal*points[:,1]/-points[:,2]])

def residual(parameters):
    # FOV is weakly constrained by a prior, not measured lens intrinsics.
    return np.r_[(project(parameters)-uv).ravel(),(parameters[6]-60)/6]

fit=least_squares(residual,initial,bounds=([.2,-.5,.15,-6,-6,-6,35],[1.2,.5,1.,6,6,6,100]))
errors=np.linalg.norm(project(fit.x)-uv,axis=1)
scene['overhead']={'position_m':fit.x[:3].tolist(),'rotation_matrix':Rotation.from_rotvec(fit.x[3:6]).as_matrix().tolist(),'fovy_deg':float(fit.x[6])}
for row,error in zip(landmarks,errors):row['residual_px']=float(error)
scene['fit']['rms_px']=float(np.sqrt(np.mean(errors**2)))
scene['status']='estimated_not_calibrated';scene['fit']['held_out_validation']=False
path.write_text(json.dumps(scene,indent=2)+'\n')
print(f'Estimated camera updated: {scene["fit"]["rms_px"]:.2f} px fitting residual. No held-out validation.')
