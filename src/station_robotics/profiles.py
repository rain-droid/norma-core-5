"""Robot-specific descriptions. The controller never imports either hardware driver."""
from dataclasses import dataclass
from pathlib import Path
import copy
import re
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]


@dataclass
class Profile:
    key: str
    label: str
    description: ET.Element
    mesh_root: Path
    names: list[str]
    gripper: str
    tool_parent: str
    tool_transform: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    hover: np.ndarray
    fk_fn: object
    solve_fn: object
    source: str
    rest: object = None
    fingertip_backoff: float = 0.

    def fk(self, q):
        return self.fk_fn(np.asarray(q))

    def solve(self, xyz, seed):
        q = np.asarray(self.solve_fn(np.asarray(xyz), np.asarray(seed)))
        if q.shape != (len(self.names),) or not np.isfinite(q).all():
            raise ValueError('Invalid IK output')
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError('IK exceeded profile joint limits')
        if np.linalg.norm(self.fk(q)[:3, 3]-xyz) > .004:
            raise ValueError('IK endpoint error exceeds 4 mm')
        return q


def _so_description(model):
    from so101_station.calibration import DEFAULT_PROFILE
    result = ET.Element('robot', name='so101_pgripper')
    limits = {j.name: (j.lower, j.upper) for j in model.calibration.joints}
    def expand(value):
        value = value.replace('${prefix}', '').replace('${parent}', 'gripper_link')
        value = value.replace('${mount_xyz}', '0 0 0').replace('${mount_rpy}', '-1.57079632679 0 -1.57079632679')
        return re.sub(r"\$\{limits\['([^']+)'\]\['(lower|upper)'\]\}", lambda m: str(limits[m[1]][m[2]=='upper']), value)
    for filename, macro in [('so101_arm_macro.xacro','so101_arm'),('pgripper_macro.xacro','pgripper')]:
        root=ET.parse(DEFAULT_PROFILE/'model'/filename).getroot()
        body=next(n for n in root if n.tag.endswith('macro') and n.get('name')==macro)
        for node in body:
            if node.tag not in ('link','joint'): continue
            node=copy.deepcopy(node)
            for element in node.iter():
                element.attrib.update({k:expand(v) for k,v in element.attrib.items()})
            for child in list(node):
                if not child.tag.endswith('mesh_pair'): continue
                node.remove(child)
                is_arm=child.tag.endswith('so101_mesh_pair')
                file=('meshes/so101/' if is_arm else 'meshes/pgripper/')+child.get('filename')
                for kind in ('visual','collision'):
                    visual=ET.SubElement(node,kind)
                    ET.SubElement(visual,'origin',xyz=child.get('xyz','0 0 0'),rpy=child.get('rpy','0 0 0'))
                    geom=ET.SubElement(visual,'geometry')
                    ET.SubElement(geom,'mesh',filename=file,scale='1 1 1' if is_arm else '.001 .001 .001')
                    ET.SubElement(visual,'material',name=child.get('material','pgripper'))
            result.append(node)
    return result


def load_profile(key):
    if key=='so101':
        from so101_station.cleanup.geometry import ArmModel
        model=ArmModel()
        hover=np.array([-.4740007575,-.0246145397,.0240617227,1.5713539892,-.6734175659])
        return Profile(key,'SO-101 + parallel gripper',_so_description(model),ROOT/'assets/so101_pgripper',
            [j.name for j in model.joints],'gripper','tool0',np.eye(4),model.lower,model.upper,hover,
            model.fk,lambda xyz,q:model.solve(xyz,q,.002),
            'AtharvaBhorpe/so_arm101_pgripper @ c79830e · Apache-2.0 / MIT',
            rest=np.array([-.0108,-1.7291,1.5864,-1.6416,0.]),fingertip_backoff=0.)
    if key=='elrobot':
        from elrobot.kinematics import Kinematics, ARM_JOINT_NAMES, GRIPPER_JOINT_NAME, TIP_LINK
        path=ROOT/'assets/elrobot/elrobot_follower.urdf';kin=Kinematics(path)
        def solve(xyz,seed):
            result=kin.ik(xyz,approach=[0,0,-1],q_seed=seed,q_ref=seed,ref_weight=.0001,random_seeds=4,rng=np.random.default_rng(7))
            if not result.ok: raise ValueError(result.describe())
            return result.q
        hover=solve([0,.20,.08],np.zeros(kin.n))
        return Profile(key,'ElRobot + parallel gripper',ET.parse(path).getroot(),path.parent,
            ARM_JOINT_NAMES,GRIPPER_JOINT_NAME,TIP_LINK,kin.T_tip_tool,kin.lower+.05,kin.upper-.05,hover,
            kin.fk,solve,'NormaCore ElRobot · repository URDF / MIT',
            rest=np.array([-.04353,-1.56219,-1.01721,1.26380,-.03994,.56499,-.22479]),fingertip_backoff=.012)
    raise ValueError('Unknown robot profile; choose so101 or elrobot')
