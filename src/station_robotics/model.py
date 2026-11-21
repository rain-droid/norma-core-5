"""URDF to MJCF conversion with CAD, inertias, mimic constraints and position actuators."""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def fmt(values):
    return ' '.join(f'{float(x):.10g}' for x in values)


def quat(matrix):
    return fmt(np.roll(Rotation.from_matrix(matrix).as_quat(),1))


def origin(node):
    if node is None:return {'pos':'0 0 0','quat':'1 0 0 0'}
    return {'pos':node.get('xyz','0 0 0'),'quat':fmt(np.roll(Rotation.from_euler('xyz',np.fromstring(node.get('rpy','0 0 0'),sep=' ')).as_quat(),1))}


def camera_rotation(position, target):
    z=np.asarray(position)-target;z/=np.linalg.norm(z)
    x=np.cross([0,0,1],z)
    if np.linalg.norm(x)<1e-6:x=np.array([1.,0,0])
    x/=np.linalg.norm(x);y=np.cross(z,x)
    return np.column_stack([x,y,z])


def build_model(profile, scene=None, task="alignment"):
    root=ET.Element('mujoco',model=profile.key)
    ET.SubElement(root,'compiler',angle='radian',autolimits='true',balanceinertia='true')
    ET.SubElement(root,'option',timestep='.002',integrator='implicitfast',gravity='0 0 -9.81',iterations='50')
    visual=ET.SubElement(root,'visual')
    ET.SubElement(visual,'global',offwidth='960',offheight='640')
    ET.SubElement(visual,'headlight',ambient='.35 .35 .35',diffuse='.65 .65 .65',specular='.2 .2 .2')
    ET.SubElement(visual,'quality',shadowsize='2048',offsamples='4')
    default=ET.SubElement(root,'default')
    ET.SubElement(default,'joint',damping='.3',armature='.005',frictionloss='.035')
    ET.SubElement(default,'geom',friction='.8 .005 .0001',condim='3',solref='.01 1')
    asset=ET.SubElement(root,'asset');mesh_names={}
    ET.SubElement(asset,'texture',name='table_grid',type='2d',builtin='checker',width='512',height='512',rgb1='.22 .25 .25',rgb2='.24 .27 .27')
    ET.SubElement(asset,'material',name='table_mat',texture='table_grid',texrepeat='12 12',texuniform='true',reflectance='.06')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='.1 -.3 1.3',dir='0 .2 -1',diffuse='.85 .85 .85',castshadow='true')
    ET.SubElement(world,'geom',name='table',type='plane',size='1 1 .02',material='table_mat',contype='2',conaffinity='3')
    center=profile.fk(profile.hover)[:3,3]
    for name,pos in [('overview',center+np.array([.40,-.43,.30])),('overhead',center+np.array([.30,-.15,.60]))]:
        aim=np.array([center[0]*.65,center[1]*.65,.09]) if name=='overview' else np.r_[center[:2],0]
        ET.SubElement(world,'camera',name=name,pos=fmt(pos),quat=quat(camera_rotation(pos,aim)),fovy='48')
    if scene:
        from .scenes import table_point
        overhead=world.find("camera[@name='overhead']");c=scene['overhead']
        overhead.set('pos',fmt(c['position_m']));overhead.set('quat',quat(c['rotation_matrix']));overhead.set('fovy',str(c['fovy_deg']))
        # Neutral wood-like tabletop and a visual-only cover footprint from the recording.
        asset.find("texture[@name='table_grid']").set('rgb1','.50 .41 .34')
        asset.find("texture[@name='table_grid']").set('rgb2','.50 .41 .34')
        plate=table_point(c,[315,275],scene['resolution'],.002)
        for index in range(48):
            a,b=np.array([index,index+1])*2*np.pi/48
            ends=np.r_[plate+[.105*np.cos(a),.105*np.sin(a),0],plate+[.105*np.cos(b),.105*np.sin(b),0]]
            ET.SubElement(world,'geom',name=f'recorded_cover_{index}',type='capsule',fromto=fmt(ends),size='.0012',rgba='.75 .82 .85 .4',contype='0',conaffinity='0')
        wrapper=table_point(c,[632,353],scene['resolution'],.004)
        ET.SubElement(world,'geom',name='recorded_wrapper',type='box',pos=fmt(wrapper),size='.066 .022 .003',euler='0 0 -.35',rgba='.22 .40 .34 1',contype='0',conaffinity='0')
    links={n.get('name'):n for n in profile.description.findall('link')}
    children={};by_child={}
    for joint in profile.description.findall('joint'):
        parent=joint.find('parent').get('link');child=joint.find('child').get('link')
        children.setdefault(parent,[]).append(joint);by_child[child]=joint
    base=next(name for name in links if name not in by_child)
    equality=ET.SubElement(root,'equality');actuator=ET.SubElement(root,'actuator')
    colors={'servo':'.10 .12 .13 1','printed':'.78 .67 .39 1','pgripper':'.83 .50 .40 1','silver':'.60 .64 .65 1'}
    if scene:colors.update(printed='.08 .09 .10 1',pgripper='.70 .36 .40 1')
    def add_link(name,parent,incoming=None):
        body=ET.SubElement(parent,'body',name=name,**origin(incoming.find('origin') if incoming is not None else None))
        if incoming is not None and incoming.get('type')!='fixed':
            limit=incoming.find('limit');jointname=incoming.get('name')
            limits=[float(limit.get('lower')),float(limit.get('upper'))]
            if jointname in profile.names:
                i=profile.names.index(jointname);limits=[profile.lower[i],profile.upper[i]]
            ET.SubElement(body,'joint',name=jointname,type='slide' if incoming.get('type')=='prismatic' else 'hinge',axis=incoming.find('axis').get('xyz'),range=fmt(limits))
            mimic=incoming.find('mimic')
            if mimic is not None:
                ET.SubElement(equality,'joint',joint1=jointname,joint2=mimic.get('joint'),polycoef=f"{mimic.get('offset','0')} {mimic.get('multiplier','1')} 0 0 0",solref='.004 1')
            elif jointname in profile.names or jointname==profile.gripper:
                ET.SubElement(actuator,'position',name=jointname,joint=jointname,kp='180',kv='8',ctrlrange=fmt(limits),forcerange='-2.94 2.94')
        link=links[name];inertial=link.find('inertial')
        if inertial is not None:
            inertia=inertial.find('inertia');vals=[float(inertia.get(k,'0')) for k in ['ixx','iyy','izz','ixy','ixz','iyz']]
            # Upstream ElRobot gear uses a zero inertia tensor; positive numerical floor.
            vals[:3]=[max(v,1e-8) for v in vals[:3]]
            ET.SubElement(body,'inertial',pos=(inertial.find('origin').get('xyz','0 0 0') if inertial.find('origin') is not None else '0 0 0'),mass=inertial.find('mass').get('value'),fullinertia=fmt(vals))
        for index,node in enumerate(link.findall('visual')):
            geometry=node.find('geometry');mesh=geometry.find('mesh');material=node.find('material');mat=material.get('name','silver') if material is not None else 'silver'
            color=colors.get(mat,'.10 .12 .13 1' if 'ST3215' in mat else colors['silver'])
            params={**origin(node.find('origin')),'rgba':color,'contype':'0','conaffinity':'0','group':'2','mass':'0'}
            if mesh is not None:
                path=(profile.mesh_root/mesh.get('filename')).resolve()
                if not path.exists():raise FileNotFoundError(f'Missing CAD asset: {path}')
                key=(str(path),mesh.get('scale','1 1 1'))
                if key not in mesh_names:
                    mesh_names[key]=f'mesh{len(mesh_names)}';ET.SubElement(asset,'mesh',name=mesh_names[key],file=str(path),scale=key[1],maxhullvert='64')
                params.update(type='mesh',mesh=mesh_names[key])
            elif geometry.find('box') is not None:params.update(type='box',size=fmt(np.fromstring(geometry.find('box').get('size'),sep=' ')/2))
            elif geometry.find('cylinder') is not None:
                c=geometry.find('cylinder');params.update(type='cylinder',size=f"{c.get('radius')} {float(c.get('length'))/2}")
            else:continue
            ET.SubElement(body,'geom',name=f'{name}_visual{index}',**params)
            # Convex CAD collision hulls are conservative approximations, separate from visuals.
            collision={**params,'contype':'1','conaffinity':'2','group':'3','rgba':'.8 .2 .2 .2'}
            ET.SubElement(body,'geom',name=f'{name}_collision{index}',**collision)
        if task=='cleanup' and incoming is not None and incoming.get('type')=='prismatic' and incoming.find('mimic') is not None:
            # Flat contact pads approximate the inner finger surfaces; the CAD hull
            # includes the rack/backing and is unsuitable as a single grasp contact shape.
            for geom in body.findall('geom'):
                if geom.get('group')=='3':geom.set('contype','0');geom.set('conaffinity','0')
            sign=np.sign(float(incoming.find('mimic').get('multiplier')))
            jo=incoming.find('origin');pos=np.fromstring(jo.get('xyz','0 0 0'),sep=' ')
            R=Rotation.from_euler('xyz',np.fromstring(jo.get('rpy','0 0 0'),sep=' ')).as_matrix()
            pad=np.array([.0017+sign*.0288,.077,0.])
            ET.SubElement(body,'geom',name=name+'_contact_pad',type='box',pos=fmt(R.T@(pad-pos)),quat=quat(R.T),size='.002 .0265 .010',rgba='.3 .3 .3 .2',group='3',mass='0',contype='1',conaffinity='2',friction='1.5 .02 .003',condim='6',solref='.004 1',solimp='.99 .999 .001')
        if name==profile.tool_parent:
            T=profile.tool_transform
            ET.SubElement(body,'site',name='tool',pos=fmt(T[:3,3]),quat=quat(T[:3,:3]),size='.002',rgba='.3 .95 .75 1',group='4')
            if not scene:
                p=np.array([0,-.05,-.08]);target=np.array([0,0,.05]);R=camera_rotation(p,target)
                ET.SubElement(body,'camera',name='wrist',pos=fmt(T[:3,3]+T[:3,:3]@p),quat=quat(T[:3,:3]@R),fovy='65')
        if scene and name==scene['wrist']['body']:
            c=scene['wrist']
            ET.SubElement(body,'camera',name='wrist',pos=fmt(c['position_m']),quat=quat(c['rotation_matrix']),fovy=str(c['fovy_deg']))
        for joint in children.get(name,[]):add_link(joint.find('child').get('link'),body,joint)
    add_link(base,world)
    if task=='cleanup':
        root.find('option').set('cone','elliptic');root.find('option').set('impratio','10');root.find('option').set('noslip_iterations','3')
        # Reduced simulated grip torque: rigid objects must stall the jaws instead of
        # being crushed through soft contact constraints. This is not a measured motor limit.
        gripact=actuator.find(f"position[@joint='{profile.gripper}']")
        gripact.set('kp','4');gripact.set('kv','.015');gripact.set('forcerange','-.08 .08')
        gripjoint=world.find(f".//joint[@name='{profile.gripper}']");gripjoint.set('damping','.003');gripjoint.set('frictionloss','.005')
        for element in equality:
            element.set('solref','.003 1');element.set('solimp','.99 .999 .001')
        for element in root.iter('geom'):
            if 'jaw' in element.get('name','').lower() and element.get('group')=='3':
                element.set('friction','1.2 .015 .002');element.set('condim','4');element.set('solref','.004 1')
        from .cleanup_scene import add_cleanup_scene
        for name in ('overview','overhead'):
            c=world.find(f"camera[@name='{name}']")
            pos=center+np.array([.32,-.25,.48]) if name=='overview' else center+np.array([.15,-.10,.60])
            c.set('pos',fmt(pos));c.set('quat',quat(camera_rotation(pos,np.r_[center[:2],0])));c.set('fovy','48')
        asset.find("texture[@name='table_grid']").set('rgb1','.46 .36 .27')
        asset.find("texture[@name='table_grid']").set('rgb2','.46 .36 .27')
        add_cleanup_scene(root,profile)
        return ET.tostring(root,encoding='unicode')
    # Rigid trash surrogate; no claims about flexible paper grasp dynamics.
    paper=ET.SubElement(world,'body',name='paper',pos=fmt(np.r_[center[:2]+[.018,-.012],.02]))
    ET.SubElement(paper,'freejoint',name='paper_free')
    for i,(p,s) in enumerate([([0,0,0],[.015,.018,.016]),([.008,.006,.006],[.010,.012,.010]),([-.009,-.006,.003],[.009,.012,.012])]):
        ET.SubElement(paper,'geom',name=f'paper{i}',type='ellipsoid',pos=fmt(p),size=fmt(s),rgba='.88 .36 .21 1',mass='.004',contype='0',conaffinity='0')
    ET.SubElement(paper,'geom',name='paper_contact',type='box',pos='0 0 -.007',size='.014 .014 .009',mass='.001',contype='2',conaffinity='3',group='3')
    return ET.tostring(root,encoding='unicode')
