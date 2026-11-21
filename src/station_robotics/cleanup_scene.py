"""Rigid tabletop props for the MuJoCo cleanup task. Labels stay in evaluation only."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from .model import fmt


def add_cleanup_scene(root, profile):
    world=root.find('worldbody');asset=root.find('asset')
    center=profile.fk(profile.hover)[:2,3];radial=center/np.linalg.norm(center);side=np.array([-radial[1],radial[0]])
    positions=[center+side*.042,center-side*.040,center+radial*.10+side*.02,center-radial*.055-side*.09]
    # Opaque simulation IDs are independent of policy labels. Never publish this mapping to the agent.
    specs=[('item_a',positions[0],.016),('item_b',positions[1],.008),('item_c',positions[2],.016),('item_d',positions[3],.005)]
    bodies=[]
    for name,xy,z in specs:
        body=ET.SubElement(world,'body',name=name,pos=fmt([*xy,z]))
        ET.SubElement(body,'freejoint',name=name+'_free');bodies.append(body)
    def geom(body,**kw):
        defaults=dict(contype='2',conaffinity='3',friction='1.1 .015 .002',condim='4',mass='.005',solref='.006 1')
        defaults.update(kw);return ET.SubElement(body,'geom',**defaults)
    # Faceted, irregular white paper; rigid approximation, not deformable paper.
    rng=np.random.default_rng(17);v=rng.normal(size=(45,3));v/=np.linalg.norm(v,axis=1)[:,None];v*=rng.uniform(.011,.018,(45,1));v[:,2]=np.maximum(v[:,2],-.013)
    hull=ConvexHull(v)
    ET.SubElement(asset,'mesh',name='crumple_mesh',vertex=fmt(v.ravel()),face=' '.join(str(int(i)) for i in hull.simplices.ravel()))
    geom(bodies[0],type='mesh',mesh='crumple_mesh',rgba='.87 .86 .80 1',contype='0',conaffinity='0')
    geom(bodies[0],type='box',size='.012 .012 .011',group='3',rgba='.9 .9 .9 0',mass='.001')
    # Crimped empty snack wrapper, with a contrasting central printed panel.
    geom(bodies[1],type='box',size='.025 .012 .005',rgba='.18 .40 .64 1')
    geom(bodies[1],type='box',pos='0 0 .0052',size='.016 .009 .0003',rgba='.87 .74 .28 1',mass='.0001',contype='0',conaffinity='0')
    for sign in [-1,1]:
        for j in range(5):geom(bodies[1],type='box',pos=fmt([sign*.027,(j-2)*.0045,0]),size='.003 .0015 .0018',rgba='.65 .67 .72 1',mass='.0001')
    # Recognizable computer mouse: domed body, separate buttons and scroll wheel.
    geom(bodies[2],type='ellipsoid',size='.023 .037 .015',rgba='.13 .15 .17 1',mass='.07')
    geom(bodies[2],type='box',pos='0 0 .014',size='.0007 .028 .0008',rgba='.45 .47 .49 1',mass='.0001',contype='0',conaffinity='0')
    geom(bodies[2],type='ellipsoid',pos='0 .014 .016',size='.0025 .006 .002',rgba='.65 .67 .69 1',mass='.0001',contype='0',conaffinity='0')
    # Pen: long barrel, metal tip and cap clip. Kept as a dynamic body for disturbance auditing.
    geom(bodies[3],type='capsule',fromto='-.046 0 0 .035 0 0',size='.0035',rgba='.08 .13 .22 1',mass='.012')
    geom(bodies[3],type='capsule',fromto='.035 0 0 .047 0 0',size='.002',rgba='.65 .67 .70 1',mass='.001')
    geom(bodies[3],type='box',pos='-.03 0 .004',size='.014 .001 .0007',rgba='.65 .67 .7 1',mass='.0001',contype='0',conaffinity='0')
    for element in bodies[3].findall('geom'):
        element.set('contype','0');element.set('conaffinity','0')
    geom(bodies[3],type='box',size='.046 .0035 .0025',rgba='.1 .1 .1 0',group='3',mass='.001')
    # Open bin: five actual collision pieces. No teleport or object attachment.
    binxy=center+side*.155-radial*.025
    binbody=ET.SubElement(world,'body',name='bin',pos=fmt([*binxy,0]))
    for pos,size in [([0,0,.004],[.048,.048,.004]),([-.051,0,.025],[.003,.054,.025]),([.051,0,.025],[.003,.054,.025]),([0,-.051,.025],[.048,.003,.025]),([0,.051,.025],[.048,.003,.025])]:
        geom(binbody,type='box',pos=fmt(pos),size=fmt(size),rgba='.30 .47 .40 1',mass='0')
    return root
