"""Read-only hand-guided table calibration; no motor writes or calibration mat."""
import argparse
import asyncio
import json
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

from .cleanup.backend import StationBackend
from .cleanup.geometry import ArmModel, CleanupError, TableMap, vector


async def capture(backend, directory, contact_confirmed):
    if not contact_confirmed:
        raise CleanupError('operator must confirm both fingertips touch bare table')
    before=backend.state()
    frame=await backend.look()
    await asyncio.sleep(.25)
    after=backend.state()
    if np.max(np.abs(np.array(before['q'])-after['q']))>.01:
        raise CleanupError('arm moved during capture; hold the contact still')
    if any(j['torque'] for j in after['report']['joints'] if j['motor_id']!=5):
        raise CleanupError('hand-guiding requires the other five motors to be unpowered')
    tip=backend.model.fk(after['q'])
    if tip[:3,2] @ [0,0,-1] < np.cos(np.deg2rad(10)):
        raise CleanupError('orient gripper down so both fingertips touch the table')
    if not -.1 <= (time.time_ns()-frame.captured_ns)/1e9 <= 1:
        raise CleanupError('stale camera frame')
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    path=directory/(uuid.uuid4().hex+'.json')
    image_path=path.with_suffix('.jpg').resolve();image_path.write_bytes(frame.jpeg)
    record={'camera_id':frame.camera_id,'image_size':[frame.width,frame.height],
            'captured_ns':frame.captured_ns,'bus':backend.bus,'q':after['q'],
            'tip_xyz_m':tip[:3,3].tolist(),'image_path':str(image_path),
            'pixel':None,'contact_confirmed':True}
    path.write_text(json.dumps(record,indent=2)+'\n')
    return {'record':str(path.resolve()),**record}


def fit(records, drop_pixel):
    if len(records)<7:
        raise CleanupError('need six spread-out touches plus one independent check touch')
    first=records[0]
    for r in records:
        if r.get('excluded'):
            raise CleanupError('remove excluded touch records before fitting')
        if not r.get('contact_confirmed') or any(r[k]!=first[k] for k in ('camera_id','image_size','bus')):
            raise CleanupError('touches must use one fixed camera, resolution and robot bus')
        pixel=vector(r['pixel'],2,'annotated fingertip midpoint pixel')
        if not (0<=pixel[0]<first['image_size'][0] and 0<=pixel[1]<first['image_size'][1]):
            raise CleanupError('touch pixel outside image')
        vector(r['tip_xyz_m'],3,'touch position')
    xyz=np.array([r['tip_xyz_m'] for r in records])
    floor=float(np.median(xyz[:,2]))
    if np.max(np.abs(xyz[:,2]-floor))>.004:
        raise CleanupError('touch heights disagree by >4 mm; check contact, model and table level')
    training=records[:-1]
    hull=cv2.convexHull(xyz[:-1,:2].astype(np.float32)).reshape(-1,2)
    config={'sim_only':False,'camera_fixed':True,'camera_id':first['camera_id'],
            'image_size':first['image_size'],'pixel_points':[r['pixel'] for r in training],
            'table_points_m':xyz[:-1,:2].tolist(),'workspace_polygon_m':hull.tolist(),
            'drop_xy_m':hull.mean(axis=0).tolist(),'table_z_m':floor,
            'grasp_tip_z_m':floor+.008,'travel_z_m':floor+.075,'drop_tip_z_m':floor+.04}
    table=TableMap(config)
    def project(pixel):
        p=table.H @ np.r_[vector(pixel,2,'pixel'),1]
        if abs(p[2])<1e-8:raise CleanupError('pixel maps to infinity')
        xy=p[:2]/p[2];table.check_xy(xy);return xy
    error=float(np.linalg.norm(project(records[-1]['pixel'])-xyz[-1,:2]))
    if error>.005:
        raise CleanupError(f'independent touch check misses by {error*1000:.1f} mm')
    config['drop_xy_m']=project(drop_pixel).tolist()
    TableMap(config)
    config['touch_check_error_m']=error
    config['bench_verified']=False
    return config


async def live(args):
    backend=StationBackend(args.host,args.bus_serial,args.camera_queue,ArmModel(),False)
    try:
        await backend.start()
        return await capture(backend,args.out,args.contact_confirmed)
    finally:await backend.close()


def main():
    ap=argparse.ArgumentParser(description=__doc__);sub=ap.add_subparsers(dest='command',required=True)
    c=sub.add_parser('capture');c.add_argument('--host',default='127.0.0.1:8888')
    c.add_argument('--bus-serial',required=True);c.add_argument('--camera-queue',required=True)
    c.add_argument('--contact-confirmed',action='store_true');c.add_argument('--out',default='logs/touch-calibration')
    m=sub.add_parser('mark');m.add_argument('record',type=Path);m.add_argument('--pixel',nargs=2,type=float,required=True)
    f=sub.add_parser('fit');f.add_argument('records',nargs='+',type=Path)
    f.add_argument('--drop-pixel',nargs=2,type=float,required=True);f.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    try:
        if args.command=='capture':result=asyncio.run(live(args))
        elif args.command=='mark':
            result=json.loads(args.record.read_text());result['pixel']=vector(args.pixel,2,'pixel').tolist()
            args.record.write_text(json.dumps(result,indent=2)+'\n')
        else:
            result=fit([json.loads(p.read_text()) for p in args.records],args.drop_pixel)
            args.out.parent.mkdir(parents=True,exist_ok=True)
            args.out.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2));return 0
    except (ValueError,OSError,KeyError,TypeError,TimeoutError) as exc:
        print(json.dumps({'error':str(exc)}));return 1


if __name__=='__main__':raise SystemExit(main())
