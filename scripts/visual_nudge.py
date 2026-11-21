"""One inspected <=10 mm correction, then camera evidence; no gripping."""
import argparse,asyncio,json
from pathlib import Path
import numpy as np
from so101_station.cleanup.backend import StationBackend
from so101_station.cleanup.geometry import ArmModel
from so101_station.cleanup.executor import Executor

async def main(a):
    delta=np.array([a.x,a.y,a.z])/1000
    if np.linalg.norm(delta)>.01001:raise ValueError('one correction is limited to 10 mm')
    m=ArmModel();b=StationBackend('127.0.0.1:8888','5B91029197','/3f030e6315da479a787bb3ba3401932c12eb6355cdcea95826ce7542d66fd712/usbvideo/bc7fa9721806aa6a25f910651a7f06cb',m,True);ex=Executor(b,m,.002)
    try:
        await b.start();s=b.state();start=np.array(s['q']);xyz=m.fk(start)[:3,3]
        if not s['torque']:raise ValueError('arm must already be holding')
        if (xyz+delta)[2]<.012:raise ValueError('target too low for visual correction')
        planner=ArmModel();planner.lower+=.04;planner.upper-=.04;q=planner.solve(xyz+delta,start,.002)
        path=np.linspace(start,q,max(2,int(np.ceil(np.max(np.abs(q-start))/.025)))+1)[1:]
        for p in path:m.validate(p,.002)
        await ex.path(path.tolist());await asyncio.sleep(.5)
        for name,queue in [('top','bc7fa9721806aa6a25f910651a7f06cb'),('wrist','1941c8f6b02f6c2624e37302625e3bbb')]:
            b.camera_queue='/3f030e6315da479a787bb3ba3401932c12eb6355cdcea95826ce7542d66fd712/usbvideo/'+queue
            frame=await b.look();p=Path('logs/taught-pickup')/(a.name+'-'+name+'.jpg');p.write_bytes(frame.jpeg);print(str(p.resolve()),flush=True)
        print(json.dumps({'q':b.state()['q'],'xyz':m.fk(b.state()['q'])[:3,3].tolist()}))
    except BaseException:
        ex.stopped.set()
        try:await b.hold()
        except Exception:pass
        raise
    finally:await b.close()

p=argparse.ArgumentParser();p.add_argument('--x',type=float,default=0);p.add_argument('--y',type=float,default=0);p.add_argument('--z',type=float,default=0);p.add_argument('--name',required=True)
asyncio.run(main(p.parse_args()))
