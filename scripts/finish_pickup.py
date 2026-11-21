"""Grasp at the visually corrected pose, verify lift, then use taught drop."""
import asyncio,json,signal
from pathlib import Path
import numpy as np
from so101_station.cleanup.backend import StationBackend
from so101_station.cleanup.geometry import ArmModel
from so101_station.cleanup.executor import Executor

async def main():
    root=Path('logs/taught-pickup');plan=json.loads((root/'plan.json').read_text())
    m=ArmModel();planner=ArmModel();planner.lower+=.04;planner.upper-=.04
    b=StationBackend('127.0.0.1:8888','5B91029197','/3f030e6315da479a787bb3ba3401932c12eb6355cdcea95826ce7542d66fd712/usbvideo/bc7fa9721806aa6a25f910651a7f06cb',m,True)
    ex=Executor(b,m,.002);stage='plan'
    for sig in (signal.SIGINT,signal.SIGTERM):asyncio.get_running_loop().add_signal_handler(sig,ex.stopped.set)
    def line(a,z):
        path=np.linspace(a,z,max(2,int(np.ceil(np.max(np.abs(np.array(z)-a))/.025)))+1)[1:]
        for q in path:m.validate(q,.002)
        return path.tolist()
    async def photo(name):
        await asyncio.sleep(.5);f=await b.look();p=(root/name).resolve();p.write_bytes(f.jpeg);print('IMAGE '+str(p),flush=True)
    try:
        await b.start();s=b.state();q=np.array(s['q']);xyz=m.fk(q)[:3,3]
        if not s['torque'] or np.linalg.norm(xyz-[.2525,.0815,.0229])>.008:raise ValueError('corrected grasp pose changed')
        ph=planner.solve([*xyz[:2],.07],q,.002)
        d=plan['drop'];dh=planner.solve([*d['tool_xyz_m'][:2],.07],np.array(d['q']),.002)
        paths={'lift':line(q,ph),'carry':line(ph,dh),'drop':line(dh,d['q']),'retreat':line(d['q'],dh)}
        (root/'corrected-plan.json').write_text(json.dumps({'pickup_q':q.tolist(),'paths':paths},indent=2))
        async with asyncio.timeout(180):
            stage='grasp';print('STAGE grasp',flush=True);print(json.dumps(await ex.gripper(0,grasp=True)),flush=True)
            stage='lift';await ex.path(paths['lift']);await photo('second-lift-check.jpg')
            print('WAITING CARRY',flush=True)
            if (await asyncio.to_thread(input)).strip()!='CARRY':raise ValueError('carry cancelled')
            for stage in ('carry','drop'):
                print('STAGE '+stage,flush=True);await ex.path(paths[stage])
            stage='release';await ex.gripper(.9)
            stage='retreat';await ex.path(paths['retreat']);await photo('second-after-drop.jpg')
            print('MOTION_COMPLETE_VERIFY_IMAGE',flush=True)
    except BaseException as e:
        ex.stopped.set();hold_error=None
        try:await b.hold()
        except Exception as h:hold_error=str(h)
        print(json.dumps({'stage':stage,'error':str(e),'hold_error':hold_error}),flush=True);raise
    finally:await b.close()

asyncio.run(main())
