"""Execute the inspected local two-pose pickup, with hold/visual checkpoints."""
import asyncio
import fcntl
import hashlib
import json
import signal
from pathlib import Path
import numpy as np
from so101_station.cleanup.backend import StationBackend
from so101_station.cleanup.geometry import ArmModel, CleanupError
from so101_station.cleanup.executor import Executor


async def main():
    root=Path('logs/taught-pickup');plan=json.loads((root/'plan.json').read_text())
    host,bus='127.0.0.1:8888','5B91029197'
    lock=open('/tmp/so101-cleanup-'+hashlib.sha256(f'{host}/{bus}'.encode()).hexdigest()[:16]+'.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    model=ArmModel();backend=StationBackend(host,bus,'/3f030e6315da479a787bb3ba3401932c12eb6355cdcea95826ce7542d66fd712/usbvideo/bc7fa9721806aa6a25f910651a7f06cb',model,True)
    ex=Executor(backend,model,plan['floor']);stage='preflight';enabled=False
    for sig in (signal.SIGINT,signal.SIGTERM):asyncio.get_running_loop().add_signal_handler(sig,ex.stopped.set)
    async def photo(name):
        await asyncio.sleep(.5);f=await backend.look();p=(root/(name+'.jpg')).resolve();p.write_bytes(f.jpeg);print('IMAGE '+str(p),flush=True)
    async def checkpoint(word):
        print('WAITING '+word,flush=True)
        if (await asyncio.to_thread(input)).strip()!=word:raise CleanupError('checkpoint cancelled')
        ex.check()
    try:
        await backend.start();state=backend.state()
        # Hand-supported joints may drift after teaching. Hold their measured
        # pose, and validate a new initial lift; never jump to the old pose.
        model.validate(state['q'],plan['floor'])
        end=np.array(plan['paths']['raise_drop'][-1]);start=np.array(state['q'])
        steps=max(1,int(np.ceil(np.max(np.abs(end-start))/.03)))
        plan['paths']['raise_drop']=np.linspace(start,end,steps+1)[1:].tolist()
        for path in plan['paths'].values():
            for q in path:model.validate(q,plan['floor'])
        stage='hold';await backend.enable(ex.stopped);enabled=True
        print('HOLDING_ALL_SIX '+json.dumps(backend.state()['ticks']),flush=True)
        await checkpoint('GO')
        async with asyncio.timeout(180):
            stage='open';await ex.gripper(.9)
            for stage in ('raise_drop','above_pickup','descend'):
                print('STAGE '+stage,flush=True);await ex.path(plan['paths'][stage])
            stage='grasp';print('STAGE grasp',flush=True);print(json.dumps(await ex.gripper(0,grasp=True)),flush=True)
            stage='lift';await ex.path(plan['paths']['lift']);await photo('lift-check')
            await checkpoint('CARRY')
            for stage in ('carry','drop'):
                print('STAGE '+stage,flush=True);await ex.path(plan['paths'][stage])
            stage='release';await ex.gripper(.9)
            stage='retreat';await ex.path(plan['paths']['retreat']);await photo('after-drop')
            print('MOTION_COMPLETE_VERIFY_IMAGE',flush=True)
    except BaseException as e:
        ex.stopped.set();hold_error=None
        if enabled:
            try:await backend.hold()
            except Exception as h:hold_error=str(h)
        print(json.dumps({'failed_stage':stage,'error':str(e),'hold_error':hold_error}),flush=True)
        raise
    finally:
        await backend.close();lock.close()


asyncio.run(main())
