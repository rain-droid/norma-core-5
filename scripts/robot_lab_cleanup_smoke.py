"""Physics/Station regression, using explicit image-annotation fixtures, not an AI benchmark."""
import asyncio
from collections import deque
import json
from pathlib import Path
from station_robotics.cleanup_world import CleanupWorld
from station_robotics.station_protocol import SimStation,OwnedClient,StationActuators,usb

ROOT=Path(__file__).resolve().parents[1]


async def run(robot,objects):
    out=ROOT/'logs/robot-lab/cleanup/acceptance';out.mkdir(parents=True,exist_ok=True)
    world=CleanupWorld(robot);world.observe();pending=deque()
    def snapshot():return {'binding':world.binding,'q':world.data.qpos[world.qidx].copy(),'generation':world.generation,'frames':{**world.frames,'overhead':world.raw_overhead},'resolution':[world.width,world.height]}
    server=SimStation(snapshot,world.apply_goals);port=await server.start(0);client=OwnedClient(f'127.0.0.1:{port}');world.writer=pending.append
    try:
        await client.wait_ready(timeout=3);actuators=StationActuators(client,'MUJOCO')
        stream=client.read_from_tail('usbvideo/overhead',offset=b'\x00',limit=1,step=1,buf_size=2)
        entry=await asyncio.wait_for(stream.data.get(),2)
        image=bytes(usb.RxEnvelopeReader(entry.Data).get_frames().get_frames_data()[-1]);(out/f'{robot}-before.jpg').write_bytes(image)
        obs=world.capture();world.submit(obs['observation_id'],objects);world.start();held_checks=[]
        for i in range(1400):
            world.step(.04)
            while pending:await actuators.goals(pending.popleft())
            if i%2==0:world.observe()
            if world.stage=='verify_grasp':
                # Evaluation-only truth. This is deliberately outside the production policy.
                name='item_a' if not world.outcomes else 'item_b'
                height_before=float(world.data.body(name).xpos[2]);world.step(20.);world.observe()
                height_after=float(world.data.body(name).xpos[2]);held=height_before>.04 and height_after>.04
                evidence=world.inspect_grasp();held_checks.append({'object':name,'height_before_m':height_before,'height_after_20s_m':height_after,'held':held})
                for camera in ('wrist','overhead'):(out/f'{robot}-{name}-held-{camera}.jpg').write_bytes(world.frames[camera])
                world.assess_grasp(evidence['observation_id'],held)
            if not world.running:break
        world.step(2.);world.observe();evaluation=world.evaluate()
        result={'robot':robot,'stage':world.stage,'error':world.error,'station_writes':actuators.writes,'held_checks':held_checks,'evaluation':evaluation,'annotation_source':'explicit regression fixtures; not automatic recognition'}
        (out/f'{robot}-result.json').write_text(json.dumps(result,indent=2)+'\n')
        for camera in ('overview','overhead','wrist'):(out/f'{robot}-after-{camera}.jpg').write_bytes(world.frames[camera])
        print(json.dumps(result),flush=True)
        return result
    finally:await client.close();await server.close();world.close()


async def main():
    profiles=json.loads((ROOT/'examples/robot-lab-cleanup-observations.json').read_text())['profiles']
    results=[await run(robot,objects) for robot,objects in profiles.items()]
    (ROOT/'logs/robot-lab/cleanup/acceptance/results.json').write_text(json.dumps(results,indent=2)+'\n')
    assert all(r['stage']=='verify' and all(r['evaluation']['trash_in_bin'].values()) and max(r['evaluation']['protected_displacement_m'].values())<.002 and all(c['held'] for c in r['held_checks']) and r['station_writes']>50 for r in results),results
    print('Both Station-controlled cleanups passed, including 20-second holds and protected-object checks.')


if __name__=='__main__':asyncio.run(main())
