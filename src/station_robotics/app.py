"""Local MuJoCo demo. There is deliberately no hardware backend or Station URL option."""
import argparse
import asyncio
import base64
from contextlib import asynccontextmanager
import json
from pathlib import Path
import queue
import threading
import time
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, Response
from pydantic import BaseModel, Field
import uvicorn
from .simulation import Simulation
from .scenes import load_scene, reference_path
from .cleanup_world import CleanupWorld
from .station_protocol import SimStation,OwnedClient,StationActuators,usb


class Lab:
    def __init__(self,profile):
        self.profile=profile;self.task='alignment';self.actuators=None;self.loop=None;self.wire_snapshot={};self.commands=queue.Queue();self.lock=threading.Lock();self.state={'stage':'loading','hardware_connected':False}
        self.frames={};self.match_frames={};self.references={};self.quit=threading.Event();self.thread=threading.Thread(target=self.work,daemon=True)

    def station_snapshot(self):
        with self.lock:return self.wire_snapshot.copy()

    def send_goals(self,ticks):
        if self.actuators is None:raise ValueError('Station transport is not ready')
        future=asyncio.run_coroutine_threadsafe(self.actuators.goals(ticks),self.loop)
        try:future.result(timeout=2)
        except Exception:future.cancel();raise

    def publish_wire(self,sim):
        with self.lock:
            self.wire_snapshot={'binding':sim.binding,'q':sim.data.qpos[sim.qidx].copy(),'frames':{**sim.frames,'overhead':sim.raw_overhead},'resolution':[sim.width,sim.height],'generation':sim.generation}

    def station_camera(self,camera):
        async def read():
            stream=self.actuators.client.read_from_tail('usbvideo/'+camera,offset=b'\x00',limit=1,step=1,buf_size=2)
            entry=await asyncio.wait_for(stream.data.get(),2)
            if entry is None:raise ValueError('Station camera stream ended')
            return bytes(usb.RxEnvelopeReader(entry.Data).get_frames().get_frames_data()[-1])
        future=asyncio.run_coroutine_threadsafe(read(),self.loop)
        try:return future.result(timeout=3)
        except Exception:future.cancel();raise

    def invoke(self,name,args):
        answer=queue.Queue(maxsize=1);self.commands.put(('tool',(name,args,answer)))
        try:return answer.get(timeout=12)
        except queue.Empty:raise ValueError('Simulator did not answer in time')

    def work(self):
        sim=None
        try:
            sim=Simulation(self.profile);sim.observe();last=time.monotonic();next_frame=last
            while not self.quit.is_set():
                try:
                    command,value=self.commands.get_nowait()
                    if command in ('task','profile'):
                        if sim.running:raise ValueError('Pause before switching task or robot')
                        if command=='task':self.task=value
                        else:self.profile=value
                        sim.close();sim=CleanupWorld(self.profile) if self.task=='cleanup' else Simulation(self.profile)
                        if self.task=='cleanup':sim.writer=self.send_goals
                        sim.observe()
                    elif command=='station_goals':
                        if isinstance(sim,CleanupWorld):sim.apply_goals(*value)
                    elif command=='tool':
                        name,args,answer=value
                        try:
                            if not isinstance(sim,CleanupWorld):raise ValueError('Select the cleanup task first')
                            payload={};jpeg=None;extra_images=[]
                            if name=='observe':payload=sim.capture();jpeg=sim.raw_overhead
                            elif name=='classify':payload=sim.submit(args['observation_id'],args['objects'])
                            elif name=='execute':sim.start();payload={'started':True,'transport':'Station queue API'}
                            elif name=='stop':sim.pause();payload=sim.state()
                            elif name=='status':payload=sim.state()
                            elif name=='verify_grasp':payload=sim.inspect_grasp();jpeg=sim.frames['wrist'];extra_images=[sim.raw_overhead]
                            elif name=='assess_grasp':sim.assess_grasp(args.get('observation_id'),args.get('held'));payload=sim.state()
                            elif name=='verify':
                                if sim.stage!='verify':raise ValueError('Wait for release and return to rest before verifying')
                                sim.observe();payload=sim.capture();payload['outcomes']=sim.outcomes;jpeg=sim.raw_overhead
                            elif name=='assess':
                                if sim.stage!='verify' or not sim.observation or args.get('observation_id')!=sim.observation['id']:raise ValueError('Inspect a fresh final verification image first')
                                passed=args.get('trash_in_bin') is True and args.get('protected_untouched') is True
                                sim.stage='complete' if passed else 'needs_review';sim.log('Codex visually confirmed cleanup and preserved objects.' if passed else 'Final visual check needs review.');payload=sim.state()
                            else:raise ValueError('Unknown cleanup tool')
                            if jpeg is not None:
                                self.publish_wire(sim)
                                jpeg=self.station_camera('wrist' if name=='verify_grasp' else 'overhead')
                                if extra_images:extra_images=[self.station_camera('overhead')]
                            payload['station_writes']=self.actuators.writes
                            answer.put({'ok':True,'result':payload,'image_b64':base64.b64encode(jpeg).decode() if jpeg else None,'images_b64':[base64.b64encode(im).decode() for im in extra_images]})
                        except Exception as exc:answer.put({'ok':False,'error':str(exc)})
                    elif command=='scene':
                        if isinstance(sim,CleanupWorld):raise ValueError('Camera comparison is an alignment experiment')
                        if sim.running:raise ValueError('Pause before changing the camera setup')
                        load_scene(value,sim.profile.key)
                        replacement=Simulation(sim.profile.key,scene=value)
                        sim.close();sim=replacement;sim.observe()
                    elif command=='run':sim.start()
                    elif command=='step':
                        if isinstance(sim,CleanupWorld):raise ValueError('Use the cleanup tool sequence')
                        sim.start(step=True)
                    elif command=='pause':
                        if isinstance(sim,CleanupWorld):sim.pause()
                        else:sim.running=False;sim.log('Paused after the current commanded movement. The simulator continues holding.')
                    elif command=='reset':sim.reset()
                    elif command=='relocate':
                        if isinstance(sim,CleanupWorld):sim.reset(shuffle=True)
                        else:sim.relocate(value)
                    elif command=='settings':
                        if isinstance(sim,CleanupWorld):raise ValueError('Alignment settings do not apply to cleanup')
                        if sim.running:raise ValueError('Pause before changing the experiment')
                        for key,v in value.items():setattr(sim,key,v)
                        sim.reset()
                except queue.Empty:pass
                except Exception as exc:
                    sim.running=False;sim.last_error=str(exc)
                    if isinstance(sim,CleanupWorld):sim.error=str(exc)
                    sim.log(str(exc),'error')
                now=time.monotonic();dt=min(.05,now-last)
                if dt>=.002:sim.step(dt);last=now
                if now>=next_frame:
                    sim.observe()
                    with self.lock:
                        self.state=sim.state();self.frames=sim.frames.copy();self.match_frames=getattr(sim,'match_frames',{}).copy()
                        self.references={c:reference_path(getattr(sim,'scene',None),c) for c in ('overhead','wrist')}
                        if isinstance(sim,CleanupWorld):
                            self.wire_snapshot={'binding':sim.binding,'q':sim.data.qpos[sim.qidx].copy(),'frames':{**sim.frames,'overhead':sim.raw_overhead},'resolution':[sim.width,sim.height],'generation':sim.generation}
                            self.state['station_writes']=self.actuators.writes if self.actuators else 0
                        else:self.wire_snapshot={}
                    next_frame=time.monotonic()+.09
                time.sleep(.003)
        except Exception as exc:
            with self.lock:self.state={'stage':'error','last_error':str(exc),'hardware_connected':False}
        finally:
            if sim:sim.close()


class Action(BaseModel):
    command:str
    pixel:list[float]|None=None


class Settings(BaseModel):
    mode:str='vision'
    coarse_error_mm:float=Field(12,ge=0,le=20)
    deadband_ticks:float=Field(3,ge=0,le=8)


def create_app(profile='so101',station_port=8890):
    lab=Lab(profile)
    @asynccontextmanager
    async def lifespan(app):
        lab.loop=asyncio.get_running_loop()
        station=SimStation(lab.station_snapshot,lambda q,g:lab.commands.put(('station_goals',(q,g))))
        port=await station.start(station_port);client=OwnedClient(f'127.0.0.1:{port}')
        try:
            await client.wait_ready(timeout=3);lab.actuators=StationActuators(client,'MUJOCO');lab.thread.start();yield
        finally:
            lab.quit.set()
            if lab.thread.is_alive():await asyncio.to_thread(lab.thread.join,3)
            await client.close();await station.close()
    app=FastAPI(title='Station Robot Lab — MuJoCo',lifespan=lifespan)
    @app.get('/')
    def index():return HTMLResponse((Path(__file__).parent/'static/index.html').read_text())
    @app.post('/api/task/{task_id}')
    def task_route(task_id:str):
        if task_id not in ('alignment','cleanup'):raise HTTPException(400,'Unknown task')
        lab.commands.put(('task',task_id));return {'queued':True}
    @app.post('/api/cleanup/{name}')
    def cleanup_tool(name:str,args:dict):
        try:return lab.invoke(name,args)
        except ValueError as exc:raise HTTPException(400,str(exc)) from exc
    @app.get('/api/state')
    def state():
        with lab.lock:return json.loads(json.dumps(lab.state))
    @app.get('/api/frame/{camera}')
    def frame(camera:str):
        if camera not in ('overview','wrist','overhead'):raise HTTPException(404,'Unknown camera')
        with lab.lock:data=lab.frames.get(camera)
        if data is None:raise HTTPException(503,'MuJoCo renderer is starting')
        return Response(data,media_type='image/jpeg',headers={'Cache-Control':'no-store'})
    @app.get('/api/recording/{camera}')
    def recording(camera:str):
        with lab.lock:path=lab.references.get(camera)
        if path is None:raise HTTPException(404,'No saved recording for this camera setup')
        return Response(path.read_bytes(),media_type='image/jpeg')
    @app.get('/api/match/{camera}')
    def match(camera:str):
        with lab.lock:data=lab.match_frames.get(camera)
        if data is None:raise HTTPException(404,'No recorded pose for this camera setup')
        return Response(data,media_type='image/jpeg')
    @app.post('/api/scene/{scene_id}')
    def scene_route(scene_id:str):
        with lab.lock:robot=lab.state.get('robot',profile)
        try:load_scene(scene_id,robot)
        except ValueError as exc:raise HTTPException(400,str(exc)) from exc
        lab.commands.put(('scene',scene_id));return {'queued':True,'backend':'mujoco'}
    @app.post('/api/action')
    def action(a:Action):
        if a.command not in ('run','step','pause','reset','relocate'):raise HTTPException(400,'Unknown simulation action')
        with lab.lock:width,height=lab.state.get('resolution',[720,480])
        if a.pixel is not None and (len(a.pixel)!=2 or not all(0<=p<limit for p,limit in zip(a.pixel,[width,height]))):raise HTTPException(400,'Invalid pixel')
        lab.commands.put((a.command,a.pixel));return {'queued':True,'backend':'mujoco'}
    @app.post('/api/profile/{profile_id}')
    def profile_route(profile_id:str):
        if profile_id not in ('so101','elrobot'):raise HTTPException(400,'Unknown profile')
        lab.commands.put(('profile',profile_id));return {'queued':True}
    @app.post('/api/settings')
    def settings(s:Settings):
        if s.mode not in ('vision','open_loop'):raise HTTPException(400,'Unknown controller mode')
        lab.commands.put(('settings',s.model_dump()));return {'queued':True}
    @app.get('/api/capabilities')
    def capabilities():return {'backend':'mujoco','hardware_connected':False,'profiles':{'so101':{'arm_joints':5,'gripper_joints':1},'elrobot':{'arm_joints':7,'gripper_joints':1}},'actions':['run','step','pause','reset','relocate'],'scope':'Alignment and image-selected mixed-object cleanup; rigid MuJoCo contact proxies','station_endpoint':'127.0.0.1:'+str(station_port),'station_implementation':'simulator queue-protocol subset; not physical Station'}
    return app


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--robot',choices=['so101','elrobot'],default='so101');parser.add_argument('--port',type=int,default=8785);parser.add_argument('--station-port',type=int,default=8890)
    a=parser.parse_args();uvicorn.run(create_app(a.robot,a.station_port),host='127.0.0.1',port=a.port,log_level='warning')


if __name__=='__main__':main()
