"""Mixed-object MuJoCo world. Agent observations contain images, never object labels."""
import time
import uuid
from collections import deque
import cv2
import mujoco
import numpy as np
from scipy.optimize import least_squares
from .model import build_model
from .profiles import load_profile
from .simulation import Simulation
from .station_protocol import JointBinding


class CleanupWorld:
    width,height=853,480
    project=Simulation.project
    unproject=Simulation.unproject
    def __init__(self,robot='so101',render=True):
        self.profile=load_profile(robot);self.model=mujoco.MjModel.from_xml_string(build_model(self.profile,task='cleanup'));self.data=mujoco.MjData(self.model)
        self.binding=JointBinding.simulated(self.profile);self.clearance=.075+self.profile.fingertip_backoff
        self.qidx=np.array([self.model.joint(n).qposadr[0] for n in self.binding.names]);self.aidx=np.array([self.model.actuator(n).id for n in self.binding.names])
        self.renderer=mujoco.Renderer(self.model,height=self.height,width=self.width) if render else None
        self.frames={};self.images={};self.events=deque(maxlen=35);self.generation=uuid.uuid4().hex;self.revision=0
        self.annotations=[];self.plan=None;self.stage='observe';self.error=None;self.active=None;self.phase=None;self.outcomes=[];self.running=False
        self.observation=None;self.writer=None;self.segment=None;self.last_command=0.;self.sequence=[];self.goal=None;self.steps=0
        self.initial_objects={};self.reset()
    def reset(self,shuffle=False):
        mujoco.mj_resetData(self.model,self.data)
        q=np.r_[self.profile.rest,0.]
        self.data.qpos[self.qidx]=q;self.data.ctrl[self.aidx]=q
        # Mirror joints at the initial open position. Normal grasp motion is physical contact only.
        self.generation=uuid.uuid4().hex;self.revision+=1;self.annotations=[];self.plan=None;self.segment=None;self.sequence=[]
        self.stage='observe';self.phase=None;self.running=False;self.outcomes=[];self.active=None;self.error=None;self.observation=None;self.events.clear()
        if shuffle:
            rng=np.random.default_rng()
            for name in ('item_a','item_b','item_c','item_d'):
                i=self.model.joint(name+'_free').qposadr[0];self.data.qpos[i:i+2]+=rng.uniform(-.012,.012,2)
        for _ in range(500):mujoco.mj_step(self.model,self.data)
        self.initial_objects={n:self.data.body(n).xpos.copy() for n in ('item_a','item_b','item_c','item_d')}
        self.log('Observe the table. Codex must label trash, protected objects and the bin before any cleanup motion.')
    def close(self):
        if self.renderer:self.renderer.close()
    def log(self,text,kind='info'):self.events.appendleft({'time':round(self.data.time,2),'message':text,'kind':kind})
    def step(self,seconds):
        for _ in range(max(1,round(seconds/.002))):mujoco.mj_step(self.model,self.data)
        if self.running:self.advance()
    def apply_goals(self,q,generation):
        if generation!=self.generation:raise ValueError('Stale Station command for previous scene')
        self.data.ctrl[self.aidx]=q
    def solve(self,xyz,seed,yaw=None):
        target=np.asarray(xyz,float);seed=np.asarray(seed)
        def residual(q):
            T=self.profile.fk(q);r=np.r_[T[:3,3]-target,.08*(T[:3,2]-[0,0,-1]),.00005*(q-seed)]
            if yaw is not None:r=np.r_[r,.04*(T[:3,0]-[np.cos(yaw),np.sin(yaw),0])]
            return r
        for initial in [seed,self.profile.hover,np.zeros(len(seed))]:
            result=least_squares(residual,np.clip(initial,self.profile.lower+1e-6,self.profile.upper-1e-6),bounds=(self.profile.lower,self.profile.upper),max_nfev=120)
            T=self.profile.fk(result.x)
            if np.linalg.norm(T[:3,3]-target)<=.004 and T[2,2]<=-.97:return result.x
        raise ValueError('Target pose is not reachable by this robot profile')
    def observe(self):
        if not self.renderer:return
        option=mujoco.MjvOption();option.geomgroup[3]=0;option.sitegroup[4]=0
        for camera in ('overview','overhead','wrist'):
            self.renderer.update_scene(self.data,camera=camera,scene_option=option);rgb=self.renderer.render().copy();self.images[camera]=rgb
            im=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            if camera=='overhead':
                for a in self.annotations:
                    x,y,w,h=np.round(a['bbox']).astype(int);color=(140,220,170) if a['decision']=='trash' else (170,170,170)
                    cv2.rectangle(im,(x,y),(x+w,y+h),color,1);cv2.putText(im,a['label']+' / '+a['decision'],(x,max(15,y-5)),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
            self.frames[camera]=cv2.imencode('.jpg',im)[1].tobytes()
        self.raw_overhead=cv2.imencode('.jpg',cv2.cvtColor(self.images['overhead'],cv2.COLOR_RGB2BGR))[1].tobytes()
    def capture(self):
        if self.running:raise ValueError('Pause motion before taking a planning observation')
        self.observe();ident=uuid.uuid4().hex
        self.observation={'id':ident,'revision':self.revision,'created':time.monotonic(),'rgb':self.images['overhead'].copy()}
        return {'observation_id':ident,'image_size':[self.width,self.height],'camera':'overhead','revision':self.revision,'instruction':'Identify every object from the image. Mark crumpled paper and empty snack packaging as trash only when confident; mouse, pen and all uncertain objects stay. Identify the bin separately.'}
    def submit(self,observation_id,objects):
        from .cleanup_policy import validate_annotations
        if self.running:raise ValueError('Cleanup is already moving')
        obs=self.observation
        if not obs or obs['id']!=observation_id or obs['revision']!=self.revision or time.monotonic()-obs['created']>180:raise ValueError('Observation expired; observe again')
        annotations=validate_annotations(objects,self.width,self.height)
        self.annotations=annotations;self.stage='review';self.plan={'observation_id':observation_id,'created':time.monotonic()}
        self.log(f"Codex selected {sum(a['decision']=='trash' for a in annotations)} trash targets; all other objects are protected.")
        return {'accepted':True,'objects':annotations,'next':'execute_cleanup'}
    def start(self):
        if self.running:raise ValueError('Already running')
        if not self.plan or self.stage!='review':raise ValueError('Observe and classify the current scene first')
        if time.monotonic()-self.plan['created']>120:raise ValueError('Selection expired; observe again')
        if self.writer is None:raise ValueError('Station transport is not connected')
        self.pending=[a for a in self.annotations if a['decision']=='trash'];self.bin=next(a for a in self.annotations if a['decision']=='bin')
        self.running=True;self.error=None;self.stage='planning';self.phase='next';self.advance()
    def pause(self):
        self.running=False;self.stage='paused';self.segment=None;self.sequence=[]
        if self.writer:self.writer(self.binding.encode(np.clip(self.data.qpos[self.qidx],self.binding.lower,self.binding.upper)))
        self.log('Paused; commanded the current measured pose. Re-observe before continuing.')
        self.plan=None
    def _move(self,xyz,grip,duration=.7,yaw=None):
        start=self.data.ctrl[self.aidx].copy();seed=start[:-1]
        q=self.solve(xyz,seed,yaw)
        self.segment={'from':start,'to':np.r_[q,grip],'start':self.data.time,'duration':duration};self.last_command=-1
    def _joint_move(self,q,duration=1.0):
        self.segment={'from':self.data.ctrl[self.aidx].copy(),'to':np.asarray(q),'start':self.data.time,'duration':duration};self.last_command=-1
    def _tick_segment(self):
        seg=self.segment;t=(self.data.time-seg['start'])/seg['duration']
        if self.data.time-self.last_command>=.04:
            u=np.clip(t,0,1);u=u*u*(3-2*u);q=seg['from']+u*(seg['to']-seg['from']);self.writer(self.binding.encode(q));self.last_command=self.data.time
        if t>=1.3:
            if self.phase=='close':
                speed=abs(self.data.qvel[self.model.joint(self.profile.gripper).dofadr[0]])
                if speed>.08 and self.data.time-seg['start']<4.:return False
                if speed>.08:raise ValueError('Gripper is still moving; no stable grasp evidence')
            self.segment=None;return True
        return False
    def _fresh_target(self,a):
        # RGB template confirmation. No scene body IDs or exact object poses are used.
        x,y,w,h=np.round(a['bbox']).astype(int);original=self.observation['rgb'][y:y+h,x:x+w]
        current=self.images['overhead'][max(0,y-12):min(self.height,y+h+12),max(0,x-12):min(self.width,x+w+12)]
        scores=cv2.matchTemplate(current,original,cv2.TM_CCOEFF_NORMED);_,score,_,loc=cv2.minMaxLoc(scores)
        if score<.7:raise ValueError('Selected object changed or became occluded; reacquire instead of guessing')
        left,top=max(0,x-12)+loc[0],max(0,y-12)+loc[1]
        # Refine a human/model box with foreground contrast, using RGB only.
        patch=self.images['overhead'][top:top+h,left:left+w]
        surround=self.images['overhead'][max(0,top-5):min(self.height,top+h+5),max(0,left-5):min(self.width,left+w+5)]
        border=np.concatenate([surround[:3].reshape(-1,3),surround[-3:].reshape(-1,3),surround[:,:3].reshape(-1,3),surround[:,-3:].reshape(-1,3)])
        bg=np.median(border,axis=0);mask=(np.linalg.norm(patch.astype(float)-bg,axis=2)>55).astype(np.uint8)
        n,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
        valid=[i for i in range(1,n) if stats[i,cv2.CC_STAT_AREA]>=12]
        offset=centers[max(valid,key=lambda i:stats[i,cv2.CC_STAT_AREA])] if valid else np.array([w/2,h/2])
        return np.array([left,top])+offset
    def advance(self):
        try:
            if self.segment and not self._tick_segment():return
            if self.sequence:
                command=self.sequence.pop(0);self.phase=command['phase'];self.stage=self.phase;self.log(command.get('message',self.phase))
                if 'joints' in command:self._joint_move(command['joints'],command.get('duration',1.))
                else:self._move(command['xyz'],command['grip'],command.get('duration',.7),command.get('yaw'))
                return
            if self.phase=='next':
                if not self.pending:
                    self.sequence=[{'phase':'rest','joints':np.r_[self.profile.rest,0.].tolist(),'duration':1.6}];self.phase='finish';return
                self.active=self.pending.pop(0);a=self.active;pixel=self._fresh_target(a)
                height=a['height_mm']/1000;target=self.unproject(pixel,'overhead',height)
                self.pick_xy=target[:2];self.pick_height=height;self.target_rgb=a
                drop=self.unproject([self.bin['bbox'][0]+self.bin['bbox'][2]/2,self.bin['bbox'][1]+self.bin['bbox'][3]/2],'overhead',0.)
                self.drop_xy=drop[:2]
                # Keep the swept vertical pickup region away from every protected observation.
                for other in self.annotations:
                    if other['decision'] not in ('keep','unknown'):continue
                    xy=self.unproject([other['bbox'][0]+other['bbox'][2]/2,other['bbox'][1]+other['bbox'][3]/2],'overhead',other['height_mm']/1000)[:2]
                    if np.linalg.norm(xy-self.pick_xy)<.07:raise ValueError('Trash is too close to a protected object; reposition or re-observe')
                z=max(.002,height-.008)+self.profile.fingertip_backoff
                # Preflight all requested Cartesian endpoints before the first action.
                yaw=np.deg2rad(a.get('jaw_yaw_deg',0)) if a.get('jaw_yaw_deg') is not None else None
                for point in [np.r_[self.pick_xy,self.clearance],np.r_[self.pick_xy,z],np.r_[self.drop_xy,self.clearance]]:self.solve(point,self.profile.hover,yaw)
                self.sequence=[{'phase':'approach','xyz':np.r_[self.pick_xy,self.clearance].tolist(),'grip':0.,'duration':1.,'yaw':yaw},
                    {'phase':'descend','xyz':np.r_[self.pick_xy,z].tolist(),'grip':0.,'duration':.85,'yaw':yaw},
                    {'phase':'close','xyz':np.r_[self.pick_xy,z].tolist(),'grip':2.15,'duration':1.65,'yaw':yaw},
                    {'phase':'lift','xyz':np.r_[self.pick_xy,self.clearance].tolist(),'grip':2.15,'duration':.85,'yaw':yaw}]
                self.phase='picking';return
            if self.phase=='lift':
                grip=float(self.data.qpos[self.qidx[-1]])
                if grip>2.08:raise ValueError('Empty closure after lift; reacquire the object before retrying')
                self.running=False;self.stage='verify_grasp';self.grasp_observation=None
                self.log('Lift complete. Inspect wrist and overhead images to confirm the object is held before carrying.');return
            if self.phase=='retreat':
                self.outcomes.append({'label':self.active['label'],'status':'released_pending_visual_verification'})
                self.active=None;self.phase='next';self.stage='planning';return
            if self.phase=='rest':
                self.running=False;self.stage='verify';self.log('Returned to folded rest. Codex must inspect the final image to verify both trash items are in the bin and protected objects remain.');return
        except Exception as exc:
            self.error=str(exc);self.running=False;self.stage='blocked';self.segment=None;self.sequence=[];self.log(self.error)
    def inspect_grasp(self):
        if self.stage!='verify_grasp':raise ValueError('No grasp is awaiting verification')
        self.observe();self.grasp_observation=uuid.uuid4().hex;self.grasp_observed_at=time.monotonic()
        return {'observation_id':self.grasp_observation,'label':self.active['label'],'jaw_angle_rad':float(self.data.qpos[self.qidx[-1]]),'instruction':'Inspect both images. Is the intended trash visibly held above the table? Jaw width alone is not proof.'}

    def assess_grasp(self,observation_id,held):
        if self.stage!='verify_grasp' or not self.grasp_observation or observation_id!=self.grasp_observation:raise ValueError('Inspect the current grasp first')
        if time.monotonic()-self.grasp_observed_at>30:raise ValueError('Grasp image expired; inspect again')
        if held is True and float(self.data.qpos[self.qidx[-1]])>2.08:raise ValueError('Jaws have since closed empty; inspect the grasp again')
        self.grasp_observation=None
        if held is not True:
            self.stage='blocked';self.error='Codex did not confirm a held object; re-observe and retry.';self.log(self.error);return
        self.sequence=[{'phase':'carry','xyz':np.r_[self.drop_xy,self.clearance].tolist(),'grip':2.15,'duration':1.1},
                       {'phase':'release','xyz':np.r_[self.drop_xy,self.clearance].tolist(),'grip':0.,'duration':1.2},
                       {'phase':'retreat','xyz':np.r_[self.drop_xy,self.clearance].tolist(),'grip':0.,'duration':.6}]
        self.running=True;self.stage='carry';self.phase='carrying';self.log('Codex confirmed the held trash from fresh images; carrying to the bin.')

    def state(self):
        return {'backend':'mujoco','task':'cleanup','robot':self.profile.key,'robot_label':self.profile.label,'stage':self.stage,'running':self.running,'resolution':[self.width,self.height],
                'objects':self.annotations,'outcomes':self.outcomes,'events':list(self.events),'last_error':self.error,'hardware_connected':False,'station_transport':True,
                'observation_id':self.observation['id'] if self.observation else None,'generation':self.generation,'revision':self.revision}
    def evaluate(self):
        """Test-only ground truth. Never included in agent observations or motion planning."""
        center=self.data.body('bin').xpos
        positions={n:self.data.body(n).xpos.copy() for n in self.initial_objects}
        return {'trash_in_bin':{n:bool(np.max(abs(positions[n][:2]-center[:2]))<.045 and .005<positions[n][2]<.07) for n in ('item_a','item_b')},
                'protected_displacement_m':{n:float(np.linalg.norm(positions[n]-self.initial_objects[n])) for n in ('item_c','item_d')},'positions':{k:v.tolist() for k,v in positions.items()}}
