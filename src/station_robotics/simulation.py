"""MuJoCo physics, RGB perception and observable visual-control state machine.

No Station import, network robot connection, or hardware mode exists here.
"""
from collections import deque
import time
import cv2
import mujoco
import numpy as np
from .profiles import load_profile
from .model import build_model
from .servo import VisualServo
from .scenes import load_scene, table_point, available_scenes


class Simulation:
    width,height=720,480
    def __init__(self,profile='so101',render=True,scene='baseline'):
        self.profile=load_profile(profile)
        self.scene_name=scene;self.scene=load_scene(scene,profile)
        self.width,self.height=self.scene['resolution'] if self.scene else (720,480)
        self.model=mujoco.MjModel.from_xml_string(build_model(self.profile,self.scene))
        self.data=mujoco.MjData(self.model)
        self.qidx=np.array([self.model.joint(n).qposadr[0] for n in self.profile.names])
        self.vidx=np.array([self.model.joint(n).dofadr[0] for n in self.profile.names])
        self.aidx=np.array([self.model.actuator(n).id for n in self.profile.names])
        self.paper_q=self.model.joint('paper_free').qposadr[0]
        self.paper_v=self.model.joint('paper_free').dofadr[0]
        self.renderer=mujoco.Renderer(self.model,height=self.height,width=self.width) if render else None
        self.frames={};self.images={};self.events=deque(maxlen=50);self.history=[]
        self.mode='vision';self.deadband_ticks=3.;self.coarse_error_mm=12.
        self.center=self.profile.fk(self.profile.hover)[:3,3].copy()
        self.target_xy=self.center[:2]+[.018,-.012]
        self.reset()
        self.match_frames=self.render_recording() if self.scene and render else {}

    def render_recording(self):
        # Separate data: comparison uses archived encoder pose, never changes the running experiment.
        data=mujoco.MjData(self.model)
        for name,q in self.scene['recorded_joints'].items():
            data.qpos[self.model.joint(name).qposadr[0]]=q
        for i in range(self.model.neq):
            if self.model.eq_type[i]==mujoco.mjtEq.mjEQ_JOINT:
                a,b=self.model.eq_obj1id[i],self.model.eq_obj2id[i]
                data.qpos[self.model.jnt_qposadr[a]]=self.model.eq_data[i,0]+self.model.eq_data[i,1]*data.qpos[self.model.jnt_qposadr[b]]
        paper=table_point(self.scene['overhead'],[522,287],self.scene['resolution'],.022)
        data.qpos[self.paper_q:self.paper_q+7]=[*paper,1,0,0,0]
        mujoco.mj_forward(self.model,data);result={}
        option=mujoco.MjvOption();option.geomgroup[3]=0;option.sitegroup[4]=0
        for camera in ('overhead','wrist'):
            self.renderer.update_scene(data,camera=camera,scene_option=option)
            rgb=self.renderer.render().copy()
            ok,jpg=cv2.imencode('.jpg',cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            result[camera]=jpg.tobytes()
        return result

    def log(self,message,kind='info'):
        self.events.appendleft({'time':round(self.data.time,2),'message':message,'kind':kind})

    def reset(self):
        mujoco.mj_resetData(self.model,self.data)
        self.data.qpos[self.qidx]=self.profile.hover
        self.data.ctrl[self.aidx]=self.profile.hover
        self.data.qpos[self.paper_q:self.paper_q+7]=[*self.target_xy,.022,1,0,0,0]
        # Initialize mimic jaw positions consistently with the open gear pose.
        self.command_q=self.profile.hover.copy();self.servo=VisualServo();self.stage='ready'
        self.running=False;self.steps_left=None;self.anchor=self.center.copy();self.anchor_q=self.command_q.copy()
        self.sample_errors=[];self.history=[];self.events.clear();self.goal=None;self.target=None
        self.last_error=None;self.error_px=None;self.quiet_since=None;self.move_started=0.;self.last_observed=-1.
        mujoco.mj_forward(self.model,self.data)
        for _ in range(300):mujoco.mj_step(self.model,self.data)
        self.log('Ready. Pick a target position, then run or step through the correction loop.')

    def close(self):
        if self.renderer:self.renderer.close()

    def project(self,point,camera):
        cid=self.model.camera(camera).id;R=self.data.cam_xmat[cid].reshape(3,3)
        p=R.T@(np.asarray(point)-self.data.cam_xpos[cid]);f=.5*self.height/np.tan(np.deg2rad(self.model.cam_fovy[cid]/2))
        if p[2]>=-.001:return None
        return np.array([self.width/2+f*p[0]/-p[2],self.height/2-f*p[1]/-p[2]])

    def unproject(self,pixel,camera,z=.022):
        cid=self.model.camera(camera).id;R=self.data.cam_xmat[cid].reshape(3,3);o=self.data.cam_xpos[cid]
        f=.5*self.height/np.tan(np.deg2rad(self.model.cam_fovy[cid]/2))
        ray=R@np.array([(pixel[0]-self.width/2)/f,-(pixel[1]-self.height/2)/f,-1])
        if abs(ray[2])<1e-6:raise ValueError('Camera ray misses the table')
        return o+ray*((z-o[2])/ray[2])

    @staticmethod
    def detect(rgb):
        # Deliberately simple color perception of the orange rigid trash proxy.
        # No MuJoCo object IDs or positions enter this detector.
        x=rgb.astype(float);r,g,b=x[:,:,0],x[:,:,1],x[:,:,2]
        mask=((r>65)&(r>1.85*g)&(g>1.25*b)).astype(np.uint8)*255
        n,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
        candidates=[i for i in range(1,n) if stats[i,cv2.CC_STAT_AREA]>15]
        if not candidates:return None
        i=max(candidates,key=lambda k:stats[k,cv2.CC_STAT_AREA])
        return centers[i],stats[i,:4].tolist()

    def reference(self):
        tcp=self.data.site('tool').xpos.copy();axis=self.data.site('tool').xmat.reshape(3,3)[:,2]
        if axis[2]>-.5:raise ValueError('Tool is not pointing toward the table')
        footprint=tcp+axis*((.022-tcp[2])/axis[2])
        return self.project(footprint,'wrist')

    def observe(self):
        if not self.renderer:raise RuntimeError('RGB rendering is required for visual feedback')
        for camera in ('overview','wrist','overhead'):
            option=mujoco.MjvOption();option.geomgroup[3]=0;option.sitegroup[4]=0
            self.renderer.update_scene(self.data,camera=camera,scene_option=option)
            self.images[camera]=self.renderer.render().copy()
        detection=self.detect(self.images['wrist']);self.goal=self.reference()
        self.target=detection[0] if detection else None
        self.error_px=float(np.linalg.norm(self.target-self.goal)) if self.target is not None and self.goal is not None else None
        for camera,rgb in self.images.items():
            im=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            if camera=='wrist' and self.goal is not None:
                g=tuple(np.round(self.goal).astype(int));cv2.drawMarker(im,g,(186,239,143),cv2.MARKER_CROSS,24,2)
                if detection:
                    p=tuple(np.round(self.target).astype(int));x,y,w,h=detection[1]
                    cv2.rectangle(im,(x,y),(x+w,y+h),(90,160,240),2)
                    cv2.arrowedLine(im,p,g,(186,239,143),2,tipLength=.17)
                    cv2.circle(im,p,5,(90,160,240),-1)
            elif camera=='overhead':
                found=self.detect(rgb)
                if found:
                    x,y,w,h=found[1];cv2.rectangle(im,(x,y),(x+w,y+h),(90,160,240),2)
            ok,jpg=cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,85]);self.frames[camera]=jpg.tobytes()
        self.last_observed=self.data.time
        if self.running:self.advance()

    def command(self,offset):
        xyz=self.anchor+np.r_[offset,0]/1000
        if np.linalg.norm(xyz[:2]-self.anchor[:2])>.05:raise ValueError('Local 50 mm neighborhood exhausted')
        q=self.profile.solve(xyz,self.anchor_q)
        # A single fixed command origin is retained across all corrections.
        self.command_q=q
        old=self.data.ctrl[self.aidx];threshold=self.deadband_ticks*2*np.pi/4096
        self.data.ctrl[self.aidx]=np.where(abs(q-old)>=threshold,q,old)
        self.move_started=self.data.time;self.quiet_since=None

    def settled(self):
        if np.max(abs(self.data.qvel[self.vidx]))>.035:self.quiet_since=None
        elif self.quiet_since is None:self.quiet_since=self.data.time
        if self.data.time-self.move_started>3.:raise ValueError('Motion did not become stationary')
        return self.data.time-self.move_started>.4 and self.quiet_since is not None and self.data.time-self.quiet_since>.12

    def sample(self):
        if self.target is None or self.goal is None:raise ValueError('Target lost in wrist view; pause and reacquire')
        error=self.target-self.goal
        self.history.append({'time':round(self.data.time,3),'error_px':float(np.linalg.norm(error)),
            'pixel':self.target.tolist(),'goal':self.goal.tolist(),'command_offset_mm':self.servo.offset.tolist(),
            'joint_residual_ticks':(abs(self.data.qpos[self.qidx]-self.command_q)*4096/(2*np.pi)).tolist()})
        return error

    def advance(self):
        try:
            if not self.settled():return
            error=self.sample()
            if self.stage=='coarse':
                if self.mode=='open_loop':
                    self.stage='complete';self.running=False;self.log(f'Single move finished. Remaining image error: {np.linalg.norm(error):.1f} px.');return
                self.anchor_q=self.data.ctrl[self.aidx].copy();self.anchor=self.profile.fk(self.anchor_q)[:3,3]
                self.sample_errors=[error];self.servo.offset=np.array([4.,0.]);self.command(self.servo.offset)
                self.stage='probe_x';self.log('Observe X response: one 4 mm command, then a fresh image.')
            elif self.stage=='probe_x':
                self.sample_errors.append(error);self.servo.offset=np.array([4.,4.]);self.command(self.servo.offset)
                self.stage='probe_y';self.log('Observe Y response from the same command origin.')
            elif self.stage=='probe_y':
                self.sample_errors.append(error);self.servo.fit(self.sample_errors)
                self.stage='correct';self.log('Camera response learned. Corrections now use measured pixel error.')
            elif self.stage=='correct':
                delta=self.servo.correction(error)
                if delta is None:
                    if self.servo.stable>=2:
                        self.stage='complete';self.running=False;self.log(f'Aligned in two fresh frames: {np.linalg.norm(error):.1f} px. No grasp has been attempted.','success')
                    else:self.log('Inside the image tolerance. Checking a second fresh frame.')
                else:
                    self.command(self.servo.offset)
                    self.log(f'Correction {self.servo.moves}: X {delta[0]:+.1f}, Y {delta[1]:+.1f} mm; error {np.linalg.norm(error):.1f} px.')
            if self.steps_left is not None:
                self.steps_left-=1
                if self.steps_left<=0:self.running=False
        except Exception as exc:
            self.running=False;self.stage='blocked';self.last_error=str(exc);self.log(str(exc),'error')

    def start(self,step=False):
        if self.stage in ('complete','blocked'):self.reset();self.observe()
        if self.stage=='ready':
            found=self.detect(self.images['overhead'])
            if found is None:raise ValueError('No trash proxy detected in overhead view')
            target=self.unproject(found[0],'overhead')
            bias=np.array([1.,-.65])*self.coarse_error_mm/1000
            xyz=np.r_[target[:2]+bias,self.center[2]]
            self.anchor=self.center.copy();self.anchor_q=self.data.ctrl[self.aidx].copy()
            self.command((xyz[:2]-self.anchor[:2])*1000)
            self.stage='coarse';self.log(f'Overhead approach with {self.coarse_error_mm:.0f} mm injected calibration bias.')
        if not self.running:
            # Pausing the lesson must not consume the next observation deadline.
            # Preserve the existing servo goals and fixed command origin.
            self.move_started=self.data.time;self.quiet_since=None
        self.running=True;self.steps_left=1 if step else None

    def relocate(self,pixel=None):
        if self.running:raise ValueError('Pause or reset before moving the target')
        xy=self.unproject(pixel,'overhead')[:2] if pixel is not None else self.center[:2]+np.random.default_rng().uniform(-.025,.025,2)
        if np.linalg.norm(xy-self.center[:2])>.045:raise ValueError('Choose a point inside the local 45 mm test area')
        self.target_xy=xy;self.reset()

    def step(self,seconds=.02):
        for _ in range(max(1,round(seconds/self.model.opt.timestep))):mujoco.mj_step(self.model,self.data)
        if not np.isfinite(self.data.qpos).all():raise RuntimeError('Simulation diverged')

    def state(self):
        return {'backend':'mujoco','mujoco_version':mujoco.__version__,'robot':self.profile.key,'robot_label':self.profile.label,
            'arm_joints':len(self.profile.names),'stage':self.stage,'running':self.running,'mode':self.mode,
            'time':round(self.data.time,2),'error_px':self.error_px,'moves':self.servo.moves,'history':self.history,
            'target_pixel':None if self.target is None else self.target.tolist(),'goal_pixel':None if self.goal is None else self.goal.tolist(),
            'matrix':None if self.servo.matrix is None else self.servo.matrix.tolist(),'events':list(self.events),
            'coarse_error_mm':self.coarse_error_mm,'deadband_ticks':self.deadband_ticks,'source':self.profile.source,
            'joints':[{'name':n,'measured':float(self.data.qpos[i]),'commanded':float(q)} for n,i,q in zip(self.profile.names,self.qidx,self.command_q)],
            'hardware_connected':False,'last_error':self.last_error,
            'resolution':[self.width,self.height],'scene':self.scene_name,'available_scenes':available_scenes(self.profile.key),
            'camera_setup':{'status':'synthetic' if not self.scene else self.scene['status'],
                'label':'Synthetic baseline' if not self.scene else self.scene['label'],
                'fit':None if not self.scene else self.scene['fit']}}
