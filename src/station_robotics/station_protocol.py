"""NormaCore wire adapter and an explicit simulator-only subset of Station's queue API.

Uses the repository's generated messages and unmodified upstream Client transport.
The emulator implements setup/ping, latest/follow reads, and ST3215 RAM goal writes.
It is not the full NormaCore Station service. It never forwards to physical Station.
"""
import asyncio
import logging
from pathlib import Path
import struct
import sys
import time
import uuid
import numpy as np

VENDOR=Path(__file__).resolve().parents[2]/'vendor/norma_core'
if str(VENDOR) not in sys.path:sys.path.insert(0,str(VENDOR))
from software.station.shared.station_py.client import Client, send_commands
from target.gen_python.protobuf.normfs import normfs as n
from target.gen_python.protobuf.station import commands,drivers,inference
from target.gen_python.protobuf.drivers.st3215 import st3215 as st
from target.gen_python.protobuf.drivers.usbvideo import usbvideo as usb,frame


class JointBinding:
    """Explicit per-joint calibration; no joint count is assumed by the transport."""
    def __init__(self,names,lower,upper,zero=None,sign=None):
        self.names=list(names);self.lower=np.asarray(lower);self.upper=np.asarray(upper)
        self.ids=list(range(1,len(names)+1));self.zero=np.asarray(zero if zero is not None else [2048]*len(names),float)
        self.sign=np.asarray(sign if sign is not None else np.ones(len(names)),float)
    @classmethod
    def simulated(cls,profile):
        lim=profile.description.find(f"joint[@name='{profile.gripper}']/limit")
        lo=np.r_[profile.lower,float(lim.get('lower'))];hi=np.r_[profile.upper,float(lim.get('upper'))]
        # Simulation-specific ticks. Never transplant these zeros into hardware calibration.
        zero=2048-(lo+hi)/2*4096/(2*np.pi)
        return cls(profile.names+[profile.gripper],lo,hi,zero)
    def encode(self,q):
        q=np.asarray(q,float)
        if q.shape!=self.lower.shape or not np.isfinite(q).all() or np.any(q<self.lower-1e-5) or np.any(q>self.upper+1e-5):raise ValueError('Joint goal outside binding limits')
        return {i:int(round(z+s*a*4096/(2*np.pi))) for i,z,s,a in zip(self.ids,self.zero,self.sign,q)}
    def decode(self,ticks):
        if set(ticks)!=set(self.ids):raise ValueError('Goal must include exactly the configured motor IDs')
        q=np.array([(ticks[i]-z)/s*2*np.pi/4096 for i,z,s in zip(self.ids,self.zero,self.sign)])
        if not np.isfinite(q).all() or np.any(q<self.lower-.0016) or np.any(q>self.upper+.0016):raise ValueError('Station ticks exceed joint limits')
        return np.clip(q,self.lower,self.upper)


class OwnedClient(Client):
    def __init__(self,address):
        self.tasks=set();super().__init__(address,logging.getLogger('robot-lab.station'))
    async def manage_connection(self):
        t=asyncio.current_task();self.tasks.add(t)
        try:await super().manage_connection()
        finally:self.tasks.discard(t)
    async def _read_async(self,*args,**kwargs):
        t=asyncio.current_task();self.tasks.add(t)
        try:await super()._read_async(*args,**kwargs)
        finally:self.tasks.discard(t)
    async def close(self):
        self.close_connection()
        tasks=list(self.tasks)
        for t in tasks:t.cancel()
        await asyncio.gather(*tasks,return_exceptions=True)


class StationActuators:
    def __init__(self,client,bus_serial):self.client=client;self.bus_serial=bus_serial;self.writes=0
    async def goals(self,ticks):
        cmd=st.Command(target_bus_serial=self.bus_serial,sync_write=st.ST3215SyncWriteCommand(address=0x2a,motors=[st.ST3215SyncWriteCommand_MotorWrite(motor_id=i,value=int(v).to_bytes(2,'little')) for i,v in ticks.items()]))
        await send_commands(self.client,[commands.DriverCommand(type=drivers.StationCommandType.STC_ST3215_COMMAND,body=cmd.encode())]);self.writes+=1


class SimStation:
    def __init__(self,snapshot,apply):
        self.snapshot=snapshot;self.apply=apply;self.server=None;self.clients=set();self.read_tasks=set();self.writes=0
    async def start(self,port=8890):
        self.server=await asyncio.start_server(self.connection,'127.0.0.1',port)
        return self.server.sockets[0].getsockname()[1]
    async def close(self):
        self.server.close();await self.server.wait_closed()
        for writer in list(self.clients):writer.close()
        for task in list(self.read_tasks):task.cancel()
        await asyncio.gather(*list(self.read_tasks),return_exceptions=True)
    async def connection(self,reader,writer):
        self.clients.add(writer);lock=asyncio.Lock();tasks=[]
        async def send(msg):
            data=msg.encode()
            async with lock:writer.write(struct.pack('<Q',len(data))+data);await writer.drain()
        async def stream(req):
            ident=req.get_read_id();queue=req.get_queue_id();limit=req.get_limit()
            if queue not in ('st3215/inference','inference-states','usbvideo/overhead','usbvideo/wrist'):
                await send(n.ServerResponse(read=n.ReadResponse(read_id=ident,result=n.ReadResponse_Result.RR_QUEUE_NOT_FOUND)));return
            await send(n.ServerResponse(read=n.ReadResponse(read_id=ident,result=n.ReadResponse_Result.RR_START)))
            count=0
            while not writer.is_closing():
                packet=self.packet(queue)
                if packet is not None:
                    await send(n.ServerResponse(read=n.ReadResponse(read_id=ident,result=n.ReadResponse_Result.RR_ENTRY,id=n.Id(raw=time.time_ns().to_bytes(8,'little')),data=packet)))
                    count+=1
                    if limit and count>=limit:break
                await asyncio.sleep(.05)
            await send(n.ServerResponse(read=n.ReadResponse(read_id=ident,result=n.ReadResponse_Result.RR_END)))
        try:
            while True:
                length=struct.unpack('<Q',await reader.readexactly(8))[0]
                if length>4_000_000:raise ValueError('Oversized message')
                req=n.ClientRequestReader(await reader.readexactly(length))
                if req._setup_buf is not None:await send(n.ServerResponse(setup=n.SetupResponse(version=1,instance_id=uuid.uuid4().hex)))
                elif req._ping_buf is not None:await send(n.ServerResponse(ping=n.PingResponse(local_stamp_ns=time.time_ns())))
                elif req._read_buf is not None:
                    task=asyncio.create_task(stream(req.get_read()));tasks.append(task);self.read_tasks.add(task);task.add_done_callback(self.read_tasks.discard)
                elif req._write_buf is not None:
                    w=req.get_write();ok=True;ids=[]
                    try:
                        if w.get_queue_id()!='commands':raise ValueError('Only Station commands queue accepts writes')
                        for packet in w.get_packets() or []:
                            self.command(bytes(packet));ids.append(n.Id(raw=uuid.uuid4().bytes))
                    except (ValueError,TypeError):ok=False
                    await send(n.ServerResponse(write=n.WriteResponse(write_id=w.get_write_id(),result=n.WriteResponse_Result.WR_DONE if ok else n.WriteResponse_Result.WR_SERVER_ERROR,ids=ids)))
        except (asyncio.IncompleteReadError,ConnectionError,ValueError):pass
        finally:
            for t in tasks:t.cancel()
            await asyncio.gather(*tasks,return_exceptions=True);self.clients.discard(writer);writer.close()
    def command(self,packet):
        snapshot=self.snapshot();binding=snapshot['binding'];goals=[]
        pack=commands.StationCommandsPackReader(memoryview(packet))
        for cmd in pack.get_commands() or []:
            if cmd.get_type()!=drivers.StationCommandType.STC_ST3215_COMMAND:raise ValueError('Unsupported driver')
            c=st.CommandReader(memoryview(bytes(cmd.get_body())))
            if c.get_target_bus_serial()!='MUJOCO':raise ValueError('Simulator bus only')
            sw=c.get_sync_write()
            if sw is None or sw.get_address()!=0x2a:raise ValueError('Simulator implements goal-position RAM writes only')
            motors=sw.get_motors() or [];ticks={m.get_motor_id():int.from_bytes(bytes(m.get_value()),'little') for m in motors}
            if len(ticks)!=len(motors) or any(len(bytes(m.get_value()))!=2 for m in motors):raise ValueError('Malformed goal')
            goals.append(binding.decode(ticks))
        if not goals:raise ValueError('Empty command')
        for q in goals:self.apply(q,snapshot['generation']);self.writes+=1
    def packet(self,queue):
        s=self.snapshot()
        if not s:return None
        now=time.time_ns()
        if queue=='st3215/inference':
            b=s['binding'];q=np.clip(s['q'],b.lower,b.upper);ticks=b.encode(q);motors=[]
            for i in b.ids:
                dump=bytearray(75);dump[5]=i;dump[0x28]=1;dump[0x38:0x3a]=ticks[i].to_bytes(2,'little');dump[0x3e]=120;dump[0x3f]=30
                motors.append(st.InferenceState_MotorState(id=i,state=bytes(dump),system_stamp_ns=now,range_min=0,range_max=4095,range_freezed=True))
            return st.InferenceState(buses=[st.InferenceState_BusState(bus=st.ST3215Bus(serial_number='MUJOCO',port_name='simulation'),motors=motors,system_stamp_ns=now)]).encode()
        if queue=='inference-states':return inference.InferenceRx(entries=[inference.InferenceRx_Entry(queue='usbvideo/'+c,type=drivers.QueueDataType.QDT_USB_VIDEO_FRAMES) for c in ('overhead','wrist')]).encode()
        camera=queue.split('/')[-1];jpg=s['frames'].get(camera)
        if not jpg:return None
        w,h=s['resolution']
        return usb.RxEnvelope(type=usb.RxEnvelopeType.ET_FRAMES,camera=usb.Camera(unique_id='mujoco-'+camera),frames=frame.FramesPack(format=frame.FrameFormat(width=w,height=h,kind=frame.FrameFormatKind.FF_JPEG),frames_data=[jpg])).encode()
