import asyncio
import time

import pytest

from elrobot.norma import st3215
from so101_station.calibration import load_calibration
from so101_station.station import inspect_state
from so101_station.cleanup.geometry import ArmModel, CleanupError
from so101_station.bench import wrist_test


class BenchFixture:
    bus = 'BENCH-TEST'

    def __init__(self):
        self.model = ArmModel()
        self.data = {}
        self.writes = []
        self.disturb = False
        for j in load_calibration().joints:
            d=bytearray(0x47)
            d[5]=j.motor_id
            for address,value in [(9,j.range_min),(11,j.range_max),
                                  (31,abs(j.homing_offset)|(0x800 if j.homing_offset<0 else 0)),
                                  (0x38,2647 if j.motor_id==5 else j.center_tick),(0x30,1000)]:
                d[address:address+2]=value.to_bytes(2,'little')
            d[0x3E],d[0x3F]=124,30
            self.data[j.motor_id]=d

    @property
    def latest(self):
        motors=[st3215.InferenceState_MotorState(id=i,system_stamp_ns=time.time_ns(),state=bytes(d))
                for i,d in self.data.items()]
        return st3215.InferenceState(buses=[st3215.InferenceState_BusState(
            bus=st3215.ST3215Bus(serial_number=self.bus),motors=motors)]).encode()

    def state(self):
        r=inspect_state(self.model.calibration,self.latest,self.bus)
        if not r['profile_matches_live_state']:
            raise CleanupError('; '.join(r['issues']))
        return {'report':r,'ticks':{j['motor_id']:j['tick'] for j in r['joints']}}

    async def write(self,address,values,stop=None):
        if stop is not None and stop.is_set():
            raise CleanupError('stopped')
        assert set(values)=={5}
        self.writes.append((address,dict(values)))
        d=self.data[5]
        d[address:address+len(values[5])]=values[5]
        if address==0x2A and d[0x28]:
            d[0x38:0x3A]=values[5]
            if self.disturb:
                self.data[1][0x38:0x3A]=(2070).to_bytes(2,'little')


async def instant(_):
    await asyncio.sleep(0)


def test_wrist_round_trip_enables_only_motor_five_and_observes_return():
    b=BenchFixture()
    r=asyncio.run(wrist_test(b,execute=True,expected_tick=2647,pause=instant))
    assert r['status']=='moved_and_returned'
    assert r['reached_tick']==2636 and r['returned_tick']==2647
    assert b.data[5][0x28]==1 and all(b.data[i][0x28]==0 for i in [1,2,3,4,6])
    assert all(a in {0x28,0x29,0x2A,0x2E,0x30} and set(v)=={5} for a,v in b.writes)


def test_preview_and_failed_preconditions_cannot_enable_any_motor():
    b=BenchFixture()
    r=asyncio.run(wrist_test(b))
    assert not r['execute'] and not b.writes
    for kwargs in ({'expected_tick':2400},{'expected_tick':2647,'delta_deg':2}):
        with pytest.raises(CleanupError):
            asyncio.run(wrist_test(b,execute=True,**kwargs))
    assert not b.writes
    b.data[2][0x28]=1
    with pytest.raises(CleanupError,match='unpowered'):
        asyncio.run(wrist_test(b,execute=True,expected_tick=2647))
    assert not b.writes


def test_passive_joint_motion_aborts_and_holds_only_wrist():
    b=BenchFixture(); b.disturb=True
    with pytest.raises(CleanupError,match='another joint moved'):
        asyncio.run(wrist_test(b,execute=True,expected_tick=2647,pause=instant))
    assert b.writes[-1][0]==0x2A
    assert b.writes[-1][1][5]==b.data[5][0x38:0x3A]
    assert all(set(v)=={5} for _,v in b.writes)
