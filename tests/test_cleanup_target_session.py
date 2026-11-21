"""Target tools integrate with session gates without opening a real robot connection."""
import asyncio
import json

import httpx
import pytest

from so101_station.cleanup.geometry import CleanupError
from so101_station.cleanup.service import create_app
from test_cleanup_visual import make


def select(h, obs, action_id='select', target_id='paper'):
    return h.select_target(action_id,obs['observation_id'],target_id,'crumpled_paper','wrist',[280,180,64,64])


def test_target_tools_do_not_move_capture_or_advance_task(tmp_path):
    async def run():
        h=make(tmp_path);obs=await h.observe();revision=h.revision;stage=h.stage
        async def unexpected_capture(*args):
            raise AssertionError('Target tools must reuse supplied observation')
        h.backend.look=unexpected_capture
        selected=select(h,obs)
        assert selected['execution']=='completed'
        assert selected['detail']['position_space']=='image_pixels'
        assert selected['detail']['robot_xyz_m'] is None
        assert selected['detail']['grasp_ready'] is False
        located=h.locate_target('locate',obs['observation_id'],'paper')
        assert located['execution']=='completed'
        assert not h.backend.commands and h.revision==revision and h.stage==stage
        assert select(h,obs)==selected
        with pytest.raises(CleanupError,match='conflict'):
            select(h,obs,target_id='different')
    asyncio.run(run())


def test_target_requests_obey_freshness_and_single_writer_gate(tmp_path):
    async def run():
        h=make(tmp_path);obs=await h.observe()
        h.busy=True
        with pytest.raises(CleanupError,match='busy'):
            select(h,obs)
        h.busy=False;h.observations[obs['observation_id']]['created']-=31
        with pytest.raises(CleanupError,match='stale_observation'):
            select(h,obs)
        assert not h.backend.commands
    asyncio.run(run())


def test_probe_uses_remembered_wrist_target_and_reselection_invalidates_map(tmp_path):
    async def run():
        h=make(tmp_path);obs=await h.observe()
        selected=select(h,obs)
        assert selected['execution']=='completed'
        result=await h.probe_alignment('probe-id',obs['observation_id'],scene_clear=True,target_id='paper')
        assert result['execution']=='completed',result.get('reason')
        data=result['detail']
        assert data['target_binding']=={'target_id':'paper','generation':selected['detail']['generation']}
        evidence=json.loads((h.out/f"probe-{data['probe_id']}.json").read_text())
        assert evidence['target_binding']==data['target_binding']
        latest=result['observation']
        again=select(h,latest,'reselect')
        assert again['detail']['generation']>selected['detail']['generation']
        count=len(h.backend.commands)
        denied=await h.align_target('old-target',latest['observation_id'],data['probe_id'],data['reference_pixel'],True)
        assert denied['execution']=='rejected' and len(h.backend.commands)==count
    asyncio.run(run())


def test_probe_requires_one_target_source_and_current_locate(tmp_path):
    async def run():
        h=make(tmp_path);obs=await h.observe();select(h,obs)
        both=await h.probe_alignment('both',obs['observation_id'],[280,180,64,64],True,target_id='paper')
        assert both['execution']=='rejected' and 'exactly one' in both['reason']
        current=both['observation']
        stale=await h.probe_alignment('unlocated',current['observation_id'],scene_clear=True,target_id='paper')
        assert stale['execution']=='rejected' and not h.backend.commands
        assert h.locate_target('fresh-locate',stale['observation']['observation_id'],'paper')['execution']=='completed'
    asyncio.run(run())


def test_target_tools_http_contract_without_network_or_motor_writes(tmp_path):
    async def run():
        h=make(tmp_path);app=create_app(h)
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app),base_url='http://test') as client:
            obs=(await client.post('/tools/observe',json={'args':{}})).json()['result']
            response=await client.post('/tools/select_target',json={'args':dict(action_id='select',observation_id=obs['observation_id'],target_id='paper',label='crumpled_paper',view='wrist',bbox=[280,180,64,64])})
            result=response.json()
            assert result['ok'] and result['result']['detail']['observation_id']==obs['observation_id']
            rejected=await client.post('/tools/select_target',json={'args':dict(action_id='useful-item',observation_id=obs['observation_id'],target_id='phone',label='phone',view='wrist',bbox=[280,180,64,64])})
            assert not rejected.json()['ok'] and not h.backend.commands
    asyncio.run(run())
