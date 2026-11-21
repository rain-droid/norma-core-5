"""Reusable CLI for fresh wrist probing and local image alignment; no raw motor access."""
import argparse
import json
from pathlib import Path
import uuid

import httpx


def compact(value):
    if isinstance(value, dict):
        return {k: compact(v) for k,v in value.items() if k != 'images'}
    if isinstance(value, list):
        return [compact(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8772')
    parser.add_argument('--out', type=Path, default=Path('logs/visual-client'))
    sub = parser.add_subparsers(dest='command',required=True)
    sub.add_parser('observe')
    for name in ('prepare','park','probe','align','select','locate'):
        child = sub.add_parser(name)
        child.add_argument('--observation-id',required=True,help='ID of the fresh images you inspected')
        child.add_argument('--request-id',help='Reuse the same ID and arguments after a network timeout')
        if name in ('prepare','park','probe','align'):
            child.add_argument('--scene-clear',action='store_true',required=True,help='Confirm inspected horizontal corridor is clear')
        if name in ('prepare','park'):
            child.add_argument('--empty-gripper-confirmed',action='store_true',required=True)
            child.add_argument('--operator-confirmed',action='store_true',help='Actual user direction to restart after a fault')
        elif name == 'probe':
            child.add_argument('--axis',choices=['x','y','xy'],default='xy')
            target = child.add_mutually_exclusive_group(required=True)
            target.add_argument('--bbox',nargs=4,type=int,metavar=('X','Y','WIDTH','HEIGHT'))
            target.add_argument('--target-id')
            child.add_argument('--tracker',choices=['patch','features'],default='patch')
        elif name in ('select','locate'):
            child.add_argument('--target-id',required=True)
            if name == 'select':
                child.add_argument('--label',choices=['crumpled_paper','snack_wrapper'],required=True)
                child.add_argument('--view',choices=['overhead','wrist'],required=True)
                child.add_argument('--bbox',nargs=4,type=int,required=True,metavar=('X','Y','WIDTH','HEIGHT'))
                child.add_argument('--tracker',choices=['patch','features'],default='patch')
        else:
            child.add_argument('--probe-id',required=True)
            child.add_argument('--goal-pixel',nargs=2,type=float,required=True,metavar=('U','V'))
    a = parser.parse_args()
    tool = {'observe':'observe','prepare':'prepare_alignment','park':'park','probe':'probe_alignment','align':'align_target','select':'select_target','locate':'locate_target'}[a.command]
    args = {}
    request_id = getattr(a,'request_id',None) or uuid.uuid4().hex
    if a.command != 'observe':
        args.update(action_id=request_id, observation_id=a.observation_id)
        if a.command in ('prepare','park','probe','align'):args.update(scene_clear=a.scene_clear)
        if a.command in ('prepare','park'):args.update(empty_gripper_confirmed=a.empty_gripper_confirmed,operator_confirmed=a.operator_confirmed)
        elif a.command == 'probe':args.update(bbox=a.bbox,tracker=a.tracker,target_id=a.target_id,axis=a.axis)
        elif a.command == 'select':args.update(target_id=a.target_id,label=a.label,view=a.view,bbox=a.bbox,tracker=a.tracker)
        elif a.command == 'locate':args.update(target_id=a.target_id)
        else:args.update(probe_id=a.probe_id,goal_pixel=a.goal_pixel)
    a.out.mkdir(parents=True,exist_ok=True)
    request_path = a.out / f'{request_id}-request.json'
    payload = {'tool':tool,'args':args}
    if request_path.exists() and json.loads(request_path.read_text()) != payload:
        parser.error('request ID already belongs to different arguments')
    request_path.write_text(json.dumps(payload,indent=2))
    print(json.dumps({'request_id':request_id,'request_file':str(request_path.resolve())}),flush=True)
    with httpx.Client(timeout=190) as client:
        response = client.post(f'{a.url}/tools/{tool}',json={'args':args})
        response.raise_for_status()
        result = compact(response.json())
    (a.out/f'{request_id}-result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return 0 if result.get('ok') else 1


if __name__ == '__main__':
    raise SystemExit(main())
