"""Image-first Codex tools for the local Station/MuJoCo cleanup lab."""
import argparse
import base64
import json
from pathlib import Path
import time
import httpx

URL='http://127.0.0.1:8785'


def request(name,args=None,url=URL):
    with httpx.Client(timeout=15) as client:
        response=client.post(url+'/api/cleanup/'+name,json=args or {});response.raise_for_status();return response.json()


def prepare(robot,url=URL):
    with httpx.Client(timeout=15) as c:
        c.post(url+'/api/profile/'+robot).raise_for_status();c.post(url+'/api/task/cleanup').raise_for_status()
        end=time.monotonic()+15
        while time.monotonic()<end:
            s=c.get(url+'/api/state').json()
            if s.get('task')=='cleanup' and s.get('robot')==robot:return s
            time.sleep(.1)
        raise TimeoutError('Cleanup scene did not load')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--url',default=URL)
    sub=p.add_subparsers(dest='command',required=True)
    a=sub.add_parser('prepare');a.add_argument('--robot',choices=['so101','elrobot'],default='so101')
    for name in ['observe','verify','verify_grasp']:
        a=sub.add_parser(name);a.add_argument('--out',type=Path,default=Path('logs/robot-lab/cleanup/agent'))
    a=sub.add_parser('classify');a.add_argument('--observation-id',required=True);a.add_argument('--file',type=Path,required=True)
    for name in ['execute','status','stop']:sub.add_parser(name)
    a=sub.add_parser('assess');a.add_argument('--observation-id',required=True);a.add_argument('--trash-in-bin',action='store_true');a.add_argument('--protected-untouched',action='store_true')
    a=sub.add_parser('assess_grasp');a.add_argument('--observation-id',required=True);a.add_argument('--held',action='store_true')
    args=p.parse_args()
    if args.command=='prepare':result={'ok':True,'result':prepare(args.robot,args.url)}
    else:
        body={}
        if args.command=='classify':body={'observation_id':args.observation_id,'objects':json.loads(args.file.read_text())}
        elif args.command=='assess_grasp':body={'observation_id':args.observation_id,'held':args.held}
        elif args.command=='assess':body={'observation_id':args.observation_id,'trash_in_bin':args.trash_in_bin,'protected_untouched':args.protected_untouched}
        result=request(args.command,body,args.url)
    extra=result.pop('images_b64',[]);encoded=result.pop('image_b64',None)
    if encoded:
        args.out.mkdir(parents=True,exist_ok=True);ident=result['result']['observation_id'];path=args.out/(ident+'.jpg');path.write_bytes(base64.b64decode(encoded));result['result']['image_path']=str(path.resolve())
        result['result']['image_paths']=[str(path.resolve())]
        for index,im in enumerate(extra):
            path=args.out/(ident+f'-{index+1}.jpg');path.write_bytes(base64.b64decode(im));result['result']['image_paths'].append(str(path.resolve()))
        (args.out/(ident+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not result['ok']:raise SystemExit(1)


if __name__=='__main__':main()
