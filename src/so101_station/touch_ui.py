"""Local click-to-annotate touch calibration. Station connection is read-only."""
import argparse
import asyncio
import json
import re
import uuid
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from pydantic import BaseModel
import uvicorn

from .cleanup.backend import StationBackend
from .cleanup.geometry import ArmModel, vector
from .touch import capture


class Pixel(BaseModel):
    pixel: list[float]


class Contact(BaseModel):
    confirmed: bool = False


def create_app(directory, host, bus, camera):
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=True)
    app=FastAPI();lock=asyncio.Lock()

    def path(ident):
        if not re.fullmatch('[a-f0-9]{32}',ident):raise HTTPException(404,'Unknown point')
        p=directory/(ident+'.json')
        if not p.exists():raise HTTPException(404,'Unknown point')
        return p

    def record(p):
        r=json.loads(p.read_text());r['id']=p.stem;r['image_url']='/image/'+p.stem
        return r

    @app.get('/')
    async def index():return FileResponse(Path(__file__).parent/'ui/touch.html')

    @app.get('/points')
    async def points():return [record(p) for p in sorted(directory.glob('*.json'),key=lambda p:p.stat().st_mtime)]

    @app.get('/image/{ident}')
    async def image(ident):
        if ident.startswith('preview-') and re.fullmatch('preview-[a-f0-9]{32}',ident):
            p=directory/'previews'/(ident+'.jpg')
        else:p=path(ident).with_suffix('.jpg')
        if not p.exists():raise HTTPException(404,'Image missing')
        return FileResponse(p,media_type='image/jpeg')

    async def read(touch=False):
        async with lock:
            backend=StationBackend(host,bus,camera,ArmModel(),False)
            try:
                await backend.start()
                if touch:
                    r=await capture(backend,directory,True)
                    return record(Path(r['record']))
                f=await backend.look();ident='preview-'+uuid.uuid4().hex
                (directory/'previews').mkdir(exist_ok=True)
                (directory/'previews'/(ident+'.jpg')).write_bytes(f.jpeg)
                return {'id':ident,'image_url':'/image/'+ident,'image_size':[f.width,f.height],
                        'captured_ns':f.captured_ns,'contact_confirmed':False,'pixel':None}
            except (ValueError,TimeoutError,OSError) as e:raise HTTPException(400,str(e))
            finally:await backend.close()

    @app.post('/photo')
    async def photo():return await read()

    @app.post('/capture')
    async def touch(body:Contact):
        if not body.confirmed:raise HTTPException(400,'Confirm both tips touch bare table first')
        return await read(True)

    @app.post('/points/{ident}/pixel')
    async def mark(ident,body:Pixel):
        p=path(ident);r=record(p)
        try:pixel=vector(body.pixel,2,'midpoint')
        except ValueError as e:raise HTTPException(400,str(e))
        if not (0<=pixel[0]<r['image_size'][0] and 0<=pixel[1]<r['image_size'][1]):
            raise HTTPException(400,'Click inside the photo')
        r.update(pixel=pixel.tolist(),annotation_source='operator_click')
        r.pop('annotation_note',None);p.write_text(json.dumps(r,indent=2)+'\n');return r

    @app.post('/points/{ident}/exclude')
    async def exclude(ident):
        p=path(ident);r=record(p);r['excluded']=not r.get('excluded',False)
        p.write_text(json.dumps(r,indent=2)+'\n');return r

    return app


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='127.0.0.1:8888');p.add_argument('--bus-serial',required=True)
    p.add_argument('--camera-queue',required=True);p.add_argument('--directory',default='logs/touch-calibration')
    p.add_argument('--port',type=int,default=8780);a=p.parse_args()
    uvicorn.run(create_app(a.directory,a.host,a.bus_serial,a.camera_queue),host='127.0.0.1',port=a.port)


if __name__=='__main__':main()
