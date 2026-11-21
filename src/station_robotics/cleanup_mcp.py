"""Native Codex vision loop: tool images are recognized by Codex, without a second model API."""
import asyncio
import json
import os
import httpx
import mcp.types as types
from mcp.server import Server
from mcp.server.stdio import stdio_server

URL=os.environ.get('STATION_ROBOT_LAB_URL','http://127.0.0.1:8785')
INSTRUCTIONS='''You operate the local MuJoCo cleanup simulator through a Station-compatible queue endpoint. This is not physical hardware. Start with cleanup_observe and inspect the returned RGB image. Identify every object visually; do not infer labels from colors, IDs, order or previous scenes. Only clearly crumpled paper and clearly empty snack wrappers are eligible trash. A computer mouse, pen, and all uncertain items must remain. Mark the open bin separately. Submit pixel boxes and confidence through cleanup_classify. Only then execute. At verify_grasp, inspect cleanup_verify_grasp images and use cleanup_assess_grasp before carrying; partial jaw closure is not proof. Observe status until it stops, inspect cleanup_verify images and assess whether both trash objects are in the bin and belongings remain. Never report success from command completion alone. A blocked grasp needs a fresh observation and a corrected selection; do not blindly repeat. Physics metadata and evaluation truth are not available to this agent.'''
OBJECT={'type':'object','required':['label','decision','confidence','bbox','height_mm'],'properties':{'label':{'type':'string'},'decision':{'type':'string','enum':['trash','keep','unknown','bin']},'confidence':{'type':'number','minimum':0,'maximum':1},'bbox':{'type':'array','items':{'type':'number'},'minItems':4,'maxItems':4,'description':'Pixel x,y,width,height in the returned image'},'height_mm':{'type':'number','minimum':0,'maximum':60},'jaw_yaw_deg':{'type':'number','minimum':-180,'maximum':180,'description':'Optional closing-axis yaw in the calibrated robot base frame; omit if uncertain'}}}
SPECS={
'observe':('Get a fresh raw tabletop image and observation ID. Inspect it before selecting anything.',{}),
'classify':('Submit image-grounded trash/protected/bin decisions for all objects. Nothing moves.',{'observation_id':{'type':'string'},'objects':{'type':'array','items':OBJECT}}),
'execute':('Execute the reviewed selection through the Station client and MuJoCo contact physics.',{}),
'status':('Get progress, events and failure evidence. No hidden simulator object labels.',{}),
'stop':('Stop the cleanup sequence and command a hold at the measured simulated pose.',{}),
'verify_grasp':('Get wrist and overhead images of the lifted object; judge whether the intended trash is actually held.',{}),
'assess_grasp':('Continue carrying only after you visually confirm the intended trash is held.',{'observation_id':{'type':'string'},'held':{'type':'boolean'}}),
'verify':('Get a fresh final image after release and return to folded rest.',{}),
'assess':('Record your visual assessment of the returned verification image.',{'observation_id':{'type':'string'},'trash_in_bin':{'type':'boolean'},'protected_untouched':{'type':'boolean'}})}


async def list_tools(ctx,params):
    return types.ListToolsResult(tools=[types.Tool(name='cleanup_'+name,description=description,input_schema={'type':'object','properties':props,'required':list(props),'additionalProperties':False}) for name,(description,props) in SPECS.items()])


async def call_tool(ctx,params):
    name=params.name.removeprefix('cleanup_')
    if name not in SPECS:return types.CallToolResult(content=[types.TextContent(type='text',text='Unknown tool')],is_error=True)
    try:
        async with httpx.AsyncClient(timeout=15) as c:
            r=await c.post(URL+'/api/cleanup/'+name,json=params.arguments or {});r.raise_for_status();data=r.json()
        img=data.pop('image_b64',None);extra=data.pop('images_b64',[]);content=[types.TextContent(type='text',text=json.dumps(data))]
        for image in ([img] if img else [])+extra:content.append(types.ImageContent(type='image',data=image,mime_type='image/jpeg'))
        return types.CallToolResult(content=content,is_error=not data.get('ok'))
    except Exception as e:return types.CallToolResult(content=[types.TextContent(type='text',text=str(e))],is_error=True)

server=Server('station-cleanup',instructions=INSTRUCTIONS,on_list_tools=list_tools,on_call_tool=call_tool)
async def run():
    async with stdio_server() as (read,write):await server.run(read,write,server.create_initialization_options())
def main():asyncio.run(run())
if __name__=='__main__':main()
