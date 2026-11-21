"""Semantic selection contract: the agent supplies image-grounded observations."""
import math

TRASH_LABELS={'crumpled paper','empty snack wrapper'}
PROTECTED_LABELS={'computer mouse','pen'}


def validate_annotations(objects,width,height):
    if not isinstance(objects,list) or not 1<=len(objects)<=20:raise ValueError('Provide all visible objects, at most 20')
    result=[]
    for i,a in enumerate(objects):
        label=str(a.get('label','')).strip().lower();decision=a.get('decision');confidence=a.get('confidence',0);bbox=a.get('bbox',[]);h=a.get('height_mm',15)
        if decision not in ('trash','keep','unknown','bin') or not label:raise ValueError('Every object needs a label and decision')
        if not isinstance(confidence,(int,float)) or not math.isfinite(confidence) or not 0<=confidence<=1:raise ValueError('Invalid confidence')
        if len(bbox)!=4 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in bbox):raise ValueError('Use pixel bbox [x,y,width,height]')
        x,y,w,bh=bbox
        if x<0 or y<0 or w<5 or bh<5 or x+w>width or y+bh>height:raise ValueError('Object box lies outside the observed image')
        if not isinstance(h,(int,float)) or not math.isfinite(h) or not 0<=h<=60:raise ValueError('Invalid estimated object height')
        if decision=='trash' and (label not in TRASH_LABELS or confidence<.9):raise ValueError('Only confidently identified crumpled paper or empty snack wrappers may be selected')
        if label in PROTECTED_LABELS and decision!='keep':raise ValueError('Mouse and pen must be protected')
        if decision=='bin' and confidence<.9:raise ValueError('Identify the destination bin confidently')
        yaw=a.get('jaw_yaw_deg')
        if yaw is not None and (not isinstance(yaw,(int,float)) or not math.isfinite(yaw) or not -180<=yaw<=180):raise ValueError('Invalid jaw yaw')
        result.append({'id':f'object-{i+1}','label':label,'decision':decision,'confidence':confidence,'bbox':list(bbox),'height_mm':h,'jaw_yaw_deg':yaw})
    if sum(a['decision']=='bin' for a in result)!=1:raise ValueError('Identify exactly one destination bin')
    if not any(a['decision']=='trash' for a in result):raise ValueError('No confidently identified trash; leave the scene unchanged')
    for a in result:
        if a['decision']!='trash':continue
        x,y,w,h=a['bbox']
        for b in result:
            if b is a or b['decision']=='bin':continue
            xx,yy,ww,hh=b['bbox']
            if max(x,xx)<min(x+w,xx+ww) and max(y,yy)<min(y+h,yy+hh):raise ValueError('Trash selection overlaps another object; refine the boxes')
    return result
