import json
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,'/work/robocon_linux_training_v3/demo')
from sim_demo import Grid

ROOT=Path(__file__).resolve().parent
CONFIG=json.loads((ROOT/'config.json').read_text())

def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grid():
    cfg=dict(CONFIG)
    cfg['obstacles_m']=[[a+5.5,b+5.5,c+5.5,d+5.5] for a,b,c,d in [o['box'] for o in CONFIG['obstacles_world']]]
    return Grid(cfg,'baseline')

def plan_world(start,goal):
    g=grid();raw=g.astar((start[0]+5.5,start[1]+5.5),(goal[0]+5.5,goal[1]+5.5))
    simple=g.simplify(raw)
    pts=[(x-5.5,y-5.5) for x,y in simple]
    assert len(pts)>1 and all(g.line(a,b) for a,b in zip(simple,simple[1:])), 'Invalid route'
    return pts,[(x-5.5,y-5.5) for x,y in raw]

def profile(length,t,vmax=2.,acc=2.):
    ta=min(vmax/acc,math.sqrt(length/acc));v=ta*acc;tc=max(0.,(length-acc*ta*ta)/v);total=2*ta+tc
    if t<ta:return .5*acc*t*t,acc*t,total
    if t<ta+tc:return .5*acc*ta*ta+v*(t-ta),v,total
    if t<total:return length-.5*acc*(total-t)**2,acc*(total-t),total
    return length,0.,total

def wrap(x):return math.atan2(math.sin(x),math.cos(x))

def clearance_world(x,y,radius=None):
    if radius is None:radius=math.hypot(.58,.68)/2
    values=[5.5-abs(x)-radius,5.5-abs(y)-radius]
    for obj in CONFIG['obstacles_world']:
        a,b,c,d=obj['box'];values.append(math.hypot(max(a-x,0,x-c),max(b-y,0,y-d))-radius)
    return min(values)
