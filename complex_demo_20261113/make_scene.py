import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as E
from core import CONFIG,ROOT

out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
mode=sys.argv[2];yaw=float(os.environ.get('COMPLEX_YAW_DEG',CONFIG['restart_yaw_deg']))
spawn=[*CONFIG['start_world'],.12,0,0,0] if mode=='mapping' else [*CONFIG['restart_world'],.12,0,0,yaw*3.141592653589793/180]
env=dict(os.environ,ROBOCON_SPAWN=' '.join(map(str,spawn)))
subprocess.run(['python3','/work/make_mobile_world.py'],check=True,env=env)
tree=E.parse('/work/mobile_world.sdf');world=tree.getroot().find('world')
E.register_namespace('gz','http://gazebosim.org/schema')
def item(parent,tag,text=None,**attrs):
    e=E.SubElement(parent,tag,**attrs)
    if text is not None:e.text=str(text)
    return e

for name,pose in [('training_ref_A','-3.2 -1 0.4 0 0 0'),('training_ref_B','1 -3.2 0.55 0 0 0')]:
    world.find(f'model[@name="{name}"]/pose').text=pose
for obj in CONFIG['obstacles_world']:
    if obj['name']!='Red obstacle':continue
    a,b,c,d=obj['box'];h=obj['height'];m=item(world,'model',name='red_obstacle')
    item(m,'static','true');item(m,'pose',f'{(a+c)/2} {(b+d)/2} {h/2} 0 0 0');link=item(m,'link',name='link')
    for kind in ['collision','visual']:
        element=item(link,kind,name=kind);item(item(item(element,'geometry'),'box'),'size',f'{c-a} {d-b} {h}')
        if kind=='visual':
            mat=item(element,'material');item(mat,'ambient','0.85 0.16 0.12 1');item(mat,'diffuse','0.85 0.16 0.12 1')
for name,xy,color in [('start',CONFIG['start_world'],'0.1 0.7 0.45 1'),('goal',CONFIG['goal_world'],'1 0.7 0.05 1')]:
    m=item(world,'model',name=name+'_floor_marker');item(m,'static','true');item(m,'pose',f'{xy[0]} {xy[1]} 0.008 0 0 0')
    link=item(m,'link',name='link');vis=item(link,'visual',name='marker');cylinder=item(item(vis,'geometry'),'cylinder')
    item(cylinder,'radius',.32);item(cylinder,'length',.012)
    mat=item(vis,'material');item(mat,'ambient',color);item(mat,'diffuse',color)
plugin=world.find('.//model[@name="club_robot"]/plugin[@name="gz::sim::systems::MecanumDrive"]')
for name,value in [('min_linear_acceleration',-3),('max_linear_acceleration',3),('min_angular_acceleration',-2),('max_angular_acceleration',2)]:plugin.find(name).text=str(value)
world.find('model[@name="training_overview_camera"]/link/sensor/update_rate').text=os.environ.get('COMPLEX_CAMERA_HZ','8')
tree.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
(out/'spawn.json').write_text(__import__('json').dumps({'mode':mode,'pose':spawn,'yaw_deg':yaw}))
print(out/'world.sdf')
