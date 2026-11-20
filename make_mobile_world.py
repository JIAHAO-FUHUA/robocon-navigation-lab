"""Complete a native SDF mecanum model using converted pinned Livox xacro."""
import copy
import sys
import xml.etree.ElementTree as E
from pathlib import Path
E.register_namespace('gz','http://gazebosim.org/schema')

root=Path('/work')
sys.path.insert(0,str(root/'robocon_linux_training_v3/tools'))
from generate_world import generate
import json

def item(parent,tag,text=None,**attributes):
    elem=E.SubElement(parent,tag,**attributes)
    if text is not None:elem.text=str(text)
    return elem

world=E.fromstring(generate(json.loads((root/'robocon_linux_training_v3/config/field_spec.json').read_text())))
# Training room walls outside the official 11 m floor; not official field fixtures.
for name,size,xyz in [('room_n',[12.2,.1,2],[0,6,1]),('room_s',[12.2,.1,2],[0,-6,1]),
                      ('room_e',[.1,12.2,2],[6,0,1]),('room_w',[.1,12.2,2],[-6,0,1])]:
    fixture=item(world.find('world'),'model',name=name)
    item(fixture,'static','true');item(fixture,'pose',' '.join(map(str,xyz))+' 0 0 0')
    link=item(fixture,'link',name='link')
    for kind in ['collision','visual']:
        element=item(link,kind,name=kind)
        shape=item(item(element,'geometry'),'box');item(shape,'size',' '.join(map(str,size)))
        if kind=='visual':
            material=item(element,'material');item(material,'ambient','0.65 0.68 0.72 1');item(material,'diffuse','0.65 0.68 0.72 1')
for color_name,color in [('L1','0.55 0.65 0.75 1'),('L2','0.20 0.42 0.68 1')]:
    material=world.find(f'.//model[@name="{color_name}"]/link/visual/material')
    material.find('ambient').text=color;material.find('diffuse').text=color
model=E.parse(root/'mobile_sensor.sdf').getroot().find('model')
model.set('name','club_robot')
item(model,'pose',os_spawn if (os_spawn:=__import__('os').environ.get('ROBOCON_SPAWN')) else '-4.4 -4.4 0.12 0 0 0')
for name,x,y,sign in [('front_left',.2,.25,-1),('front_right',.2,-.25,1),('rear_left',-.2,.25,1),('rear_right',-.2,-.25,-1)]:
    link=item(model,'link',name=name+'_wheel')
    item(link,'pose',f'{x} {y} -0.03 -1.57079632679 0 0')
    inertia=item(link,'inertial');item(inertia,'mass',.4)
    ii=item(inertia,'inertia')
    for field,v in [('ixx',.001),('iyy',.001),('izz',.0016),('ixy',0),('ixz',0),('iyz',0)]:item(ii,field,v)
    for kind in ['collision','visual']:
        element=item(link,kind,name=kind)
        geom=item(element,'geometry')
        shape=item(geom,'sphere' if kind=='collision' else 'cylinder');item(shape,'radius',.09)
        if kind=='visual':item(shape,'length',.05)
        else:
            ode=item(item(item(element,'surface'),'friction'),'ode')
            item(ode,'mu',1.0);item(ode,'mu2',0.0)
            direction=item(ode,'fdir1',f'1 {sign} 0')
            direction.set('{http://gazebosim.org/schema}expressed_in','base_link')
    joint=item(model,'joint',name=name+'_wheel_joint',type='revolute')
    item(joint,'parent','base_link');item(joint,'child',name+'_wheel')
    axis=item(joint,'axis');item(axis,'xyz','0 0 1')
    limit=item(axis,'limit');item(limit,'lower',-1e16);item(limit,'upper',1e16)
plugin=item(model,'plugin',filename='gz-sim-mecanum-drive-system',name='gz::sim::systems::MecanumDrive')
for tag,v in [('front_left_joint','front_left_wheel_joint'),('front_right_joint','front_right_wheel_joint'),
              ('back_left_joint','rear_left_wheel_joint'),('back_right_joint','rear_right_wheel_joint'),
              ('wheel_separation',.5),('wheelbase',.4),('wheel_radius',.09),
              ('min_linear_acceleration',-.5),('max_linear_acceleration',.5),
              ('min_angular_acceleration',-.6),('max_angular_acceleration',.6)]:item(plugin,tag,v)
truth=item(model,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
for tag,v in [('publish_link_pose','false'),('publish_sensor_pose','false'),('publish_collision_pose','false'),
              ('publish_visual_pose','false'),('publish_nested_model_pose','false'),('use_pose_vector_msg','true'),
              ('publish_model_pose','true'),('static_publisher','false'),('update_frequency','20'),('topic','/model/club_robot/evaluation_pose')]:item(truth,tag,v)
world.find('world').append(model)
camera=item(world.find('world'),'model',name='training_overview_camera')
item(camera,'static','true');item(camera,'pose','0 -13 12 0 0.7454 1.57079632679')
camera_link=item(camera,'link',name='link')
sensor=item(camera_link,'sensor',name='overview',type='camera')
item(sensor,'always_on','true');item(sensor,'update_rate',2);item(sensor,'topic','/training/overview')
cam=item(sensor,'camera');item(cam,'horizontal_fov',1.05)
image=item(cam,'image');item(image,'width',1000);item(image,'height',800);item(image,'format','R8G8B8')
clip=item(cam,'clip');item(clip,'near',.1);item(clip,'far',100)
E.indent(world)
E.ElementTree(world).write(root/'mobile_world.sdf',encoding='utf-8',xml_declaration=True)
print('Created /work/mobile_world.sdf')
