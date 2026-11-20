"""Real-data video dashboard with visible world destination, loop and A* route."""
from pathlib import Path
legacy=Path('/work/video_demo_20261113/record_dashboard.py').read_text()
prefix=legacy.split('story={};mtime=0;last_trail=0.;first_camera=None')[0]
prefix=prefix.replace("Path('/work/latest_map.txt')","Path('/work/complex_demo_20261113/latest_map.txt')")
exec(prefix)
from geometry_msgs.msg import PoseArray
from core import CONFIG,ROOT
data.update(truth=[],camera_stamp=0.,task={})
def true_pose(m):
    if not m.poses:return
    p=m.poses[0];q=p.orientation;stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    row=(stamp,p.position.x,p.position.y,yaw)
    if not data['truth'] or stamp>data['truth'][-1][0]:data['truth'].append(row)
node.create_subscription(PoseArray,'/evaluation/ground_truth',true_pose,10)
node.create_subscription(String,'/club/task_status',lambda m:data.update(task=json.loads(m.data)),10)
old_camera=camera
def camera_with_stamp(m):
    old_camera(m);data['camera_stamp']=m.header.stamp.sec+m.header.stamp.nanosec/1e9
# Replace the observer's camera subscription to retain source timestamps for overlays.
for subscription in list(node.subscriptions):
    if subscription.topic_name=='/training/overview':node.destroy_subscription(subscription)
node.create_subscription(ROSImage,'/training/overview',camera_with_stamp,visual_qos)
story={};mtime=0
def field_pixel(x,y,z=.02):
    pitch=.7454;cp,sp=math.cos(pitch),math.sin(pitch)
    forward=cp*(y+13)-sp*(z-12);up=sp*(y+13)+cp*(z-12);f=1000/(2*math.tan(1.05/2))
    u=500+f*x/forward;v=400-f*up/forward
    return 44+(u-30)*1100/940,260+(v-135)*600/555
def world_pixel(x,y):return 1213+(x+5.5)/11*320,847-(y+5.5)/11*320
def dashed(d,points,color,width=4,dash=13):
    for a,b in zip(points,points[1:]):
        a=np.array(a);b=np.array(b);delta=b-a;length=np.linalg.norm(delta)
        if length==0:continue
        for s in np.arange(0,length,dash*1.7):
            p=a+delta*(s/length);q=a+delta*(min(length,s+dash)/length);d.line([tuple(p),tuple(q)],fill=color,width=width)
def project(points):
    p=points-np.array([4.4,4.4,0.])
    return np.stack([1532+31*(.866*p[:,0]-.5*p[:,1]),365+19*(.32*p[:,0]+.55*p[:,1]-.90*p[:,2])],axis=1).astype(int)
def scatter(image,points,color):
    if not len(points):return
    pix=project(points);valid=(pix[:,0]>1196)&(pix[:,0]<1878)&(pix[:,1]>235)&(pix[:,1]<475);pix=pix[valid]
    array=np.asarray(image).copy();array[pix[:,1],pix[:,0]]=color;array[np.minimum(pix[:,1]+1,474),pix[:,0]]=color
    image.paste(Image.fromarray(array))
def render():
    global story,mtime
    file=run/'story_state.json'
    if file.exists() and file.stat().st_mtime_ns!=mtime:story=json.loads(file.read_text());mtime=file.stat().st_mtime_ns
    im=Image.new('RGB',(1920,1080),'#F2F5F9');d=ImageDraw.Draw(im)
    d.rectangle((0,0,1920,112),fill=navy)
    drawtext(d,(34,15),story.get('title','Robocon | Full-loop mapping and obstacle avoidance'),40,'white',True)
    drawtext(d,(36,71),'PolyU FENG AI & Robotics Club  |  FAST-LIO2 → ICP → A* → Estimated-pose control',22,'#C5D7E9')
    labels=['FIELD + GOAL','SENSORS','2 m/s LOOP','RELOCALIZE','A* PLAN','AUTO DRIVE','RESULT'];active=story.get('step',0)
    for i,label in enumerate(labels):
        x=35+i*265;fill=cyan if i==active else ('#D6E7EC' if i<active else '#E0E6ED')
        d.rounded_rectangle((x,124,x+248,161),7,fill=fill);drawtext(d,(x+12,128),f'{i+1:02d}  {label}',19,'white' if i==active else navy,True)
    d.rounded_rectangle((28,178,1161,872),10,fill='white')
    drawtext(d,(48,187),'GAZEBO  |  Destination and measured trail',24,navy,True)
    drawtext(d,(48,221),'S: start / 起点      G: goal / 目的地      Red: obstacle / 障碍',21,gray)
    if data['camera']:
        im.paste(data['camera'].crop((30,135,970,690)).resize((1100,600),Image.Resampling.BILINEAR),(44,260));d=ImageDraw.Draw(im)
        if args.mode=='mapping':dashed(d,[field_pixel(*p) for p in CONFIG['mapping_loop_world']],cyan,5)
        route=[(p[0]-CONFIG['map_from_world_translation'][0],p[1]-CONFIG['map_from_world_translation'][1]) for p in data['path']]
        if args.mode!='mapping' and len(route)>1:d.line([field_pixel(*p) for p in route],fill=orange,width=6)
        visible=[p for p in data['truth'] if p[0]<=data['camera_stamp']+.005]
        if len(visible)>1:d.line([field_pixel(p[1],p[2]) for p in visible],fill='#287DD0',width=4)
        for label,xy,color in [('S 起点',CONFIG['start_world'],green),('G 目的地',CONFIG['goal_world'],orange)]:
            x,y=field_pixel(*xy);d.ellipse((x-10,y-10,x+10,y+10),outline=color,width=4)
            d.rounded_rectangle((x+14,y-40,x+157,y-3),6,fill=navy);drawtext(d,(x+23,y-38),label,24,'white',True)
        if args.mode=='navigation':
            xx,yy=field_pixel(3.3,0,.8);drawtext(d,(xx-35,yy-25),'障碍',21,'#F7E5E1',True)
        speed=0.
        if len(data['truth'])>2:
            a,b=data['truth'][-3],data['truth'][-1];dt=b[0]-a[0]
            if dt>0:speed=math.hypot(b[1]-a[1],b[2]-a[2])/dt
        d.rounded_rectangle((847,275,1128,362),9,fill=navy)
        drawtext(d,(864,279),'Measured speed / 实测速率',17,'#C5D7E9')
        drawtext(d,(864,306),f'{speed:.2f} m/s',34,'white',True)
        drawtext(d,(58,819),'青：建图路线   橙：A*规划   蓝：实际轨迹',22,navy,True)
    d.rounded_rectangle((1180,178,1892,497),10,fill='#112035')
    drawtext(d,(1200,189),'Live map' if args.mode=='mapping' else 'Saved map + live ICP alignment',24,'white',True)
    pts=data['points'][::max(1,len(data['points'])//55000)];scatter(im,pts,(116,143,167))
    live=data['live']
    if args.mode!='mapping' and data['health']:
        h=data['health'];live=live@rotation(h['offset_quaternion']).T+np.array(h['offset_translation'])
    scatter(im,live,(50,215,220));d=ImageDraw.Draw(im)
    d.rounded_rectangle((1180,510,1892,872),10,fill='white')
    d.rectangle((1213,527,1533,847),fill='#F5F7FA',outline='#8DA1B3',width=2)
    radius=math.hypot(.58,.68)/2+CONFIG['safety_margin_m'];scale=320/11
    for obj in CONFIG['obstacles_world']:
        a,b,c,e=obj['box'];x1,y1=world_pixel(a,e);x2,y2=world_pixel(c,b)
        d.rounded_rectangle((x1-radius*scale,y1-radius*scale,x2+radius*scale,y2+radius*scale),radius=int(radius*scale),fill='#F4DBD5')
        d.rectangle((x1,y1,x2,y2),fill='#D65545' if obj['name']=='Red obstacle' else '#9DAFC0')
    if args.mode=='mapping':dashed(d,[world_pixel(*p) for p in CONFIG['mapping_loop_world']],cyan,3,8)
    route=[(p[0]-CONFIG['map_from_world_translation'][0],p[1]-CONFIG['map_from_world_translation'][1]) for p in data['path']]
    if args.mode!='mapping' and len(route)>1:
        dashed(d,[world_pixel(*route[0]),world_pixel(*CONFIG['goal_world'])],'#909EAA',2,6)
        cross=((route[0][0]+CONFIG['goal_world'][0])/2,(route[0][1]+CONFIG['goal_world'][1])/2)
        cx,cy=world_pixel(*cross);d.line([(cx-6,cy-6),(cx+6,cy+6)],fill='#C63E32',width=3);d.line([(cx-6,cy+6),(cx+6,cy-6)],fill='#C63E32',width=3)
        d.line([world_pixel(*p) for p in route],fill=orange,width=5)
    if len(data['truth'])>1:d.line([world_pixel(p[1],p[2]) for p in data['truth']],fill='#287DD0',width=3)
    for label,xy,color in [('S',CONFIG['start_world'],green),('G',CONFIG['goal_world'],orange)]:
        x,y=world_pixel(*xy);d.ellipse((x-6,y-6,x+6,y+6),fill=color);drawtext(d,(x+8,y-20),label,21,navy,True)
    if data['truth']:
        p=data['truth'][-1];x,y=world_pixel(p[1],p[2]);d.ellipse((x-6,y-6,x+6,y+6),fill=blue)
        d.line([(x,y),(x+15*math.cos(p[3]),y-15*math.sin(p[3]))],fill=navy,width=3)
    drawtext(d,(1555,535),'FULL FIELD  11 × 11 m',19,navy,True)
    drawtext(d,(1555,578),'巡航目标 / Cruise target',19,gray)
    drawtext(d,(1555,608),'2.00 m/s',34,navy,True)
    task=data['task']
    drawtext(d,(1555,663),f'Segment {task["segment"]+1} / {task["segments"]}' if task else 'Waiting / 等待',24,navy,True)
    if data['truth']:drawtext(d,(1555,707),f'Heading {math.degrees(data["truth"][-1][3]):.0f}°',24,navy)
    h=data['health'];status='ICP matched' if h and h['match_ok'] else ('FAST-LIO2 mapping' if args.mode=='mapping' else 'Awaiting ICP')
    drawtext(d,(1555,754),status,22,green if h and h['match_ok'] else gray)
    drawtext(d,(1555,805),'粉色：膨胀避让范围',17,gray)
    if args.mode!='mapping' and data['path']:drawtext(d,(1555,838),'灰虚线：直线受阻',17,gray)
    d.rectangle((0,890,1920,1080),fill=navy)
    drawtext(d,(36,902),story.get('cn','场地目的地、路线与实际运动轨迹'),30,'white')
    drawtext(d,(36,952),story.get('en','Destination, planned route and measured robot motion'),24,'#C5D7E9')
    drawtext(d,(36,1003),f'sim {data["clock"]:.1f} s  |  LiDAR {data["lidar_n"]:,} pts/frame  |  {status}',21,'#78D2DD')
    drawtext(d,(36,1042),'Teaching layout · 2 m/s on straights, slower turns · known-layout A* grid · estimated-pose control',19,'#B0C0CF')
    return im

exec(legacy[legacy.index('encoder=subprocess.Popen'):])
