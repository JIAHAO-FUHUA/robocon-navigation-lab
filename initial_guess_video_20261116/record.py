"""Real-data video dashboard with visible world destination, loop and A* route."""
from pathlib import Path
legacy=Path('/work/video_demo_20261113/record_dashboard.py').read_text()
prefix=legacy.split('story={};mtime=0;last_trail=0.;first_camera=None')[0]
prefix=prefix.replace("Path('/work/latest_map.txt')","Path('/work/initial_guess_video_20261116/latest_map.txt')")
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
# English-only dashboard. The truth observer never publishes motion commands.
MAGENTA='#B453C4'
startup_snapshot={}

def label(d,xy,value,size=21,color=navy,strong=False):
    drawtext(d,xy,value,size,color,strong)

def arrow(d,p,yaw,length,color,width=3):
    q=(p[0]+length*math.cos(yaw),p[1]-length*math.sin(yaw))
    d.line([p,q],fill=color,width=width)
    for turn in [-.5,.5]:
        d.line([q,(q[0]-9*math.cos(yaw+turn),q[1]+9*math.sin(yaw+turn))],fill=color,width=width)

def actual_start():
    return data['truth'][0][1:4] if data['truth'] else None

def initial_estimate():
    global startup_snapshot
    file=run/'startup_result.json'
    if file.exists():
        if not startup_snapshot:startup_snapshot=json.loads(file.read_text())
        return startup_snapshot['initial_estimate_world_pose']
    p=pose()
    if p is None:return None
    return (p[0]-CONFIG['map_from_world_translation'][0],p[1]-CONFIG['map_from_world_translation'][1],p[2])

def paragraphs(d,value,x,y,width,size=24,line_height=40,max_lines=2):
    words=value.split();lines=[];line=''
    for word in words:
        candidate=(line+' '+word).strip()
        if d.textlength(candidate,font=fonts[size])>width and line:
            lines.append(line);line=word
        else:line=candidate
    if line:lines.append(line)
    assert len(lines)<=max_lines,value
    for i,line in enumerate(lines):label(d,(x,y+i*line_height),line,size,'white' if i==0 else '#C5D7E9')

def draw_start_zoom(d,actual,estimate):
    left,top,right,bottom=1213,527,1533,847
    d.rectangle((left,top,right,bottom),fill='#F5F7FA',outline='#8DA1B3',width=2)
    origin=(1326,734);scale=355
    def pt(p):return origin[0]+(p[0]-CONFIG['start_world'][0])*scale,origin[1]-(p[1]-CONFIG['start_world'][1])*scale
    for value in [-.2,0,.2,.4]:
        x=origin[0]+value*scale;y=origin[1]-value*scale
        d.line([(x,top+32),(x,bottom-24)],fill='#D9E2EB',width=1)
        d.line([(left+20,y),(right-10,y)],fill='#D9E2EB',width=1)
    label(d,(1226,535),'START AREA: enlarged',17,navy,True)
    seed=origin
    d.line([(seed[0]-7,seed[1]-7),(seed[0]+7,seed[1]+7)],fill=MAGENTA,width=4)
    d.line([(seed[0]-7,seed[1]+7),(seed[0]+7,seed[1]-7)],fill=MAGENTA,width=4)
    arrow(d,seed,0,53,MAGENTA)
    label(d,(seed[0]-81,seed[1]+15),'Zero seed',17,MAGENTA,True)
    if actual:
        p=pt(actual);d.ellipse((p[0]-10,p[1]-10,p[0]+10,p[1]+10),outline=blue,width=3)
        arrow(d,p,actual[2],53,blue)
        label(d,(p[0]-20,p[1]-42),'Actual',17,blue,True)
        dashed(d,[seed,p],'#8D9CAB',2,7)
    if estimate:
        p=pt(estimate);d.ellipse((p[0]-4,p[1]-4,p[0]+4,p[1]+4),fill=green)
        arrow(d,p,estimate[2],43,green,2)
        label(d,(p[0]-8,p[1]+20),'ICP',17,green,True)
    label(d,(1226,811),'0.20 m offset on each axis',17,gray)

def pose_card(d,actual,estimate):
    label(d,(1555,531),'INITIAL POSE COMPARISON',19,navy,True)
    ox,oy=CONFIG['start_world']
    label(d,(1555,570),'Actual start (simulation)',19,blue,True)
    if actual:
        label(d,(1555,599),f'({actual[0]-ox:+.3f}, {actual[1]-oy:+.3f}, 0.000) m',19,navy)
        label(d,(1555,625),f'Yaw: {math.degrees(actual[2]):+.2f} deg',21,navy)
    label(d,(1555,670),'Input sent to ICP',19,MAGENTA,True)
    label(d,(1555,699),'(0.000, 0.000, 0.000) m',19,navy)
    label(d,(1555,725),'Yaw: 0.00 deg',21,navy)
    label(d,(1555,770),'ICP result (relative to S)',19,green,True)
    if estimate:
        label(d,(1555,799),f'({estimate[0]-ox:+.3f}, {estimate[1]-oy:+.3f}) m',19,navy)
        label(d,(1555,825),f'Yaw: {math.degrees(estimate[2]):+.2f} deg',21,navy)
    else:label(d,(1555,806),'Waiting for a valid match...',19,gray)

def render():
    global story,mtime
    file=run/'story_state.json'
    if file.exists() and file.stat().st_mtime_ns!=mtime:
        story=json.loads(file.read_text());mtime=file.stat().st_mtime_ns
    im=Image.new('RGB',(1920,1080),'#F2F5F9');d=ImageDraw.Draw(im)
    d.rectangle((0,0,1920,112),fill=navy)
    title=story.get('title','Inaccurate initial pose: zero seed versus actual start').split('|')[0].strip()
    label(d,(34,15),title,40,'white',True)
    label(d,(36,71),'PolyU FENG AI & Robotics Club  |  FAST-LIO2 > ICP > A* > Estimated-pose control',22,'#C5D7E9')
    labels=['FIELD + GOAL','SENSORS','2 m/s LOOP','ZERO-SEED ICP','A* PLAN','AUTO DRIVE','RESULT']
    active=story.get('step',3)
    for i,value in enumerate(labels):
        x=35+i*265;fill=cyan if i==active else '#D6E7EC' if i<active else '#E0E6ED'
        d.rounded_rectangle((x,124,x+248,161),7,fill=fill)
        label(d,(x+12,128),f'{i+1:02d}  {value}',19,'white' if i==active else navy,True)
    actual=actual_start();estimate=initial_estimate()
    d.rounded_rectangle((28,178,1161,872),10,fill='white')
    label(d,(48,187),'GAZEBO  |  Correct the initial pose, then drive to G',24,navy,True)
    label(d,(48,221),'Blue: actual start    Magenta: zero seed    Green: ICP result    G: destination',21,gray)
    if data['camera']:
        im.paste(data['camera'].crop((30,135,970,690)).resize((1100,600),Image.Resampling.BILINEAR),(44,260))
        d=ImageDraw.Draw(im)
        route=[(p[0]-CONFIG['map_from_world_translation'][0],p[1]-CONFIG['map_from_world_translation'][1]) for p in data['path']]
        if len(route)>1:d.line([field_pixel(*p) for p in route],fill=orange,width=6)
        visible=[p for p in data['truth'] if p[0]<=data['camera_stamp']+.005]
        if len(visible)>1:d.line([field_pixel(p[1],p[2]) for p in visible],fill='#287DD0',width=4)
        x,y=field_pixel(*CONFIG['goal_world']);d.ellipse((x-10,y-10,x+10,y+10),outline=orange,width=4)
        d.rounded_rectangle((x+14,y-40,x+157,y+4),6,fill=navy);label(d,(x+23,y-38),'G  GOAL',24,'white',True)
        x,y=field_pixel(*CONFIG['start_world']);d.line([(x-7,y-7),(x+7,y+7)],fill=MAGENTA,width=3);d.line([(x-7,y+7),(x+7,y-7)],fill=MAGENTA,width=3)
        d.line([(x,y),(x-63,y-38)],fill=MAGENTA,width=2)
        d.rounded_rectangle((x-205,y-67,x-57,y-30),5,fill=navy);label(d,(x-196,y-66),'ZERO SEED',19,'#E7B8F0',True)
        if actual:
            x,y=field_pixel(actual[0],actual[1]);d.ellipse((x-9,y-9,x+9,y+9),outline=blue,width=3)
            d.line([(x,y),(x+64,y-77)],fill=blue,width=2)
            d.rounded_rectangle((x+58,y-106,x+238,y-68),5,fill=navy);label(d,(x+68,y-103),'ACTUAL START',19,'#A5CEFF',True)
        if estimate:
            x,y=field_pixel(estimate[0],estimate[1]);d.ellipse((x-4,y-4,x+4,y+4),fill=green)
        speed=0.
        if len(data['truth'])>2:
            a,b=data['truth'][-3],data['truth'][-1];dt=b[0]-a[0]
            if dt>0:speed=math.hypot(b[1]-a[1],b[2]-a[2])/dt
        d.rounded_rectangle((847,275,1128,362),9,fill=navy)
        label(d,(864,279),'Measured speed',17,'#C5D7E9');label(d,(864,306),f'{speed:.2f} m/s',34,'white',True)
        d.rounded_rectangle((51,814,1137,855),5,fill=navy)
        label(d,(64,821),'Magenta: zero seed  |  Green: ICP pose  |  Orange: A* route  |  Blue: actual trail',19,'white',True)
    d.rounded_rectangle((1180,178,1892,497),10,fill='#112035')
    label(d,(1200,189),'Saved map + live ICP alignment',24,'white',True)
    pts=data['points'][::max(1,len(data['points'])//55000)];scatter(im,pts,(116,143,167))
    live=data['live']
    if data['health']:
        h=data['health'];live=live@rotation(h['offset_quaternion']).T+np.array(h['offset_translation'])
    scatter(im,live,(50,215,220));d=ImageDraw.Draw(im)
    d.rounded_rectangle((1180,510,1892,872),10,fill='white')
    if active==3:draw_start_zoom(d,actual,estimate)
    else:
        d.rectangle((1213,527,1533,847),fill='#F5F7FA',outline='#8DA1B3',width=2)
        radius=math.hypot(.58,.68)/2+CONFIG['safety_margin_m'];scale=320/11
        for obj in CONFIG['obstacles_world']:
            a,b,c,e=obj['box'];x1,y1=world_pixel(a,e);x2,y2=world_pixel(c,b)
            d.rounded_rectangle((x1-radius*scale,y1-radius*scale,x2+radius*scale,y2+radius*scale),radius=int(radius*scale),fill='#F4DBD5')
            d.rectangle((x1,y1,x2,y2),fill='#D65545' if obj['name']=='Red obstacle' else '#9DAFC0')
        route=[(p[0]-CONFIG['map_from_world_translation'][0],p[1]-CONFIG['map_from_world_translation'][1]) for p in data['path']]
        if len(route)>1:d.line([world_pixel(*p) for p in route],fill=orange,width=5)
        if len(data['truth'])>1:d.line([world_pixel(p[1],p[2]) for p in data['truth']],fill='#287DD0',width=3)
        for value,xy,color in [('I',CONFIG['start_world'],MAGENTA),('G',CONFIG['goal_world'],orange)]:
            x,y=world_pixel(*xy);d.ellipse((x-6,y-6,x+6,y+6),fill=color);label(d,(x+8,y-20),value,21,navy,True)
        if data['truth']:
            p=data['truth'][-1];x,y=world_pixel(p[1],p[2]);d.ellipse((x-6,y-6,x+6,y+6),fill=blue);arrow(d,(x,y),p[3],15,navy)
        label(d,(1223,532),'FULL FIELD: 11 x 11 m',17,navy,True)
    pose_card(d,actual,estimate)
    d.rectangle((0,890,1920,1080),fill=navy)
    paragraphs(d,story.get('en','Actual start is displaced and rotated. ICP receives a zero initial pose.'),36,902,1848)
    health=data['health'];status='ICP verified: ready' if startup_snapshot else 'ICP matching' if health and health.get('match_ok') else 'Waiting for ICP'
    label(d,(36,1003),f'sim {data["clock"]:.1f} s  |  LiDAR {data["lidar_n"]:,} pts/frame  |  {status}',21,'#78D2DD')
    label(d,(36,1042),'Truth: display/evaluation only | Control: estimated poses | Known-layout A* grid | 2 m/s straights, slower turns',19,'#B0C0CF')
    return im

exec(legacy[legacy.index('encoder=subprocess.Popen'):])
