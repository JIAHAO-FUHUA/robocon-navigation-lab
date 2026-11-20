"""Record real ROS camera/cloud/control data in a teaching dashboard; no motion output."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry,Path as NavPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image as ROSImage,PointCloud2,Imu
from sensor_msgs_py import point_cloud2
from livox_ros_driver2.msg import CustomMsg
from std_msgs.msg import String

ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);ap.add_argument('--mode',required=True);args=ap.parse_args()
run=args.run;W,H,FPS=1920,1080,12
fontfile='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
boldfile='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
fonts={size:ImageFont.truetype(fontfile,size) for size in [17,19,21,22,23,24,26,28,30,34,40]}
bold={size:ImageFont.truetype(boldfile,size) for size in [19,22,24,28,34,40]}
navy='#172C43';cyan='#22B8C8';blue='#2A76BF';orange='#F0B44C';green='#30AF7D';gray='#758598'
rclpy.init();node=Node('training_video_observer')
data={'camera':None,'points':np.empty((0,3)),'live':np.empty((0,3)),'odom':None,'health':None,'pose':None,
      'clock':0.,'lidar_n':0,'offset_ms':0.,'imu_n':0,'lidar_count':0,'cloud_count':0,'cmd':[0,0,0],'path':[], 'trail':[]}
def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def camera(m):
    channels=4 if 'a8' in m.encoding else 3
    a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*channels].reshape(m.height,m.width,channels)[:,:,:3]
    if m.encoding.startswith('bgr'):a=a[:,:,::-1]
    data['camera']=Image.fromarray(a.copy())
def cloud(m):
    data['cloud_count']+=1
    points=point_cloud2.read_points_numpy(m,field_names=('x','y','z'),skip_nans=True)
    points=np.asarray(points).reshape(-1,3)
    points=points[np.all(np.isfinite(points),axis=1)]
    data['live']=points[::max(1,len(points)//11000)].copy()
    if args.mode=='mapping' and data['cloud_count']%4==0:
        merged=np.vstack([data['points'],data['live']])
        _,indices=np.unique(np.floor(merged/.09).astype(np.int32),axis=0,return_index=True)
        data['points']=merged[np.sort(indices)][::max(1,len(indices)//60000)]
def odom(m):data['odom']=m
def lidar(m):
    data['lidar_count']+=1;data['lidar_n']=m.point_num
    if data['lidar_count']%10==1:
        data['offset_ms']=max((p.offset_time for p in m.points),default=0)/1e6
        if data['odom'] is None:data['live']=np.array([(p.x,p.y,p.z) for p in m.points[::3]],dtype=np.float32)
def health(m):data['health']=json.loads(m.data)
def imu(m):data['imu_n']+=1
def clock(m):data['clock']=m.clock.sec+m.clock.nanosec/1e9
def cmd(m):data['cmd']=[m.linear.x,m.linear.y,m.angular.z]
def path(m):data['path']=[(p.pose.position.x,p.pose.position.y) for p in m.poses]
visual_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(ROSImage,'/training/overview',camera,visual_qos)
node.create_subscription(PointCloud2,'/fastlio2/world_cloud',cloud,visual_qos)
node.create_subscription(Odometry,'/fastlio2/lio_odom',odom,10)
node.create_subscription(CustomMsg,'/livox/lidar',lidar,visual_qos)
node.create_subscription(Imu,'/livox/imu',imu,QoSProfile(depth=50,reliability=ReliabilityPolicy.BEST_EFFORT))
node.create_subscription(Clock,'/clock',clock,1)
node.create_subscription(Twist,'/club/cmd_vel',cmd,10)
node.create_subscription(String,'/club/localization_health',health,10)
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(NavPath,'/club/planned_path',path,qos)
def read_pcd(path):
    with Path(path).open('rb') as f:
        header={}
        while True:
            line=f.readline().decode('ascii').strip();parts=line.split()
            if parts:header[parts[0]]=parts[1:]
            if line.startswith('DATA'):break
        assert header['DATA']==['binary']
        a=np.frombuffer(f.read(),dtype=np.float32).reshape(-1,len(header['FIELDS']))
        return a[:,:3].copy()
if args.mode=='navigation':
    data['points']=read_pcd(Path('/work/latest_map.txt').read_text().strip())

def pose():
    m=data['odom']
    if m is None:return None
    if args.mode=='navigation' and not (data['health'] and data['health'].get('match_ok')):return None
    p=m.pose.pose.position;q=m.pose.pose.orientation
    r=rotation([q.x,q.y,q.z,q.w]);xyz=np.array([p.x,p.y,p.z])
    h=data['health']
    if args.mode=='navigation' and h:
        rr=rotation(h['offset_quaternion']);xyz=rr@xyz+np.array(h['offset_translation']);r=rr@r
    xyz=xyz-r@np.array([.011,.02329,.25288])
    return float(xyz[0]),float(xyz[1]),math.atan2(r[1,0],r[0,0])

def drawtext(d,xy,text,size=24,color=navy,strong=False):d.text(xy,str(text),font=(bold if strong and size in bold else fonts)[size],fill=color)
def project(points):
    p=points-np.array([4.4,4.4,0.])
    return np.stack([1532+37*(.866*p[:,0]-.5*p[:,1]),430+37*(.32*p[:,0]+.55*p[:,1]-.90*p[:,2])],axis=1).astype(int)
def scatter(image,points,color):
    if not len(points):return
    pix=project(points);mask=(pix[:,0]>1198)&(pix[:,0]<1875)&(pix[:,1]>240)&(pix[:,1]<625)
    pix=pix[mask]
    array=np.asarray(image).copy();array[pix[:,1],pix[:,0]]=color
    # A second pixel keeps dots visible on a projector.
    array[np.minimum(pix[:,1]+1,624),pix[:,0]]=color
    image.paste(Image.fromarray(array))
def map_point(x,y):return 1240+(x+.3)/1.9*610,828-(y+.3)/1.3*112

story={};mtime=0;last_trail=0.;first_camera=None
def render():
    global story,mtime,last_trail,first_camera
    file=run/'story_state.json'
    if file.exists() and file.stat().st_mtime_ns!=mtime:
        story=json.loads(file.read_text());mtime=file.stat().st_mtime_ns
    im=Image.new('RGB',(W,H),'#F2F5F9');d=ImageDraw.Draw(im)
    d.rectangle((0,0,W,112),fill=navy)
    drawtext(d,(34,15),story.get('title','Robocon 2027 | Live simulation'),40,'white',True)
    drawtext(d,(36,71),'PolyU FENG AI & Robotics Club  |  Gazebo + FAST-LIO2 + ICP + A* + Position control',22,'#C5D7E9')
    labels=['FIELD','SENSORS','MAPPING','RELOCALIZE','PLAN','AUTO DRIVE','RESULT']
    active=story.get('step',0)
    for i,label in enumerate(labels):
        x=35+i*265;fill=cyan if i==active else ('#D6E7EC' if i<active else '#E0E6ED')
        d.rounded_rectangle((x,124,x+248,161),7,fill=fill)
        drawtext(d,(x+12,128),f'{i+1:02d}  {label}',19,'white' if i==active else navy,True)
    d.rounded_rectangle((28,178,1161,872),10,fill='white')
    drawtext(d,(48,187),'GAZEBO  |  Live simulation camera',24,navy,True)
    drawtext(d,(48,221),'11 m × 11 m  ·  L1 0.6 m  ·  L2 0.9 m',21,gray)
    frame=data['camera']
    if frame:
        if first_camera is None:first_camera=time.time()
        cropped=frame.crop((30,135,970,690)).resize((1100,600),Image.Resampling.BILINEAR)
        im.paste(cropped,(44,260));d=ImageDraw.Draw(im)
        # A literal zoom of the live camera, showing the small robot more clearly.
        inset=frame.crop((150,455,450,640)).resize((390,241),Image.Resampling.BILINEAR)
        im.paste(inset,(744,600));d=ImageDraw.Draw(im)
        d.rectangle((740,568,1138,600),fill=navy);drawtext(d,(751,571),'Robot close-up / 机器人局部',19,'white')
        d.rectangle((740,568,1138,845),outline='white',width=3)
    else:drawtext(d,(310,480),'Waiting for simulation camera...',28,gray)
    d.rounded_rectangle((1180,178,1892,646),10,fill='#112035')
    drawtext(d,(1201,189),'Accumulated point cloud' if args.mode=='mapping' else 'Saved map + live ICP alignment',24,'white',True)
    graypoints=data['points'][::max(1,len(data['points'])//55000)]
    scatter(im,graypoints,(116,143,167))
    live=data['live']
    if args.mode=='navigation' and data['health']:
        hh=data['health'];live=live@rotation(hh['offset_quaternion']).T+np.array(hh['offset_translation'])
    scatter(im,live,(50,215,220))
    d=ImageDraw.Draw(im)
    drawtext(d,(1201,611),f"Map points shown: {len(graypoints):,}   |   cyan: current scan",19,'#C2D5E4')
    d.rounded_rectangle((1180,659,1892,872),10,fill='white')
    drawtext(d,(1201,667),'A* route + estimated robot pose  /  局部路线',22,navy,True)
    for x in [0,.5,1,1.5]:
        a=map_point(x,-.3);b=map_point(x,1.0);d.line([a,b],fill='#E0E7ED',width=1)
    for y in [0,.5,1]:
        a=map_point(-.3,y);b=map_point(1.6,y);d.line([a,b],fill='#E0E7ED',width=1)
    route=data['path']
    if len(route)>1:
        d.line([map_point(*p) for p in route],fill=orange,width=7)
        unique=[]
        for p in route:
            if not unique or math.hypot(p[0]-unique[-1][0],p[1]-unique[-1][1])>.01:unique.append(p)
        for i,p in enumerate(unique[1:]):
            x,y=map_point(*p);d.ellipse((x-8,y-8,x+8,y+8),fill=orange);drawtext(d,(x+10,y-24),f'T{i+1}',19,navy,True)
    pp=pose()
    if pp is not None:
        if time.monotonic()-last_trail>.08:data['trail'].append(pp[:2]);last_trail=time.monotonic()
        if len(data['trail'])>1:d.line([map_point(*p) for p in data['trail']],fill=blue,width=3)
        x,y=map_point(*pp[:2]);yaw=pp[2]
        d.ellipse((x-8,y-8,x+8,y+8),fill=blue)
        d.line([(x,y),(x+20*math.cos(yaw),y-20*math.sin(yaw))],fill=navy,width=4)
        drawtext(d,(1202,837),f'map x={pp[0]:.2f} m   y={pp[1]:.2f} m   yaw={math.degrees(pp[2]):.1f}°',19,gray)
        if data['path']:drawtext(d,(1625,839),'橙色规划 / 蓝色估计',17,gray)
    d.rectangle((0,890,W,1080),fill=navy)
    drawtext(d,(36,902),story.get('cn','真实仿真传感器与定位控制数据'),30,'white')
    drawtext(d,(36,952),story.get('en','Live simulation, sensor data and estimated robot pose'),24,'#C5D7E9')
    health=data['health'];status='ICP matched' if health and health.get('match_ok') else ('ICP awaiting initial pose' if args.mode=='navigation' else 'FAST-LIO2 mapping' if data['odom'] is not None else 'Sensors active / SLAM not started')
    telemetry=f"sim {data['clock']:6.1f} s  |  LiDAR {data['lidar_n']:,} pts/frame  |  point offsets ≤{data['offset_ms']:.0f} ms  |  {status}"
    drawtext(d,(36,1003),telemetry,21,'#78D2DD')
    drawtext(d,(36,1042),'Teaching layout · flat-floor prototype · ICP initial pose supplied · no ground-truth control input',19,'#B0C0CF')
    return im

encoder=subprocess.Popen(['ffmpeg','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r',str(FPS),'-i','-',
                          '-an','-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-threads','2','-movflags','+faststart',str(run/'recording.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=(run/'encoder.log').open('w'))
started=time.monotonic();nextframe=started;frames=0
try:
    while not (run/'record.stop').exists() and time.monotonic()-started<250:
        while time.monotonic()<nextframe:rclpy.spin_once(node,timeout_sec=.002)
        rclpy.spin_once(node,timeout_sec=0)
        encoder.stdin.write(render().tobytes());frames+=1;nextframe+=1/FPS
        if time.monotonic()-nextframe>.5:nextframe=time.monotonic()
finally:
    encoder.stdin.close();encoder.wait(timeout=20)
    (run/'video_metrics.json').write_text(json.dumps({'frames':frames,'fps':FPS,'video_duration_s':frames/FPS,'wall_duration_s':time.monotonic()-started,
                                                   'camera_received':data['camera'] is not None,'lidar_messages':data['lidar_count'],'imu_messages':data['imu_n'],'cloud_messages':data['cloud_count'],'encoder_exit':encoder.returncode},indent=2))
    node.destroy_node();rclpy.shutdown()
