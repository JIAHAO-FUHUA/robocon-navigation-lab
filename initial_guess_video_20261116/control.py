"""Full-loop mapping or obstacle-aware navigation, estimated pose only."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry,Path as NavPath
from std_msgs.msg import String
from livox_ros_driver2.msg import CustomMsg
from interface.srv import Relocalize,SaveMaps
from core import CONFIG,ROOT,rotation,plan_world,profile,wrap,grid
import sys
sys.path.insert(0,'/work/video_demo_20261113')
from story import mark

ap=argparse.ArgumentParser();ap.add_argument('mode');ap.add_argument('--run',type=Path,required=True);ap.add_argument('--record',action='store_true');args=ap.parse_args()
run=args.run;rclpy.init();node=Node('complex_estimated_pose_control',parameter_overrides=[Parameter('use_sim_time',value=True)])
state={'odom':None,'wall':0.,'scan':-1.,'health':None}
pub=node.create_publisher(Twist,'/club/cmd_vel',10)
posepub=node.create_publisher(PoseStamped,'/club/localized_pose',10)
statuspub=node.create_publisher(String,'/club/task_status',10)
pathpub=node.create_publisher(NavPath,'/club/planned_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
def stamp(s):return s.sec+s.nanosec/1e9
def odom(m):state.update(odom=m,wall=time.monotonic())
node.create_subscription(Odometry,'/fastlio2/lio_odom',odom,1)
node.create_subscription(CustomMsg,'/livox/lidar',lambda m:state.update(scan=stamp(m.header.stamp)),1)
node.create_subscription(String,'/club/localization_health',lambda m:state.update(health=json.loads(m.data)),10)
def now():return node.get_clock().now().nanoseconds/1e9
def spin():rclpy.spin_once(node,timeout_sec=.01)
def command(vx=0.,vy=0.,w=0.):
    msg=Twist();msg.linear.x=float(vx);msg.linear.y=float(vy);msg.angular.z=float(w);pub.publish(msg)
def pose():
    m=state['odom'];p=m.pose.pose.position;q=m.pose.pose.orientation
    r=rotation([q.x,q.y,q.z,q.w]);xyz=np.array([p.x,p.y,p.z])
    if args.mode!='mapping':
        h=state['health'];rr=rotation(h['offset_quaternion']);xyz=rr@xyz+np.array(h['offset_translation']);r=rr@r
    xyz-=r@np.array([.011,.02329,.25288]);xyz-=np.array(CONFIG['map_from_world_translation'])
    return xyz[0],xyz[1],math.atan2(r[1,0],r[0,0])
def hold(seconds,step,title,cn,en):
    mark(run,step,title,cn,en);end=time.monotonic()+seconds;last=0
    while time.monotonic()<end:
        if time.monotonic()-last>.1:command();last=time.monotonic()
        spin()
def publish_route(pts):
    msg=NavPath();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
    tx,ty,_=CONFIG['map_from_world_translation']
    for x,y in pts:
        p=PoseStamped();p.header=msg.header;p.pose.position.x=x+tx;p.pose.position.y=y+ty;p.pose.orientation.w=1.;msg.poses.append(p)
    pathpub.publish(msg)

handle=(run/'control.csv').open('w');writer=csv.writer(handle);writer.writerow(['sim_time','world_x_est','world_y_est','yaw_est','vx_body','vy_body','wz','segment','tracking_error','pose_age'])
result={'status':'NOT_STARTED','mode':args.mode,'cruise_target_mps':2.,'truth_used_for_control':False,'waypoints_reached':[]}
try:
    deadline=time.monotonic()+60
    while (state['odom'] is None or now()-stamp(state['odom'].header.stamp)>.15) and time.monotonic()<deadline:spin()
    if state['odom'] is None:raise RuntimeError('No LIO odometry')
    if args.mode=='mapping':
        points=CONFIG['mapping_loop_world'];publish_route(points)
        hold(2,2,'02  Full-loop mapping  |  2 m/s绕场建图','青色为预定建图路线；直线巡航2m/s，转角与到点前减速。','Follow one complete loop: 2 m/s on straights, decelerating at turns.')
    else:
        if args.record:hold(8,3,'Actual start differs from the ICP seed','','Actual offset: (+0.20, +0.20, 0.00) m, yaw -15 degrees. ICP receives a zero position and zero orientation.')
        client=node.create_client(Relocalize,'/localizer/relocalize')
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('No ICP service')
        req=Relocalize.Request();req.pcd_path=(ROOT/'latest_map.txt').read_text().strip()
        # Deliberately fixed at zero. Do not derive this seed from the true spawn.
        angle=0.0
        req.x=0.0;req.y=0.0;req.z=0.0
        req.roll=0.0;req.pitch=0.0;req.yaw=0.0
        seed={k:float(getattr(req,k)) for k in ('x','y','z','roll','pitch','yaw')}
        assert all(value == 0.0 for value in seed.values())
        (run/'initial_guess_request.json').write_text(json.dumps(seed,indent=2))
        result['icp_initial_guess_request']=seed
        result['requested_case']=CONFIG['test_case']
        if args.record:mark(run,3,'ICP starts from a zero initial guess','','Align the current LiDAR scans to the saved map. The actual start pose is not sent to ICP.')
        relocalization_started=time.monotonic()
        future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=15)
        if not future.result() or not future.result().success:raise RuntimeError('ICP request rejected')
        until=time.monotonic()+30
        while not (state['health'] and state['health']['match_ok']) and time.monotonic()<until:spin()
        if not state['health'] or not state['health']['match_ok']:raise RuntimeError('ICP failed')
        result['relocalization_first_success_wall_s']=time.monotonic()-relocalization_started
        # Stationary startup check. No true spawn or Gazebo truth enters this gate.
        until=time.monotonic()+30.0
        last_match=-1.0;last_candidate=None;stable_pairs=0;last_zero=0.0
        samples=[]
        while time.monotonic()<until:
            if time.monotonic()-last_zero>.1:
                command();last_zero=time.monotonic()
            spin();h=state['health']
            if not h or not h['match_ok'] or h['request_id']!=1:continue
            match=h['last_success_stamp']
            if match<=last_match or now()-match>1.5:continue
            if now()-state['scan']>.30 or now()-stamp(state['odom'].header.stamp)>.30:continue
            candidate=np.array(pose());last_match=match
            if last_candidate is not None:
                consistent=np.linalg.norm(candidate[:2]-last_candidate[:2])<.02 and abs(wrap(candidate[2]-last_candidate[2]))<math.radians(.5)
                stable_pairs=stable_pairs+1 if consistent else 0
            last_candidate=candidate
            samples.append({'scan_stamp':match,'estimated_world_pose':candidate.tolist(),'consecutive_stable_pairs':stable_pairs})
            if stable_pairs>=2:break
        else:raise RuntimeError('ICP_STARTUP_NOT_STABLE')
        result['startup_verification_samples']=samples
        result['startup_gate']='Two consecutive stable changes across at least three distinct fresh ICP matches; <2cm and <0.5deg; no truth feedback.'
        result['startup_verified_wall_s']=time.monotonic()-relocalization_started

        request_id=state['health']['request_id'];result['initial_health']=state['health']
        result['actual_spawn_yaw_deg']=CONFIG['restart_yaw_deg']
        result['initial_estimate_world_pose']=list(pose())
        result['initial_estimate_sim_stamp']=stamp(state['odom'].header.stamp)
        (run/'startup_result.json').write_text(json.dumps({k:result[k] for k in ('icp_initial_guess_request','initial_estimate_world_pose','initial_estimate_sim_stamp','startup_verification_samples','startup_verified_wall_s')},indent=2))

        if args.mode=='relocalization_test':
            hold(4 if args.record else 2,3,f'ICP matched at {math.degrees(angle):.0f}°  |  换向重定位成功',
                 f'原点附近以{math.degrees(angle):.0f}°启动，已对齐固定地图；此段只验证重定位。',
                 f'Heading {math.degrees(angle):.0f}° relocalization verified. This clip tests relocalization only.')
            result['status']='COMPLETE'
            result['estimated_world_pose']=list(pose());raise SystemExit(0)
        if args.record:hold(8,3,'Relocalized from an inaccurate initial guess','','The fresh ICP matches are consistent. Compare the green estimated pose with the blue actual start before moving.')
        x,y,_=pose();points,raw=plan_world((x,y),CONFIG['goal_world'])
        straight_clear=grid().line((x+5.5,y+5.5),(CONFIG['goal_world'][0]+5.5,CONFIG['goal_world'][1]+5.5))
        assert not straight_clear,'Demonstration requires a blocked direct route'
        result['astar']={'world_path':points,'raw_world_path':raw,'direct_line_clear':straight_clear,
                         'length_m':sum(math.dist(a,b) for a,b in zip(points,points[1:]))}
        (run/'planned_path.json').write_text(json.dumps(result['astar'],indent=2));publish_route(points)
        if args.record:hold(6,4,'A* plans a route around the obstacles','','The direct route is blocked. The orange A* path goes around the platform and the red obstacle to G.')
    # Planning is CPU work: drain queues and require a new sensor/pose pair before motion.
    until=time.monotonic()+10
    while time.monotonic()<until:
        spin()
        if state['odom'] is not None and 0<=now()-stamp(state['odom'].header.stamp)<.15 and 0<=now()-state['scan']<.15:break
    else:raise RuntimeError('No fresh sensor/pose pair after planning')
    mark(run,2 if args.mode=='mapping' else 5,'02  Mapping at 2 m/s  |  完成整圈' if args.mode=='mapping' else '05  Navigate to G  |  沿A*路线自动到点',
         '蓝色为实测轨迹，数字显示速度；整圈建图后回到起点S。' if args.mode=='mapping' else '对照橙色规划路线和蓝色实际轨迹，观察机器人绕开障碍到达G。',
         'Blue: measured robot trail. Complete the loop and return to S.' if args.mode=='mapping' else 'Compare the orange plan with the blue measured trail as the robot reaches G.')
    start_time=now();start_wall=time.monotonic();last_time=now();previous=np.zeros(2);last_status=-1.
    kp=2.3;vmax=CONFIG['cruise_mps'];acc=CONFIG['reference_acceleration_mps2'];max_tracking=0.
    for segment,(a,b) in enumerate(zip(points,points[1:])):
        a=np.array(a);b=np.array(b);direction=(b-a)/np.linalg.norm(b-a);length=float(np.linalg.norm(b-a));begin=now();stable=0.
        while True:
            spin();t=now();dt=t-last_time
            if time.monotonic()-state['wall']>.5:raise RuntimeError('ODOM_WALL_TIMEOUT')
            if t<last_time:raise RuntimeError('CLOCK_RESET')
            if dt<.019:continue
            last_time=t
            if time.monotonic()-start_wall>240:raise RuntimeError('TASK_TIMEOUT')
            age=t-stamp(state['odom'].header.stamp)
            if age<0 or age>.30:raise RuntimeError(f'POSE_STALE age={age:.3f}')
            if t-state['scan']>.30:raise RuntimeError('SCAN_STALE')
            if args.mode!='mapping':
                h=state['health']
                if not h['match_ok'] or h['request_id']!=request_id or t-h['last_success_stamp']>2.5:raise RuntimeError('ICP_NOT_HEALTHY')
            x,y,yaw=pose();distance,ref_speed,total=profile(length,t-begin,vmax,acc)
            # Compare the measured pose to the reference at the measurement's timestamp.
            # At 2m/s, a 0.1s-old pose must not be treated as a current-position error of 0.2m.
            measured_distance,_,_=profile(length,max(0.,stamp(state['odom'].header.stamp)-begin),vmax,acc)
            ref=a+direction*measured_distance;err=ref-np.array([x,y]);max_tracking=max(max_tracking,float(np.linalg.norm(err)))
            velocity=direction*ref_speed+kp*err
            norm=np.linalg.norm(velocity)
            if norm>vmax:velocity*=vmax/norm
            delta=velocity-previous;limit=CONFIG['command_acceleration_mps2']*min(dt,.05)
            if np.linalg.norm(delta)>limit:delta*=limit/np.linalg.norm(delta)
            previous+=delta;c,s=math.cos(yaw),math.sin(yaw)
            vx,vy=c*previous[0]+s*previous[1],-s*previous[0]+c*previous[1]
            omega=max(-.8,min(.8,2.*wrap(-yaw)))
            command(vx,vy,omega)
            if t-last_status>.2:
                msg_status=String();msg_status.data=json.dumps({'segment':segment,'segments':len(points)-1,'reference_speed':ref_speed,'command_speed':float(np.linalg.norm(previous)),'waypoint_world':b.tolist()});statuspub.publish(msg_status);last_status=t
            writer.writerow([t,x,y,yaw,vx,vy,omega,segment,float(np.linalg.norm(err)),age]);handle.flush()
            msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=state['odom'].header.stamp
            msg.pose.position.x=x+CONFIG['map_from_world_translation'][0];msg.pose.position.y=y+CONFIG['map_from_world_translation'][1]
            msg.pose.orientation.z=math.sin(yaw/2);msg.pose.orientation.w=math.cos(yaw/2);posepub.publish(msg)
            od=state['odom'].twist.twist.linear;speed=math.hypot(od.x,od.y)
            error=float(np.linalg.norm(b-np.array([x,y])))
            if t-begin>total and error<.06 and speed<.08 and abs(wrap(yaw))<.05:stable+=dt
            else:stable=0.
            if stable>.3:break
        command();previous[:]=0
        result['waypoints_reached'].append({'segment':segment,'point_world':b.tolist(),'estimated_error_m':error,'sim_time':t})
    command();result['elapsed_sim_s']=now()-start_time;result['max_reference_tracking_error_m']=max_tracking
    result['estimated_final_world_pose']=list(pose());result['status']='COMPLETE'
    if args.mode=='mapping':
        hold(3,2,'02  Loop complete  |  回到S并保存地图','整圈已完成并回到起点，等待最后点云进入PGO，再保存map.pcd。','Loop complete at S. Save the sensor-generated map for the restart test.')
        directory=run/'map';directory.mkdir()
        client=node.create_client(SaveMaps,'/pgo/save_maps')
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('No save service')
        req=SaveMaps.Request();req.file_path=str(directory);req.save_patches=False
        future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=30)
        if not future.result() or not future.result().success or not (directory/'map.pcd').exists():raise RuntimeError('Map save failed')
        (ROOT/'latest_map.txt').write_text(str(directory/'map.pcd'))
        result['map_pcd']=str(directory/'map.pcd')
        if args.record:hold(5,2,'02  Map saved  |  整圈地图已保存','下一步从S附近改变朝向启动，使用刚保存的地图重定位。','Next: restart near S with a different heading and relocalize in this map.')
    elif args.record:
        hold(8,6,'Goal reached after correcting the initial pose','','The robot stops at G. Compare the orange planned route with the blue measured trajectory.')
except Exception as exc:
    result['status']='FAILED';result['error']=str(exc)
finally:
    for _ in range(5):command();spin()
    (run/'task_result.json').write_text(json.dumps(result,indent=2))
    summary={k:v for k,v in result.items() if k not in ['astar','initial_health']}
    if 'astar' in result:summary['astar_summary']={'segments':len(result['astar']['world_path'])-1,'length_m':result['astar']['length_m'],'direct_line_clear':result['astar']['direct_line_clear']}
    print(json.dumps(summary,indent=2),flush=True)
    handle.close();node.destroy_node();rclpy.shutdown()
if result['status']!='COMPLETE':raise SystemExit(2)
