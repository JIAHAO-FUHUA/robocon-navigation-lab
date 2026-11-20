import argparse
import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from core import ROOT,CONFIG
sys.path.insert(0,'/work/video_demo_20261113')
from story import mark

ap=argparse.ArgumentParser();ap.add_argument('mode');ap.add_argument('--record',action='store_true');args=ap.parse_args()
root=Path('/work')
for required in ('mobile_sensor.sdf', 'mobile_sensor.urdf'):
    if not (root / required).is_file():
        raise SystemExit(f'Missing {required}. Run bash /work/scripts/prepare_sim.sh first.')
run=ROOT/'sessions'/f'{args.mode}_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}'
run.mkdir(parents=True);(ROOT/f'latest_{args.mode}.txt').write_text(str(run));shutil.copy2(ROOT/'config.json',run/'config.json')
procs=[];files=[];recorder=None
def start(name,cmd):
    f=(run/(name+'.log')).open('w');files.append(f)
    p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append((name,p));return p
def clean():
    if recorder and recorder.poll() is None:
        (run/'record.stop').touch()
        try:recorder.wait(timeout=20)
        except subprocess.TimeoutExpired:pass
    for name,p in reversed(procs):
        if p.poll() is None:
            try:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
    for name,p in reversed(procs):
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:os.killpg(p.pid,signal.SIGTERM)
            except ProcessLookupError:pass
    for f in files:f.close()
try:
    subprocess.run(['python3',str(ROOT/'make_scene.py'),str(run),args.mode],check=True)
    shutil.copy2(root/'config/demo_bridge.yaml',run/'bridge.yaml')
    start('gazebo',['gz','sim','-s','-r',str(run/'world.sdf')])
    start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(run/'bridge.yaml')])
    start('rsp',['ros2','run','robot_state_publisher','robot_state_publisher','/work/mobile_sensor.urdf','--ros-args','-p','use_sim_time:=true'])
    start('static_tf',['ros2','run','tf2_ros','static_transform_publisher','--x','-0.011','--y','-0.02329','--z','-0.25288','--frame-id','localization_imu','--child-frame-id','base_link'])
    time.sleep(3)
    if args.record:
        if args.mode=='mapping':
            mark(run,0,'01  S → G  |  起点、目的地与障碍',
                 '先沿青色路线绕场一圈建图，回到S；换向重定位后，再用A*绕障到G。',
                 'Map one full loop back to S; restart with a new heading, then use A* to reach G.')
        else:
            yaw=float(os.environ.get('COMPLEX_YAW_DEG',CONFIG['restart_yaw_deg']))
            mark(run,3,f'03  Restart at {yaw:.0f}°  |  原点附近换向启动',
                 f'起点附近重新启动，朝向改为{yaw:.0f}°；目的地G保持不变。',
                 f'Restart near S with a {yaw:.0f}° heading. Destination G is unchanged.')
        recorder=start('recorder',['python3',str(ROOT/'record.py'),'--run',str(run),'--mode','mapping' if args.mode=='mapping' else 'navigation'])
        time.sleep(7)
        if args.mode=='mapping':
            mark(run,1,'01  Sensors ready  |  激光与IMU',
                 'MID360激光10Hz、IMU200Hz；点云来自当前场景，控制采用估计位姿。',
                 'MID360 LiDAR at 10Hz and IMU at 200Hz. Control uses the estimated pose.')
            time.sleep(4)
    start('bag',['ros2','bag','record','-o',str(run/'raw'),'/clock','/livox/lidar','/livox/imu','/tf','/tf_static','/club/cmd_vel','/fastlio2/lio_odom','/club/localization_health','/evaluation/ground_truth','/club/planned_path','/club/localized_pose','/pgo/loop_markers'])
    cfg=root/'robocon_linux_training_v3/config'
    lio=(cfg/'lio_sim.yaml').read_text().replace('lidar_filter_num: 6','lidar_filter_num: 2').replace('scan_resolution: 0.15','scan_resolution: 0.08').replace('map_resolution: 0.3','map_resolution: 0.15')
    (run/'lio.yaml').write_text(lio)
    mapping=args.mode=='mapping'
    module=(cfg/('pgo_sim.yaml' if mapping else 'localizer_sim.yaml')).read_text()
    if mapping:module=module.replace('loop_time_tresh: 60.0','loop_time_tresh: 8.0')
    (run/'module.yaml').write_text(module)
    start('lio',['ros2','launch','club_bringup','training.launch.py','mode:='+('mapping' if mapping else 'localization'),
                 'lio_config:='+str(run/'lio.yaml'),'module_config:='+str(run/'module.yaml'),'use_sim_time:=true','rviz:=false'])
    start('evaluation',['python3','/work/record_evaluation.py',str(run)])
    cmd=['python3',str(ROOT/'control.py'),args.mode,'--run',str(run)]
    if args.record:cmd.append('--record')
    result=subprocess.run(cmd)
    (run/'run_status.json').write_text(json.dumps({'returncode':result.returncode,'run':str(run)}))
    print('COMPLEX RUN',run,'RETURN',result.returncode,flush=True)
    if result.returncode:raise SystemExit(result.returncode)
finally:clean()
