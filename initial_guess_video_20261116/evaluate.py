import csv
import json
import math
from pathlib import Path
import sys
import numpy as np
from core import CONFIG,ROOT,clearance_world,wrap

run=Path(sys.argv[1]);task=json.loads((run/'task_result.json').read_text())
rows=[r for r in csv.DictReader((run/'evaluation.csv').open()) if r['source']=='truth']
positions=np.array([[float(r['x']),float(r['y'])] for r in rows]);times=np.array([float(r['stamp']) for r in rows])
delta=np.diff(times);valid=delta>1e-6
speeds=np.linalg.norm(np.diff(positions,axis=0)[valid],axis=1)/delta[valid]
task['truth_speed_max_mps']=float(np.max(speeds)) if len(speeds) else 0.
task['truth_speed_p95_mps']=float(np.quantile(speeds,.95)) if len(speeds) else 0.
task['truth_path_length_m']=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum())
task['minimum_physical_clearance_m']=float(min(clearance_world(x,y) for x,y in positions))
task['collision_free_by_conservative_geometry']=task['minimum_physical_clearance_m']>0
for waypoint in task.get('waypoints_reached',[]):
    idx=int(np.argmin(np.abs(times-waypoint['sim_time'])))
    waypoint['truth_world']=positions[idx].tolist();waypoint['truth_error_m']=math.dist(positions[idx],waypoint['point_world'])
if task.get('waypoints_reached'):
    task['truth_final_error_m']=task['waypoints_reached'][-1]['truth_error_m']
if task['mode']=='relocalization_test':
    idx=len(rows)-1;r=rows[idx];qx,qy,qz,qw=[float(r[k]) for k in ['qx','qy','qz','qw']]
    yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));p=task['estimated_world_pose']
    task['truth_initial_position_error_m']=math.dist(p[:2],positions[-1]);task['truth_initial_yaw_error_deg']=abs(math.degrees(wrap(p[2]-yaw)))
if 'initial_estimate_world_pose' in task:
    idx=int(np.argmin(np.abs(times-task['initial_estimate_sim_stamp'])))
    r=rows[idx];qx,qy,qz,qw=[float(r[k]) for k in ['qx','qy','qz','qw']]
    truth_yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    est=task['initial_estimate_world_pose']
    task['truth_initial_world_pose']=[float(r['x']),float(r['y']),truth_yaw]
    task['initial_relocalization_position_error_m']=math.dist(est[:2],positions[idx])
    task['initial_relocalization_yaw_error_deg']=abs(math.degrees(wrap(est[2]-truth_yaw)))
    task['initial_relocalization_timestamp_difference_s']=float(abs(times[idx]-task['initial_estimate_sim_stamp']))
    task['measured_start_offset_from_S_m']=[float(r['x'])-CONFIG['start_world'][0],float(r['y'])-CONFIG['start_world'][1]]
    task['measured_start_yaw_deg']=math.degrees(truth_yaw)
(run/'validated_result.json').write_text(json.dumps(task,indent=2))
summary={k:v for k,v in task.items() if k not in ['astar','initial_health','waypoints_reached']}
if 'astar' in task:summary['planned_path_length_m']=task['astar']['length_m']
print(json.dumps(summary,indent=2))
