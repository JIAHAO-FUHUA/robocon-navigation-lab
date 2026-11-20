#!/usr/bin/env bash
set -e
source /work/env.sh
cd /work
mkdir -p logs

if [[ ! -s /work/course_map/map.pcd ]]; then
  printf 'Missing /work/course_map/map.pcd. Check the course download.\n' >&2
  exit 1
fi
xacro mobile_robot.urdf.xacro > mobile_sensor.urdf
gz sdf -p mobile_sensor.urdf > mobile_sensor.sdf
test -s mobile_sensor.urdf && test -s mobile_sensor.sdf

for demo in complex_demo_20261113 initial_guess_video_20261116; do
  if [[ ! -e "$demo/latest_map.txt" ]]; then
    printf '%s\n' /work/course_map/map.pcd > "$demo/latest_map.txt"
  fi
done
printf '%s\n' 'Simulation model and map pointers ready.'
