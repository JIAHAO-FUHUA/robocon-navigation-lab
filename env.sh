#!/usr/bin/env bash
export ROS_DOMAIN_ID=73
export GZ_PARTITION=robocon_v3
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1
# Prefer the container's virtual display after start_virtual_display.sh.
export DISPLAY=${ROBOCON_DISPLAY:-:99}
source /opt/ros/jazzy/setup.bash
source /work/jazzy_ws/install/setup.bash
export LD_LIBRARY_PATH=/work/deps_install/lib:/work/deps_install/lib64:${LD_LIBRARY_PATH:-}
export GZ_SIM_RESOURCE_PATH=/work/jazzy_ws/install/livox_gz_sim/share:${GZ_SIM_RESOURCE_PATH:-}
