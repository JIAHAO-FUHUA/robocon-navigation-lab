# RTAB-Map RGB-D simulation

Use an Ubuntu 24.04 desktop with ROS 2 Jazzy. Open a separate Ubuntu terminal for this exercise.

## Install the packages

```bash
sudo apt update
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-turtlebot3-teleop ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz-image ros-jazzy-rviz2
```

## Build and save one map

```bash
cd ~/robocon-navigation-lab
source /opt/ros/jazzy/setup.bash
RTAB_OVERLAY=$(python3 extras/prepare_rtabmap.py)
export AMENT_PREFIX_PATH="$RTAB_OVERLAY:$AMENT_PREFIX_PATH"
export ROS_HOME="$(mktemp -d "$HOME/rtabmap_lab.XXXXXX")"
ros2 launch rtabmap_demos turtlebot3_sim_rgbd_demo.launch.py
```

The script copies the installed TurtleBot3 model into a local overlay and adds depth output and a spawn yaw option. In Gazebo, make sure the simulation is playing. Use 2D Nav Goal in RViz to move around an open section and turn back toward the start. You can also teleoperate in another terminal using `ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=/cmd_vel` after sourcing Jazzy and setting `TURTLEBOT3_MODEL=waffle`.

Press Ctrl+C and wait for Gazebo and RTAB-Map to exit. In the same terminal confirm the saved database with `ls -lh "$ROS_HOME/rtabmap.db"`.

## Reuse the saved map

Run the original start once.

```bash
ros2 launch rtabmap_demos turtlebot3_sim_rgbd_demo.launch.py localization:=true
```

Stop it and allow the processes to exit. Restart from the offset pose in the same terminal. The start changes from X −2.0 m, Y 0.5 m and yaw 0° to X −1.8 m, Y 0.7 m and yaw −15°.

```bash
ROBOCON_SPAWN_YAW=-0.261799 ros2 launch rtabmap_demos \
  turtlebot3_sim_rgbd_demo.launch.py localization:=true x_pose:=-1.8 y_pose:=0.7
```

Check the terminal for confirmation that RTAB-Map uses the saved database with `Mem/IncrementalMemory=false`. Wait for a visual match in RViz. Open a second terminal with `source /opt/ros/jazzy/setup.bash`. Check the estimate with `ros2 run tf2_ros tf2_echo map base_footprint` and the simulated pose with `gz model -m waffle -p`. The saved map starts at the robot’s original pose. Subtract the starting X and Y of the mapping run from Gazebo’s world coordinates before comparing them with the map. Keep that relation fixed for both restarts.

If an older Gazebo process is still open, close its window and stop its launch terminal before continuing. Keep the same ROS_HOME across both restarts. Save a copy of the database before starting a new mapping run.

The launch and Waffle camera configuration follow the [RTAB-Map simulation example](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/turtlebot3/turtlebot3_sim_rgbd_demo.launch.py).
