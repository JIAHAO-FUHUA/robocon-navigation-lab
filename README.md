# Robocon 2027 Navigation Lab

Practice mapping, restarting and driving a simulated robot around an obstacle. Then adapt the same workflow to the Builder Robot (BR) route to the handover area and two tower positions.

## 1. Get the course code and start Ubuntu

Download this repository from GitHub or extract the course ZIP. Put the project in your Ubuntu home folder as `~/robocon-navigation-lab`. The folder that contains this README also contains `scripts/`, `config/` and the two demo folders. Use Ubuntu 24.04 with Docker and internet access for the first build. The container uses ROS 2 Jazzy and Gazebo Harmonic.

Run these commands in the **Ubuntu host** terminal. If Docker is already installed, start with `sudo docker run --rm hello-world`.

```bash
sudo apt update
sudo apt install -y docker.io git unzip
sudo systemctl enable --now docker
sudo docker run --rm hello-world
```

Check that you see `Hello from Docker`. From the project folder, create the container once.

```bash
cd ~/robocon-navigation-lab
sudo docker run -it --name robocon-intro --shm-size=1g \
  -v "$PWD:/work" -w /work ros:jazzy-ros-base-noble bash
```

After the prompt changes, run these commands **inside the container**.

```bash
pwd
bash /work/scripts/setup_course.sh
```

`pwd` should show `/work`. Setup downloads the upstream sources, builds the workspace, applies the course compatibility changes and generates the robot model. Wait for `COURSE SETUP COMPLETE`. If setup stops, find the first error before continuing. Ask for a prepared classroom machine if the first build runs beyond the session.

On a later day, reopen the container from the **Ubuntu host**.

```bash
sudo docker start robocon-intro
sudo docker exec -it robocon-intro bash
```

Run the remaining commands **inside the container**. Start a virtual display before the first simulation.

```bash
source /work/env.sh
ros2 pkg executables fastlio2
ros2 pkg executables pgo
ros2 pkg executables localizer
bash /work/start_virtual_display.sh
```

Look for `lio_node`, `pgo_node`, `localizer_node` and the OpenGL renderer. Use one simulation at a time. The virtual display allows the simulator to run in the background and save a video.

### Upstream code you download

Setup runs `scripts/download_upstream.sh` after installing Git. Read that script to see the exact repositories, checkout directories and commit IDs. The full developer repositories are downloaded into `jazzy_ws/` on your computer.

| Upstream project | What it provides | Revision |
| --- | --- | --- |
| [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2) | LiDAR/IMU mapping, pose graph and ICP localizer | `7baa59f7339ba88ce3b646fe35b23f61031c5d3f` |
| [livox_gz_sim](https://github.com/mathaimp/livox_gz_sim) | Simulated MID-360 LiDAR and IMU | `3f06f1ac4d3f6854f7275ca9aa097484791a1abd` |
| [Sophus](https://github.com/strasdat/Sophus) | Pose mathematics | `de0f8d3d92bf776271e16de56d1803940ebccab9` |
| [GTSAM](https://github.com/borglab/gtsam) | Pose graph optimization | `c57988fe554e7213c77fe379c1d7c483de26ad33` |
| [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) | Two ROS message definitions for simulated scans | `21445540f0d100dc86a7e6df312dd70bbdb4afdf` |

The course includes Gazebo scene generation, the A* grid and position controller, the simulation runner, the dashboard, the saved teaching map and compatibility scripts. The Livox simulation helper builds the two downloaded messages and retains the upstream license. It uses no physical LiDAR in this lab. See the [Gazebo Harmonic and ROS compatibility guide](https://gazebosim.org/docs/harmonic/ros_installation/) for the platform pairing.

## 2. Run the first navigation test with the course map

Use the original field and the original configuration for the first run. This restores the map pointer to the included teaching map.

```bash
source /work/env.sh
export ROS_DOMAIN_ID=79
export GZ_PARTITION=robocon_bias_video
cd /work/initial_guess_video_20261116
printf '%s\n' /work/course_map/map.pcd > latest_map.txt
python3 run.py navigation --record
```

The robot starts 0.20 m away from S on both axes, turned 15° clockwise. ICP receives a zero guess relative to S and aligns the scan with the existing map. Then A* plans and position feedback moves the robot. Watch for `RETURN 0`.

## 3. Build your own map and navigate on it

```bash
source /work/env.sh
export ROS_DOMAIN_ID=76
export GZ_PARTITION=robocon_complex
cd /work/complex_demo_20261113
python3 run.py mapping --record
```

When you see `RETURN 0`, run `cat latest_map.txt`. The returned path should end in `map.pcd`. The next block copies this pointer to the navigation demo and runs the same ICP → A* → controller sequence.

```bash
source /work/env.sh
export ROS_DOMAIN_ID=79
export GZ_PARTITION=robocon_bias_video
cd /work/initial_guess_video_20261116
cp /work/complex_demo_20261113/latest_map.txt .
python3 run.py navigation --record
```

## 4. Check the results

After the run, evaluate it while using the same scene settings.

```bash
cd /work/initial_guess_video_20261116
run_dir=$(cat latest_navigation.txt)
printf '%s\n' "$run_dir"
python3 evaluate.py "$run_dir"
```

- `status: COMPLETE` means the script completed the route. Check `truth_final_error_m` for the distance to the goal and `minimum_physical_clearance_m` for the evaluated clearance. Simulated ground truth measures the result. The controller uses the estimated pose.
- Open `recording.mp4` and find the orange plan and blue trail. On the Ubuntu host, replace `/work` in the printed path with `~/robocon-navigation-lab`.
- If a run stops, read the terminal error and `task_result.json` if available. The Gazebo and localization logs are in the same run folder. Record the first error with the run path.

Next use [Change the field and plan the BR exercise](docs/change-field.md) for the target, obstacle and checkpoint activities.
