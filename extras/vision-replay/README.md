# Robocon Vision Replay Starter

Download and extract the [Vision Replay Starter ZIP](https://github.com/PolyU-FENG-AI-Robotics-Club/robocon-navigation-lab/releases/download/v1.0.0/Robocon_Vision_Replay_Starter.zip) on Ubuntu 24.04 x86-64. It includes two Gazebo recordings with left and right images, IMU readings, calibration and simulator truth. Both algorithms use the same recorded images. The simulator pose is used to evaluate the result.

## ORB-SLAM3

From the extracted folder, run:

```bash
bash run_orb.sh
```

Install Docker before the first run. The first run downloads and builds the pinned ORB-SLAM3 and Pangolin sources in a container. Allow time for the build and use the log if a dependency is unavailable. An NVIDIA GPU is optional. The Dockerfile identifies the pinned source commits. The local compatibility changes and replay driver add format handling and expose diagnostics for the map-reuse exercise.

The script prints a new folder under `orb_runs`. Check `student_map/map.osa`, `student_restart/poses.csv` and `student_offset/poses.csv`. A row with `state=2` indicates tracking is OK. `contains_old_anchor=1` indicates that the active map contains the original map reference. Check both before judging map reuse. A failed match is a valid result to report.

## cuVSLAM

Use a compatible NVIDIA GPU and driver, with Python 3.12 and `python3-venv` installed. In the extracted folder:

```bash
nvidia-smi
bash run_cuvslam.sh
```

The script installs the pinned cuVSLAM 17.0.1 CUDA 13 wheel and dependencies in a private Python environment. The first install needs Internet access. It replays the Gazebo recordings with stereo and IMU, saves a native map and checks both restart poses. Look in the printed `cuvslam_runs` folder for `results.json`. The replay uses Gazebo data and the standalone SDK. For the Isaac Sim extension, use the hardware and software listed in the NVIDIA tutorial.
