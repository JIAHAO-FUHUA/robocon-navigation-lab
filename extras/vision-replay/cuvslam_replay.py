"""Create a fresh cuVSLAM map and test two restarts on Gazebo camera data."""
import argparse
import csv
import json
from pathlib import Path
import threading
import time

import cuvslam as cu
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def transform(rotation, position):
    output = np.eye(4)
    output[:3, :3] = rotation
    output[:3, 3] = position
    return output


def pose_matrix(pose):
    return transform(Rotation.from_quat(pose.rotation).as_matrix(), pose.translation)


def tracker(calibration):
    cameras = []
    for offset in (0.0, calibration["baseline_m"]):
        camera = cu.Camera(
            size=(calibration["width"], calibration["height"]),
            focal=(calibration["fx"], calibration["fy"]),
            principal=(calibration["cx"], calibration["cy"]),
        )
        camera.rig_from_camera.translation[0] = offset
        cameras.append(camera)
    imu = cu.ImuCalibration()
    imu.rig_from_imu = cu.Pose(
        translation=calibration["rig_from_imu_translation_m"],
        rotation=calibration["rig_from_imu_quaternion_xyzw"],
    )
    imu.frequency = calibration["imu_hz"]
    imu.accelerometer_noise_density = 2e-3
    imu.gyroscope_noise_density = 1.6968e-4
    imu.accelerometer_random_walk = 3e-3
    imu.gyroscope_random_walk = 1.9393e-5
    return cu.Tracker(
        cu.Rig(cameras, [imu]),
        cu.Tracker.OdometryConfig(
            async_sba=False,
            rectified_stereo_camera=True,
            odometry_mode=cu.Tracker.OdometryMode.Inertial,
        ),
        cu.Tracker.SlamConfig(sync_mode=True, planar_constraints=True),
    )


def run_frames(dataset, slam):
    imu_samples = np.loadtxt(dataset / "imu.csv", delimiter=",", skiprows=1)
    imu_index = 0
    with (dataset / "frames.csv").open(newline="") as handle:
        for row in csv.DictReader(handle):
            stamp = int(row["timestamp_ns"])
            while imu_index < len(imu_samples) and imu_samples[imu_index, 0] <= stamp:
                sample = imu_samples[imu_index]
                imu = cu.ImuMeasurement()
                imu.timestamp_ns = int(sample[0])
                imu.angular_velocities = sample[1:4]
                imu.linear_accelerations = sample[4:7]
                slam.register_imu_measurement(0, imu)
                imu_index += 1
            images = [cv2.imread(str(dataset / row[side]), cv2.IMREAD_GRAYSCALE) for side in ("left", "right")]
            if any(image is None for image in images):
                raise RuntimeError(f"Missing image in {dataset}, timestamp {stamp}")
            odom, pose = slam.track(stamp, images)
            if pose is None and odom.world_from_rig is not None:
                pose = odom.world_from_rig.pose
            yield stamp, images, pose


def truth_at(dataset, stamp):
    poses = np.loadtxt(dataset / "truth.csv", delimiter=",", skiprows=1)
    i = np.argmin(np.abs(poses[:, 0] - stamp))
    return transform(Rotation.from_quat(poses[i, 4:8]).as_matrix(), poses[i, 1:4])


def save_map(source, base_from_camera, map_dir, calibration):
    slam = tracker(calibration)
    first_reference = None
    for index, (stamp, _, pose) in enumerate(run_frames(source, slam)):
        if index % 400 == 0:
            print("Mapping frame", index, flush=True)
        if pose is not None and first_reference is None:
            first_reference = stamp, pose_matrix(pose)
    if first_reference is None:
        raise RuntimeError("The mapping replay produced no camera pose")
    saved = threading.Event()
    success = []
    slam.save_map(str(map_dir), lambda ok: (success.append(bool(ok)), saved.set()))
    if not saved.wait(30) or not success[0]:
        raise RuntimeError("Could not save the map")
    stamp, first_pose = first_reference
    # This alignment is established once from the mapping start, for evaluation.
    world_from_map = truth_at(source, stamp) @ base_from_camera @ np.linalg.inv(first_pose)
    return world_from_map


def test_restart(dataset, base_from_camera, world_from_map, map_dir, calibration):
    slam = tracker(calibration)
    completed = threading.Event()
    reply = {}
    settings = cu.Tracker.SlamLocalizationSettings(
        horizontal_search_radius=0.8, vertical_search_radius=0.4,
        horizontal_step=0.1, vertical_step=0.1, angular_step_rads=0.04,
    )

    def receive(pose, error):
        reply["pose"] = None if pose is None else pose_matrix(pose)
        reply["error"] = error
        completed.set()

    request = None
    for stamp, images, pose in run_frames(dataset, slam):
        if request is None and stamp > 2_000_000_000 and pose is not None:
            request = stamp, time.perf_counter()
            guess = cu.Pose(translation=[0, 0, 0], rotation=[0, 0, 0, 1])
            slam.localize_in_map(str(map_dir), stamp, guess, images, settings, lambda: None, receive)
        if completed.wait(0.005):
            break
    if request is None:
        raise RuntimeError("No valid restart pose before requesting localisation")
    if not completed.is_set() or reply["pose"] is None:
        return {"failure": reply.get("error") or "No map match"}

    target_stamp, requested_at = request
    estimated_base = world_from_map @ reply["pose"] @ np.linalg.inv(base_from_camera)
    truth = truth_at(dataset, target_stamp)
    xy_error = float(np.linalg.norm(estimated_base[:2, 3] - truth[:2, 3]))
    yaw_error = abs(float(Rotation.from_matrix(truth[:3, :3].T @ estimated_base[:3, :3]).as_euler("xyz",degrees=True)[2]))
    return {
        "request_stamp_ns": target_stamp,
        "actual_position_m": truth[:2, 3].tolist(),
        "estimated_position_m": estimated_base[:2, 3].tolist(),
        "position_error_cm": 100 * xy_error,
        "heading_error_deg": yaw_error,
        "observed_wait_s": time.perf_counter() - requested_at,
        "passes": xy_error <= 0.2 and yaw_error <= 5,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("calibration", type=Path)
    parser.add_argument("mapping_recording", type=Path)
    parser.add_argument("offset_recording", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    calibration = json.loads(args.calibration.read_text())
    optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    base_from_camera = transform(
        Rotation.from_euler("y", calibration["camera_pitch_rad"]).as_matrix() @ optical,
        calibration["base_from_left_camera_ros_translation"],
    )
    map_dir = args.output / "native_map"
    world_from_map = save_map(args.mapping_recording, base_from_camera, map_dir, calibration)
    results = {}
    for label, recording in [("reference", args.mapping_recording), ("offset", args.offset_recording)]:
        results[label] = test_restart(recording, base_from_camera, world_from_map, map_dir, calibration)
    summary = json.dumps(results, indent=2)
    (args.output / "results.json").write_text(summary + "\n")
    print(summary)


if __name__ == "__main__":
    main()
