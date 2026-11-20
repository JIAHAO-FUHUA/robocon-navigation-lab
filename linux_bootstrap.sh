#!/usr/bin/env bash
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
export ROBOCON_HANDOFF=/work/robocon_linux_training_v3
export ROBOCON_WS=/work/jazzy_ws
export ROBOCON_DEPS=/work/deps_install
mkdir -p /work/logs "$ROBOCON_WS/src" "$ROBOCON_WS/deps" "$ROBOCON_DEPS"
echo 'STAGE 1 apt dependencies'
apt-get update
apt-get install -y --no-install-recommends build-essential cmake git pkg-config libeigen3-dev libpcl-dev libyaml-cpp-dev libboost-all-dev libtbb-dev libfmt-dev python3-dev python3-colcon-common-extensions python3-numpy mesa-utils xvfb xauth ros-jazzy-ros-gz ros-jazzy-gz-sim-vendor ros-jazzy-gz-transport-vendor ros-jazzy-gz-msgs-vendor ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-pcl-conversions ros-jazzy-message-filters ros-jazzy-rviz2 ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-default-plugins
source /opt/ros/jazzy/setup.bash
echo 'STAGE 2 sources'
bash /work/scripts/download_upstream.sh
driver="$ROBOCON_WS/src/livox_ros_driver2"
mkdir -p "$driver/msg"
install -m 644 "$ROBOCON_HANDOFF/ros2/livox_ros_driver2/"{CMakeLists.txt,package.xml,NOTICE.txt} "$driver/"
install -m 644 "$ROBOCON_WS/deps/livox_ros_driver2_upstream/msg/"{CustomMsg.msg,CustomPoint.msg} "$driver/msg/"
install -m 644 "$ROBOCON_WS/deps/livox_ros_driver2_upstream/LICENSE.txt" "$driver/LICENSE.upstream.txt"
if [ ! -d "$ROBOCON_WS/src/club_bringup" ]; then
  cp -a "$ROBOCON_HANDOFF/ros2/club_bringup" "$ROBOCON_WS/src/"
fi
python3 "$ROBOCON_HANDOFF/tools/patch_build.py" --source "$ROBOCON_WS/src/FASTLIO2_ROS2" --report /work/logs/build_compatibility.patch
python3 "$ROBOCON_HANDOFF/tools/patch_runtime.py" --source "$ROBOCON_WS/src/FASTLIO2_ROS2" --report /work/logs/imu_unit.patch
echo 'STAGE 3 sensor build'
cd "$ROBOCON_WS"
colcon build --base-paths src --symlink-install --executor sequential --packages-up-to livox_gz_sim --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$ROBOCON_WS/install/setup.bash"
echo 'STAGE 4 Sophus'
cmake -S "$ROBOCON_WS/deps/Sophus" -B "$ROBOCON_WS/deps/Sophus/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$ROBOCON_DEPS" -DSOPHUS_USE_BASIC_LOGGING=ON -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF
cmake --build "$ROBOCON_WS/deps/Sophus/build" -j2
cmake --install "$ROBOCON_WS/deps/Sophus/build"
echo 'STAGE 5 GTSAM'
cmake -S "$ROBOCON_WS/deps/gtsam" -B "$ROBOCON_WS/deps/gtsam/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$ROBOCON_DEPS" -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
cmake --build "$ROBOCON_WS/deps/gtsam/build" -j2
cmake --install "$ROBOCON_WS/deps/gtsam/build"
echo 'STAGE 6 LIO PGO ICP build'
export CMAKE_PREFIX_PATH="$ROBOCON_DEPS:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$ROBOCON_DEPS/lib:$ROBOCON_DEPS/lib64:${LD_LIBRARY_PATH:-}"
cd "$ROBOCON_WS"
colcon build --base-paths src --symlink-install --executor sequential --packages-up-to club_bringup fastlio2 pgo localizer --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$ROBOCON_WS/install/setup.bash"
ros2 pkg executables fastlio2
ros2 pkg executables pgo
ros2 pkg executables localizer
dpkg-query -W 'ros-jazzy-*' > /work/logs/apt_versions.txt
echo 'BOOTSTRAP COMPLETE'
