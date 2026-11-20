#!/usr/bin/env bash
set -euo pipefail

workspace=/work/jazzy_ws
mkdir -p "$workspace/src" "$workspace/deps"

clone_pin() {
  local url="$1" target="$2" commit="$3"
  if [[ ! -d "$target/.git" ]]; then
    git clone "$url" "$target"
  fi
  if [[ -z "$(git -C "$target" status --porcelain)" ]]; then
    git -C "$target" checkout --detach "$commit"
  fi
  if [[ "$(git -C "$target" rev-parse HEAD)" != "$commit" ]]; then
    printf 'Source is at a different revision. Check %s before rebuilding.\n' "$target" >&2
    return 1
  fi
  printf '%s %s\n' "$(basename "$target")" "$commit"
}

clone_pin https://github.com/liangheming/FASTLIO2_ROS2.git \
  "$workspace/src/FASTLIO2_ROS2" 7baa59f7339ba88ce3b646fe35b23f61031c5d3f
clone_pin https://github.com/mathaimp/livox_gz_sim.git \
  "$workspace/src/livox_gz_sim" 3f06f1ac4d3f6854f7275ca9aa097484791a1abd
clone_pin https://github.com/strasdat/Sophus.git \
  "$workspace/deps/Sophus" de0f8d3d92bf776271e16de56d1803940ebccab9
clone_pin https://github.com/borglab/gtsam.git \
  "$workspace/deps/gtsam" c57988fe554e7213c77fe379c1d7c483de26ad33
clone_pin https://github.com/Livox-SDK/livox_ros_driver2.git \
  "$workspace/deps/livox_ros_driver2_upstream" 21445540f0d100dc86a7e6df312dd70bbdb4afdf
