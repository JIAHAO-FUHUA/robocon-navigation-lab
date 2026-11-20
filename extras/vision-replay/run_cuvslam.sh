#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
cd "$here"
python3 -c 'import platform,sys; sys.exit(0 if sys.version_info[:2] == (3,12) and platform.machine() == "x86_64" else "Use Ubuntu 24.04 x86-64 with Python 3.12.")'
command -v nvidia-smi >/dev/null || { echo 'Install a compatible NVIDIA driver and check nvidia-smi.' >&2; exit 1; }
nvidia-smi --query-gpu=name,driver_version --format=csv
test -s gazebo_data/calibration.json || { echo 'Extract the accompanying Gazebo data.' >&2; exit 1; }
env_path="$here/.cuvslam_venv"
if [[ ! -x "$env_path/bin/python" ]]; then
  python3 -m venv "$env_path"
fi
if [[ ! -e "$env_path/lab_17_0_1_ready" ]]; then
  mkdir -p "$here/.cuvslam_download"
  wheel='cuvslam-17.0.1+cu13-cp312-abi3-manylinux_2_39_x86_64.whl'
  wheel_path="$here/.cuvslam_download/$wheel"
  curl -fL --retry 2 -o "$wheel_path" "https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.1/$wheel"
  printf '%s  %s\n' dba2079a3e013b6bb37d8c9965e4beb42007282f33863815d69689568f820ed1 "$wheel_path" | sha256sum -c -
  "$env_path/bin/python" -m pip install "$wheel_path" \
    'numpy==2.5.5' 'scipy==1.18.1' 'opencv-python-headless==5.0.0.93' \
    'nvidia-cublas==13.3.0.5' 'nvidia-cuda-nvrtc==13.2.51' 'nvidia-cusolver==12.1.0.51' \
    'nvidia-cusparse==12.7.9.17' 'nvidia-nvjitlink==13.2.51' 'PyYAML==6.0.3'
  touch "$env_path/lab_17_0_1_ready"
fi
export LD_LIBRARY_PATH="$env_path/lib/python3.12/site-packages/nvidia/cu13/lib:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 RERUN=0
mkdir -p cuvslam_runs
run=$(mktemp -d "$here/cuvslam_runs/run.XXXXXX")
"$env_path/bin/python" -u cuvslam_replay.py gazebo_data/calibration.json \
  gazebo_data/mapping_20261117_074657 gazebo_data/navigation_20261117_155645 "$run/result"
printf 'Saved the map and restart check in %s\n' "$run/result"
