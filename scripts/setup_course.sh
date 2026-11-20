#!/usr/bin/env bash
set -eo pipefail
cd /work
bash linux_bootstrap.sh
python3 add_localizer_health.py
python3 fix_tf_refresh.py
source /work/env.sh
cd /work/jazzy_ws
colcon build --symlink-install --executor sequential --packages-select localizer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
apt-get install -y ffmpeg python3-pil fonts-noto-cjk
bash /work/scripts/prepare_sim.sh
printf '%s\n' 'COURSE SETUP COMPLETE'
