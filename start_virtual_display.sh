#!/usr/bin/env bash
set -eo pipefail
if ! DISPLAY=:99 glxinfo -B >/dev/null 2>&1; then
  nohup Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp > /work/logs/xvfb.log 2>&1 &
  sleep 1
fi
DISPLAY=:99 glxinfo -B
