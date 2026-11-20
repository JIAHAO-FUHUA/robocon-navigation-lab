#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
cd "$here"
[[ $(uname -m) == x86_64 ]] || { echo 'This example requires x86-64 Ubuntu.' >&2; exit 1; }
docker_cmd=(docker)
if ! docker info >/dev/null 2>&1; then
  docker_cmd=(sudo docker)
  "${docker_cmd[@]}" info >/dev/null
fi
for recording in mapping_20261117_074657 navigation_20261117_155645; do
  test -s "gazebo_data/$recording/frames.csv" || { echo "Missing gazebo_data/$recording. Extract the entire archive." >&2; exit 1; }
done
image=robocon-student-orb:4452a3c
if ! "${docker_cmd[@]}" image inspect "$image" >/dev/null 2>&1; then
  "${docker_cmd[@]}" build -f Dockerfile.orb -t "$image" .
fi
mkdir -p orb_runs
run=$(mktemp -d "$here/orb_runs/run.XXXXXX")
"${docker_cmd[@]}" run --rm --network none --cpus 2 --user "$(id -u):$(id -g)" \
  -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 \
  --mount "type=bind,src=$here/gazebo_data,dst=/sensor_data,readonly" \
  --mount "type=bind,src=$run,dst=/orb_work/results" "$image" \
  bash -ec 'source env.sh
    python3 run_trials.py student_map mapping
    python3 run_trials.py student_restart mapping --load-map student_map
    python3 run_trials.py student_offset restart_minus15 --load-map student_map'
printf 'Saved the map and restart poses in %s\n' "$run"
