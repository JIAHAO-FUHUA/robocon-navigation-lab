#!/usr/bin/env bash
set -euo pipefail

fail() {
  printf '%s\n' "$1" >&2
  exit 1
}

project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
[[ -f "$project/scripts/setup_course.sh" && -f "$project/env.sh" && -s "$project/course_map/map.pcd" ]] ||
  fail 'Place start_lab.sh in the robocon-navigation-lab project folder, alongside README.md.'
command -v docker >/dev/null ||
  fail 'Install Docker on Ubuntu using the course README, then run bash start_lab.sh in the Ubuntu terminal.'

docker_cmd=(docker)
if ! docker info >/dev/null 2>&1; then
  command -v sudo >/dev/null ||
    fail 'Docker is unavailable. Start Docker or check your account permissions, then retry.'
  docker_cmd=(sudo docker)
  if ! "${docker_cmd[@]}" info >/dev/null 2>&1; then
    fail 'Docker is unavailable. On Ubuntu run sudo systemctl start docker, or start Docker Desktop, then retry.'
  fi
fi

container=robocon-intro
if "${docker_cmd[@]}" container inspect "$container" >/dev/null 2>&1; then
  mounted=$("${docker_cmd[@]}" container inspect --format \
    '{{range .Mounts}}{{if eq .Destination "/work"}}{{.Source}}{{end}}{{end}}' "$container")
  if [[ "$mounted" != "$project" ]]; then
    # Earlier course versions can have the same container name and a different source folder.
    read -r project_id _ < <(printf '%s' "$project" | cksum)
    container="robocon-intro-$project_id"
  fi
fi

if "${docker_cmd[@]}" container inspect "$container" >/dev/null 2>&1; then
  mounted=$("${docker_cmd[@]}" container inspect --format \
    '{{range .Mounts}}{{if eq .Destination "/work"}}{{.Source}}{{end}}{{end}}' "$container")
  [[ "$mounted" == "$project" ]] ||
    fail "Container $container is already linked to another folder. Check docker ps -a before retrying."
  if [[ $("${docker_cmd[@]}" container inspect --format '{{.State.Running}}' "$container") != true ]]; then
    printf 'Starting %s\n' "$container"
    "${docker_cmd[@]}" start "$container" >/dev/null
  fi
else
  printf 'Creating %s\n' "$container"
  "${docker_cmd[@]}" run --detach --name "$container" --shm-size=1g \
    --volume "$project:/work" --workdir /work ros:jazzy-ros-base-noble sleep infinity >/dev/null
fi

if ! "${docker_cmd[@]}" exec "$container" bash -ec '
  test -s /work/jazzy_ws/install/setup.bash
  command -v ffmpeg
  command -v Xvfb
  source /work/env.sh
  command -v gz
  ros2 pkg prefix fastlio2
  ros2 pkg prefix pgo
  ros2 pkg prefix localizer
' >/dev/null 2>&1; then
  printf '%s\n' 'Setting up the lab. The first build downloads and compiles the dependencies.'
  "${docker_cmd[@]}" exec "$container" bash /work/scripts/setup_course.sh
fi

"${docker_cmd[@]}" exec -it -e COMPLEX_CAMERA_HZ="${COMPLEX_CAMERA_HZ:-4}" "$container" \
  bash -ec '
    cd /work
    if [[ ! -s mobile_sensor.urdf || ! -s mobile_sensor.sdf ]]; then
      bash /work/scripts/prepare_sim.sh
    fi
    source /work/env.sh
    mkdir -p /work/logs
    if ! bash /work/start_virtual_display.sh > /work/logs/display_check.log 2>&1; then
      cat /work/logs/display_check.log >&2
      exit 1
    fi
    printf "\nLab ready in /work. Run the mapping or navigation commands from the README.\n"
    printf "Overview camera rate is %s Hz. Type exit to return to your Ubuntu terminal.\n\n" "$COMPLEX_CAMERA_HZ"
    exec bash -i
  '
