# Change the field and plan the BR exercise

Start with a successful baseline run from the [README](../README.md). Save its video and results for comparison. Run the commands below inside the same course container. Keep all distances in metres. The 11 m teaching field uses its centre as `(0, 0)`. In the top view, positive x points right and positive y points up. Yaw is in degrees where the key ends in `_deg`. Clockwise yaw is negative.

## 1. Change only the goal

Open `initial_guess_video_20261116/config.json` in your Ubuntu text editor. Change the two values in `goal_world` to `[4.4, 3.0]`, keep the comma after the entry, and save. The original value is `[4.4, 2.0]`. The goal is a visual marker, so keep the same point cloud.

```bash
cd /work/initial_guess_video_20261116
python3 -m json.tool config.json >/dev/null
source /work/env.sh
export ROS_DOMAIN_ID=79
export GZ_PARTITION=robocon_bias_video
python3 run.py navigation --record
run_dir=$(cat latest_navigation.txt)
python3 evaluate.py "$run_dir"
```

Check that the new goal marker and the end of the orange route move. Compare both videos and final errors. Depending on the target, much of the route can stay the same.

## 2. Move one obstacle and rebuild the map

The Gazebo scene, the A* grid and the LiDAR point cloud serve different purposes. Gazebo simulates the obstacle and sensor data. A* reads obstacle rectangles from the JSON configuration. ICP uses the saved LiDAR point cloud. If geometry changes, update the scene and grid together and collect a fresh map.

For this lab, change only the **Red obstacle**. Its exact name tells both `make_scene.py` scripts which box to create. Its `box` contains `[xmin, ymin, xmax, ymax]`. Height is a separate value. Move the original rectangle from `[3.0, -0.7, 3.6, 0.7]` to `[3.1, -0.2, 3.7, 1.2]` and see how A* changes the path around it. Keep the floor, platforms, LiDAR position, start and restart pose unchanged for this exercise.

Make the same change in both configuration files. Paste this entire block through the final `PY`, or edit the `Red obstacle` entry in each file with a text editor.

```bash
cd /work
python3 - <<'PY'
import json
from pathlib import Path

folders = ("complex_demo_20261113", "initial_guess_video_20261116")
for folder in folders:
    path = Path(folder) / "config.json"
    data = json.loads(path.read_text())
    matches = [obj for obj in data["obstacles_world"] if obj["name"] == "Red obstacle"]
    if len(matches) != 1:
        raise ValueError(f"Expected one Red obstacle in {path}")
    matches[0]["box"] = [3.1, -0.2, 3.7, 1.2]
    path.write_text(json.dumps(data, indent=2) + "\n")
    print(path, matches[0]["box"])
PY
```

The two output lines should show the same rectangle. Check that the goal is reachable and inspect the generated world before starting the simulation.

```bash
source /work/env.sh
cd /work/initial_guess_video_20261116
python3 - <<'PY'
from core import CONFIG, plan_world
path, _ = plan_world(CONFIG["restart_world"], CONFIG["goal_world"])
print("Planned stops in metres", path)
PY
python3 make_scene.py /work/logs/scene_preview navigation
```

`logs/scene_preview/world.sdf` is the generated scene. Search it for `red_obstacle`. Its centre should be `x=3.4`, `y=0.5` (allow for rounding) and its box should appear in both `visual` and `collision`. Edit the input JSON and regenerate the world for future changes. The generated SDF is overwritten on later runs.

Map the changed field (Section 3 of the README). Copy the new map pointer to the navigation example, then run navigation and evaluate it. The included `course_map/map.pcd` belongs to the original field. To compare only the obstacle change, keep the goal the same in both runs. Check that both the planned path and blue trail leave clearance around the new obstacle. If A* reports no path, check the inflated obstacle against the wall and measure the available width.

## 3. Build your group's BR field

Work in a separate copy of the project for the competition version. Start with the dimensions and locations in the competition rulebook and appendix. Pick one side of the field and document how you transform the rulebook drawing into your Gazebo coordinates. Place the start S, the ramp, the receiving position H, and tower positions A and B. A and B represent robot stopping poses near the building spots. Give each one a docking heading. The exercise ends after both towers are complete.

| To change | Start with | What you also verify |
| --- | --- | --- |
| Floor and platforms | `robocon_linux_training_v3/config/field_spec.json` and `tools/generate_world.py` in that folder | Platform top heights and limits in the generated SDF. The teaching generator creates box platforms. Add the ramp in the scene code, then check that its upper surface meets L1. |
| Environment fixtures | The scene functions in `make_mobile_world.py` and the two `make_scene.py` files | Each fixture should have matching visual and collision geometry. Several teaching fixtures and their positions are explicitly defined in Python. Update the corresponding planner geometry wherever the robot drives. |
| Floor planning and obstacles | `obstacles_world` in both demo configs and `core.py` | The starter grid uses a centred 11 m field with a 5.5 m shift and a flat driving surface. For levels, define drivable regions per level and an explicit ramp transition. An A* line across a vertical step is invalid. |
| Robot and sensor placement | `mobile_robot.urdf.xacro` and `make_mobile_world.py` | Update collision footprint, robot size in both configs, sensor transforms and clearance calculation consistently. Run `bash /work/scripts/prepare_sim.sh` to regenerate geometry, then map again. |
| Targets and waits | Create a task-point list from [the checkpoint template](checkpoints.csv) | Sequence number, x/y/z, yaw, action and dwell or ready condition. Add repeated visits to H as needed. |

Work in this order.

1. **Model and preview.** Generate the SDF. Check the ramp connection, the robot footprint at H, tower approach clearance and the L1 limits.
2. **Map the route.** Keep the same physical layout for mapping and navigation. Set `mapping_loop_world` for an accessible loop that covers the areas you will use. Test the ramp as a separate, explicit transition before adding it to the mission. If you move the mapping origin, update the transform and the approximate ICP start in the navigation example before testing.
3. **Run one leg first.** Test from H to A, stop near the correct position, and save the result. Confirm the estimated pose, heading and clearance.
4. **Chain the mission.** The starter navigation script plans to one `goal_world`. In `initial_guess_video_20261116/control.py`, locate the call to `plan_world`. Extend the sequence so each target uses the latest estimated pose, both position and heading must settle, and a wait or ready signal completes the work stop before the next target. Preserve localization checks and motion stop behaviour. First complete S → ramp → H → A → H, then add B and any repeated pickups. Use a labelled stop or a timed wait to represent gripping and stacking.
5. **Show the outcome.** Submit the field, task-point list, map, a video with planned and travelled routes, visited stop IDs, position and heading errors, elapsed time and any failed stop. State the tolerance you used and explain one change your group made.
