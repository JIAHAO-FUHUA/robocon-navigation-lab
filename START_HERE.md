# Robocon Navigation Lab

Extract the ZIP into your Ubuntu home directory. This creates `~/robocon-navigation-lab`. The checked environment is Ubuntu 24.04 x86-64 with Docker. The lab uses its own ROS 2 Jazzy container.

Run this once before class. The first launch downloads the pinned upstream projects and compiles FAST-LIO2 and its dependencies.

```bash
cd ~/robocon-navigation-lab
bash start_lab.sh
```

When the launcher prints `Lab ready in /work`, continue on slide 9 in that terminal. On subsequent sessions use the same two commands. Slides 10–16 explain how to run the route, change the goal and verify a result checker.

The optional RTAB-Map workshop uses a separate Ubuntu terminal with ROS 2 Jazzy and Gazebo Harmonic. Follow `extras/RTABMAP.md`. The stereo examples use the separate Vision Replay Starter ZIP.
