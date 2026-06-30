# SLAM Wheeled Robot — ROS2 Jazzy Navigation System

M.Tech research internship project — NIT Calicut, Dept. of Electrical Engineering.
Guide: Dr. Rahul Radhakrishnan.

A from-scratch autonomous navigation system for a differential-drive wheeled
mobile robot, built in ROS2 Jazzy, implementing four key improvements over
a prior MATLAB/RRT/DWA baseline:

| # | Improvement | Status |
|---|---|---|
| 1 | RRT → **RRT\*** global planner | ✅ Complete, benchmarked |
| 2 | MATLAB → **ROS2 + Gazebo** | ✅ Complete |
| 3 | DWA → **MPPI** local controller | ✅ Complete, benchmarked |
| 4 | Static map → **Dynamic replanning** | ✅ Complete, validated |

## Project Roadmap

| Phase | Description | Status |
|---|---|---|
| Phase 0 | ROS2 + Gazebo + Nav2 Environment Setup | ✅ Completed |
| Phase 1 | RRT* Global Path Planner Development and Benchmarking | ✅ Completed |
| Phase 2 | MPPI Local Controller Development and Benchmarking (originally planned as TEB) | ✅ Completed |
| Phase 3 | SLAM Integration (SLAM Toolbox + AMCL Localization) | ✅ Completed |
| Phase 4 | Dynamic Replanning and Obstacle Response | ✅ Completed |
| Phase 5 | Full System Integration and Performance Evaluation | ✅ Completed |
| Phase 6 | Hardware Validation on Wheeled Mobile Robot | ⏳ Pending |

**Full phase-by-phase results, metrics, and conclusions:**
**[`results/Phase-wise_Improvement.md`](results/Phase-wise_Improvement.md)**

## Why MPPI instead of TEB

The original plan called for TEB as the local planner upgrade. `teb_local_planner`
has no official, stable ROS2 Jazzy release — the upstream repo only has an
experimental branch with an open maintenance issue. Building it from source
risked days of dependency failures.

**MPPI is Nav2's own modern successor to both TEB and DWB**, confirmed by
Nav2's lead maintainer and official tuning guide. It ships natively with
`ros-jazzy-navigation2` — zero source build required, more current technology,
same improvement narrative (kinodynamic awareness, smoother avoidance, less
oscillation).

## Repository structure

```
slam-wheeled-robot-ros2/
├── README.md                     <- you are here
├── rrt_star_planner/             <- Phases 1-5 package (RRT* + MPPI + SLAM configs)
│   ├── include/rrt_star_planner/
│   │   └── rrt_star_planner.hpp
│   ├── src/
│   │   └── rrt_star_planner.cpp
│   ├── config/
│   │   ├── nav2_params_rrtstar.yaml       <- RRT* planner only
│   │   ├── nav2_params_mppi.yaml          <- MPPI controller only
│   │   ├── nav2_params_dwa_baseline.yaml  <- DWA config for comparison
│   │   ├── nav2_params_full_phase3.yaml   <- RRT* + MPPI + AMCL (static-map mode)
│   │   ├── nav2_params_live_slam.yaml     <- RRT* + MPPI, NO AMCL (live-SLAM mode)
│   │   └── slam_toolbox_params.yaml
│   ├── launch/
│   │   └── slam_nav2_rrtstar.launch.py
│   ├── scripts/
│   │   ├── benchmark_rrt_vs_rrtstar.py
│   │   ├── benchmark_controller_comparison.py
│   │   ├── plot_controller_comparison.py
│   │   ├── benchmark_dynamic_replanning.py
│   │   └── rugged_test_phase5.py
│   ├── PHASE2_README.md
│   └── CMakeLists.txt / package.xml / plugins.xml
├── rrt_planner/                  <- Vanilla RRT baseline (for benchmarking only)
│   ├── include/rrt_planner/
│   ├── src/
│   ├── config/
│   │   └── nav2_params_rrt_baseline.yaml
│   ├── scripts/
│   │   ├── benchmark_comparison.py
│   │   └── plot_comparison.py
│   └── CMakeLists.txt / package.xml / plugins.xml
├── models/
│   └── dynamic_obstacle.sdf       <- Spawnable obstacle for dynamic replanning tests
├── maps/                          <- Saved SLAM maps
│   ├── phase3_map.pgm / .yaml
│   └── phase5_final_map.pgm / .yaml
├── results/
│   ├── Phase-wise_Improvement.md       <- full results, metrics, conclusions per phase
│   ├── rrt_vs_rrtstar_comparison.png
│   ├── mppi_vs_dwa_comparison.png
│   ├── benchmark_rrt_star_*.csv
│   ├── benchmark_rrt_*.csv
│   ├── controller_*.csv
│   └── dynamic_replanning_*.csv
└── docs/
    └── DEBUGGING_LOG.md           <- full record of issues encountered + fixes
```

## Two operating modes — choose the right params file

This project has two distinct, **mutually exclusive** modes. Using the
wrong params file for the wrong mode causes a real conflict (see
Debugging Log entry on AMCL + SLAM Toolbox).

| Mode | Use case | Params file | Localization |
|---|---|---|---|
| **Live SLAM** | Building a new map, general demos | `nav2_params_live_slam.yaml` | SLAM Toolbox (built-in) |
| **Static map + AMCL** | Navigating a pre-built saved map | `nav2_params_full_phase3.yaml` | AMCL only |

**Never run SLAM Toolbox and a params file containing AMCL at the same time.**

## Quick start

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select rrt_star_planner rrt_planner --symlink-install
source install/setup.bash
```

### Launch — Live SLAM mode (RRT* + MPPI, building a new map)
```bash
# Terminal 1
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 (after Gazebo fully loads)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Terminal 3 (after SLAM registers sensor, +5s)
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  params_file:=$HOME/ros2_ws/src/rrt_star_planner/config/nav2_params_live_slam.yaml \
  use_rviz:=False

# Send a goal directly -- no /initialpose needed in this mode
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.3, y: -0.2, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

### Launch — Static map + AMCL mode (no live SLAM)
```bash
# Terminal 1 — same as above
# Terminal 2 — skip SLAM Toolbox, go straight to Nav2 with a saved map:
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/ros2_ws/maps/phase5_final_map.yaml \
  params_file:=$HOME/ros2_ws/src/rrt_star_planner/config/nav2_params_full_phase3.yaml \
  use_rviz:=False

# ⚠️ Publish /initialpose IMMEDIATELY (within ~10s) after launch --
# see warning below, this is a real timing-sensitive step.
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Dynamic obstacle testing
```bash
ros2 run ros_gz_sim create -file ~/ros2_ws/models/dynamic_obstacle.sdf \
  -name dynamic_obstacle -x 0.15 -y -0.1 -z 0.0
```

## ⚠️ Critical fixes — verify these every session, not just once

### 1. Gazebo Harmonic + ROS2 Jazzy `cmd_vel` type mismatch

This fix was found to **silently revert** after a system package update
partway through this project. **Re-verify it at the start of every new
session**, not just once.

```bash
ros2 node info /ros_gz_bridge | grep cmd_vel
# Must show: geometry_msgs/msg/Twist  -- if it shows TwistStamped, reapply:

sudo sed -i 's|ros_type_name: "geometry_msgs/msg/TwistStamped".*|ros_type_name: "geometry_msgs/msg/Twist"|' \
  /opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml
```
**Requires a full Gazebo restart to take effect** — the bridge config is only read at launch.

### 2. AMCL `/initialpose` timing race condition (static-map mode only)

When loading a pre-built static map (not live SLAM), AMCL will not publish
`map → odom` until it receives `/initialpose`. Nav2's bringup has a hard
~30-second timeout waiting for that transform — if the pose isn't sent in
time, the **entire** navigation stack bringup aborts. **Send `/initialpose`
immediately after launch, not after waiting for the system to look "ready."**

### 3. Never combine AMCL with live SLAM Toolbox

Both publish `map → odom`. Running both causes AMCL to perpetually fail
and eventually times out the whole navigation request. Use the correct
params file for your mode (see table above).

See `docs/DEBUGGING_LOG.md` for the full diagnostic trail behind all three fixes.

## Results summary

Full details, metrics, and per-phase conclusions: **[`results/Phase-wise_Improvement.md`](results/Phase-wise_Improvement.md)**

| Component | Baseline | Proposed | Benefit |
|---|---|---|---|
| Global Planner | RRT | RRT* | 56.8% smoother paths, 6x higher success rate |
| Local Controller | DWA | MPPI | ~70% lower peak angular velocity, zero oscillations |
| Platform | MATLAB | ROS2 + Gazebo | Hardware-portable, industry standard |
| Localization | Not separated from mapping | SLAM Toolbox + AMCL | Map-once, localize-forever capability |
| Obstacle handling | Static map only | Live dynamic replanning | Reacts to unmapped obstacles without re-mapping |
| System integration | Individual modules | Full stack validated together | Proven stable under sustained, multi-step operation |

## Author

Kiran S K — M.Tech Electrical Engineering (Instrumentation & Control Systems),
NIT Calicut · B.Tech Mechatronics Engineering, Paavai Engineering College
GitHub: [KiranSastha](https://github.com/KiranSastha) · [kiransk.me](https://kiransk.me)
