# SLAM Wheeled Robot — ROS2 Jazzy Navigation System

A from-scratch autonomous navigation system for a differential-drive wheeled
mobile robot, built in ROS2 Jazzy, implementing four key improvements over
a prior MATLAB/RRT/DWA baseline:

| # | Improvement | Status |
|---|---|---|
| 1 | RRT → **RRT\*** global planner | ✅ Complete, benchmarked |
| 2 | MATLAB → **ROS2 + Gazebo** | ✅ Complete |
| 3 | DWA → **MPPI** local controller | ✅ Complete, benchmarked |
| 4 | Static map → **Dynamic replanning** | ✅ Complete |

## Project Roadmap

| Phase | Description | Status |
|---|---|---|
| Phase 0 | ROS2 + Gazebo + Nav2 Environment Setup | ✅ Completed |
| Phase 1 | RRT* Global Path Planner Development and Benchmarking | ✅ Completed |
| Phase 2 | MPPI Local Controller Development and Benchmarking (originally planned as TEB) | ✅ Completed |
| Phase 3 | SLAM Integration (SLAM Toolbox + AMCL Localization) | ✅ Completed |
| Phase 4 | Dynamic Replanning and Obstacle Response | ✅ Completed |
| Phase 5 | Full System Integration and Performance Evaluation | ⏳ Pending |
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
├── rrt_star_planner/             <- Phases 1-3 package (RRT* + MPPI + SLAM configs)
│   ├── include/rrt_star_planner/
│   │   └── rrt_star_planner.hpp
│   ├── src/
│   │   └── rrt_star_planner.cpp
│   ├── config/
│   │   ├── nav2_params_rrtstar.yaml       <- RRT* planner only
│   │   ├── nav2_params_mppi.yaml          <- MPPI controller only
│   │   ├── nav2_params_dwa_baseline.yaml  <- DWA config for comparison
│   │   ├── nav2_params_full_phase3.yaml   <- RRT* + MPPI combined (use this from Phase 3 onward)
│   │   └── slam_toolbox_params.yaml
│   ├── launch/
│   │   └── slam_nav2_rrtstar.launch.py
│   ├── scripts/
│   │   ├── benchmark_rrt_vs_rrtstar.py
│   │   ├── benchmark_controller_comparison.py
│   │   └── plot_controller_comparison.py
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
├── maps/                         <- Saved SLAM maps (Phase 3)
│   ├── phase3_map.pgm
│   └── phase3_map.yaml
├── results/
│   ├── Phase-wise_Improvement.md       <- full results, metrics, conclusions per phase
│   ├── rrt_vs_rrtstar_comparison.png
│   ├── mppi_vs_dwa_comparison.png
│   ├── benchmark_rrt_star_*.csv
│   ├── benchmark_rrt_*.csv
│   └── controller_*.csv
└── docs/
    └── DEBUGGING_LOG.md           <- full record of issues encountered + fixes
```

## Quick start

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select rrt_star_planner rrt_planner --symlink-install
source install/setup.bash
```

### Launch — full stack (RRT* + MPPI), live SLAM mapping
```bash
# Terminal 1
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 (after Gazebo fully loads)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Terminal 3 (after SLAM registers sensor, +5s)
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  params_file:=$HOME/ros2_ws/src/rrt_star_planner/config/nav2_params_full_phase3.yaml \
  use_rviz:=False

# Set initial pose and send a goal
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.3, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

### Launch — static map + AMCL localization only (no live SLAM)
```bash
# Terminal 1 — same as above
# Terminal 2 — skip SLAM Toolbox, go straight to Nav2 with the saved map:
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/ros2_ws/maps/phase3_map.yaml \
  params_file:=$HOME/ros2_ws/src/rrt_star_planner/config/nav2_params_full_phase3.yaml \
  use_rviz:=False

# ⚠️ IMPORTANT: publish /initialpose IMMEDIATELY (within ~10s) after launch,
# not after waiting for the system to "settle" — see warning below.
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## ⚠️ Two one-time/per-session fixes required

### 1. Gazebo Harmonic + ROS2 Jazzy `cmd_vel` type mismatch (one-time, requires sudo)

TurtleBot3's stock Gazebo bridge config publishes `/cmd_vel` as
`TwistStamped`, but Nav2's velocity pipeline (`controller_server` →
`velocity_smoother` → `collision_monitor`) publishes plain `Twist`.
This silently drops every velocity command — the robot will appear to
plan correctly but never physically move.

```bash
sudo cp /opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml \
        /opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml.bak

sudo sed -i 's|geometry_msgs/msg/TwistStamped" # If you use Twist.*|geometry_msgs/msg/Twist"|' \
        /opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml
```

Verify:
```bash
ros2 node info /ros_gz_bridge | grep cmd_vel
# Should show: geometry_msgs/msg/Twist
```

### 2. AMCL `/initialpose` timing race condition (every session, using a static map)

When loading a pre-built static map (not live SLAM), AMCL will not publish
`map → odom` until it receives `/initialpose`. Nav2's bringup has a hard
~30-second timeout waiting for that transform — if the pose isn't sent in
time, the **entire** navigation stack bringup aborts, even though AMCL
itself never errors. **Send `/initialpose` immediately after launch, not
after waiting for the system to look "ready."**

See `docs/DEBUGGING_LOG.md` for the full diagnostic trail behind both fixes.

## Results summary

Full details, metrics, and per-phase conclusions: **[`results/Phase-wise_Improvement.md`](results/Phase-wise_Improvement.md)**

| Component | Baseline | Proposed | Benefit |
|---|---|---|---|
| Global Planner | RRT | RRT* | 56.8% smoother paths, 6x higher success rate |
| Local Controller | DWA | MPPI | ~70% lower peak angular velocity, zero oscillations |
| Platform | MATLAB | ROS2 + Gazebo | Hardware-portable, industry standard |
| Localization | Not separated from mapping | SLAM Toolbox + AMCL | Map-once, localize-forever capability |

## Author

Kiran S K · B.Tech Mechatronics Engineering, Paavai Engineering College · [Portfolio](https://kiransk.me)
