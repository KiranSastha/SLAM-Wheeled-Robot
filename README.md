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
| 3 | DWA → **MPPI** local controller | 🔄 In progress |
| 4 | Static map → **Dynamic replanning** | ⏳ Planned |

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
├── rrt_star_planner/             <- Phase 1 + Phase 2 package
│   ├── include/rrt_star_planner/
│   │   └── rrt_star_planner.hpp
│   ├── src/
│   │   └── rrt_star_planner.cpp
│   ├── config/
│   │   ├── nav2_params_rrtstar.yaml      <- complete Nav2 params, RRT* planner
│   │   ├── nav2_params_mppi.yaml         <- complete Nav2 params, MPPI controller
│   │   ├── nav2_params_dwa_baseline.yaml <- DWA config for comparison
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
├── results/
│   ├── benchmark_rrt_star_*.csv
│   ├── benchmark_rrt_*.csv
│   └── rrt_vs_rrtstar_comparison.png
└── docs/
    └── DEBUGGING_LOG.md           <- full record of issues encountered + fixes
```

## Quick start

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select rrt_star_planner rrt_planner --symlink-install
source install/setup.bash

# Launch (3 terminals, in order, with waits between each)
# Terminal 1
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 (after Gazebo fully loads)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Terminal 3 (after SLAM registers sensor, +5s)
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  params_file:=$HOME/ros2_ws/src/rrt_star_planner/config/nav2_params_rrtstar.yaml \
  use_rviz:=False

# Set initial pose and send a goal
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.3, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

## ⚠️ One-time system fix required (Gazebo Harmonic + ROS2 Jazzy)

TurtleBot3's stock Gazebo bridge config publishes `/cmd_vel` as
`TwistStamped`, but Nav2's velocity pipeline (`controller_server` →
`velocity_smoother` → `collision_monitor`) publishes plain `Twist`.
This silently drops every velocity command — the robot will appear to
plan correctly but never physically move.

**Fix (one-time, requires sudo):**
```bash
sudo cp /opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml \
        /opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml.bak

sudo sed -i 's|geometry_msgs/msg/TwistStamped" # If you use Twist.*|geometry_msgs/msg/Twist"|' \
        /opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml
```

Verify after applying:
```bash
ros2 node info /ros_gz_bridge | grep cmd_vel
# Should show: geometry_msgs/msg/Twist
```

See `docs/DEBUGGING_LOG.md` for the full diagnostic trail that led to
discovering this.

## Results — RRT vs RRT* (20 trials each)

| Metric | RRT (baseline) | RRT* (improved) | Change |
|---|---|---|---|
| Success rate | 2/20 (10%) | 12/20 (60%) | **+500%** |
| Avg planning time | ~110 ms (40–180 range) | ~53 ms (40–68 range) | **~52% faster, far more consistent** |
| Avg path smoothness | 6.68 rad | 2.89 rad | **56.8% smoother** |
| Avg path length | 1.82 m | 1.86 m | No clear advantage at this sample size |

RRT*'s rewiring mechanism produced dramatically more reliable planning
(6x higher success rate) and much smoother resulting paths, at a lower
average planning-time cost than vanilla RRT, whose few successful runs
were highly inconsistent (40–180ms).

Full chart: `results/rrt_vs_rrtstar_comparison.png`

## Author

Kiran S K — M.Tech Electrical Engineering (Instrumentation & Control Systems),
NIT Calicut · B.Tech Mechatronics Engineering, Paavai Engineering College
GitHub: [KiranSastha](https://github.com/KiranSastha) · [kiransk.me](https://kiransk.me)
