# rrt_star_planner

RRT* global planner Nav2 plugin — **ROS2 Jazzy / Ubuntu 24.04**.

Implements asymptotically-optimal path planning on a 2D occupancy grid
with near-neighbour rewiring and adaptive radius.

## RRT vs RRT* — key difference

| | RRT | RRT* |
|---|---|---|
| Path quality | Feasible only | Asymptotically optimal |
| Near search | Nearest 1 node | All nodes within r(n) |
| Rewiring | None | Yes — updates parent if cost improves |
| Expected path length | Higher | 15-30% shorter |

## Build

```bash
cd ~/ros2_ws/src
# (already unzipped here)
cd ~/ros2_ws
colcon build --packages-select rrt_star_planner --symlink-install
source install/setup.bash
```

## Verify plugin registered

```bash
ros2 run pluginlib find_factories nav2_core::GlobalPlanner
# Expected: rrt_star_planner/RRTStarPlanner
```

## Run

```bash
# Terminal 1 — Gazebo
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 — SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Terminal 3 — Nav2 with RRT*
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  params_file:=$HOME/ros2_ws/src/rrt_star_planner/config/nav2_params_rrtstar.yaml
```

Then in RViz2: set **2D Pose Estimate** then **Nav2 Goal**.

Expected terminal output:
```
[planner_server]: RRT* path found | cost=2.847m  nodes=1253  time=187ms
```

## Benchmark (Day 5)

```bash
python3 ~/ros2_ws/src/rrt_star_planner/scripts/benchmark_rrt_vs_rrtstar.py
```

Results saved to `~/ros2_ws/results/rrtstar_benchmark_<timestamp>.csv`

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| max_iterations | 3000 | Increase for large maps |
| step_size | 0.15 m | Smaller = more precise, slower |
| goal_tolerance | 0.25 m | Acceptance radius at goal |
| goal_bias | 0.10 | Probability of sampling goal directly |
| rewire_radius | 0.75 m | Neighbourhood search radius |
| lethal_threshold | 253 | Costmap obstacle threshold |
