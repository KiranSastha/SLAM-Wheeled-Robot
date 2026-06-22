# Phase 2 - MPPI Local Controller (replaces DWA)

## Why MPPI instead of TEB

teb_local_planner has no official, stable ROS2 Jazzy release. The upstream
repo (rst-tu-dortmund/teb_local_planner) only has an experimental ros2-master
branch with an open, unresolved maintenance issue (#373). Building it from
source on Jazzy risks days of dependency/build failures.

MPPI (Model Predictive Path Integral controller) is Nav2's own modern
successor to both TEB and DWB:

"MPPI controller... is the functional successor of the TEB and DWB
controllers, providing predictive time-varying trajectories reminiscent
of TEB while providing tunable critic functions similar to DWB."
-- Nav2 RosCon 2023 talk, Steve Macenski (Nav2 lead maintainer)

"MPPI however does have moderately higher compute costs, but it is
highly recommended to go this route and has received considerable
development resources and attention due to its power."
-- Official Nav2 Tuning Guide

It ships natively with ros-jazzy-navigation2 - zero source build required.

## Report framing (Improvement 3 - updated)

Original: "DWA -> TEB"
Updated: "DWA -> MPPI" - same justification (kinodynamic awareness, smoother
obstacle avoidance, less oscillation), but using Nav2's actual current-
generation controller rather than an unmaintained ROS1 port.

## Files

- config/nav2_params_mppi.yaml - full Nav2 params with MPPI controller + RRT* planner
- config/nav2_params_dwa_baseline.yaml - DWA controller config for comparison
- scripts/benchmark_controller_comparison.py - records oscillation + jerk metrics
- scripts/plot_controller_comparison.py - generates comparison chart

## How to run

### 1. Launch with MPPI

Terminal 1 (Gazebo) and Terminal 2 (SLAM) same as before. Terminal 3:

    ros2 launch turtlebot3_navigation2 navigation2.launch.py \
      use_sim_time:=True \
      params_file:=$HOME/ros2_ws/src/rrt_star_planner/config/nav2_params_mppi.yaml

### 2. Verify MPPI loaded

    ros2 param get /controller_server FollowPath.plugin

Expected: nav2_mppi_controller::MPPIController

### 3. Record benchmark (new terminal, BEFORE sending the goal)

    python3 ~/ros2_ws/src/rrt_star_planner/scripts/benchmark_controller_comparison.py MPPI 60

Then immediately send a Nav2 Goal in RViz2.

### 4. Switch to DWA baseline and repeat

Ctrl+C the Nav2 terminal, relaunch with nav2_params_dwa_baseline.yaml,
then run the benchmark script again with label DWA. Send the SAME goal.

### 5. Generate comparison chart

    python3 ~/ros2_ws/src/rrt_star_planner/scripts/plot_controller_comparison.py

## Key MPPI parameters

| Parameter | Value | Meaning |
|---|---|---|
| batch_size | 2000 | Number of sampled trajectories per iteration |
| time_steps | 56 | Prediction horizon length |
| model_dt | 0.05 | Time step duration (s) |
| motion_model | DiffDrive | Matches TurtleBot3 differential drive |
| vx_max / wz_max | 0.26 / 1.82 | Same limits as DWA baseline (fair comparison) |
| CostCritic.cost_weight | 3.81 | Obstacle avoidance strength |
| PathAlignCritic.cost_weight | 14.0 | How closely to follow the RRT* path |
