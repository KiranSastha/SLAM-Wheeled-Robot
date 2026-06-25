# SLAM Wheeled Robot — Project Handbook

**A complete guide for anyone reading this project for the first time**

---

## Table of Contents

1. [What This Project Is](#1-what-this-project-is)
2. [How Autonomous Navigation Works](#2-how-autonomous-navigation-works)
3. [The Technology Stack — Why These Tools](#3-the-technology-stack--why-these-tools)
4. [Core Concepts Explained](#4-core-concepts-explained)
   - 4.1 [ROS2 and Nav2](#41-ros2-and-nav2)
   - 4.2 [Gazebo Simulation](#42-gazebo-simulation)
   - 4.3 [RRT and RRT* — Global Planning](#43-rrt-and-rrt--global-planning)
   - 4.4 [DWA and MPPI — Local Control](#44-dwa-and-mppi--local-control)
   - 4.5 [SLAM — Simultaneous Localization and Mapping](#45-slam--simultaneous-localization-and-mapping)
   - 4.6 [AMCL — Adaptive Monte Carlo Localization](#46-amcl--adaptive-monte-carlo-localization)
5. [Repository Structure](#5-repository-structure)
6. [Environment Setup](#6-environment-setup)
7. [Running the System](#7-running-the-system)
   - 7.1 [Mode A — Live SLAM Mapping](#71-mode-a--live-slam-mapping)
   - 7.2 [Mode B — Static Map with AMCL](#72-mode-b--static-map-with-amcl)
8. [Known Issues and Fixes](#8-known-issues-and-fixes)
9. [Phase-wise Results](#9-phase-wise-results)
   - 9.1 [Phase 0 — Environment Setup](#91-phase-0--environment-setup)
   - 9.2 [Phase 1 — RRT* Global Planner](#92-phase-1--rrt-global-planner)
   - 9.3 [Phase 2 — MPPI Local Controller](#93-phase-2--mppi-local-controller)
   - 9.4 [Phase 3 — SLAM Integration](#94-phase-3--slam-integration)
10. [What Is Still Pending](#10-what-is-still-pending)
11. [Glossary](#11-glossary)

---

## 1. What This Project Is

This project implements a complete autonomous navigation system for a differential-drive wheeled mobile robot, built entirely in ROS2 Jazzy and simulated in Gazebo. It started from a MATLAB-based baseline that used a vanilla RRT global planner and a DWA local controller, and systematically improved every component of that stack.

The four improvements made over the baseline are:

| # | What Changed | From | To |
|---|---|---|---|
| 1 | Global Planner | RRT | RRT* |
| 2 | Simulation Platform | MATLAB | ROS2 + Gazebo |
| 3 | Local Controller | DWA | MPPI |
| 4 | Localization | Static/embedded | SLAM Toolbox + AMCL |

The result is a navigation stack that is smoother, more reliable, hardware-portable, and capable of operating on a pre-built map without re-mapping every session.

The robot used for simulation is the **TurtleBot3 Burger**, a standard ROS2 research platform with differential drive (two independently driven wheels). All results, benchmarks, and parameters in this project are validated in simulation. Hardware deployment on a physical wheeled robot is planned for a later phase.

---

## 2. How Autonomous Navigation Works

Before diving into the code, it helps to understand what a mobile robot navigation system actually does, conceptually.

When a robot needs to get from point A to point B, three problems must be solved simultaneously:

**Problem 1 — Where am I?** (Localization)
The robot needs to know its position on a map. It does this using sensor data (typically a LiDAR or depth sensor) and comparing what it currently sees to what it expects to see based on the map.

**Problem 2 — Where should I go?** (Global Planning)
Given the robot's current position and a goal position, a global planner computes a high-level path through the environment — a sequence of waypoints from start to goal, avoiding known obstacles.

**Problem 3 — How do I get there right now?** (Local Control)
The global plan is an ideal path, but the real world is messy. The local controller takes the global plan and the robot's current position and computes actual wheel velocity commands at high frequency to follow that path while avoiding unexpected obstacles.

These three problems are solved by different components running in parallel:

```
Sensors (LiDAR) → SLAM/AMCL (localization)
                         ↓
               Global Planner (RRT*)
                         ↓
             Local Controller (MPPI)
                         ↓
            Velocity Commands → Robot Wheels
```

This project addresses all three using Nav2, ROS2's standard navigation framework.

---

## 3. The Technology Stack — Why These Tools

### ROS2 Jazzy (vs MATLAB)

The baseline used MATLAB for simulation. MATLAB is a fine prototyping tool, but it is not portable — code written in MATLAB cannot run on actual robot hardware without a full rewrite or expensive toolboxes. ROS2 (Robot Operating System 2) is the industry and research standard for robot software. Code written in ROS2 can run on a laptop, a simulation, or directly on a robot's onboard computer with minimal changes. Moving to ROS2 Jazzy (the current LTS release) makes the entire stack hardware-deployable.

### Gazebo Harmonic (vs MATLAB Simulation)

Gazebo is a physics-based 3D robot simulator that integrates natively with ROS2. It simulates sensor noise, robot dynamics, collision physics, and real-time sensor output — making simulation results much more predictive of real hardware behavior than a simplified MATLAB model.

### Nav2

Nav2 is ROS2's navigation framework. It provides a complete, configurable pipeline for autonomous navigation including global planners, local controllers, costmaps, recovery behaviors, and lifecycle management. Rather than building all of this from scratch, this project uses Nav2 as the framework and plugs in custom planners (RRT*) while also using Nav2's built-in MPPI controller.

### TurtleBot3 Burger

TurtleBot3 Burger is a small, lightweight, differential-drive research robot maintained by ROBOTIS. Its ROS2 Gazebo simulation package is well-maintained and widely used, making it an ideal platform for developing and validating navigation algorithms before hardware deployment.

---

## 4. Core Concepts Explained

### 4.1 ROS2 and Nav2

**ROS2** stands for **Robot Operating System 2**. Despite the name, it is not an operating system — it is a middleware framework that provides the plumbing for robot software: a publish-subscribe message passing system, a service call system, a parameter server, and lifecycle management for nodes.

Everything in a ROS2 system is a **node** — a process that communicates with others by publishing and subscribing to **topics** (message streams) or calling **services** and **actions** (request-response mechanisms).

Key ROS2 concepts used in this project:

| Concept | What It Is | Example in This Project |
|---|---|---|
| Node | A single running process | `slam_toolbox`, `controller_server` |
| Topic | A named message stream (pub/sub) | `/cmd_vel`, `/scan`, `/map` |
| Action | A long-running task with feedback | `/navigate_to_pose` |
| Launch file | A Python script that starts multiple nodes | `slam_nav2_rrtstar.launch.py` |
| Parameter file | A YAML config file for node settings | `nav2_params_full_phase3.yaml` |
| `colcon` | ROS2's build tool | Used to build the custom planner packages |

**Nav2** (Navigation 2) is the ROS2 navigation stack. It is a collection of nodes and a framework that together implement the full navigation pipeline. The key Nav2 servers used here are:

- `planner_server` — runs the global planner (RRT* in this project)
- `controller_server` — runs the local controller (MPPI in this project)
- `map_server` — serves a static pre-built map to the rest of the system
- `amcl` — localizes the robot on the static map
- `bt_navigator` — orchestrates the navigation task using a Behavior Tree
- `collision_monitor` — a safety layer that modifies velocity commands before they reach the robot

### 4.2 Gazebo Simulation

Gazebo Harmonic is the physics simulator used in this project. When you launch the simulation, Gazebo starts a virtual TurtleBot3 Burger in a virtual environment (the `turtlebot3_world`). This virtual robot publishes the same ROS2 topics a real robot would — `/scan` for LiDAR data, `/odom` for wheel odometry, and subscribes to `/cmd_vel` for velocity commands.

The **ROS-Gazebo bridge** (`ros_gz_bridge`) is a critical component — it translates between Gazebo's internal message format and ROS2 message types. One known issue with this bridge in the Jazzy/Harmonic combination is a message type mismatch on `/cmd_vel` that silently breaks movement. This is documented and fixed in [Section 8](#8-known-issues-and-fixes).

### 4.3 RRT and RRT* — Global Planning

#### What is RRT?

**RRT** stands for **Rapidly-exploring Random Tree**. It is a sampling-based path planning algorithm invented by Steven LaValle in 1998. Instead of searching a grid or a graph, RRT builds a tree by randomly sampling points in the robot's free space and connecting them to the nearest existing node in the tree.

The basic algorithm:
1. Start with a tree rooted at the robot's current position.
2. Sample a random point in the free space.
3. Find the nearest existing node in the tree.
4. Extend the tree toward the random point by a fixed step size.
5. If the extension does not collide with an obstacle, add it to the tree.
6. Repeat until the goal is reached or a time limit is hit.

RRT is fast and works well in high-dimensional spaces. However, the paths it produces are jagged and non-optimal — the algorithm makes no attempt to find the best path, only any valid path.

#### What is RRT*?

**RRT\*** (pronounced "RRT star") is an asymptotically optimal extension of RRT introduced by Karaman and Frazzoli in 2011. It adds two key operations on top of RRT:

1. **Choose parent** — When adding a new node, instead of connecting it to the nearest node, it searches a neighborhood and connects it through whichever nearby node produces the lowest cost path from the start.

2. **Rewire** — After adding the new node, it checks whether any nearby nodes would have a lower-cost path if they were connected through the new node instead of their current parent, and rewires the tree if so.

These two operations mean the tree continuously improves its paths as more nodes are added. Given enough time, RRT* converges to the optimal path. In practice, even with limited iterations, it produces significantly smoother and shorter paths than vanilla RRT.

**In this project**, the RRT* planner is implemented as a custom Nav2 plugin in C++ (`rrt_star_planner`). It is registered with Nav2 via `plugins.xml` and configured in the Nav2 parameter YAML files.

#### RRT* Configuration (nav2_params_rrtstar.yaml)

The key parameters that control the RRT* planner:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "rrt_star_planner/RRTStarPlanner"
      max_iterations: 5000        # Max tree nodes before giving up
      step_size: 0.05             # How far to extend each new branch (meters)
      goal_tolerance: 0.1         # How close to goal counts as success (meters)
      search_radius: 0.3          # Neighborhood radius for rewiring (meters)
```

**What these parameters do:**
- `max_iterations` — Higher values give better paths but take longer. 5000 is a good balance for real-time navigation.
- `step_size` — Smaller values produce smoother trees but require more iterations to cover the same distance. Too large and the planner struggles with narrow passages.
- `goal_tolerance` — The planner stops when a tree node lands within this distance of the goal. Too tight and the planner may run out of iterations before reaching it.
- `search_radius` — The radius within which nodes are considered for rewiring. Larger radii produce better paths but add computational cost per iteration.

### 4.4 DWA and MPPI — Local Control

The local controller is responsible for converting the global plan into actual velocity commands for the robot at high frequency (typically 20 Hz or more). It must follow the global plan while reacting to local obstacles and respecting the robot's physical movement constraints (it cannot instantly accelerate, it cannot drive sideways, etc.).

#### What is DWA?

**DWA** stands for **Dynamic Window Approach**. It was introduced by Fox, Burgard, and Thrun in 1997. DWA works by:

1. Considering a "dynamic window" of velocity commands that the robot can actually execute given its current speed and acceleration limits.
2. Sampling a set of candidate (linear velocity, angular velocity) pairs from that window.
3. Forward-simulating each candidate for a short time horizon.
4. Scoring each simulated trajectory on criteria like goal progress, obstacle clearance, and velocity.
5. Sending the highest-scoring command to the robot.

DWA is fast and widely used. Its main weakness is that it uses short, simple arc trajectories (constant velocity pairs), which leads to oscillatory, jerky behavior — especially when the robot needs to make tight turns or navigate near obstacles. It also tends to produce frequent direction reversals and high angular velocity spikes.

#### What is MPPI?

**MPPI** stands for **Model Predictive Path Integral**. It was introduced by Williams et al. at Georgia Tech in 2016 and is now Nav2's recommended modern local controller, replacing both DWA and TEB.

MPPI works fundamentally differently from DWA:

1. **Sample many rollouts** — At each control step, MPPI samples thousands of random control sequences (sequences of velocity commands over a time horizon, e.g. 5 seconds into the future).
2. **Simulate all rollouts in parallel** — Each sampled sequence is forward-simulated using the robot's motion model.
3. **Score each rollout** — A cost function evaluates each rollout on goal progress, obstacle avoidance, path following, and smoothness.
4. **Compute a weighted average** — The final control command is a weighted average of all sampled sequences, where better (lower-cost) sequences receive exponentially higher weights. This is the "path integral" part — the controller integrates over all paths proportionally to their cost.

The result is a controller that is inherently smooth (because it averages over many trajectories), predictive (because it plans over a long horizon), and graceful around obstacles (because it can consider many paths simultaneously). It also has zero oscillations by nature — the averaging process filters out high-frequency noise in the control output.

**In this project**, MPPI ships natively with `ros-jazzy-navigation2` — no source build required.

#### MPPI Configuration (nav2_params_mppi.yaml)

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56              # Number of steps in each rollout
      model_dt: 0.05              # Time between steps in a rollout (seconds)
      batch_size: 2000            # Number of random rollouts sampled per control step
      vx_std: 0.2                 # Std deviation for linear velocity noise sampling
      wz_std: 0.4                 # Std deviation for angular velocity noise sampling
      vx_max: 0.5                 # Max forward speed (m/s)
      vx_min: -0.35               # Max reverse speed (m/s)
      wz_max: 1.9                 # Max turning speed (rad/s)
      iteration_count: 1          # Optimization iterations (1 is standard)
      temperature: 0.3            # Controls how sharply good trajectories are weighted
```

**What these parameters do:**
- `time_steps × model_dt` — Total planning horizon. 56 × 0.05 = 2.8 seconds ahead. Longer horizons produce smoother behavior but increase compute cost.
- `batch_size` — More rollouts → better decisions but higher CPU cost. 2000 is a practical balance for a standard CPU.
- `vx_std` / `wz_std` — How much random variation is injected into sampled control sequences. Higher values explore more of the velocity space but can produce noisier behavior.
- `temperature` — A lower value (closer to 0) makes the controller sharply prefer the single best trajectory. A higher value (closer to 1) produces a smoother blend of trajectories.

#### Why MPPI instead of TEB?

The original plan called for TEB (Timed Elastic Band) as the local planner upgrade. TEB has no official, stable ROS2 Jazzy release — the upstream repository only has an experimental branch with an open maintenance issue. Building it from source risked significant dependency failures. MPPI is Nav2's own modern successor to both TEB and DWB, confirmed by Nav2's lead maintainer and official tuning guide, and ships natively with the Nav2 package.

### 4.5 SLAM — Simultaneous Localization and Mapping

**SLAM** stands for **Simultaneous Localization and Mapping**. The name captures the problem precisely: how does a robot build a map of an unknown environment while simultaneously figuring out where it is in that map? These two problems are circular — you need a map to localize, and you need to know your location to build an accurate map.

Modern SLAM algorithms solve this with probabilistic methods that maintain uncertainty estimates about both the map and the robot's pose, and continuously refine both as new sensor data arrives.

**SLAM Toolbox** is the package used in this project. It is the officially recommended SLAM solution for ROS2 and supports:
- Online async mapping (map built in real time as the robot explores)
- Map serialization (saving the map to disk in `.pgm` and `.yaml` format)
- Map reuse (loading a saved map for later localization-only use)

The map produced by SLAM Toolbox is an **occupancy grid** — a 2D grid where each cell is marked as free (white), occupied (black, i.e. a wall or obstacle), or unknown (grey). Nav2's costmap layers use this occupancy grid to determine where the robot can and cannot navigate.

In this project, the SLAM map produced in Phase 3 is a **94 × 103 cell grid at 0.05m resolution**, covering the simulated TurtleBot3 world.

#### Map files

After SLAM mapping, two files are saved:

```
maps/phase3_map.pgm   ← The actual map image (greyscale occupancy grid)
maps/phase3_map.yaml  ← Metadata: resolution, origin, thresholds
```

The YAML file looks like:
```yaml
image: phase3_map.pgm
resolution: 0.05          # Each cell = 5cm × 5cm in the real world
origin: [-1.25, -1.25, 0] # World coordinates of the map's bottom-left corner
negate: 0
occupied_thresh: 0.65     # Cells with probability > 0.65 are treated as occupied
free_thresh: 0.25         # Cells with probability < 0.25 are treated as free
```

### 4.6 AMCL — Adaptive Monte Carlo Localization

**AMCL** stands for **Adaptive Monte Carlo Localization**. Once a map has been built, you do not need to run SLAM every time — you can localize purely using AMCL.

AMCL works by maintaining a **particle filter** — a set of hypotheses (particles) about where the robot might be on the map. Each particle represents one possible (x, y, θ) pose of the robot. At startup, particles are distributed around the initial pose estimate. As the robot moves and receives sensor data, the particles are:

1. **Propagated** — Each particle is moved according to the robot's wheel odometry (with noise to account for odometry drift).
2. **Weighted** — Each particle is scored based on how well the current LiDAR scan matches the map from that particle's hypothesized location. Particles in locations that match the scan well get higher weight.
3. **Resampled** — Particles are resampled proportionally to their weights, so the particle cloud converges toward the true pose over time.

The "adaptive" part means AMCL dynamically adjusts the number of particles based on uncertainty — more particles when the robot is lost, fewer when it is confidently localized.

AMCL requires an **initial pose estimate** (`/initialpose` topic) to seed the particle filter at startup. Without this, particles are uniformly distributed across the entire map and the robot cannot localize quickly.

**Critical timing note:** When loading a static map, AMCL must receive `/initialpose` within approximately 30 seconds of Nav2 launch, or the navigation bringup aborts. See Section 8 for details.

---

## 5. Repository Structure

```
slam-wheeled-robot-ros2/
│
├── README.md                        ← Project overview and quick start
│
├── rrt_star_planner/                ← MAIN PACKAGE (use this for Phases 1–3)
│   ├── include/rrt_star_planner/
│   │   └── rrt_star_planner.hpp     ← C++ header: RRT* class declaration
│   ├── src/
│   │   └── rrt_star_planner.cpp     ← C++ source: RRT* algorithm implementation
│   ├── config/
│   │   ├── nav2_params_rrtstar.yaml       ← Phase 1: RRT* planner only
│   │   ├── nav2_params_mppi.yaml          ← Phase 2: MPPI controller only
│   │   ├── nav2_params_dwa_baseline.yaml  ← DWA config (for comparison benchmarks)
│   │   ├── nav2_params_full_phase3.yaml   ← Phase 3+: RRT* + MPPI combined (USE THIS)
│   │   └── slam_toolbox_params.yaml       ← SLAM Toolbox configuration
│   ├── launch/
│   │   └── slam_nav2_rrtstar.launch.py    ← Main launch file
│   ├── scripts/
│   │   ├── benchmark_rrt_vs_rrtstar.py         ← Runs Phase 1 benchmark
│   │   ├── benchmark_controller_comparison.py   ← Runs Phase 2 benchmark
│   │   └── plot_controller_comparison.py        ← Generates Phase 2 comparison chart
│   ├── PHASE2_README.md             ← Phase 2 specific notes
│   ├── plugins.xml                  ← Registers RRT* as a Nav2 planner plugin
│   ├── CMakeLists.txt               ← CMake build config for the C++ planner
│   └── package.xml                  ← ROS2 package manifest (dependencies)
│
├── rrt_planner/                     ← BASELINE PACKAGE (vanilla RRT, benchmarking only)
│   ├── include/rrt_planner/
│   ├── src/
│   ├── config/
│   │   └── nav2_params_rrt_baseline.yaml  ← RRT baseline Nav2 config
│   ├── scripts/
│   │   ├── benchmark_comparison.py    ← Runs Phase 1 comparison benchmark
│   │   └── plot_comparison.py         ← Generates Phase 1 comparison chart
│   ├── plugins.xml
│   ├── CMakeLists.txt
│   └── package.xml
│
├── maps/
│   ├── phase3_map.pgm               ← Saved occupancy grid (image)
│   └── phase3_map.yaml              ← Map metadata (resolution, origin, thresholds)
│
├── results/
│   ├── Phase-wise_Improvement.md         ← Full results write-up per phase
│   ├── rrt_vs_rrtstar_comparison.png     ← Phase 1 benchmark chart
│   ├── mppi_vs_dwa_comparison.png        ← Phase 2 benchmark chart
│   ├── benchmark_rrt_star_*.csv          ← Phase 1 raw data
│   ├── benchmark_rrt_*.csv               ← Phase 1 baseline raw data
│   └── controller_*.csv                  ← Phase 2 raw data
│
└── docs/
    └── DEBUGGING_LOG.md             ← Full record of issues encountered and how they were fixed
```

**Which config file should I use?**

| Situation | Config File |
|---|---|
| Running Phase 1 only (RRT* planner, default DWA controller) | `nav2_params_rrtstar.yaml` |
| Running Phase 2 only (default planner, MPPI controller) | `nav2_params_mppi.yaml` |
| Comparing against DWA baseline | `nav2_params_dwa_baseline.yaml` |
| Running Phase 3 or any full system run | `nav2_params_full_phase3.yaml` ← **use this** |

---

## 6. Environment Setup

### Prerequisites

Before cloning this repository, the following must be installed on an Ubuntu 24.04 machine:

**ROS2 Jazzy** (full desktop install)
```bash
# Follow the official ROS2 Jazzy installation guide:
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt install ros-jazzy-desktop
```

**TurtleBot3 packages**
```bash
sudo apt install ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-navigation2
```

**Nav2 and SLAM Toolbox**
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

**ROS-Gazebo Bridge**
```bash
sudo apt install ros-jazzy-ros-gz-bridge
```

### Setting Up the Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/KiranSastha/SLAM-Wheeled-Robot.git .

# Build both packages
cd ~/ros2_ws
colcon build --packages-select rrt_star_planner rrt_planner --symlink-install

# Source the workspace
source install/setup.bash
```

> **Tip:** Add `source ~/ros2_ws/install/setup.bash` to your `~/.bashrc` so you do not have to run it every terminal session.

### Environment Variable

TurtleBot3 packages require this environment variable to be set:
```bash
export TURTLEBOT3_MODEL=burger
```

Add this to `~/.bashrc` alongside the workspace source line.

---

## 7. Running the System

The system can be launched in two modes depending on whether you want to build a new map or use the saved one.

### 7.1 Mode A — Live SLAM Mapping

Use this when you want the robot to explore and build a map in real time.

**Terminal 1 — Start Gazebo**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Wait for Gazebo to fully load (the 3D world with the robot should be visible).

**Terminal 2 — Start SLAM Toolbox**
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
Wait for SLAM Toolbox to register the `/scan` topic. You will see a log line like `Registering sensor`.

**Terminal 3 — Start Nav2 (wait 5 seconds after Terminal 2)**
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  params_file:=$HOME/ros2_ws/src/rrt_star_planner/config/nav2_params_full_phase3.yaml \
  use_rviz:=False
```

**Set initial pose**
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

**Send a navigation goal**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.3, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

**Saving the map (after exploration)**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_map
```
This saves `my_map.pgm` and `my_map.yaml`.

### 7.2 Mode B — Static Map with AMCL

Use this when you already have a saved map and want to localize and navigate on it without re-mapping.

**Terminal 1 — Start Gazebo** (same as Mode A)

**Terminal 2 — Start Nav2 with the saved map**
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/ros2_ws/maps/phase3_map.yaml \
  params_file:=$HOME/ros2_ws/src/rrt_star_planner/config/nav2_params_full_phase3.yaml \
  use_rviz:=False
```

**⚠️ CRITICAL: Publish `/initialpose` immediately** (within ~10 seconds of Nav2 launch, not after it looks settled)
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Then send a navigation goal as in Mode A.

---

## 8. Known Issues and Fixes

These are real issues encountered during development. Both must be addressed for the system to work correctly.

---

### Fix 1 — Gazebo cmd_vel Type Mismatch (One-time, requires sudo)

**What happens without this fix:**
The robot plans a path correctly. The plan appears in logs as accepted. But the robot never moves physically in Gazebo. No error is shown.

**Root cause:**
TurtleBot3's stock Gazebo bridge config publishes `/cmd_vel` as `geometry_msgs/msg/TwistStamped`. However, Nav2's velocity pipeline (controller_server → velocity_smoother → collision_monitor) publishes plain `geometry_msgs/msg/Twist` (no timestamp header). The bridge silently drops every velocity command because the types do not match.

**The fix (run once per machine):**
```bash
# Backup the original
sudo cp /opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml \
        /opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml.bak

# Apply the fix
sudo sed -i 's|geometry_msgs/msg/TwistStamped" # If you use Twist.*|geometry_msgs/msg/Twist"|' \
        /opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml
```

**Verify the fix:**
```bash
ros2 node info /ros_gz_bridge | grep cmd_vel
# Expected output: geometry_msgs/msg/Twist
```

This is a one-time fix per machine. Once applied, it persists across sessions.

---

### Fix 2 — AMCL /initialpose Timing Race Condition (Every session, static map mode)

**What happens without this fix:**
When using a pre-built static map (Mode B), the entire Nav2 navigation stack bringup aborts with a timeout error, even though AMCL itself never shows an error message.

**Root cause:**
Nav2's bringup sequence waits for AMCL to publish the `map → odom` transform before activating the global costmap. AMCL will not publish that transform until it receives `/initialpose`. Nav2's bringup has a hard ~30-second timeout for costmap activation. If `/initialpose` is not published in time, the bringup fails and all nodes shut down.

The confusing part: AMCL does not log an error. The failure manifests as a timeout in an unrelated-looking component, which makes diagnosis non-obvious.

**The fix:**
Publish `/initialpose` immediately after launching Nav2 — ideally within 10 seconds, before the system has fully "settled." Do not wait for log output to look ready before publishing.

```bash
# Publish this RIGHT AWAY after starting Nav2 — don't wait
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

This fix must be applied every session when using Mode B (static map). In Mode A (live SLAM), SLAM Toolbox publishes the `map → odom` transform directly, so this issue does not apply.

---

## 9. Phase-wise Results

### 9.1 Phase 0 — Environment Setup

**Objective:** Migrate the navigation stack from MATLAB to ROS2 + Gazebo.

**What was done:**
- Installed ROS2 Jazzy, TurtleBot3 packages, Nav2, SLAM Toolbox, and Gazebo Harmonic.
- Set up the ROS2 workspace and verified basic robot operation in Gazebo.
- Validated the Nav2 default stack (NavFn planner + DWA controller) as a working baseline.

**Outcome:** A functioning ROS2 + Gazebo environment with a working baseline navigation stack. This is the foundation all subsequent phases build on.

---

### 9.2 Phase 1 — RRT* Global Planner

**Objective:** Replace the default Nav2 global planner (NavFn) with a custom RRT* implementation and demonstrate improvement over vanilla RRT.

**Benchmark setup:** 20 trials per planner, same start and goal positions, same environment.

| Metric | RRT | RRT* | Result |
|---|---|---|---|
| Path Length | ≈ 1.82 m | ≈ 1.86 m | Comparable |
| Planning Time | ≈ 110 ms | ≈ 55 ms | RRT* is 2× faster |
| Path Smoothness (total curvature) | ≈ 6.6 rad | ≈ 2.5 rad | 56.8% improvement |
| Success Rate (20 trials) | 2/20 (10%) | 12/20 (60%) | 6× improvement |

**Key observations:**
- RRT* produces dramatically smoother paths, measured as total absolute curvature (sum of direction changes along the path in radians). Lower is smoother.
- The 6× improvement in success rate is striking — vanilla RRT succeeded on only 2 out of 20 trials. This is largely because RRT's jagged paths frequently triggered Nav2's path validation or caused the controller to fail.
- RRT* planning time is actually lower than RRT despite doing more work per node. This is because the rewiring step finds better paths earlier, allowing the planner to terminate sooner.
- Path lengths are nearly identical — RRT* does not find dramatically shorter paths, but makes the path much more navigable.

**Chart:** `results/rrt_vs_rrtstar_comparison.png`

---

### 9.3 Phase 2 — MPPI Local Controller

**Objective:** Replace the Nav2 default DWA local controller with MPPI and demonstrate smoother robot motion.

**Benchmark setup:** Multiple navigation runs with identical start/goal pairs, logging velocity command streams.

| Metric | DWA | MPPI | Result |
|---|---|---|---|
| Peak Angular Velocity | ±1.8 rad/s | ±0.5 rad/s | ~70% reduction |
| Command Oscillations | High (3 events) | None (0 events) | Eliminated |
| Steering Reversals | Frequent | Minimal | Significantly reduced |
| Velocity Continuity | Discontinuous | Continuous | Improved |
| Recovery Behaviors Triggered | 15 (aborted 1 run) | 0 | Eliminated |

**Key observations:**
- DWA produced angular velocity spikes of ±1.8 rad/s — aggressive turn commands that cause the physical robot to lurch. MPPI's peak is ±0.5 rad/s, a 70% reduction, meaning much gentler turns.
- DWA triggered recovery behaviors on one run (15 recoveries, eventually aborting). MPPI completed every run with zero recoveries.
- MPPI's velocity profile is continuous and smooth — there are no abrupt jumps in commanded speed or direction. DWA's profile shows repeated discontinuities.
- This smoother behavior directly translates to hardware readiness: a robot that lurches and oscillates in simulation will behave the same way on hardware, putting more wear on motors and making the robot unpredictable.

**Chart:** `results/mppi_vs_dwa_comparison.png`

---

### 9.4 Phase 3 — SLAM Integration

**Objective:** Validate the full "build once, localize forever" SLAM workflow — live mapping with SLAM Toolbox, map save, map reload, and localization-only operation with AMCL.

| Stage | Outcome |
|---|---|
| Live map construction (SLAM Toolbox) | ✅ Successful — 94×103 cell map at 0.05m resolution |
| Map save (`map_saver_cli`) | ✅ Successful — `.pgm` + `.yaml` generated |
| Static map reload (`map_server`) | ✅ Successful |
| AMCL localization on static map | ✅ Successful — `map → odom` transform stable |
| RRT* + MPPI navigation on static map | ✅ Successful — Goal finished with status: SUCCEEDED, 0 recoveries |

**Key observations:**
- AMCL correctly localizes the robot on the previously-saved static map without any live SLAM process running. The mapping and localization stages are independently functional.
- The AMCL `/initialpose` timing issue (Fix 2 above) was the only integration problem encountered. Once understood and resolved, the full pipeline operated correctly on the first subsequent attempt.
- The complete pipeline — map load → AMCL localize → RRT* plan → MPPI control → goal reached — operated end-to-end without manual intervention.

**Practical implication:** For hardware deployment (Phase 6), the robot does not need to re-map every time it is turned on. It maps once, saves the map, and every subsequent session loads the saved map and localizes using AMCL. This is the expected operational mode for a real deployment.

---

## 10. What Is Still Pending

The following phases are planned but not yet implemented:

**Phase 4 — Dynamic Replanning and Obstacle Response**
The current stack plans around static map obstacles only. This phase will add dynamic replanning — the ability to detect unexpected obstacles (not on the map) and replan around them in real time. This likely involves the Nav2 `collision_monitor` and a recovery behavior tree.

**Phase 5 — Full System Integration and Performance Evaluation**
A comprehensive end-to-end evaluation of the complete stack, running multiple diverse navigation tasks, measuring accumulated metrics, and producing a final performance report.

**Phase 6 — Hardware Validation**
Deploying the stack on a physical wheeled mobile robot. The map built in simulation will need to be replaced with a real-world map. The primary validation task is confirming that the parameters tuned in simulation translate to acceptable hardware behavior, and re-tuning where they do not.

---

## 11. Glossary

| Term | Full Form | What It Means in This Project |
|---|---|---|
| ROS2 | Robot Operating System 2 | Middleware framework for robot software. Provides message passing, services, and lifecycle management between nodes. |
| Nav2 | Navigation 2 | ROS2's official navigation stack. Manages global planning, local control, costmaps, and recovery behaviors. |
| SLAM | Simultaneous Localization and Mapping | Building a map while also figuring out where you are in it. Used in Phase 3 to produce the occupancy grid map. |
| AMCL | Adaptive Monte Carlo Localization | Particle-filter-based localization on a pre-built static map. Estimates the robot's pose without re-mapping. |
| RRT | Rapidly-exploring Random Tree | Sampling-based path planning algorithm. Fast but produces jagged, non-optimal paths. Used as the baseline. |
| RRT* | Rapidly-exploring Random Tree Star | Asymptotically optimal extension of RRT. Adds parent selection and rewiring for smoother, higher-quality paths. |
| DWA | Dynamic Window Approach | Local controller that samples velocity commands and picks the highest-scoring one. Produces oscillatory motion. Used as the baseline. |
| MPPI | Model Predictive Path Integral | Local controller that samples thousands of trajectory rollouts and takes a weighted average. Produces smooth, continuous motion. |
| TEB | Timed Elastic Band | A local planning algorithm originally planned for this project. Replaced by MPPI due to no stable ROS2 Jazzy release. |
| Costmap | — | A 2D grid overlaid on the map that marks areas as free or forbidden for navigation. Nav2 maintains a global costmap and a local costmap. |
| Occupancy Grid | — | The map format used by Nav2. Each cell is free (white), occupied (black), or unknown (grey). |
| `colcon` | — | ROS2's build tool. Used to compile the C++ planner packages. |
| `cmd_vel` | Command Velocity | The ROS2 topic (`/cmd_vel`) on which velocity commands are published to the robot. Type: `geometry_msgs/msg/Twist`. |
| `Twist` | — | A ROS2 message type containing linear and angular velocity components. The basic format for robot velocity commands. |
| TF | Transform | ROS2's coordinate frame system. Navigation relies on transforms between frames: `map → odom → base_link`. |
| `/initialpose` | — | A ROS2 topic used to give AMCL its initial guess of where the robot is on the map. Must be sent promptly at startup. |
| Differential Drive | — | A robot drive configuration with two independently controlled wheels (left and right). The TurtleBot3 Burger uses this. |
| TurtleBot3 Burger | — | The simulated robot platform used in this project. A small, lightweight differential-drive robot by ROBOTIS. |
| Gazebo | — | Physics-based 3D robot simulator. Used to simulate the TurtleBot3 and its environment in this project. |
| Gazebo Harmonic | — | The specific Gazebo version compatible with ROS2 Jazzy. |
| `ros_gz_bridge` | ROS-Gazebo Bridge | A node that translates messages between Gazebo's internal format and ROS2 topics. |
| LiDAR | Light Detection and Ranging | The sensor used by the TurtleBot3 to detect obstacles. Publishes a `/scan` topic with distance readings in all directions. |
| Particle Filter | — | The probabilistic algorithm underlying AMCL. Represents robot pose uncertainty as a set of weighted hypotheses (particles). |
| DXA | — | Unit of measurement in DOCX/Word processing (not related to this project). |
| Phase | — | A development stage in this project's roadmap. Phases 0–3 are complete; Phases 4–6 are pending. |

---

*Handbook last updated: Phase 3 completion. For the full technical debugging record, see `docs/DEBUGGING_LOG.md`.*
