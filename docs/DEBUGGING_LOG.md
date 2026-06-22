# Debugging Log — Phase 1 (RRT*) Integration

This log documents real issues encountered integrating a custom Nav2
global planner plugin on ROS2 Jazzy + Gazebo Harmonic, and how each was
diagnosed and fixed. Kept because the diagnostic process itself
demonstrates the debugging methodology, which is valuable to show in
a viva/defense — these are not typical "tutorial" bugs.

## 1. ROS2 API changes: Humble → Jazzy

**Symptom:** Compile errors —
`'CostmapROS' is not a member of 'nav2_costmap_2d'`

**Cause:** Jazzy renamed `nav2_costmap_2d::CostmapROS` to `Costmap2DROS`,
and `GlobalPlanner::createPlan()` gained a third `cancel_checker`
argument not present in Humble-era tutorials/examples.

**Fix:** Updated plugin header/cpp to the Jazzy API signature:
```cpp
void configure(..., std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
nav_msgs::msg::Path createPlan(..., std::function<bool()> cancel_checker) override;
```

## 2. CMakeLists referencing a non-existent `launch/` directory

**Symptom:** `ament_cmake_symlink_install_directory() can't find .../launch`

**Cause:** `install(DIRECTORY config launch scripts ...)` listed a
directory that didn't exist yet in the package.

**Fix:** Removed `launch` from the install line until the directory
was actually created, then added it back once populated. Also required
clearing stale `build/`/`install/` artifacts — CMake caches the
configure step and doesn't always pick up CMakeLists changes without
a clean rebuild.

## 3. Empty YAML lists break Jazzy's launch parameter parser

**Symptom:**
```
Expected 'value' to be one of [float, int, str, bool, bytes], but got '()' of type 'tuple'
```

**Cause:** Any empty YAML list (`polygons: []`, `observation_sources: []`,
`dock_plugins: []`, `docks: []`) gets parsed by Jazzy's launch system as
Python's empty tuple `()`, which fails the ROS2 parameter type
validator. This is undocumented and produces a completely generic,
unhelpful error message with no indication of which field caused it.

**Diagnosis method:** Wrote a small script to scan the params YAML for
any line matching `: []$` — found the exact offending lines instantly
instead of bisecting the file manually.

**Fix:** Every optional/empty list parameter needs at least one
non-empty placeholder value:
```yaml
dock_plugins: ["dummy_dock"]
dummy_dock:
  plugin: "opennav_docking::SimpleChargingDock"
docks: ["dummy_dock_instance"]
dummy_dock_instance:
  type: "dummy_dock"
  frame: "map"
  pose: [0.0, 0.0, 0.0]
```

## 4. Stock `turtlebot3_navigation2` launch brings up nodes regardless of your params file

**Symptom:** `collision_monitor`, `docking_server`, and `route_server`
all failed bringup one after another across multiple debugging
sessions, even when the person's params file didn't reference them.

**Cause:** The Jazzy-era `turtlebot3_navigation2/launch/navigation2.launch.py`
has its own internal node list it always tries to bring up, independent
of the params file's `lifecycle_manager_navigation.node_names`. Removing
a node from your params file does not stop the launch file from trying
to load and configure it — it just means that node has no valid
parameters, which then fails configuration and aborts the *entire*
bringup (taking down already-active nodes like `bt_navigator`).

**Fix:** Rather than removing these nodes, added minimal valid
parameter blocks for each so they configure successfully.

## 5. RViz2 GLSL shader crash (GPU-driver specific, unrelated to ROS2)

**Symptom:**
```
GLSL link result: active samplers with a different type refer to the same texture image unit
```
RViz2 becomes unresponsive or crashes outright when displaying the map.

**Cause:** OGRE/RViz2 shader compatibility issue with this machine's
GPU driver — unrelated to Nav2, SLAM, or any of our configuration.

**Fix:** Bypassed RViz2 entirely for testing — used CLI tools instead:
```bash
ros2 topic pub --once /initialpose ...
ros2 action send_goal /navigate_to_pose ... --feedback
```
This is strictly better for benchmarking anyway since the benchmark
scripts already talk to Nav2 purely through the action API.

## 6. The real root cause: Twist vs TwistStamped message-type mismatch

**Symptom:** Every lifecycle node reports `active`, AMCL accepts poses,
`compute_path_to_pose` succeeds, but the robot never physically moves
in Gazebo. `navigation_time` increases but `distance_remaining` never
changes; `number_of_recoveries` climbs.

**Diagnosis method:**
1. Checked `/cmd_vel` topic info — found **two message types**
   registered on the same topic name: `Twist` and `TwistStamped`.
2. Traced the full velocity pipeline node-by-node using `ros2 node info`:
   `controller_server` → `/cmd_vel_nav` (Twist) → `velocity_smoother` →
   `/cmd_vel_smoothed` (Twist) → `collision_monitor` → `/cmd_vel` (Twist)
3. Checked the actual Gazebo bridge subscriber:
   `ros_gz_bridge` subscribes to `/cmd_vel` expecting **TwistStamped**.
4. Found the bridge config source file, which contained its own comment
   warning about exactly this:
   `ros_type_name: "geometry_msgs/msg/TwistStamped" # If you use Twist, you need to change the type to Twist`

**Cause:** TurtleBot3's stock bridge YAML defaults to `TwistStamped`
for Gazebo Harmonic compatibility, but the entire Nav2 velocity
pipeline (controller_server, velocity_smoother, collision_monitor)
publishes plain `Twist`. Every single velocity command was being
silently dropped by the bridge with no error, warning, or log message
anywhere in the stack.

**Fix:**
```bash
sudo cp .../turtlebot3_burger_bridge.yaml .../turtlebot3_burger_bridge.yaml.bak
sudo sed -i 's|TwistStamped.*|Twist"|' .../turtlebot3_burger_bridge.yaml
```
Verified via:
```bash
ros2 node info /ros_gz_bridge | grep cmd_vel
# geometry_msgs/msg/Twist
```

After this single fix, navigation worked end-to-end on the first
subsequent attempt — `Goal finished with status: SUCCEEDED`.

## 7. Incomplete baseline params file (partial override vs complete file)

**Symptom:** `Couldn't load critics! Caught exception: No critics
defined for FollowPath` — only on the vanilla RRT baseline launch,
never on the RRT* launch.

**Cause:** The baseline comparison params file was originally written
as a 14-line *partial override* containing only the `planner_server`
section, assuming Nav2 would merge it with defaults. It does not —
Nav2 requires `controller_server.FollowPath.critics` to be explicitly
defined or the controller fails to configure, aborting the entire
bringup.

**Fix:** Rebuilt the baseline file as a complete params file (same
full structure as the working RRT* file), changing only the
`GridBased.plugin` field to point at the vanilla `RRTPlanner` instead
of `RRTStarPlanner`.

## Key lessons for future phases

1. **Always log to a file** (`2>&1 | tee log.txt`) from the first
   launch attempt — partial/fragmented terminal scrollback cost
   significant time across this debugging session.
2. **Empty YAML lists are a recurring trap on Jazzy** — check any new
   params file for `: []` before launching.
3. **Partial params overrides are unreliable** — always start new
   config files from a known-complete, working file and modify only
   the specific section needed.
4. **Message-type mismatches fail silently** — when a system reports
   all-green but physically does nothing, check `ros2 topic info
   <topic> --verbose` for multiple registered types on one topic name.
5. **Full clean restarts work better than incremental patching** once
   more than one lifecycle node has failed in a session — stale
   `ros2 daemon` state and zombie processes caused several false leads.
