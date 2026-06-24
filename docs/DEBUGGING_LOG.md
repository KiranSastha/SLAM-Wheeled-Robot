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


---

# Debugging Log — Phase 2 (MPPI) Integration

## 8. MPPI CostCritic segfault — `consider_footprint` / costmap shape mismatch

**Symptom:** Nav2 container crashes with `SIGSEGV` immediately after loading
`ConstraintCritic`, right after a warning:
```
[controller_server]: Inconsistent configuration in collision checking.
Please verify the robot's shape settings in both the costmap and the cost critic.
```

**Cause:** `CostCritic.consider_footprint: true` requires the costmap to
define an actual `footprint` polygon. This project's costmap config uses
a simple circular `robot_radius: 0.22` everywhere (no footprint polygon
defined anywhere). With `consider_footprint: true` and no footprint to
read, the footprint-cost lookup inside `CostCritic` segfaults instead of
failing gracefully.

**Diagnosis method:** Searched for the exact warning text combined with
"segfault" — found official Nav2 Jazzy example configs that explicitly
use `consider_footprint: false` for radius-based (non-footprint) robots,
confirming the mismatch.

**Fix:**
```yaml
CostCritic:
  consider_footprint: false   # was: true
```

## 9. MPPI params file missing required Jazzy node configs

**Symptom:**
```
[collision_monitor]: Error while getting parameters: parameter 'observation_sources' is not initialized
[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

**Cause:** `nav2_params_mppi.yaml` was originally authored as a separate
file rather than branched from the already-fixed, complete
`nav2_params_rrtstar.yaml`. It never inherited the `collision_monitor`,
`route_server`, and `docking_server` blocks (see Phase 1 issue #4) — the
same category of bug, just not yet ported to this file.

**Fix:** Copied the complete, working blocks for all three nodes from
the verified-working RRT* params file, and added them to
`lifecycle_manager_navigation.node_names`.

**Lesson reinforced:** Any new Nav2 params file in this project should
always be created by copying the most recently fixed, complete file and
editing only the specific section needed — never started from a fresh
partial file. This is the third time a partial-override file has caused
a bringup failure (RRT baseline, MPPI, almost DWA baseline too).

## 10. DWA baseline comparison file had the same partial-override problem (caught proactively)

**Symptom:** None yet encountered — caught and fixed *before* launching,
by checking the file's line count (62 lines, clearly too short to be a
complete Nav2 params file) before running it.

**Fix:** Rather than patch the partial file again, simply reused the
already-complete, working `nav2_params_rrtstar.yaml` directly as the DWA
baseline file — since that file's `controller_server.FollowPath` was
already configured as `dwb_core::DWBLocalPlanner` (DWA) the whole time.
No edits needed; the "DWA baseline" file IS the RRT* file, which already
contains a correct, complete DWA config.

## 11. DWA aborted a navigation goal MPPI completed successfully (research finding, not a bug)

**Symptom:** Sending the same goal `(1.0, 0.5)` to DWA resulted in
`number_of_recoveries: 15`, the robot stuck oscillating near (but not at)
the goal, ending in `Goal finished with status: ABORTED` (error_code 208).
MPPI completed an equivalent-distance goal with `number_of_recoveries: 0`.

**Not a configuration bug** — this is a genuine behavioral difference
between the two controllers and is reported as a result, not fixed. A
second, easier DWA goal succeeded, giving a fairer two-trial dataset.

**Result used in report (Chapter 5):** MPPI's angular velocity stayed
within ±0.5 rad/s with 0 oscillations; DWA swung between ±1.8 rad/s with
3 oscillation events and one outright navigation failure on a harder goal.


---

# Debugging Log — Phase 3 (SLAM Toolbox + AMCL) Integration

## 12. AMCL `/initialpose` timing race condition with Nav2 bringup

**Symptom:** On every fresh launch using a saved static map (`map_server` +
`nav2_params_full_phase3.yaml`), the bringup consistently aborted after
~30 seconds with:
```
[global_costmap.global_costmap]: Timed out waiting for transform from
base_link to map to become available, tf error: Invalid frame ID "map"...
[lifecycle_manager_navigation]: Failed to bring up all requested nodes.
Aborting bringup.
```
AMCL itself logged, repeatedly and correctly:
```
[amcl]: AMCL cannot publish a pose or update the transform.
Please set the initial pose...
```

**Cause:** This is not a configuration bug. AMCL behaves exactly as
designed — it will not publish the `map → odom` transform until it
receives an `/initialpose` message. However, `global_costmap` (and by
extension the rest of `lifecycle_manager_navigation`'s bringup sequence)
has a hard ~30-second timeout waiting for `base_link → map` to become
available. If `/initialpose` is not published within that window, the
entire navigation stack bringup aborts — even though AMCL itself never
failed or errored.

This creates a race condition: a person following typical Nav2
documentation (launch first, then set initial pose afterward via RViz2)
will reliably hit this failure, because the natural sequence — launch,
wait for the system to "settle," then click 2D Pose Estimate — almost
always exceeds the 30-second window. Sending the pose only after seeing
the system in a final/settled state is, ironically, what causes the
failure.

**Diagnosis method:** Repeated clean (no manual lifecycle intervention)
launches confirmed the abort happened identically every time, ruling out
session-specific corruption. Reading the full log from the very first
line — not just the final error — showed AMCL was healthy and simply
waiting throughout the entire 30-second window.

**Fix:** Publish `/initialpose` immediately (within the first 5–10
seconds) after launching Nav2 — do not wait for the system to appear
"ready" first:
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/ros2_ws/maps/phase3_map.yaml \
  params_file:=.../nav2_params_full_phase3.yaml \
  use_rviz:=False
# In a second terminal, immediately (don't wait):
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

**Lesson reinforced:** When using a pre-built static map with AMCL (as
opposed to live SLAM, which has no such race condition since it doesn't
depend on an external pose message to begin publishing transforms),
always send the initial pose proactively and immediately rather than
reactively after observing a "ready" state — there may not be a visible
indicator that the clock is already running out.
