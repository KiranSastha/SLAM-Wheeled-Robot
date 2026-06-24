# Phase-wise Improvement Summary

## Project Roadmap

| Phase | Description | Status |
|---|---|---|
| Phase 0 | ROS2 + Gazebo + Nav2 Environment Setup | ✅ Completed |
| Phase 1 | RRT* Global Path Planner Development and Benchmarking | ✅ Completed |
| Phase 2 | MPPI Local Controller Development and Benchmarking (originally planned as TEB) | ✅ Completed |
| Phase 3 | SLAM Integration (SLAM Toolbox + AMCL Localization) | ✅ Completed |
| Phase 4 | Dynamic Replanning and Obstacle Response | ⏳ Pending |
| Phase 5 | Full System Integration and Performance Evaluation | ⏳ Pending |
| Phase 6 | Hardware Validation on Wheeled Mobile Robot | ⏳ Pending |

---

## Phase 1 Results — RRT* vs RRT

### Objective
To evaluate the performance of the RRT* global planner against the baseline RRT planner.

| Metric | RRT | RRT* | Result |
|---|---|---|---|
| Path Length | ≈ 1.82 m | ≈ 1.86 m | Comparable |
| Planning Time | ≈ 110 ms | ≈ 55 ms | Faster |
| Path Smoothness | ≈ 6.6 rad | ≈ 2.5 rad | 56.8% Improvement |
| Success Rate (20 trials) | 2/20 (10%) | 12/20 (60%) | 6x Improvement |

### Observations
- RRT* generated significantly smoother paths.
- Path lengths remained similar between both planners.
- Planning time remained suitable for real-time navigation.
- RRT* succeeded far more consistently than vanilla RRT across repeated trials.
- Benchmark conducted over 20 trials demonstrated consistent performance.

### Conclusion
RRT* improved overall path quality by producing substantially smoother trajectories while maintaining comparable path lengths and real-time planning capability, alongside a dramatically higher success rate. The planner was successfully integrated into the Nav2 framework and validated in simulation.

**Chart:** `results/rrt_vs_rrtstar_comparison.png`

---

## Phase 2 Results — MPPI vs DWA

### Objective
To evaluate MPPI as a replacement for the conventional DWA local planner.

> **Note:** TEB was the originally planned local planner upgrade. It was replaced with MPPI
> because `teb_local_planner` has no official, stable ROS2 Jazzy release. MPPI is Nav2's
> own modern successor to both TEB and DWB, confirmed by Nav2's lead maintainer and
> official tuning guide, and ships natively with `ros-jazzy-navigation2`.

| Metric | DWA | MPPI | Result |
|---|---|---|---|
| Peak Angular Velocity | ±1.8 rad/s | ±0.5 rad/s | ~70% Reduction |
| Command Oscillations | High (3 events) | Low (0 events) | Improved |
| Steering Reversals | Frequent | Minimal | Improved |
| Velocity Continuity | Discontinuous | Continuous | Improved |
| Motion Smoothness | Moderate | High | Improved |

### Observations
- DWA produced aggressive steering commands and frequent oscillations.
- MPPI generated smooth and continuous velocity commands.
- Steering effort was significantly reduced.
- Motion became more predictable and suitable for physical robot deployment.
- DWA aborted one navigation attempt after exhausting 15 recovery behaviors; MPPI
  completed an equivalent goal with zero recoveries.

### Conclusion
MPPI outperformed DWA by generating smoother control actions with fewer oscillations and lower peak steering commands. The controller demonstrated improved trajectory tracking behavior and enhanced suitability for real-world mobile robot navigation.

**Chart:** `results/mppi_vs_dwa_comparison.png`

---

## Phase 3 Results — SLAM Toolbox + AMCL Localization

### Objective
To validate the full "build once, localize forever" SLAM workflow: construct a map using
live SLAM, save it, then reload it and localize purely with AMCL — without live mapping.

| Stage | Outcome |
|---|---|
| Live map construction (SLAM Toolbox) | ✅ Successful — 94×103 cell map at 0.05m resolution saved |
| Map save (`map_saver_cli`) | ✅ Successful — `.pgm` + `.yaml` generated |
| Static map reload (`map_server`) | ✅ Successful |
| AMCL localization on static map | ✅ Successful — `map → odom` transform published and stable |
| RRT* + MPPI navigation on static map | ✅ Successful — `Goal finished with status: SUCCEEDED`, 0 recoveries |

### Observations
- AMCL correctly localizes the robot on a previously-saved static map without any live
  SLAM process running, confirming the mapping and localization stages are independently
  functional.
- A significant integration issue was identified and resolved: AMCL requires `/initialpose`
  to be published *before* the Nav2 bringup's ~30-second `global_costmap` activation
  timeout, or the entire navigation bringup aborts. This is a timing/sequencing issue in
  the bringup process rather than a configuration error, and is not clearly documented
  in Nav2's own startup behavior.
- Once the timing issue was understood, the full pipeline (map load → AMCL localize →
  RRT* plan → MPPI control) operated correctly on the first subsequent attempt.

### Conclusion
The SLAM Toolbox + AMCL integration is functionally complete. The system can build a map
once and reuse it indefinitely for localization-only operation, which is the practical
mode of operation expected for real hardware deployment in Phase 6 (rather than
re-mapping on every run).

**Map files:** `maps/phase3_map.pgm`, `maps/phase3_map.yaml` (or equivalent path in repo)

---

## Overall Outcome After Phase 3

### Baseline Navigation Stack (original plan)
```
RRT → DWA → MATLAB simulation only, no localization separation
```

### Proposed Navigation Stack (implemented)
```
RRT* → MPPI → ROS2/Gazebo → SLAM Toolbox (mapping) + AMCL (localization)
```

### Improvements Achieved

| Component | Baseline | Proposed | Benefit |
|---|---|---|---|
| Global Planner | RRT | RRT* | Smoother paths, 6x higher success rate |
| Local Controller | DWA | MPPI | Smoother control actions, fewer oscillations |
| Platform | MATLAB | ROS2 + Gazebo | Hardware-portable, industry standard |
| Localization | Not separated from mapping | SLAM Toolbox + AMCL | Map-once, localize-forever capability |
| Path Quality | Moderate | Improved | Better navigation |
| Motion Smoothness | Moderate | High | Better robot behavior |
| Real-World Suitability | Moderate | High | Better hardware readiness |

---

*Last updated: Phase 3 completion. See `docs/DEBUGGING_LOG.md` for the full technical
debugging record behind these results.*
