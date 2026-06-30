#!/usr/bin/env python3
"""
rugged_test_phase5.py
=======================
Focused rugged/stress test for Phase 5 integration testing. Covers:

  1. Repeated goal cycling -- sends N different valid goals back-to-back
     in the same session, checking for degradation over time (increasing
     planning time, increasing recoveries, or outright failures appearing
     later in the sequence that didn't appear early on).

  2. Forced failure recovery -- deliberately sends an unreachable/invalid
     goal, confirms it fails cleanly (doesn't hang/crash the action
     server), then immediately sends a known-good goal afterward to
     confirm the system recovers and accepts new goals normally.

  3. Resource monitoring -- samples CPU and memory usage of the key
     long-running nodes (component_container_isolated, slam_toolbox)
     at the start and end of the test, to flag any obvious growth
     consistent with a memory leak over the test duration.

Usage:
  python3 rugged_test_phase5.py

Results saved to ~/ros2_ws/results/rugged_test_<timestamp>.csv
and a summary printed to console.
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import csv, os, sys, time, subprocess
from datetime import datetime

# Known-reachable goals from this map (mix of short/medium trips)
VALID_GOALS = [
    (0.3, -0.2),
    (0.5, 0.5),
    (0.5, -0.5),
    (0.3, -0.2),
    (0.5, 0.5),
]

# Deliberately invalid/unreachable goal -- far outside the explored map
INVALID_GOAL = (50.0, 50.0)


def make_pose(x, y, frame='map'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = 1.0
    return p


def get_process_stats(process_name_substring):
    """Returns (cpu_percent, mem_mb) for the first matching process,
    or (None, None) if not found. Uses ps directly -- no extra deps."""
    try:
        out = subprocess.check_output(
            ["ps", "-eo", "pid,comm,%cpu,rss,args"],
            text=True
        )
        for line in out.splitlines()[1:]:
            if process_name_substring in line:
                parts = line.split(None, 4)
                if len(parts) >= 4:
                    cpu = float(parts[2])
                    rss_kb = float(parts[3])
                    return cpu, rss_kb / 1024.0  # convert to MB
    except Exception as e:
        print(f"  (could not read process stats: {e})")
    return None, None


class RuggedTestNode(Node):
    def __init__(self):
        super().__init__('rugged_test_node')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._client.wait_for_server()
        self.get_logger().info('Server ready.')
        self.last_feedback = None

    def feedback_callback(self, feedback_msg):
        self.last_feedback = feedback_msg.feedback

    def send_goal(self, goal_pose, timeout_s=30.0):
        self.last_feedback = None
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        t0 = time.perf_counter()
        send_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_s)

        if not send_future.done():
            return {'status': 'SEND_TIMEOUT', 'wall_time_s': timeout_s,
                    'nav_time_s': -1, 'recoveries': -1, 'error_code': -1}

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {'status': 'REJECTED', 'wall_time_s': 0,
                    'nav_time_s': -1, 'recoveries': -1, 'error_code': -1}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        wall_time = time.perf_counter() - t0

        if not result_future.done():
            # Cancel the stuck goal so it doesn't linger for the next test
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            return {'status': 'RESULT_TIMEOUT', 'wall_time_s': wall_time,
                    'nav_time_s': -1, 'recoveries': -1, 'error_code': -1}

        result = result_future.result()
        feedback = self.last_feedback

        status_map = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        return {
            'status': status_map.get(result.status, f'status_{result.status}'),
            'wall_time_s': round(wall_time, 2),
            'nav_time_s': round(
                feedback.navigation_time.sec +
                feedback.navigation_time.nanosec / 1e9, 2
            ) if feedback else -1,
            'recoveries': feedback.number_of_recoveries if feedback else -1,
            'error_code': result.result.error_code,
        }


def run_rugged_test():
    rclpy.init()
    node = RuggedTestNode()
    results = []

    print('\n' + '=' * 75)
    print('  PHASE 5 RUGGED TEST -- repeated cycling + forced failure + resources')
    print('=' * 75)

    # ── Part A: Resource baseline ──────────────────────────────────────
    print('\n--- Part A: Resource baseline (start) ---')
    cpu1, mem1 = get_process_stats('component_container')
    cpu2, mem2 = get_process_stats('slam_toolbox')
    print(f'  component_container_isolated: CPU={cpu1}%  MEM={mem1:.1f}MB' if cpu1 is not None
          else '  component_container_isolated: not found')
    print(f'  slam_toolbox:                  CPU={cpu2}%  MEM={mem2:.1f}MB' if cpu2 is not None
          else '  slam_toolbox: not found')
    baseline_mem = {'container_mb': mem1, 'slam_mb': mem2}

    # ── Part B: Repeated goal cycling ──────────────────────────────────
    print('\n--- Part B: Repeated goal cycling ---')
    print(f"  {'#':<4} {'Goal':>14} {'WallTime(s)':>12} {'NavTime(s)':>11} {'Recoveries':>11} {'Status':>14}")
    print('  ' + '-' * 70)

    for i, (gx, gy) in enumerate(VALID_GOALS, 1):
        r = node.send_goal(make_pose(gx, gy))
        print(f"  {i:<4} ({gx:.1f},{gy:.1f}) {r['wall_time_s']:>12.2f} "
              f"{r['nav_time_s']:>11.2f} {r['recoveries']:>11} {r['status']:>14}")
        results.append({
            'phase': 'B_cycling', 'trial': i, 'goal_x': gx, 'goal_y': gy,
            **r
        })
        time.sleep(0.5)

    # ── Part C: Forced failure + recovery ──────────────────────────────
    print('\n--- Part C: Forced failure (unreachable goal) ---')
    r_fail = node.send_goal(make_pose(*INVALID_GOAL), timeout_s=15.0)
    print(f"  Invalid goal {INVALID_GOAL} -> {r_fail['status']}")
    results.append({'phase': 'C_forced_failure', 'trial': 1,
                     'goal_x': INVALID_GOAL[0], 'goal_y': INVALID_GOAL[1], **r_fail})

    print('\n--- Part C: Recovery check (known-good goal immediately after) ---')
    r_recover = node.send_goal(make_pose(*VALID_GOALS[0]))
    print(f"  Recovery goal {VALID_GOALS[0]} -> {r_recover['status']}")
    results.append({'phase': 'C_recovery_check', 'trial': 1,
                     'goal_x': VALID_GOALS[0][0], 'goal_y': VALID_GOALS[0][1], **r_recover})

    recovered_ok = r_recover['status'] == 'SUCCEEDED'

    # ── Part D: Resource check (end) ───────────────────────────────────
    print('\n--- Part D: Resource check (end) ---')
    cpu1b, mem1b = get_process_stats('component_container')
    cpu2b, mem2b = get_process_stats('slam_toolbox')
    print(f'  component_container_isolated: CPU={cpu1b}%  MEM={mem1b:.1f}MB' if cpu1b is not None
          else '  component_container_isolated: not found')
    print(f'  slam_toolbox:                  CPU={cpu2b}%  MEM={mem2b:.1f}MB' if cpu2b is not None
          else '  slam_toolbox: not found')

    # ── Summary ─────────────────────────────────────────────────────────
    print('\n' + '=' * 75)
    print('  SUMMARY')
    print('=' * 75)
    cycling = [r for r in results if r['phase'] == 'B_cycling']
    ok_cycling = [r for r in cycling if r['status'] == 'SUCCEEDED']
    print(f'  Goal cycling success rate: {len(ok_cycling)}/{len(cycling)}')

    print(f'  Forced failure handled cleanly: '
          f'{"YES" if r_fail["status"] in ("ABORTED", "RESULT_TIMEOUT") else "UNEXPECTED: " + r_fail["status"]}')
    print(f'  Recovered after forced failure: {"YES" if recovered_ok else "NO -- system did not recover!"}')

    if mem1 is not None and mem1b is not None:
        delta = mem1b - mem1
        print(f'  component_container memory change: {delta:+.1f} MB '
              f'({"possible leak" if delta > 50 else "stable"})')
    if mem2 is not None and mem2b is not None:
        delta = mem2b - mem2
        print(f'  slam_toolbox memory change: {delta:+.1f} MB '
              f'({"possible leak" if delta > 50 else "stable"})')
    print('=' * 75)

    # Save results
    out = os.path.expanduser('~/ros2_ws/results')
    os.makedirs(out, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out, f'rugged_test_{ts}.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=results[0].keys())
        w.writeheader()
        w.writerows(results)
    print(f'\n  Saved -> {csv_path}\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run_rugged_test()
