#!/usr/bin/env python3
"""
benchmark_dynamic_replanning.py (v6 - longer goal distance)
===============================================================
v5 fixed the /odom vs /amcl_pose issue, but revealed the REAL root
cause: Nav2's xy_goal_tolerance (0.25m in this project's params) means
Nav2 considers a goal "reached" once the robot is within 25cm of it.
With HOME=(0,0) and GOAL=(0.3,-0.2) -- a trip of only ~0.36m -- the
25cm tolerance is a huge fraction of the total distance, so the robot
can satisfy "arrived home" or "arrived at goal" without travelling
anywhere close to the full distance.

Fix: use a longer-distance HOME/GOAL pair where 0.25m tolerance is a
small fraction of the trip, forcing genuine end-to-end travel.

HOME = (0.0, 0.0)
GOAL = (0.8, 0.8)   -- approx 1.13m trip, tolerance is ~22% of distance
                       (previous pair was ~0.36m trip, tolerance ~69%!)

Both points were individually confirmed reachable in earlier manual
testing (Phase 3/4 exploration), so this should be a safe, repeatable
pair for benchmarking dynamic replanning.

Usage:
  python3 benchmark_dynamic_replanning.py NO_OBSTACLE 5
  python3 benchmark_dynamic_replanning.py WITH_OBSTACLE 5
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import csv, os, sys, time, math
from datetime import datetime

HOME = (0.0, 0.0)
GOAL = (0.8, 0.8)          # ~1.13m trip -- tolerance is now a small fraction
HOME_TOLERANCE_M = 0.20    # still generous, but now genuinely meaningful


def make_pose(x, y, frame='map'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = 1.0
    return p


class ReplanningBenchmark(Node):
    def __init__(self):
        super().__init__('replanning_benchmark_node')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._client.wait_for_server()
        self.get_logger().info('Server ready.')
        self.last_feedback = None
        self.current_odom_pose = None
        self.home_odom_reference = None
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self._wait_for_first_odom()

    def _odom_callback(self, msg):
        self.current_odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def _wait_for_first_odom(self, timeout_s=5.0):
        deadline = time.time() + timeout_s
        while self.current_odom_pose is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.current_odom_pose is not None:
            self.home_odom_reference = self.current_odom_pose
            self.get_logger().info(
                f'Captured home odom reference: {self.home_odom_reference}')
        else:
            self.get_logger().error('Never received /odom! Is Gazebo running?')

    def get_current_odom(self):
        rclpy.spin_once(self, timeout_sec=0.3)
        return self.current_odom_pose

    def distance_from_home_odom(self):
        pose = self.get_current_odom()
        if pose is None or self.home_odom_reference is None:
            return float('inf')
        dx = pose[0] - self.home_odom_reference[0]
        dy = pose[1] - self.home_odom_reference[1]
        return math.hypot(dx, dy)

    def feedback_callback(self, feedback_msg):
        self.last_feedback = feedback_msg.feedback

    def send_goal(self, goal_pose):
        self.last_feedback = None
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        t0 = time.perf_counter()
        send_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal was REJECTED by action server!')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wall_time = time.perf_counter() - t0

        result = result_future.result()
        feedback = self.last_feedback

        return {
            'wall_time_s': round(wall_time, 3),
            'nav_time_s': round(
                feedback.navigation_time.sec +
                feedback.navigation_time.nanosec / 1e9, 3
            ) if feedback else -1,
            'recoveries': feedback.number_of_recoveries if feedback else -1,
            'error_code': result.result.error_code,
            'status': result.status,
        }

    def return_home_verified(self, max_retries=3):
        for attempt in range(1, max_retries + 1):
            self.get_logger().info(
                f'Returning to home position (attempt {attempt})...')
            home_pose = make_pose(*HOME)
            result = self.send_goal(home_pose)

            if result is None:
                self.get_logger().warn(
                    'Return-home goal rejected, retrying...')
                time.sleep(1.0)
                continue

            time.sleep(0.5)
            dist = self.distance_from_home_odom()
            self.get_logger().info(
                f'Distance from home (odom frame): {dist:.3f} m')

            if dist <= HOME_TOLERANCE_M:
                return True
            else:
                self.get_logger().warn(
                    f'Not actually home (dist={dist:.3f}m > '
                    f'{HOME_TOLERANCE_M}m tolerance), retrying...')

        self.get_logger().error(
            'FAILED to verify return to home after max retries!')
        return False


def run_benchmark(label, n_trials):
    rclpy.init()
    node = ReplanningBenchmark()
    results = []

    print('\n' + '=' * 70)
    print(f'  DYNAMIC REPLANNING BENCHMARK: {label}  --  {n_trials} trials')
    print(f'  Home {HOME} -> Goal {GOAL}  (~{math.hypot(GOAL[0]-HOME[0], GOAL[1]-HOME[1]):.2f}m trip)')
    print('=' * 70)
    print(f"  {'#':<4} {'WallTime(s)':>12} {'NavTime(s)':>11} {'Recoveries':>11} {'Status':>10}")
    print('-' * 70)

    for i in range(1, n_trials + 1):
        if i > 1:
            home_ok = node.return_home_verified()
            if not home_ok:
                print(f'  {i:<4}  SKIPPED (could not verify return home)')
                results.append({'trial': i, 'label': label, 'wall_time_s': -1,
                                 'nav_time_s': -1, 'recoveries': -1,
                                 'error_code': -1, 'status': 'HOME_FAILED',
                                 'start_dist_from_home_m': -1})
                continue

        start_dist = node.distance_from_home_odom()

        goal_pose = make_pose(*GOAL)
        r = node.send_goal(goal_pose)
        if r is None:
            print(f'  {i:<4}  GOAL REJECTED')
            results.append({'trial': i, 'label': label, 'wall_time_s': -1,
                             'nav_time_s': -1, 'recoveries': -1,
                             'error_code': -1, 'status': 'REJECTED',
                             'start_dist_from_home_m': round(start_dist, 3)})
            continue

        status_str = 'SUCCEEDED' if r['status'] == 4 else (
            'ABORTED' if r['status'] == 6 else f"status_{r['status']}")
        print(f"  {i:<4} {r['wall_time_s']:>12.2f} {r['nav_time_s']:>11.2f} "
              f"{r['recoveries']:>11} {status_str:>10}  "
              f"(start_dist_from_home={start_dist:.2f}m)")

        results.append({
            'trial': i, 'label': label,
            'wall_time_s': r['wall_time_s'],
            'nav_time_s': r['nav_time_s'],
            'recoveries': r['recoveries'],
            'error_code': r['error_code'],
            'status': status_str,
            'start_dist_from_home_m': round(start_dist, 3),
        })

    node.return_home_verified()

    ok = [r for r in results if r['status'] == 'SUCCEEDED']
    print('-' * 70)
    if ok:
        avg_nav = sum(r['nav_time_s'] for r in ok) / len(ok)
        avg_rec = sum(r['recoveries'] for r in ok) / len(ok)
        print(f'  AVG (successful only)  nav_time={avg_nav:.2f}s  recoveries={avg_rec:.1f}')
    print(f'  Success: {len(ok)}/{len(results)}')
    print('=' * 70)

    out = os.path.expanduser('~/ros2_ws/results')
    os.makedirs(out, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out, f'dynamic_replanning_{label}_{ts}.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=results[0].keys())
        w.writeheader()
        w.writerows(results)
    print(f'\n  Saved -> {csv_path}\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python3 benchmark_dynamic_replanning.py <LABEL> [n_trials]')
        sys.exit(1)
    label = sys.argv[1]
    n_trials = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    run_benchmark(label, n_trials)
