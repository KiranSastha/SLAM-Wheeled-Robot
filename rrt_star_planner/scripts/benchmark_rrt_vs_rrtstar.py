#!/usr/bin/env python3
"""
benchmark_rrt_vs_rrtstar.py
Calls Nav2 compute_path_to_pose 20 times and records:
  - Path length (metres)
  - Planning time (ms)
  - Path smoothness (sum of heading changes, radians)

Saves results to ~/ros2_ws/results/rrtstar_benchmark_<timestamp>.csv

Usage (with Nav2 stack running):
  python3 ~/ros2_ws/src/rrt_star_planner/scripts/benchmark_rrt_vs_rrtstar.py
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math, csv, os, time
from datetime import datetime

# Adjust these to valid free-space positions in your Gazebo world
WAYPOINT_PAIRS = [
    ((0.0,  0.0), (1.5,  0.5)),
    ((0.0,  0.0), (2.0, -0.5)),
    ((0.0,  0.0), (1.0,  1.0)),
    ((0.5,  0.5), (2.0,  0.0)),
    ((0.0,  0.0), (1.8,  0.8)),
]
TRIALS_PER_PAIR = 4   # 5 pairs x 4 trials = 20 total


def make_pose(x, y, frame='map'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = 1.0
    return p


def path_length(path):
    total = 0.0
    for i in range(1, len(path.poses)):
        dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x
        dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y
        total += math.hypot(dx, dy)
    return total


def path_smoothness(path):
    """Sum of absolute heading changes (rad) — lower is smoother."""
    total = 0.0
    poses = path.poses
    for i in range(1, len(poses) - 1):
        dx1 = poses[i].pose.position.x   - poses[i-1].pose.position.x
        dy1 = poses[i].pose.position.y   - poses[i-1].pose.position.y
        dx2 = poses[i+1].pose.position.x - poses[i].pose.position.x
        dy2 = poses[i+1].pose.position.y - poses[i].pose.position.y
        a1, a2 = math.atan2(dy1, dx1), math.atan2(dy2, dx2)
        diff = abs(a2 - a1)
        if diff > math.pi:
            diff = 2*math.pi - diff
        total += diff
    return total


class BenchmarkNode(Node):
    def __init__(self):
        super().__init__('rrt_benchmark_node')
        self._client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.get_logger().info('Waiting for compute_path_to_pose action server...')
        self._client.wait_for_server()
        self.get_logger().info('Server ready. Starting benchmark.')

    def compute_path(self, start, goal):
        gm = ComputePathToPose.Goal()
        gm.start = start
        gm.goal  = goal
        gm.planner_id = 'GridBased'
        gm.use_start  = True
        t0  = time.perf_counter()
        fut = self._client.send_goal_async(gm)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            return None, 0.0
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result.path, (time.perf_counter() - t0) * 1000.0


def run_benchmark():
    rclpy.init()
    node    = BenchmarkNode()
    results = []

    print('\n' + '='*68)
    print('  RRT* PLANNER BENCHMARK  --  20 trials')
    print('='*68)
    print(f'  {"#":<4} {"Start":>14} {"Goal":>14} {"Len(m)":>9} {"ms":>8} {"Smooth":>9}')
    print('-'*68)

    n = 0
    for (sx, sy), (gx, gy) in WAYPOINT_PAIRS:
        for _ in range(TRIALS_PER_PAIR):
            n += 1
            path, ms = node.compute_path(make_pose(sx, sy), make_pose(gx, gy))
            if path is None or len(path.poses) == 0:
                print(f'  {n:<4}  FAILED')
                results.append({
                    'trial': n, 'sx': sx, 'sy': sy, 'gx': gx, 'gy': gy,
                    'length_m': -1, 'time_ms': round(ms, 2),
                    'smoothness_rad': -1, 'ok': 0
                })
                continue
            L = path_length(path)
            S = path_smoothness(path)
            print(f'  {n:<4} ({sx:.1f},{sy:.1f})->({gx:.1f},{gy:.1f})'
                  f'  {L:>8.3f}  {ms:>7.1f}  {S:>8.3f}')
            results.append({
                'trial': n, 'sx': sx, 'sy': sy, 'gx': gx, 'gy': gy,
                'length_m': round(L, 4), 'time_ms': round(ms, 2),
                'smoothness_rad': round(S, 4), 'ok': 1
            })
            time.sleep(0.3)

    ok = [r for r in results if r['ok']]
    print('-'*68)
    if ok:
        print(f'  AVG  length={sum(r["length_m"] for r in ok)/len(ok):.3f}m  '
              f'time={sum(r["time_ms"] for r in ok)/len(ok):.1f}ms  '
              f'smooth={sum(r["smoothness_rad"] for r in ok)/len(ok):.3f}rad')
    print(f'  Success: {len(ok)}/{len(results)}')
    print('='*68)

    out = os.path.expanduser('~/ros2_ws/results')
    os.makedirs(out, exist_ok=True)
    ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out, f'rrtstar_benchmark_{ts}.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)
    print(f'\n  Results saved -> {csv_path}\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run_benchmark()
