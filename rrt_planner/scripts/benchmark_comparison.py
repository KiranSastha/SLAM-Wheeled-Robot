#!/usr/bin/env python3
"""
benchmark_comparison.py
Benchmarks RRT vs RRT* by calling Nav2's compute_path_to_pose
action 20 times per planner and comparing path length, time, smoothness.

Usage:
  python3 benchmark_comparison.py RRT_Star
  python3 benchmark_comparison.py RRT
Results saved to ~/ros2_ws/results/
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math, csv, os, time, sys
from datetime import datetime

WAYPOINT_PAIRS = [
    ((0.0,  0.0), (1.5,  0.5)),
    ((0.0,  0.0), (2.0, -0.5)),
    ((0.0,  0.0), (1.0,  1.0)),
    ((0.5,  0.5), (2.0,  0.0)),
    ((0.0,  0.0), (1.8,  0.8)),
]
TRIALS_PER_PAIR = 4


def make_pose(x, y, frame="map"):
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
        super().__init__("rrt_benchmark_node")
        self._client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.get_logger().info("Waiting for compute_path_to_pose...")
        self._client.wait_for_server()
        self.get_logger().info("Server ready.")

    def compute_path(self, start, goal):
        gm = ComputePathToPose.Goal()
        gm.start = start
        gm.goal = goal
        gm.planner_id = "GridBased"
        gm.use_start = True
        t0 = time.perf_counter()
        fut = self._client.send_goal_async(gm)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            return None, 0.0
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result.path, (time.perf_counter() - t0) * 1000.0


def run_benchmark(planner_label):
    rclpy.init()
    node = BenchmarkNode()
    results = []

    print("\n" + "="*70)
    print(f"  BENCHMARK: {planner_label}  --  20 trials")
    print("="*70)
    print(f"  {'#':<4} {'Start':>14} {'Goal':>14} {'Len(m)':>9} {'ms':>8} {'Smooth':>9}")
    print("-"*70)

    n = 0
    for (sx, sy), (gx, gy) in WAYPOINT_PAIRS:
        for _ in range(TRIALS_PER_PAIR):
            n += 1
            path, ms = node.compute_path(make_pose(sx, sy), make_pose(gx, gy))
            if path is None or len(path.poses) == 0:
                print(f"  {n:<4}  FAILED")
                results.append({"planner": planner_label, "trial": n,
                                "sx": sx, "sy": sy, "gx": gx, "gy": gy,
                                "length_m": -1, "time_ms": round(ms,2),
                                "smoothness_rad": -1, "ok": 0})
                continue
            L = path_length(path)
            S = path_smoothness(path)
            print(f"  {n:<4} ({sx:.1f},{sy:.1f})->({gx:.1f},{gy:.1f})"
                  f"  {L:>8.3f}  {ms:>7.1f}  {S:>8.3f}")
            results.append({"planner": planner_label, "trial": n,
                            "sx": sx, "sy": sy, "gx": gx, "gy": gy,
                            "length_m": round(L,4), "time_ms": round(ms,2),
                            "smoothness_rad": round(S,4), "ok": 1})
            time.sleep(0.3)

    ok = [r for r in results if r["ok"]]
    avg_len = sum(r["length_m"] for r in ok)/len(ok) if ok else 0
    avg_time = sum(r["time_ms"] for r in ok)/len(ok) if ok else 0
    avg_smooth = sum(r["smoothness_rad"] for r in ok)/len(ok) if ok else 0
    print("-"*70)
    print(f"  AVG  length={avg_len:.3f}m  time={avg_time:.1f}ms  smooth={avg_smooth:.3f}rad")
    print(f"  Success: {len(ok)}/{len(results)}")
    print("="*70)

    out = os.path.expanduser("~/ros2_ws/results")
    os.makedirs(out, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    label_safe = planner_label.lower().replace(" ","_").replace("*","star")
    csv_path = os.path.join(out, f"benchmark_{label_safe}_{ts}.csv")
    with open(csv_path,"w",newline="") as f:
        w = csv.DictWriter(f, fieldnames=results[0].keys())
        w.writeheader()
        w.writerows(results)
    print(f"\n  Saved -> {csv_path}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    label = sys.argv[1] if len(sys.argv) > 1 else "RRT_Star"
    run_benchmark(label)
