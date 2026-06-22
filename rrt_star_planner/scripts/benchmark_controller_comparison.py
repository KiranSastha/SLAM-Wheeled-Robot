#!/usr/bin/env python3
"""
benchmark_controller_comparison.py

Benchmarks MPPI vs DWA local controllers by subscribing to /cmd_vel
during navigation and recording oscillation events and jerk (smoothness).

Usage:
  python3 benchmark_controller_comparison.py MPPI 60
  python3 benchmark_controller_comparison.py DWA 60

Send the Nav2 goal in RViz2 right after starting this script.
Results saved to ~/ros2_ws/results/controller_<label>_<timestamp>.csv
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import csv, os, sys, time
from datetime import datetime


class ControllerBenchmark(Node):
    def __init__(self, label, duration):
        super().__init__("controller_benchmark_node")
        self.label = label
        self.duration = duration
        self.records = []
        self.last_wz = 0.0
        self.oscillation_count = 0
        self.start_time = time.time()
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.get_logger().info(f"Recording {label} for {duration}s. Send the Nav2 goal now!")

    def cmd_vel_callback(self, msg):
        t = time.time() - self.start_time
        vx, wz = msg.linear.x, msg.angular.z
        if (self.last_wz > 0.05 and wz < -0.05) or (self.last_wz < -0.05 and wz > 0.05):
            self.oscillation_count += 1
        self.records.append({"t": round(t, 3), "vx": round(vx, 4), "wz": round(wz, 4)})
        self.last_wz = wz

    def finish(self):
        out = os.path.expanduser("~/ros2_ws/results")
        os.makedirs(out, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out, f"controller_{self.label}_{ts}.csv")
        if self.records:
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=self.records[0].keys())
                w.writeheader()
                w.writerows(self.records)
        if len(self.records) > 1:
            vx_jerk = sum(abs(self.records[i]["vx"] - self.records[i-1]["vx"]) for i in range(1, len(self.records))) / (len(self.records) - 1)
            wz_jerk = sum(abs(self.records[i]["wz"] - self.records[i-1]["wz"]) for i in range(1, len(self.records))) / (len(self.records) - 1)
        else:
            vx_jerk = wz_jerk = 0.0
        print("\n" + "=" * 60)
        print(f"  CONTROLLER BENCHMARK: {self.label}")
        print("=" * 60)
        print(f"  Samples recorded:     {len(self.records)}")
        print(f"  Oscillation events:   {self.oscillation_count}")
        print(f"  Avg |dvx| (jerk):     {vx_jerk:.4f} m/s per sample")
        print(f"  Avg |dwz| (jerk):     {wz_jerk:.4f} rad/s per sample")
        print(f"  Saved -> {path}")
        print("=" * 60 + "\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 benchmark_controller_comparison.py <LABEL> [duration_sec]")
        sys.exit(1)
    label = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    rclpy.init()
    node = ControllerBenchmark(label, duration)
    try:
        end_time = time.time() + duration
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
