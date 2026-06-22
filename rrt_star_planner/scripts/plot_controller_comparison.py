#!/usr/bin/env python3
"""
plot_controller_comparison.py
Reads two controller_*.csv files (MPPI and DWA) and plots
angular velocity over time for visual smoothness comparison.

Usage: python3 plot_controller_comparison.py
"""

import os, glob, csv
import matplotlib.pyplot as plt

RESULTS_DIR = os.path.expanduser("~/ros2_ws/results")


def load(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append({"t": float(row["t"]), "vx": float(row["vx"]), "wz": float(row["wz"])})
    return rows


def auto_detect():
    files = sorted(glob.glob(os.path.join(RESULTS_DIR, "controller_*.csv")))
    if len(files) < 2:
        print(f"Need at least 2 controller_*.csv files in {RESULTS_DIR}")
        return None, None
    return files[-2], files[-1]


def main():
    f1, f2 = auto_detect()
    if not f1:
        return
    d1, d2 = load(f1), load(f2)
    label1 = os.path.basename(f1).split("_")[1]
    label2 = os.path.basename(f2).split("_")[1]

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    fig.suptitle("MPPI vs DWA - Velocity Command Smoothness", fontsize=13, fontweight="bold")

    axes[0].plot([r["t"] for r in d1], [r["wz"] for r in d1], label=label1)
    axes[0].set_title(f"{label1} - Angular velocity over time")
    axes[0].set_ylabel("wz (rad/s)")
    axes[0].axhline(0, color="gray", linewidth=0.5)
    axes[0].grid(alpha=0.3)
    axes[0].legend()

    axes[1].plot([r["t"] for r in d2], [r["wz"] for r in d2], label=label2, color="orange")
    axes[1].set_title(f"{label2} - Angular velocity over time")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("wz (rad/s)")
    axes[1].axhline(0, color="gray", linewidth=0.5)
    axes[1].grid(alpha=0.3)
    axes[1].legend()

    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, "mppi_vs_dwa_comparison.png")
    plt.savefig(out, dpi=150)
    print(f"Chart saved -> {out}")
    plt.show()


if __name__ == "__main__":
    main()
