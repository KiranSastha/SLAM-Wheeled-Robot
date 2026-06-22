#!/usr/bin/env python3
"""
plot_comparison.py
Reads the two CSV files from benchmark_comparison.py and generates
a 3-panel comparison chart (length, time, smoothness).

Usage: python3 plot_comparison.py
(auto-detects the 2 most recent benchmark_*.csv files)
"""

import sys, os, glob, csv
import matplotlib.pyplot as plt
import numpy as np

RESULTS_DIR = os.path.expanduser("~/ros2_ws/results")


def load_csv(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            if int(row["ok"]) == 1:
                rows.append({
                    "planner": row["planner"],
                    "length_m": float(row["length_m"]),
                    "time_ms": float(row["time_ms"]),
                    "smoothness_rad": float(row["smoothness_rad"]),
                })
    return rows


def auto_detect_csvs():
    csvs = sorted(glob.glob(os.path.join(RESULTS_DIR, "benchmark_*.csv")))
    if len(csvs) < 2:
        print(f"Need at least 2 CSV files in {RESULTS_DIR}")
        sys.exit(1)
    return csvs[-2], csvs[-1]


def plot(rrt_rows, rrtstar_rows, out_path):
    fig, axes = plt.subplots(1, 3, figsize=(14, 5))
    fig.suptitle("RRT vs RRT* - Path Planning Comparison (20 trials each)",
                 fontsize=13, fontweight="bold", y=1.02)

    metrics = [
        ("length_m", "Path Length (m)", "Shorter = better"),
        ("time_ms", "Planning Time (ms)", "Lower = faster"),
        ("smoothness_rad", "Path Smoothness (rad)", "Lower = smoother"),
    ]
    colors = {"RRT": "#E07B54", "RRT_Star": "#4C9BE8"}
    labels = {"RRT": "RRT (baseline)", "RRT_Star": "RRT* (improved)"}
    rrt_label = rrt_rows[0]["planner"] if rrt_rows else "RRT"
    rrtstar_label = rrtstar_rows[0]["planner"] if rrtstar_rows else "RRT_Star"

    for ax, (key, ylabel, note) in zip(axes, metrics):
        rrt_vals = [r[key] for r in rrt_rows]
        rrtstar_vals = [r[key] for r in rrtstar_rows]
        positions = [1, 2]
        bp = ax.boxplot([rrt_vals, rrtstar_vals], positions=positions, widths=0.5,
                        patch_artist=True, medianprops=dict(color="white", linewidth=2))
        bp["boxes"][0].set_facecolor(colors.get(rrt_label, "#E07B54"))
        bp["boxes"][1].set_facecolor(colors.get(rrtstar_label, "#4C9BE8"))

        for i, (vals, pos) in enumerate(zip([rrt_vals, rrtstar_vals], positions)):
            jitter = np.random.uniform(-0.08, 0.08, len(vals))
            ax.scatter([pos+j for j in jitter], vals, alpha=0.5, s=20,
                      color=list(colors.values())[i], zorder=5)

        if key != "time_ms" and rrt_vals and rrtstar_vals:
            pct = (np.mean(rrt_vals) - np.mean(rrtstar_vals)) / np.mean(rrt_vals) * 100
            color = "#27AE60" if pct > 0 else "#E74C3C"
            ax.text(1.5, ax.get_ylim()[1]*0.95, f"RRT* is\n{abs(pct):.1f}% better",
                    ha="center", fontsize=8, color=color, fontweight="bold")

        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_title(note, fontsize=9, color="#555555")
        ax.set_xticks([1, 2])
        ax.set_xticklabels([labels.get(rrt_label, rrt_label),
                            labels.get(rrtstar_label, rrtstar_label)], fontsize=9)
        ax.grid(axis="y", alpha=0.3, linestyle="--")
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Chart saved -> {out_path}")
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) == 3:
        rrt_csv, rrtstar_csv = sys.argv[1], sys.argv[2]
    else:
        rrt_csv, rrtstar_csv = auto_detect_csvs()

    rrt_rows = load_csv(rrt_csv)
    rrtstar_rows = load_csv(rrtstar_csv)
    print(f"Loaded {len(rrt_rows)} RRT trials, {len(rrtstar_rows)} RRT* trials")

    out = os.path.join(RESULTS_DIR, "rrt_vs_rrtstar_comparison.png")
    plot(rrt_rows, rrtstar_rows, out)

    print("\n" + "="*55)
    print(f"  {'Metric':<22} {'RRT':>10} {'RRT*':>10} {'Improvement':>10}")
    print("-"*55)
    for key, label in [("length_m", "Avg path length (m)"),
                       ("time_ms", "Avg plan time (ms)"),
                       ("smoothness_rad", "Avg smoothness (rad)")]:
        rv = [r[key] for r in rrt_rows]
        sv = [r[key] for r in rrtstar_rows]
        if rv and sv:
            rm, sm = sum(rv)/len(rv), sum(sv)/len(sv)
            pct = (rm-sm)/rm*100 if rm else 0
            print(f"  {label:<22} {rm:>10.3f} {sm:>10.3f} {pct:>+9.1f}%")
    print("="*55)
