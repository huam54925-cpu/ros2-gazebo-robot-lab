#!/usr/bin/env python3
"""Re-create SAC diagnostic figures from the two checked-in summary CSVs.

Usage (from the containing 2026-10-17 directory):
    python3 scripts/plot_sac_diagnostics.py

This script plots *summary observations*, not the full underlying trajectories.
No inference/model-based image generation is used.
"""
import csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "figures" / "generated"
OUT.mkdir(parents=True, exist_ok=True)

def load(name):
    with (ROOT / "data" / name).open(newline="") as stream:
        return list(csv.DictReader(stream))

def store(fig, stem):
    fig.tight_layout()
    fig.savefig(OUT / (stem + ".svg"))
    fig.savefig(OUT / (stem + ".png"), dpi=190)
    plt.close(fig)

ev = load("sac-evaluations-5000-20000.csv")
segments = load("sac-failure-50step-15000-20000.csv")
steps = [int(x["checkpoint_step"]) for x in ev]
finals = [float(x["final_distance_m"]) for x in ev]
fig, ax = plt.subplots(figsize=(9, 4.8))
ax.bar([str(s // 1000) + "k" for s in steps], finals)
ax.axhline(0.35, linestyle="--", label="0.35 m arrival threshold")
ax.set(ylabel="Final distance (m)", xlabel="Checkpoint steps",
       title="Fixed indoor goal: final distance, single episode per checkpoint")
ax.legend()
store(fig, "sac-final-distance")

middle = [(int(r["first_step"]) + int(r["last_step"])) / 2 for r in segments]
fig, ax = plt.subplots(figsize=(9, 4.8))
for s in (15000, 20000):
    v = [float(r[f"mean_abs_v_{s}_m_s"]) for r in segments]
    ax.plot(middle, v, marker="o", label=str(s))
ax.set_yscale("log")
ax.set(xlabel="Episode step, 50-step bin center",
       ylabel="Mean absolute ack.v (m/s), log scale",
       title="Near-zero command behavior in failed evaluations")
ax.legend()
store(fig, "sac-speed-by-interval")

fig, ax = plt.subplots(figsize=(9, 4.8))
for s, min_step, minimum in ((15000, 37, .7318401175707933),
                             (20000, 28, .406332398878933)):
    x = [0, min_step] + [int(r["last_step"]) for r in segments]
    y = [3.0000003553435164, minimum] + [
        float(r[f"distance_end_{s}_m"]) for r in segments
    ]
    ax.plot(x, y, marker="o", label=str(s))
ax.axhline(.35, linestyle="--", label="0.35 m arrival threshold")
ax.set(xlabel="Episode step (selected observations only)",
       ylabel="Truth-pose goal distance (m)",
       title="Selected minima and 50-step endpoints — not complete trajectories")
ax.legend()
store(fig, "sac-goal-distance-selected")
print("Figures generated in", OUT)
