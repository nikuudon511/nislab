#!/usr/bin/env python3
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

BASE = Path("results")
OUT = Path("src/automotive/examples/rmr_3method_rmr_effect_graph.pdf")

RUNS = [
    ("無制御", "moving_zone120_seed30_fix_no-control", "no-control"),
    ("反応型RMR", "moving_zone120_seed30_reactive-rmr_fair", "reactive-rmr"),
    ("予測RMR", "moving_zone120_seed30_fix_predictive-rmr-v2v", "predictive-rmr-v2v"),
]

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]


def read_observation(run_dir, method):
    candidates = [
        BASE / run_dir / method / "observation.csv",
        BASE / run_dir / "observation.csv",
    ]
    for path in candidates:
        if path.exists() and path.stat().st_size > 0:
            with path.open(newline="") as f:
                rows = list(csv.DictReader(f))
            if rows:
                return rows[-1]
    raise FileNotFoundError(f"valid observation.csv not found for {run_dir}/{method}")


labels = []
nr_tx = []
objects = []
deleted = []

for label, run_dir, method in RUNS:
    o = read_observation(run_dir, method)
    labels.append(label)
    nr_tx.append(float(o["nr_tx"]))
    objects.append(float(o["nr_cpm_objects_total"]))
    deleted.append(float(o["nr_rmr_deleted_total"]))

x = np.arange(len(labels))
width = 0.26

fig, ax1 = plt.subplots(figsize=(7.2, 4.0))

b1 = ax1.bar(x - width / 2, np.array(objects) / 1000, width,
            label="CPM内物標数", color="#4C78A8")
b2 = ax1.bar(x + width / 2, np.array(deleted) / 1000, width,
            label="RMR削除物標数", color="#F58518")

ax1.set_ylabel("物標数 [千個]")
ax1.set_xticks(x)
ax1.set_xticklabels(labels)
ax1.grid(axis="y", linestyle="--", alpha=0.35)

for bars in (b1, b2):
    for b in bars:
        h = b.get_height()
        ax1.text(b.get_x() + b.get_width() / 2, h + max(max(objects), max(deleted)) / 1000 * 0.02,
                f"{h:.0f}", ha="center", va="bottom", fontsize=11)

ax2 = ax1.twinx()
ax2.plot(x, nr_tx, marker="o", linewidth=2.0, color="#D62728", label="V2V送信パケット数")
ax2.set_ylabel("V2V送信パケット数")
ax2.set_ylim(0, max(nr_tx) * 1.25)

for xi, y in zip(x, nr_tx):
    ax2.text(xi, y + max(nr_tx) * 0.035, f"{y:.0f}",
            ha="center", va="bottom", fontsize=11, color="#D62728")

h1, l1 = ax1.get_legend_handles_labels()
h2, l2 = ax2.get_legend_handles_labels()
ax1.legend(h1 + h2, l1 + l2,
            loc="upper center",
            bbox_to_anchor=(0.5, -0.16),
            ncol=3,
            fontsize=10)

fig.tight_layout(rect=[0, 0.08, 1, 1])
fig.savefig(OUT, bbox_inches="tight")
print(f"wrote {OUT}")