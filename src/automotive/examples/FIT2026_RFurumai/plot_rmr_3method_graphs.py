#!/usr/bin/env python3
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

BASE = Path("results")

RUNS = [
    ("無制御", "moving_zone120_seed30_fix_no-control", "no-control"),
    ("反応型RMR", "moving_zone120_seed30_reactive-rmr_fair", "reactive-rmr"),
    ("予測RMR", "moving_zone120_seed30_fix_predictive-rmr-v2v", "predictive-rmr-v2v"),
]

OUT_ORR = Path("src/automotive/examples/rmr_3method_orr_graph.pdf")
OUT_TX = Path("src/automotive/examples/rmr_3method_transmission_graph.pdf")

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]


def read_summary(run_dir, method):
      candidates = [
          BASE / run_dir / "summary.csv",
          BASE / run_dir / method / "summary.csv",
      ]
      for path in candidates:
          if path.exists() and path.stat().st_size > 0:
              with path.open(newline="") as f:
                  rows = list(csv.DictReader(f))
              if rows:
                  return rows[0]
      raise FileNotFoundError(f"valid summary.csv not found for {run_dir}/{method}")


def read_observation(run_dir, method):
    path = BASE / run_dir / method / "observation.csv"
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    return rows[-1]


labels = []
orr = []
high = []
low = []
cbr = []
v2v_mb = []

for label, run_dir, method in RUNS:
    s = read_summary(run_dir, method)
    o = read_observation(run_dir, method)

    labels.append(label)
    orr.append(float(s["object_recognition_rate"]))
    high.append(float(s["high_priority_object_recognition_rate"]))
    low.append(float(s["low_priority_object_recognition_rate"]))
    cbr.append(float(s["channel_busy_ratio"]) * 100.0)
    v2v_mb.append(float(o["nr_cpm_size_bytes_total"]) / 1_000_000.0)


x = np.arange(len(labels))
width = 0.22

# ORR + CBR graph
fig, ax1 = plt.subplots(figsize=(6.8, 3.8))
bars1 = ax1.bar(x - width, orr, width, label="全体ORR", color="#4C78A8")
bars2 = ax1.bar(x, high, width, label="高重要度ORR", color="#72B7B2")
bars3 = ax1.bar(x + width, low, width, label="低重要度ORR", color="#F58518")

ax1.set_ylabel("認識率 [%]")
ax1.set_ylim(0, 110)
ax1.set_xticks(x)
ax1.set_xticklabels(labels)
ax1.grid(axis="y", linestyle="--", alpha=0.35)

for bars in (bars1, bars2, bars3):
    for b in bars:
        h = b.get_height()
        ax1.text(b.get_x() + b.get_width() / 2, h + 1.3, f"{h:.1f}",
                 ha="center", va="bottom", fontsize=11)

ax2 = ax1.twinx()
ax2.plot(x, cbr, marker="o", linewidth=2.0, color="#D62728", label="平均CBR")
ax2.set_ylabel("平均CBR [%]")
ax2.set_ylim(0, max(cbr) * 1.25)

for xi, y in zip(x, cbr):
    ax2.text(xi, y + max(cbr) * 0.035, f"{y:.1f}",
             ha="center", va="bottom", fontsize=11, color="#D62728")

h1, l1 = ax1.get_legend_handles_labels()
h2, l2 = ax2.get_legend_handles_labels()
ax1.legend(h1 + h2, l1 + l2,
           loc="upper center",
           bbox_to_anchor=(0.5, -0.16),
           ncol=4,
           fontsize=9)

fig.tight_layout(rect=[0, 0.08, 1, 1])
fig.savefig(OUT_ORR, bbox_inches="tight")
print(f"wrote {OUT_ORR}")

# Transmission graph
fig, ax = plt.subplots(figsize=(6.8, 3.8))
bars = ax.bar(x, v2v_mb, width=0.45, label="V2V送信量", color="#4C78A8")
ax.set_ylabel("V2V送信量 [MB]")
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.grid(axis="y", linestyle="--", alpha=0.35)

ax.legend(loc="upper right", fontsize=10)

fig.tight_layout()
fig.savefig(OUT_TX, bbox_inches="tight")
print(f"wrote {OUT_TX}")
