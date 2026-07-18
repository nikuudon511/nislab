#!/usr/bin/env python3
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


BASE = Path("results")
OUT = Path("src/automotive/examples/v2n2v_policy_comparison.pdf")

RUNS = [
    ("High-only", "priority_same_seed30_high-priority-only", "v2n2v-high-priority-only"),
    ("Adaptive", "priority_same_seed30_adaptive", "v2n2v-adaptive-probability"),
    ("All-V2N2V", "priority_same_seed30_all-v2n2v", "v2n2v-all-object"),
]


def read_summary(run_dir: str):
    path = BASE / run_dir / "summary.csv"
    with path.open(newline="") as f:
        row = next(csv.DictReader(f))
    return {
        "orr": float(row["object_recognition_rate"]),
        "high": float(row["high_priority_object_recognition_rate"]),
        "low": float(row["low_priority_object_recognition_rate"]),
    }


def read_volume(run_dir: str, method: str):
    path = BASE / run_dir / method / "observation.csv"
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    row = rows[-1]
    uplink = float(row["mec_uplink_bytes"])
    forwarded = float(row["mec_forwarded_bytes"])
    return (uplink + forwarded) / 1_000_000.0


labels = []
orr = []
high = []
low = []
v2n2v_mb = []

for label, run_dir, method in RUNS:
    s = read_summary(run_dir)
    labels.append(label)
    orr.append(s["orr"])
    high.append(s["high"])
    low.append(s["low"])
    v2n2v_mb.append(read_volume(run_dir, method))

x = np.arange(len(labels))
width = 0.22

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "IPAexGothic", "DejaVu Sans"]
fig, ax1 = plt.subplots(figsize=(7.2, 4.0))

b1 = ax1.bar(x - width, orr, width, label="全体ORR", color="#4C78A8")
b2 = ax1.bar(x, high, width, label="高重要度ORR", color="#72B7B2")
b3 = ax1.bar(x + width, low, width, label="低重要度ORR", color="#F58518")
ax1.set_ylabel("認識率 [%]")
ax1.set_ylim(0, 110)
ax1.set_xticks(x)
ax1.set_xticklabels(labels)
ax1.grid(axis="y", linestyle="--", alpha=0.35)

for bars in (b1, b2, b3):
    for bar in bars:
        h = bar.get_height()
        ax1.text(
            bar.get_x() + bar.get_width() / 2,
            h + 1.5,
            f"{h:.1f}",
            ha="center",
            va="bottom",
            fontsize=10,
        )

ax2 = ax1.twinx()
line = ax2.plot(
    x,
    v2n2v_mb,
    marker="o",
    linewidth=2.0,
    color="#D62728",
    label="V2N2V送信量",
)
ax2.set_ylabel("V2N2V送信量 [MB]")
ax2.set_ylim(0, max(v2n2v_mb) * 1.25)
for xi, mb in zip(x, v2n2v_mb):
    ax2.text(xi, mb + max(v2n2v_mb) * 0.035, f"{mb:.1f}", ha="center", fontsize=10)

handles1, labels1 = ax1.get_legend_handles_labels()
handles2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(handles1 + handles2, labels1 + labels2, loc="upper left", fontsize=9)

fig.tight_layout()
OUT.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(OUT, bbox_inches="tight")
print(f"wrote {OUT}")

