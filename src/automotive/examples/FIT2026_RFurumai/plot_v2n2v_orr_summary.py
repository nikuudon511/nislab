#!/usr/bin/env python3
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

OUT = Path("src/automotive/examples/v2n2v_orr_summary.pdf")

RUNS = [
    (
        "高重要度V2N2V",
        Path("results/moving_zone120_seed30_fix_v2n2v-high-priority-only/v2n2v-high-priority-only/summary.csv"),
    ),
    (
        "適応型V2N2V",
        Path("results/moving_zone120_seed30_fix_v2n2v-adaptive-probability/v2n2v-adaptive-probability/summary.csv"),
    ),
]

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
plt.rcParams["font.size"] = 12

metrics = [
    ("全体ORR", "object_recognition_rate", "#4C78A8"),
    ("高重要度ORR", "high_priority_object_recognition_rate", "#F58518"),
    ("低重要度ORR", "low_priority_object_recognition_rate", "#54A24B"),
]

methods = []
values = []
for label, path in RUNS:
    with path.open(newline="") as f:
        row = next(csv.DictReader(f))
    methods.append(label)
    values.append([float(row[key]) for _, key, _ in metrics])

x = np.arange(len(methods))
width = 0.24

fig, ax = plt.subplots(figsize=(6.8, 3.8))
for i, (metric_label, _, color) in enumerate(metrics):
    ys = [row[i] for row in values]
    bars = ax.bar(x + (i - 1) * width, ys, width, label=metric_label, color=color)
    for bar, value in zip(bars, ys):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            value + 1.2,
            f"{value:.1f}",
            ha="center",
            va="bottom",
            fontsize=11,
        )

ax.set_ylabel("認識率 [%]")
ax.set_xticks(x)
ax.set_xticklabels(methods)
ax.set_ylim(0, 110)
ax.grid(axis="y", linestyle="--", alpha=0.35)
ax.set_axisbelow(True)
ax.legend(ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.02), fontsize=10)

fig.tight_layout(rect=(0, 0, 1, 0.92))
fig.savefig(OUT, bbox_inches="tight")
print(f"wrote {OUT}")
