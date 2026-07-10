#!/usr/bin/env python3
from pathlib import Path

import matplotlib.pyplot as plt

OUT = Path("src/automotive/examples/mec_latency_percentiles.pdf")

labels = ["平均", "p50", "p90", "p95", "p99"]
values_ms = [49.6, 49, 69, 74, 84]
colors = ["#4C78A8", "#4C78A8", "#F58518", "#F58518", "#E45756"]

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
plt.rcParams["font.size"] = 12

fig, ax = plt.subplots(figsize=(6.4, 3.4))
bars = ax.bar(labels, values_ms, color=colors, width=0.62)

for bar, value in zip(bars, values_ms):
    ax.text(
        bar.get_x() + bar.get_width() / 2,
        value + 2,
        f"{value:g} ms",
        ha="center",
        va="bottom",
        fontsize=12,
    )

ax.set_ylabel("MEC遅延 [ms]")
ax.set_ylim(0, 100)
ax.grid(axis="y", linestyle="--", alpha=0.35)
ax.set_axisbelow(True)

fig.tight_layout()
fig.savefig(OUT, bbox_inches="tight")
print(f"wrote {OUT}")
