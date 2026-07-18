#!/usr/bin/env python3
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

OUT = Path("src/automotive/examples/v2n2v_capacity_usage.pdf")

methods = ["高重要度V2N2V", "適応型V2N2V"]

# 120 s seed30, 500 byte per ideal MEC packet.
uplink_mbps = np.array([2.94, 2.94])
downlink_mbps = np.array([12.04, 15.56])

itu_ul_mbps = 50.0
itu_dl_mbps = 100.0
uplink_pct = uplink_mbps / itu_ul_mbps * 100.0
downlink_pct = downlink_mbps / itu_dl_mbps * 100.0

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
plt.rcParams["font.size"] = 12

x = np.arange(len(methods))
width = 0.34

fig, ax = plt.subplots(figsize=(7.0, 4.0))

bars_ul = ax.bar(x - width / 2, uplink_pct, width, color="#4C78A8", label="Uplink占有率")
bars_dl = ax.bar(x + width / 2, downlink_pct, width, color="#F58518", label="Downlink占有率")

for bar, rate, pct in zip(bars_ul, uplink_mbps, uplink_pct):
    ax.text(
        bar.get_x() + bar.get_width() / 2,
        pct + 0.7,
        f"{pct:.1f}%\n{rate:.1f} Mbps",
        ha="center",
        va="bottom",
        fontsize=10,
    )

for bar, rate, pct in zip(bars_dl, downlink_mbps, downlink_pct):
    ax.text(
        bar.get_x() + bar.get_width() / 2,
        pct + 0.7,
        f"{pct:.1f}%\n{rate:.1f} Mbps",
        ha="center",
        va="bottom",
        fontsize=10,
    )

ax.set_ylabel("IMT-2020基準に対する占有率 [%]")
ax.set_xticks(x)
ax.set_xticklabels(methods)
ax.set_ylim(0, 20)
ax.grid(axis="y", linestyle="--", alpha=0.35)
ax.set_axisbelow(True)

ax.legend(
    ncol=2,
    loc="lower center",
    bbox_to_anchor=(0.5, 1.02),
    fontsize=10,
)

fig.tight_layout(rect=(0, 0, 1, 0.92))
fig.savefig(OUT, bbox_inches="tight")
print(f"wrote {OUT}")
