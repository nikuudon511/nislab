#!/usr/bin/env python3
import csv
from pathlib import Path

import matplotlib.pyplot as plt

BASE = Path("results")
OUT = Path("src/automotive/examples/rmr_time_series_cbr_loss.pdf")

RUNS = [
    ("無制御", "moving_zone120_seed30_fix_no-control", "no-control", "#4C78A8"),
    ("反応型RMR", "moving_zone120_seed30_reactive-rmr_fair", "reactive-rmr", "#F58518"),
    ("予測RMR", "moving_zone120_seed30_fix_predictive-rmr-v2v", "predictive-rmr-v2v", "#54A24B"),
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
                return rows
    raise FileNotFoundError(f"valid observation.csv not found for {run_dir}/{method}")


def val(row, key):
    x = row.get(key, "N/A")
    if x == "N/A" or x == "":
        return None
    return float(x)


fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.0), sharex=True)

for label, run_dir, method, color in RUNS:
    rows = read_observation(run_dir, method)

    t = []
    cbr = []
    high_loss = []
    low_loss = []

    for r in rows:
        ts = int(float(r["time_s"]))
        if ts % 10 != 0:
            continue

        t.append(ts)
        cbr.append(float(r["channel_busy_ratio_avg"]) * 100.0)

        h = val(r, "high_packet_loss")
        l = val(r, "low_packet_loss")
        high_loss.append(0.0 if h is None else h)
        low_loss.append(0.0 if l is None else l)

    axes[0].plot(t, cbr, marker="o", linewidth=2.0, label=label, color=color)
    axes[1].plot(t, high_loss, marker="o", linewidth=2.0, label=f"{label} 高重要度", color=color)
    axes[1].plot(t, low_loss, marker="x", linestyle="--", linewidth=1.8, label=f"{label} 低重要度", color=color)

axes[0].set_ylabel("平均CBR [%]")
axes[0].grid(axis="y", linestyle="--", alpha=0.35)
axes[0].legend(loc="upper left", fontsize=9)

axes[1].set_ylabel("packet loss [%]")
axes[1].set_xlabel("時刻 [s]")
axes[1].grid(axis="y", linestyle="--", alpha=0.35)
axes[1].legend(loc="upper left", fontsize=8, ncol=2)

fig.tight_layout()
fig.savefig(OUT, bbox_inches="tight")
print(f"wrote {OUT}")