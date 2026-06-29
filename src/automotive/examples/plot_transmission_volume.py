import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


plt.rcParams.update(
    {
        "font.size": 13,
        "axes.labelsize": 13,
        "xtick.labelsize": 12,
        "ytick.labelsize": 12,
        "legend.fontsize": 11,
    }
)
plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK JP",
    "IPAexGothic",
    "IPAGothic",
    "Yu Gothic",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False


ROOT = Path(__file__).resolve().parents[3]

series = [
    (
        "無制御",
        ROOT / "results/moving_zone120_seed30_fix_no-control/no-control/observation.csv",
    ),
    (
        "反応型\nRMR",
        ROOT / "results/moving_zone120_seed30_reactive-rmr_fair/reactive-rmr/observation.csv",
    ),
    (
        "予測\nRMR",
        ROOT
        / "results/moving_zone120_seed30_fix_predictive-rmr-v2v/predictive-rmr-v2v/observation.csv",
    ),
    (
        "高重要度\nV2N2V",
        ROOT
        / "results/moving_zone120_seed30_fix_v2n2v-high-priority-only/v2n2v-high-priority-only/observation.csv",
    ),
    (
        "適応型\nV2N2V",
        ROOT
        / "results/moving_zone120_seed30_fix_v2n2v-adaptive-probability/v2n2v-adaptive-probability/observation.csv",
    ),
]


def rows(path):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"empty csv: {path}")
    return rows


methods = []
v2v_mb = []
v2n2v_mb = []

for label, path in series:
    data = rows(path)
    row = data[-1]
    methods.append(label)
    v2v_mb.append(sum(float(r["nr_cpm_size_bytes_total"]) for r in data) / 1_000_000)
    v2n2v_mb.append(
        (float(row["mec_uplink_bytes"]) + float(row["mec_forwarded_bytes"])) / 1_000_000
    )

x = np.arange(len(methods))
width = 0.62

fig, ax = plt.subplots(figsize=(6.0, 2.4))

ax.bar(x, v2v_mb, width, label="V2V", color="#4c78a8")
ax.bar(x, v2n2v_mb, width, bottom=v2v_mb, label="V2N2V", color="#f58518")

for i, value in enumerate(v2v_mb):
    ax.text(i, max(value, 2.0), f"{value:.1f}", ha="center", va="bottom", fontsize=9)

for i, value in enumerate(v2n2v_mb):
    if value <= 0:
        continue
    ax.text(
        i,
        v2v_mb[i] + value / 2,
        f"{value:.1f}",
        ha="center",
        va="center",
        fontsize=9,
        color="white",
        fontweight="bold",
    )

ax.set_ylabel("送信量（MB）")
ax.set_xticks(x)
ax.set_xticklabels(methods)
ax.set_ylim(0, max(np.array(v2v_mb) + np.array(v2n2v_mb)) * 1.18)
ax.grid(axis="y", linestyle="--", alpha=0.7)
ax.legend(
    loc="upper left",
    ncol=2,
    frameon=False,
    handlelength=1.4,
)

fig.tight_layout()
fig.savefig("transmission_volume_graph.pdf", format="pdf", bbox_inches="tight")
print("Wrote transmission_volume_graph.pdf")
