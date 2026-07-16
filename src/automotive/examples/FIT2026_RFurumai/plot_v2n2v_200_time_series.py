#!/usr/bin/env python3
import csv
from pathlib import Path

import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / "src/automotive/examples/v2n2v_200_time_series.pdf"

RUNS = [
    {
        "label": "V2N2V-only",
        "path": ROOT
        / "results/v2n2v_step200_geo_v2n2v_only_10s/v2n2v-only/observation.csv",
        "color": "#4C78A8",
    },
    {
        "label": "Hybrid forced",
        "path": ROOT
        / "results/v2n2v_step200_geo_hybrid_forced_10s/v2n2v-adaptive-probability/observation.csv",
        "color": "#F58518",
    },
]

plt.rcParams["font.family"] = ["DejaVu Sans"]
plt.rcParams["font.size"] = 10


def value(row, key):
    raw = row.get(key, "")
    if raw in ("", "N/A"):
        return None
    return float(raw)


def read_series(path):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    out = {
        "t": [],
        "active": [],
        "dl_million": [],
        "dl_delta_kpps": [],
        "aoi_p99_s": [],
        "cap_p99_s": [],
        "orr": [],
        "packet_loss": [],
    }
    prev_dl = None
    prev_t = None
    for row in rows:
        t = value(row, "time_s")
        if t is None:
            continue
        dl = value(row, "mec_forwarded_packets") or 0.0
        if prev_dl is None or prev_t is None or t <= prev_t:
            dl_delta = 0.0
        else:
            dl_delta = (dl - prev_dl) / (t - prev_t) / 1000.0

        out["t"].append(t)
        out["active"].append(value(row, "active_vehicles") or 0.0)
        out["dl_million"].append(dl / 1_000_000.0)
        out["dl_delta_kpps"].append(max(0.0, dl_delta))
        out["aoi_p99_s"].append((value(row, "mec_aoi_p99_ms") or 0.0) / 1000.0)
        out["cap_p99_s"].append((value(row, "mec_capacity_delay_p99_ms") or 0.0) / 1000.0)
        out["orr"].append(value(row, "orr"))
        out["packet_loss"].append(value(row, "high_packet_loss"))

        prev_dl = dl
        prev_t = t
    return out


def plot_metric(ax, series, key, ylabel, ylim=None):
    for run, data in series:
        xs = []
        ys = []
        for t, y in zip(data["t"], data[key]):
            if y is not None:
                xs.append(t)
                ys.append(y)
        ax.plot(xs, ys, color=run["color"], linewidth=2.0, label=run["label"])
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    if ylim:
        ax.set_ylim(*ylim)


def main():
    series = [(run, read_series(run["path"])) for run in RUNS]

    fig, axes = plt.subplots(3, 2, figsize=(9.2, 8.0), sharex=True)
    axes = axes.ravel()

    plot_metric(axes[0], series, "dl_million", "MEC downlink\ncumulative [M packets]")
    plot_metric(axes[1], series, "dl_delta_kpps", "MEC downlink\nrate [k packets/s]")
    plot_metric(axes[2], series, "aoi_p99_s", "AoI p99 [s]")
    plot_metric(axes[3], series, "cap_p99_s", "Capacity delay\np99 [s]")
    plot_metric(axes[4], series, "orr", "ORR [%]", ylim=(0, 105))
    plot_metric(axes[5], series, "active", "Active vehicles")

    for ax in axes[-2:]:
        ax.set_xlabel("Time [s]")
    axes[0].legend(loc="upper left", frameon=False)
    fig.suptitle("200-vehicle V2N2V time-series comparison", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(OUT, bbox_inches="tight")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
