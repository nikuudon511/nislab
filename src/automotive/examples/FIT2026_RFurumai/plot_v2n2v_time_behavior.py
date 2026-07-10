#!/usr/bin/env python3
import csv
from pathlib import Path

import matplotlib.pyplot as plt

BASE = Path("results")
OUT_ENV = Path("src/automotive/examples/v2n2v_time_environment.pdf")
OUT_TX = Path("src/automotive/examples/v2n2v_time_transmission.pdf")
OUT_CBR_LOSS = Path("src/automotive/examples/v2n2v_time_cbr_loss.pdf")
OUT_ORR = Path("src/automotive/examples/v2n2v_time_orr.pdf")
WINDOW_SECONDS = 10

RUNS = [
    (
        "高重要度V2N2V",
        "moving_zone120_seed30_fix_v2n2v-high-priority-only",
        "v2n2v-high-priority-only",
        "#F58518",
    ),
    (
        "適応型V2N2V",
        "moving_zone120_seed30_fix_v2n2v-adaptive-probability",
        "v2n2v-adaptive-probability",
        "#54A24B",
    ),
]

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
plt.rcParams["font.size"] = 11


def fval(row, key):
    v = row.get(key, "0")
    return 0.0 if v in ("", "N/A") else float(v)


def read_rows(run_dir, method):
    path = BASE / run_dir / method / "observation.csv"
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def summarize_by_window(rows):
    windows = {}
    for row in rows:
        time_s = int(float(row["time_s"]))
        bucket = (time_s // WINDOW_SECONDS) * WINDOW_SECONDS
        data = windows.setdefault(
            bucket,
            {
                "count": 0,
                "active_last": 0.0,
                "bg_bytes_last": 0.0,
                "mec_uplink_last": 0.0,
                "mec_forward_last": 0.0,
                "nr_size_sum": 0.0,
                "cbr_avg_sum": 0.0,
                "cbr_max": 0.0,
                "lost_last": 0.0,
                "orr_last": None,
            },
        )
        data["count"] += 1
        data["active_last"] = fval(row, "active_vehicles")
        data["bg_bytes_last"] = fval(row, "interference_bytes")
        data["mec_uplink_last"] = fval(row, "mec_uplink_bytes")
        data["mec_forward_last"] = fval(row, "mec_forwarded_bytes")
        data["nr_size_sum"] += fval(row, "nr_cpm_size_bytes")
        data["cbr_avg_sum"] += fval(row, "channel_busy_ratio_avg") * 100.0
        data["cbr_max"] = max(data["cbr_max"], fval(row, "channel_busy_ratio_max") * 100.0)
        data["lost_last"] = max(
            0.0,
            (fval(row, "high_ideal_rx") - fval(row, "high_true_rx"))
            + (fval(row, "low_ideal_rx") - fval(row, "low_true_rx")),
        )
        if row.get("orr", "N/A") not in ("", "N/A"):
            data["orr_last"] = fval(row, "orr")

    out = {k: [] for k in [
        "t", "active", "bg_kb", "v2v_kb", "uplink_kb", "forward_kb",
        "cbr_avg", "cbr_max", "lost", "orr"
    ]}
    prev = {"bg": None, "uplink": None, "forward": None, "lost": None}
    for bucket in sorted(windows):
        data = windows[bucket]
        if bucket == 0:
            prev["bg"] = data["bg_bytes_last"]
            prev["uplink"] = data["mec_uplink_last"]
            prev["forward"] = data["mec_forward_last"]
            prev["lost"] = data["lost_last"]
            continue
        out["t"].append(bucket)
        out["active"].append(data["active_last"])
        out["bg_kb"].append(max(0.0, data["bg_bytes_last"] - prev["bg"]) / 1000.0)
        out["uplink_kb"].append(max(0.0, data["mec_uplink_last"] - prev["uplink"]) / 1000.0)
        out["forward_kb"].append(max(0.0, data["mec_forward_last"] - prev["forward"]) / 1000.0)
        out["v2v_kb"].append(data["nr_size_sum"] / 1000.0)
        out["cbr_avg"].append(data["cbr_avg_sum"] / data["count"])
        out["cbr_max"].append(data["cbr_max"])
        out["lost"].append(max(0.0, data["lost_last"] - prev["lost"]))
        out["orr"].append(data["orr_last"])
        prev["bg"] = data["bg_bytes_last"]
        prev["uplink"] = data["mec_uplink_last"]
        prev["forward"] = data["mec_forward_last"]
        prev["lost"] = data["lost_last"]
    return out


series = []
for label, run_dir, method, color in RUNS:
    series.append((label, color, summarize_by_window(read_rows(run_dir, method))))


def save_environment():
    fig, ax = plt.subplots(figsize=(7.4, 3.4))
    ax2 = ax.twinx()
    data = series[0][2]
    ax.plot(data["t"], data["bg_kb"], marker="o", color="#4C78A8", label="BG送信量")
    ax2.bar(data["t"], data["active"], width=6.0, color="#E45756", alpha=0.18, label="通信車両数")
    ax.set_ylabel("BG送信量 [kB/10s]")
    ax2.set_ylabel("通信車両数")
    ax.set_xlabel("時刻 [s]")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, ncol=2, loc="lower center", bbox_to_anchor=(0.5, 1.02))
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(OUT_ENV, bbox_inches="tight")


def save_transmission():
    fig, axes = plt.subplots(2, 1, figsize=(7.4, 5.2), sharex=True)
    for label, color, data in series:
        axes[0].plot(data["t"], data["v2v_kb"], marker="o", color=color, label=label)
        axes[1].plot(data["t"], data["uplink_kb"], marker="o", color=color, label=f"{label} uplink")
        axes[1].plot(data["t"], data["forward_kb"], marker="x", linestyle="--", color=color, label=f"{label} forward")
    axes[0].set_ylabel("V2V CPMサイズ量\n[kB/10s]")
    axes[1].set_ylabel("V2N2V送信量\n[kB/10s]")
    axes[1].set_xlabel("時刻 [s]")
    for ax in axes:
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.legend(fontsize=8, ncol=2, loc="upper left")
    fig.tight_layout()
    fig.savefig(OUT_TX, bbox_inches="tight")


def save_cbr_loss():
    fig, axes = plt.subplots(2, 1, figsize=(7.4, 5.0), sharex=True)
    for label, color, data in series:
        axes[0].plot(data["t"], data["cbr_avg"], marker="o", color=color, label=f"{label} avg")
        axes[0].plot(data["t"], data["cbr_max"], marker="x", linestyle="--", color=color, label=f"{label} max")
        axes[1].plot(data["t"], data["lost"], marker="o", color=color, label=label)
    axes[0].set_ylabel("CBR [%]")
    axes[1].set_ylabel("パケットロス数\n[個/10s]")
    axes[1].set_xlabel("時刻 [s]")
    for ax in axes:
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.legend(fontsize=8, ncol=2, loc="upper left")
    fig.tight_layout()
    fig.savefig(OUT_CBR_LOSS, bbox_inches="tight")


def save_orr():
    fig, ax = plt.subplots(figsize=(7.4, 3.4))
    for label, color, data in series:
        t = [x for x, y in zip(data["t"], data["orr"]) if y is not None]
        orr = [y for y in data["orr"] if y is not None]
        ax.plot(t, orr, marker="o", color=color, label=label)
    ax.set_ylabel("ORR [%]")
    ax.set_xlabel("時刻 [s]")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend(ncol=2, loc="lower center", bbox_to_anchor=(0.5, 1.02))
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(OUT_ORR, bbox_inches="tight")


save_environment()
save_transmission()
save_cbr_loss()
save_orr()
for path in (OUT_ENV, OUT_TX, OUT_CBR_LOSS, OUT_ORR):
    print(f"wrote {path}")
