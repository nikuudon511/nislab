#!/usr/bin/env python3
import csv
from pathlib import Path

import matplotlib.pyplot as plt

BASE = Path("results")
OUT = Path("src/automotive/examples/rmr_time_behavior.pdf")
OUT_ENV = Path("src/automotive/examples/rmr_time_environment.pdf")
OUT_LOAD = Path("src/automotive/examples/rmr_time_load_reduction.pdf")
OUT_LOSS = Path("src/automotive/examples/rmr_time_cbr_loss.pdf")
OUT_ZOOM = Path("src/automotive/examples/rmr_zoom_95_120.pdf")
OUT_ORR = Path("src/automotive/examples/rmr_time_orr.pdf")
WINDOW_SECONDS = 10
BG_PACKET_SIZE_BYTES = 800
BG_INTERVAL_MS = 80
BG_LOW_FACTOR = 0.2
LOW_BG_KB_PER_VEHICLE = (
    BG_PACKET_SIZE_BYTES * BG_LOW_FACTOR * (WINDOW_SECONDS * 1000.0 / BG_INTERVAL_MS) / 1000.0
)

RUNS = [
    ("無制御", "moving_zone120_seed30_fix_no-control", "no-control", "#4C78A8"),
    ("反応型RMR", "moving_zone120_seed30_reactive-rmr_fair", "reactive-rmr", "#F58518"),
    ("予測RMR", "moving_zone120_seed30_fix_predictive-rmr-v2v", "predictive-rmr-v2v", "#54A24B"),
]

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
plt.rcParams["font.size"] = 11


def read_rows(run_dir, method):
    path = BASE / run_dir / method / "observation.csv"
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def fval(row, key):
    v = row.get(key, "0")
    return 0.0 if v in ("", "N/A") else float(v)


def summarize_zoom(rows, start_s=95, end_s=119):
    selected = [row for row in rows if start_s <= int(float(row["time_s"])) <= end_s]
    t = []
    bg_kb = []
    active_vehicles = []
    cpm_size = []
    cbr_avg = []
    cbr_max = []
    prev_bg_bytes = None
    for row in selected:
        time_s = int(float(row["time_s"]))
        bg_bytes = fval(row, "interference_bytes")
        if prev_bg_bytes is None:
            prev_bg_bytes = bg_bytes
            continue
        t.append(time_s)
        bg_kb.append(max(0.0, bg_bytes - prev_bg_bytes) / 1000.0)
        prev_bg_bytes = bg_bytes
        active_vehicles.append(fval(row, "active_vehicles"))
        cpm_size.append(fval(row, "nr_cpm_size_bytes") / 1000.0)
        cbr_avg.append(fval(row, "channel_busy_ratio_avg") * 100.0)
        cbr_max.append(fval(row, "channel_busy_ratio_max") * 100.0)
    return t, bg_kb, active_vehicles, cpm_size, cbr_avg, cbr_max


def summarize_orr(rows):
    t = []
    orr = []
    for row in rows:
        value = row.get("orr", "N/A")
        if value in ("", "N/A"):
            continue
        t.append(float(row["time_s"]))
        orr.append(float(value))
    return t, orr


def summarize_by_window(rows):
    windows = {}
    for row in rows:
        time_s = int(float(row["time_s"]))
        bucket = (time_s // WINDOW_SECONDS) * WINDOW_SECONDS
        data = windows.setdefault(
            bucket,
            {
                "count": 0,
                "cbr_avg_sum": 0.0,
                "cbr_max": 0.0,
                "bg_bytes_last": 0.0,
                "active_vehicles_last": 0.0,
                "lost_packets_last": 0.0,
                "size_sum": 0.0,
                "objects_sum": 0.0,
                "deleted_sum": 0.0,
            },
        )
        data["count"] += 1
        data["cbr_avg_sum"] += fval(row, "channel_busy_ratio_avg") * 100.0
        data["cbr_max"] = max(data["cbr_max"], fval(row, "channel_busy_ratio_max") * 100.0)
        data["bg_bytes_last"] = fval(row, "interference_bytes")
        data["active_vehicles_last"] = fval(row, "active_vehicles")
        high_lost = fval(row, "high_ideal_rx") - fval(row, "high_true_rx")
        low_lost = fval(row, "low_ideal_rx") - fval(row, "low_true_rx")
        data["lost_packets_last"] = max(0.0, high_lost + low_lost)
        data["size_sum"] += fval(row, "nr_cpm_size_bytes")
        data["objects_sum"] += fval(row, "nr_cpm_objects")
        data["deleted_sum"] += fval(row, "nr_rmr_deleted_last")

    t = []
    cbr_avg = []
    cbr_max = []
    bg_kb = []
    active_vehicles = []
    lost_packets = []
    size_kb = []
    objects = []
    deleted = []
    prev_bg_bytes = None
    prev_lost_packets = None
    for bucket in sorted(windows):
        if bucket == 0:
            prev_bg_bytes = windows[bucket]["bg_bytes_last"]
            prev_lost_packets = windows[bucket]["lost_packets_last"]
            continue
        data = windows[bucket]
        t.append(bucket)
        cbr_avg.append(data["cbr_avg_sum"] / data["count"])
        cbr_max.append(data["cbr_max"])
        bg_delta = 0.0 if prev_bg_bytes is None else data["bg_bytes_last"] - prev_bg_bytes
        bg_kb.append(max(0.0, bg_delta) / 1000.0)
        prev_bg_bytes = data["bg_bytes_last"]
        active_vehicles.append(data["active_vehicles_last"])
        loss_delta = 0.0 if prev_lost_packets is None else data["lost_packets_last"] - prev_lost_packets
        lost_packets.append(max(0.0, loss_delta))
        prev_lost_packets = data["lost_packets_last"]
        size_kb.append(data["size_sum"] / 1000.0)
        objects.append(data["objects_sum"])
        deleted.append(data["deleted_sum"])
    return t, cbr_avg, cbr_max, bg_kb, active_vehicles, lost_packets, size_kb, objects, deleted


series = []
zoom_series = []
orr_series = []
for label, run_dir, method, color in RUNS:
    rows = read_rows(run_dir, method)
    series.append((label, color, summarize_by_window(rows)))
    zoom_series.append((label, color, summarize_zoom(rows)))
    orr_series.append((label, color, summarize_orr(rows)))


def style_axis(ax):
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend(fontsize=9, ncol=2, loc="upper left")


def save_environment_plot():
    fig_env, ax_bg = plt.subplots(figsize=(7.4, 3.4))
    ax_active = ax_bg.twinx()
    t, _, _, bg_kb, active_vehicles, _, _, _, _ = series[0][2]
    low_bg_kb = [
        v * LOW_BG_KB_PER_VEHICLE
        for v in active_vehicles
    ]
    ax_bg.plot(t, bg_kb, marker="o", color="#4C78A8", label="BG送信量")
    ax_bg.plot(t, low_bg_kb, linestyle="--", color="#4C78A8", alpha=0.65, label="低負荷時の推定BG送信量")
    ax_active.bar(t, active_vehicles, width=6.0, color="#E45756", alpha=0.18, label="通信車両数")
    ax_bg.set_ylabel("BG送信量 [kB/10s]")
    ax_active.set_ylabel("通信車両数")
    ax_bg.set_xlabel("時刻 [s]")
    ax_bg.grid(axis="y", linestyle="--", alpha=0.35)
    ax_active.set_ylim(
        ax_bg.get_ylim()[0] / LOW_BG_KB_PER_VEHICLE,
        ax_bg.get_ylim()[1] / LOW_BG_KB_PER_VEHICLE,
    )
    lines1, labels1 = ax_bg.get_legend_handles_labels()
    lines2, labels2 = ax_active.get_legend_handles_labels()
    ax_bg.legend(
        lines1 + lines2,
        labels1 + labels2,
        fontsize=10,
        ncol=3,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.02),
    )
    fig_env.tight_layout(rect=(0, 0, 1, 0.92))
    fig_env.savefig(OUT_ENV, bbox_inches="tight")


def save_load_reduction_plot():
    fig_load, axes_load = plt.subplots(2, 1, figsize=(7.4, 5.0), sharex=True)
    for label, color, data in series:
        t, _, _, _, _, _, size_kb, _, deleted = data
        axes_load[0].plot(t, size_kb, marker="o", color=color, label=label)
        axes_load[1].plot(t, deleted, marker="o", color=color, label=label)
    axes_load[0].set_ylabel("CPMサイズ量\n[kB/10s]")
    axes_load[1].set_ylabel("RMR削除物標数\n[個/10s]")
    axes_load[1].set_xlabel("時刻 [s]")
    for ax in axes_load:
        style_axis(ax)
    fig_load.tight_layout()
    fig_load.savefig(OUT_LOAD, bbox_inches="tight")


def save_cbr_loss_plot():
    fig_loss, axes_loss = plt.subplots(2, 1, figsize=(7.4, 5.0), sharex=True)
    for label, color, data in series:
        t, cbr_avg, cbr_max, _, _, lost_packets, _, _, _ = data
        axes_loss[0].plot(t, cbr_avg, marker="o", color=color, label=f"{label} avg")
        axes_loss[0].plot(t, cbr_max, marker="x", linestyle="--", color=color, label=f"{label} max")
        axes_loss[1].plot(t, lost_packets, marker="o", color=color, label=label)
    axes_loss[0].set_ylabel("CBR [%]")
    axes_loss[1].set_ylabel("パケットロス数\n[個/10s]")
    axes_loss[1].set_xlabel("時刻 [s]")
    for ax in axes_loss:
        ax.grid(axis="y", linestyle="--", alpha=0.35)
    axes_loss[0].legend(
        fontsize=9,
        ncol=3,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.02),
    )
    axes_loss[1].legend(fontsize=9, ncol=3, loc="upper left")
    fig_loss.tight_layout()
    fig_loss.savefig(OUT_LOSS, bbox_inches="tight")


def save_zoom_plot():
    fig_zoom, axes_zoom = plt.subplots(3, 1, figsize=(7.4, 6.4), sharex=True)
    ax_active = axes_zoom[0].twinx()
    bg_drawn = False
    for label, color, data in zoom_series:
        t, bg_kb, active_vehicles, cpm_size, cbr_avg, cbr_max = data
        if not bg_drawn:
            axes_zoom[0].plot(t, bg_kb, marker="o", color="#4C78A8", label="BG送信量")
            ax_active.bar(t, active_vehicles, width=0.7, color="#E45756", alpha=0.18, label="通信車両数")
            bg_drawn = True
        axes_zoom[1].plot(t, cpm_size, marker="o", color=color, label=label)
        axes_zoom[2].plot(t, cbr_avg, marker="o", color=color, label=f"{label} avg")
        axes_zoom[2].plot(t, cbr_max, marker="x", linestyle="--", color=color, label=f"{label} max")

    axes_zoom[0].set_ylabel("BG送信量\n[kB/s]")
    ax_active.set_ylabel("通信車両数")
    axes_zoom[1].set_ylabel("CPMサイズ\n[kB]")
    axes_zoom[2].set_ylabel("CBR [%]")
    axes_zoom[2].set_xlabel("時刻 [s]")

    for ax in axes_zoom:
        ax.grid(axis="y", linestyle="--", alpha=0.35)
    axes_zoom[1].legend(fontsize=9, ncol=3, loc="upper left")
    axes_zoom[2].legend(
        fontsize=8,
        ncol=3,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.02),
    )
    lines1, labels1 = axes_zoom[0].get_legend_handles_labels()
    lines2, labels2 = ax_active.get_legend_handles_labels()
    axes_zoom[0].legend(
        lines1 + lines2,
        labels1 + labels2,
        fontsize=9,
        ncol=2,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.02),
    )
    fig_zoom.tight_layout(rect=(0, 0, 1, 0.97))
    fig_zoom.savefig(OUT_ZOOM, bbox_inches="tight")


def save_orr_plot():
    fig_orr, ax_orr = plt.subplots(figsize=(7.4, 3.6))
    for label, color, data in orr_series:
        t, orr = data
        ax_orr.plot(t, orr, marker="o", markersize=3, color=color, label=label)
    ax_orr.set_ylabel("ORR [%]")
    ax_orr.set_xlabel("時刻 [s]")
    ax_orr.grid(axis="y", linestyle="--", alpha=0.35)
    ax_orr.legend(
        fontsize=9,
        ncol=3,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.02),
    )
    fig_orr.tight_layout(rect=(0, 0, 1, 0.92))
    fig_orr.savefig(OUT_ORR, bbox_inches="tight")


fig, axes = plt.subplots(5, 1, figsize=(7.4, 9.2), sharex=True)
bg_active_ax = axes[1].twinx()
bg_panel_drawn = False

for label, color, data in series:
    t, cbr_avg, cbr_max, bg_kb, active_vehicles, lost_packets, size_kb, objects, deleted = data

    axes[0].plot(t, cbr_avg, marker="o", color=color, label=f"{label} avg")
    axes[0].plot(t, cbr_max, marker="x", linestyle="--", color=color, label=f"{label} max")
    if not bg_panel_drawn:
        low_bg_kb = [
            v * LOW_BG_KB_PER_VEHICLE
            for v in active_vehicles
        ]
        axes[1].plot(t, bg_kb, marker="o", color="#4C78A8", label="BG送信量")
        axes[1].plot(
            t,
            low_bg_kb,
            linestyle="--",
            color="#4C78A8",
            alpha=0.65,
            label="低負荷時の推定BG送信量",
        )
        bg_active_ax.bar(
            t,
            active_vehicles,
            width=6.0,
            color="#E45756",
            alpha=0.18,
            label="通信車両数",
        )
        bg_panel_drawn = True
    axes[2].plot(t, size_kb, marker="o", color=color, label=label)
    axes[3].plot(t, objects, marker="o", color=color, label=label)
    axes[4].plot(t, deleted, marker="o", color=color, label=label)

axes[0].set_ylabel("CBR [%]")
axes[1].set_ylabel("BG送信量\n[kB/10s]")
bg_active_ax.set_ylabel("通信車両数")
axes[2].set_ylabel("CPMサイズ量\n[kB/10s]")
axes[3].set_ylabel("CPM内物標数\n[個/10s]")
axes[4].set_ylabel("RMR削除物標数\n[個/10s]")
axes[4].set_xlabel("時刻 [s]")

for ax in axes:
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend(fontsize=8, ncol=2, loc="upper left")

bg_active_ax.grid(False)
bg_active_ax.set_ylim(
    axes[1].get_ylim()[0] / LOW_BG_KB_PER_VEHICLE,
    axes[1].get_ylim()[1] / LOW_BG_KB_PER_VEHICLE,
)
lines1, labels1 = axes[1].get_legend_handles_labels()
lines2, labels2 = bg_active_ax.get_legend_handles_labels()
axes[1].legend(
    lines1 + lines2,
    labels1 + labels2,
    fontsize=8,
    ncol=3,
    loc="lower center",
    bbox_to_anchor=(0.5, 1.02),
)

fig.tight_layout()
fig.savefig(OUT, bbox_inches="tight")
print(f"wrote {OUT}")
save_environment_plot()
save_load_reduction_plot()
save_cbr_loss_plot()
save_zoom_plot()
save_orr_plot()
print(f"wrote {OUT_ENV}")
print(f"wrote {OUT_LOAD}")
print(f"wrote {OUT_LOSS}")
print(f"wrote {OUT_ZOOM}")
print(f"wrote {OUT_ORR}")
