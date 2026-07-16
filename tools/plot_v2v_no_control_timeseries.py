#!/usr/bin/env python3
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


BASE = Path("results/bg500_base3_seed30_100s_retry/no-control")
CSV = BASE / "observation.csv"
OUT = BASE / "plots"


def to_float(value):
    if value in ("", "N/A", "-nan", "nan"):
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def load_rows():
    with CSV.open(newline="") as f:
        return [{k: to_float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def plot(rows, filename, title, series, ylabel):
    xs = [r["time_s"] for r in rows]
    plt.figure(figsize=(10, 5))
    for col, label in series:
        if col not in rows[0]:
            continue
        plt.plot(xs, [r.get(col, math.nan) for r in rows], label=label, linewidth=1.8)
    plt.title(title)
    plt.xlabel("Time [s]")
    plt.ylabel(ylabel)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUT / filename, dpi=160)
    plt.close()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rows = load_rows()

    plots = [
        ("01_cbr.png", "V2V no-control: CBR", [
            ("channel_busy_ratio_avg", "CBR avg"),
            ("channel_busy_ratio_max", "CBR max"),
            ("interference_offered_cbr", "BG offered CBR"),
        ], "ratio"),
        ("02_orr_pdr.png", "V2V no-control: Recognition and PDR", [
            ("orr", "ORR"),
            ("high_pdr", "High PDR"),
            ("low_pdr", "Low PDR"),
        ], "percent"),
        ("03_loss_breakdown.png", "V2V no-control: Loss breakdown", [
            ("high_packet_loss", "High packet loss"),
            ("low_packet_loss", "Low packet loss"),
            ("ttl_violation_rate", "TTL violation"),
            ("never_received_rate", "Never received"),
        ], "percent"),
        ("04_nr_packets.png", "V2V no-control: NR packets", [
            ("nr_tx", "NR TX"),
            ("nr_rx", "NR RX"),
            ("cpm_rx", "CPM RX"),
            ("cam_rx", "CAM RX"),
        ], "packets"),
        ("05_cpm_size_objects.png", "V2V no-control: CPM size and objects", [
            ("nr_cpm_size_bytes", "NR CPM size bytes"),
            ("nr_cpm_objects", "NR CPM objects"),
            ("nr_cpm_objects_total", "NR CPM objects total"),
        ], "count / bytes"),
        ("06_bg_load.png", "V2V no-control: Background load", [
            ("interference_tx", "BG TX"),
            ("interference_bytes", "BG bytes"),
            ("interference_drops", "BG drops"),
        ], "count / bytes"),
        ("07_latency.png", "V2V no-control: NR latency", [
            ("nr_latency_ms", "NR latency mean"),
        ], "ms"),
        ("08_active_routes.png", "V2V no-control: Active vehicles and routes", [
            ("active_vehicles", "Active vehicles"),
            ("nr_route_count", "NR route"),
            ("mec_route_count", "MEC route"),
            ("dual_route_count", "Dual route"),
        ], "vehicles"),
    ]

    for filename, title, series, ylabel in plots:
        plot(rows, filename, title, series, ylabel)

    print(f"Wrote {len(plots)} plots to {OUT}")


if __name__ == "__main__":
    main()
