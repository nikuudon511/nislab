#!/usr/bin/env python3
import csv
import math
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib.pyplot as plt


BASE = Path("results/bg500_base3_seed30_100s_retry/no-control")
CSV = BASE / "observation.csv"
OUT = BASE / "slide_plots"


def value(x):
    if x in ("", "N/A"):
        return math.nan
    try:
        return float(x)
    except ValueError:
        return math.nan


def rows():
    with CSV.open(newline="") as f:
        return [{k: value(v) for k, v in r.items()} for r in csv.DictReader(f)]


def series(data, col):
    return [r.get(col, math.nan) for r in data]


def save(name):
    plt.tight_layout()
    plt.savefig(OUT / name, dpi=180)
    plt.close()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    data = rows()
    t = series(data, "time_s")

    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.plot(t, series(data, "channel_busy_ratio_avg"), label="CBR avg", color="#1f77b4", linewidth=2)
    ax1.plot(t, series(data, "channel_busy_ratio_max"), label="CBR max", color="#4c78a8", alpha=0.7)
    ax1.axhline(0.8, color="#d62728", linestyle="--", linewidth=1.5, label="CBR=0.8")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("CBR")
    ax1.set_ylim(0, 1.05)
    ax1.grid(True, alpha=0.25)
    ax2 = ax1.twinx()
    ax2.plot(t, series(data, "orr"), label="ORR", color="#2ca02c", linewidth=2)
    ax2.set_ylabel("ORR [%]")
    ax2.set_ylim(0, 100)
    lines = ax1.get_lines() + ax2.get_lines()
    ax1.legend(lines, [l.get_label() for l in lines], loc="lower right")
    plt.title("V2V no-control: CBR pressure and recognition")
    save("slide_01_cbr_vs_orr.png")

    plt.figure(figsize=(10, 5))
    plt.plot(t, series(data, "high_pdr"), label="High-priority PDR", linewidth=2)
    plt.plot(t, series(data, "low_pdr"), label="Low-priority PDR", linewidth=2)
    plt.plot(t, series(data, "orr"), label="ORR", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Rate [%]")
    plt.ylim(0, 100)
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.title("V2V no-control: recognition reliability")
    save("slide_02_orr_high_low_pdr.png")

    plt.figure(figsize=(10, 5))
    plt.plot(t, series(data, "high_packet_loss"), label="High-priority packet loss", linewidth=2)
    plt.plot(t, series(data, "low_packet_loss"), label="Low-priority packet loss", linewidth=2)
    plt.plot(t, series(data, "never_received_rate"), label="Never received", linewidth=2)
    plt.plot(t, series(data, "ttl_violation_rate"), label="TTL violation", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Rate [%]")
    plt.ylim(0, 100)
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.title("V2V no-control: loss causes under high CBR")
    save("slide_03_loss_causes.png")

    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.plot(t, series(data, "nr_tx"), label="NR TX", color="#1f77b4", linewidth=2)
    ax1.plot(t, series(data, "nr_rx"), label="NR RX", color="#17becf", linewidth=2)
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Packets")
    ax1.grid(True, alpha=0.25)
    ax2 = ax1.twinx()
    ax2.plot(t, series(data, "channel_busy_ratio_avg"), label="CBR avg", color="#d62728", linewidth=2)
    ax2.set_ylabel("CBR")
    ax2.set_ylim(0, 1.05)
    lines = ax1.get_lines() + ax2.get_lines()
    ax1.legend(lines, [l.get_label() for l in lines], loc="upper left")
    plt.title("V2V no-control: packet volume and channel load")
    save("slide_04_packets_vs_cbr.png")

    print(f"Wrote slide plots to {OUT}")


if __name__ == "__main__":
    main()
