#!/usr/bin/env bash
set -euo pipefail

CSV=${1:-/tmp/van3twin_results/final_load_sweep/combined_summary.csv}

if [[ ! -s "$CSV" ]]; then
  echo "summary csv not found: $CSV" >&2
  exit 1
fi

awk -F, '
NR == 1 {
  for (i = 1; i <= NF; ++i) {
    col[$i] = i
  }
  printf "%-18s %-30s %8s %8s %8s %8s %10s %10s %10s\n",
         "load", "method", "ORR", "High", "Low", "loss", "MEC_TX", "MEC_RX", "MEC_ms"
  next
}
{
  load = ("load" in col) ? $(col["load"]) : "-"
  method = $(col["method"])
  orr = $(col["object_recognition_rate"])
  high = $(col["high_priority_object_recognition_rate"])
  low = $(col["low_priority_object_recognition_rate"])
  loss = $(col["packet_loss_rate"])
  mec_tx = ("mec_tx" in col) ? $(col["mec_tx"]) : "-"
  mec_rx = ("mec_rx" in col) ? $(col["mec_rx"]) : "-"
  mec_ms = ("mec_latency_ms" in col) ? $(col["mec_latency_ms"]) : "-"
  printf "%-18s %-30s %8.2f %8.2f %8.2f %8.2f %10s %10s %10s\n",
         load, method, orr, high, low, loss, mec_tx, mec_rx, mec_ms
}
' "$CSV"
