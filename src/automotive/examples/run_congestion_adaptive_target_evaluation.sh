#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../../.." && pwd)
cd "$ROOT_DIR"

SIM_TIME=${SIM_TIME:-120}
OUT_DIR=${OUT_DIR:-results/congestion_adaptive_targets_$(date +%Y%m%d_%H%M%S)}
SUMO_PORT=${SUMO_PORT:-4200}

ORR_RANGE=${ORR_RANGE:-200}
PRIORITY_DISTANCE=${PRIORITY_DISTANCE:-100}
PRIORITY_CLOSING_SPEED=${PRIORITY_CLOSING_SPEED:-3}
PRIORITY_TTC=${PRIORITY_TTC:-5}
MEC_FORWARD_RANGE=${MEC_FORWARD_RANGE:-220}

mkdir -p "$OUT_DIR"

ADAPTIVE="$OUT_DIR/adaptive_target_summary.csv"
echo "congestion,method,policy,target_object,target_high,v2n_budget_ratio,object_recognition_rate,high_priority_object_recognition_rate,low_priority_object_recognition_rate,mec_tx_per_dsrc,mec_uplink_mbps,mec_total_mbps,target_met,v2n_budget_met,object_gain_vs_v2v,low_gain_vs_v2v,recovery_efficiency,source_run" > "$ADAPTIVE"

run_case() {
  local congestion=$1
  local port=$2
  local bg_size=$3
  local bg_interval=$4
  local target_object=$5
  local target_high=$6
  local v2n_budget=$7

  local case_dir="$OUT_DIR/$congestion"
  env \
    SIM_TIME="$SIM_TIME" \
    BG_PER_VEHICLE=true \
    BG_SIZE="$bg_size" \
    BG_INTERVAL_MS="$bg_interval" \
    ORR_RANGE="$ORR_RANGE" \
    PRIORITY_DISTANCE="$PRIORITY_DISTANCE" \
    PRIORITY_CLOSING_SPEED="$PRIORITY_CLOSING_SPEED" \
    PRIORITY_TTC="$PRIORITY_TTC" \
    MEC_FORWARD_RANGE="$MEC_FORWARD_RANGE" \
    OUT_DIR="$case_dir" \
    SUMO_PORT="$port" \
    bash "$SCRIPT_DIR/run_mec_recovery_policy_comparison.sh"

  awk -F, -v congestion="$congestion" -v to="$target_object" -v th="$target_high" \
    -v budget="$v2n_budget" -v source="$case_dir" '
    NR == 1 {
      for (i = 1; i <= NF; ++i) h[$i] = i
      next
    }
    {
      object = $h["object_recognition_rate"] + 0
      high = $h["high_priority_object_recognition_rate"] + 0
      ratioText = $h["mec_tx_per_dsrc"]
      ratio = ratioText + 0
      targetMet = (object >= to && high >= th) ? "yes" : "no"
      budgetMet = (ratioText == "N/A" || ratio <= budget) ? "yes" : "no"
      print congestion "," $h["method"] "," $h["policy"] "," to "," th "," budget "," \
            $h["object_recognition_rate"] "," $h["high_priority_object_recognition_rate"] "," \
            $h["low_priority_object_recognition_rate"] "," $h["mec_tx_per_dsrc"] "," \
            $h["mec_uplink_mbps"] "," $h["mec_total_mbps"] "," targetMet "," budgetMet "," \
            $h["object_gain_vs_v2v"] "," $h["low_gain_vs_v2v"] "," \
            $h["recovery_efficiency"] "," source
    }' "$case_dir/policy_comparison.csv" >> "$ADAPTIVE"
}

run_case "low" "$SUMO_PORT" 200 100 95 95 0.2
run_case "mid" "$((SUMO_PORT + 20))" 500 50 90 90 0.5
run_case "high" "$((SUMO_PORT + 40))" 1000 25 80 85 1.0

echo "Wrote $ADAPTIVE"
