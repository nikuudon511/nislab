#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../../.." && pwd)
cd "$ROOT_DIR"

SIM_TIME=${SIM_TIME:-20}
OUT_DIR=${OUT_DIR:-results/target_recognition_v2n_budget_$(date +%Y%m%d_%H%M%S)}
SUMO_PORT=${SUMO_PORT:-3820}

TARGET_OBJECT_RECOGNITION=${TARGET_OBJECT_RECOGNITION:-80}
TARGET_HIGH_RECOGNITION=${TARGET_HIGH_RECOGNITION:-90}

ORR_RANGE=${ORR_RANGE:-200}
PRIORITY_DISTANCE=${PRIORITY_DISTANCE:-100}
PRIORITY_CLOSING_SPEED=${PRIORITY_CLOSING_SPEED:-3}
PRIORITY_TTC=${PRIORITY_TTC:-5}

BG_NODES=${BG_NODES:-8}
BG_SIZE=${BG_SIZE:-1500}
BG_INTERVAL_MS=${BG_INTERVAL_MS:-15}

MEC_FORWARD_RANGE=${MEC_FORWARD_RANGE:-200}
MEC_BACKHAUL_DELAY_MS=${MEC_BACKHAUL_DELAY_MS:-10}
MEC_PROCESSING_DELAY_MS=${MEC_PROCESSING_DELAY_MS:-0}
MEC_SRS_PERIODICITY=${MEC_SRS_PERIODICITY:-320}

mkdir -p "$OUT_DIR"

SUMMARY="$OUT_DIR/target_summary.csv"
echo "method,scenario,target_object,target_high,object_recognition_rate,high_priority_object_recognition_rate,low_priority_object_recognition_rate,target_met,packet_loss_rate,dsrc_tx,mec_tx,mec_tx_per_dsrc,mec_uplink_bytes,mec_forwarded_bytes,recovery_efficiency,run_dir" > "$SUMMARY"

csv_value() {
  local file=$1
  local column=$2
  awk -F, -v col="$column" 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} NR==2 {print $h[col]}' "$file"
}

last_observation_value() {
  local file=$1
  local column=$2
  awk -F, -v col="$column" 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} h[col] > 0 {value=$h[col]} END {print value+0}' "$file"
}

ratio_or_na() {
  local numerator=$1
  local denominator=$2
  awk -v n="$numerator" -v d="$denominator" 'BEGIN {if (d > 0) print n / d; else print "N/A"}'
}

target_met() {
  local object=$1
  local high=$2
  awk -v o="$object" -v h="$high" -v to="$TARGET_OBJECT_RECOGNITION" -v th="$TARGET_HIGH_RECOGNITION" \
    'BEGIN {print (o >= to && h >= th) ? "yes" : "no"}'
}

append_result() {
  local method=$1
  local scenario=$2
  local run_dir=$3
  local reference_v2v_object=${4:-}

  local method_summary="$run_dir/summary.csv"
  local method_dir="$run_dir/$method"
  local observation="$method_dir/observation.csv"

  local object high low loss dsrc_tx mec_tx usage mec_uplink mec_forwarded recovery met
  object=$(csv_value "$method_summary" "object_recognition_rate")
  high=$(csv_value "$method_summary" "high_priority_object_recognition_rate")
  low=$(csv_value "$method_summary" "low_priority_object_recognition_rate")
  loss=$(csv_value "$method_summary" "packet_loss_rate")
  dsrc_tx=$(csv_value "$method_summary" "dsrc_tx")
  mec_tx=$(csv_value "$method_summary" "mec_tx")
  usage=$(ratio_or_na "$mec_tx" "$dsrc_tx")
  mec_uplink=$(last_observation_value "$observation" "mec_uplink_bytes")
  mec_forwarded=$(last_observation_value "$observation" "mec_forwarded_bytes")
  met=$(target_met "$object" "$high")

  if [[ -n "$reference_v2v_object" && "$usage" != "N/A" ]]; then
    recovery=$(awk -v o="$object" -v r="$reference_v2v_object" -v u="$usage" 'BEGIN {if (u > 0) print (o - r) / u; else print "N/A"}')
  else
    recovery="N/A"
  fi

  echo "$method,$scenario,$TARGET_OBJECT_RECOGNITION,$TARGET_HIGH_RECOGNITION,$object,$high,$low,$met,$loss,$dsrc_tx,$mec_tx,$usage,$mec_uplink,$mec_forwarded,$recovery,$run_dir" >> "$SUMMARY"
}

run_methods() {
  local label=$1
  local port=$2
  local methods=$3
  shift 3

  local run_dir="$OUT_DIR/$label"
  env \
    SIM_TIME="$SIM_TIME" \
    ORR_RANGE="$ORR_RANGE" \
    PRIORITY_DISTANCE="$PRIORITY_DISTANCE" \
    PRIORITY_CLOSING_SPEED="$PRIORITY_CLOSING_SPEED" \
    PRIORITY_TTC="$PRIORITY_TTC" \
    BG_NODES="$BG_NODES" \
    BG_SIZE="$BG_SIZE" \
    BG_INTERVAL_MS="$BG_INTERVAL_MS" \
    MEC_FORWARD_RANGE="$MEC_FORWARD_RANGE" \
    MEC_BACKHAUL_DELAY_MS="$MEC_BACKHAUL_DELAY_MS" \
    MEC_PROCESSING_DELAY_MS="$MEC_PROCESSING_DELAY_MS" \
    MEC_SRS_PERIODICITY="$MEC_SRS_PERIODICITY" \
    METHODS="$methods" \
    OUT_DIR="$run_dir" \
    SUMO_PORT="$port" \
    "$@" \
    "$SCRIPT_DIR/run_hybrid_step5_evaluation.sh"
}

run_methods "baseline_no_control" "$SUMO_PORT" "no-control"
append_result "no-control" "baseline" "$OUT_DIR/baseline_no_control"

run_methods "baseline_reactive_rmr" "$((SUMO_PORT + 1))" "reactive-rmr"
append_result "reactive-rmr" "baseline" "$OUT_DIR/baseline_reactive_rmr"

run_methods "baseline_predictive_v2v" "$((SUMO_PORT + 2))" "predictive-rmr-v2v"
append_result "predictive-rmr-v2v" "baseline" "$OUT_DIR/baseline_predictive_v2v"
V2V_OBJECT=$(csv_value "$OUT_DIR/baseline_predictive_v2v/summary.csv" "object_recognition_rate")

V2N_CANDIDATES=${V2N_CANDIDATES:-"offload_only:0.60:0.80:false:0.2 duplicate_late:0.75:0.90:true:0.4 duplicate_mid:0.67:0.85:true:0.4 duplicate_early:0.60:0.80:true:0.2"}

candidate_index=0
for candidate in $V2N_CANDIDATES; do
  IFS=: read -r label switch_cbr max_cbr duplicate_recovery low_v2v_min <<< "$candidate"
  run_methods "v2n2v_${label}" "$((SUMO_PORT + 10 + candidate_index))" "predictive-rmr-v2n2v" \
    SWITCH_CBR="$switch_cbr" \
    HYBRID_CBR_MAX="$max_cbr" \
    MEC_DUPLICATE_RECOVERY="$duplicate_recovery" \
    HYBRID_LOW_V2V_MIN_PROB="$low_v2v_min"
  append_result "predictive-rmr-v2n2v" "$label" "$OUT_DIR/v2n2v_${label}" "$V2V_OBJECT"
  candidate_index=$((candidate_index + 1))
done

awk -F, '
NR == 1 {next}
$8 == "yes" && $1 == "predictive-rmr-v2n2v" {
  if (!found || $13 + 0 < bestBytes) {
    found = 1
    bestBytes = $13 + 0
    best = $0
  }
}
END {
  if (found) {
    print "selected_min_v2n," best
  } else {
    print "selected_min_v2n,N/A"
  }
}' "$SUMMARY" > "$OUT_DIR/selected_min_v2n.csv"

echo "Wrote $SUMMARY"
echo "Wrote $OUT_DIR/selected_min_v2n.csv"
