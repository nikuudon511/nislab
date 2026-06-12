#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../../.." && pwd)
cd "$ROOT_DIR"

SIM_TIME=${SIM_TIME:-20}
OUT_DIR=${OUT_DIR:-results/mec_recovery_policy_comparison_$(date +%Y%m%d_%H%M%S)}
SUMO_PORT=${SUMO_PORT:-3960}

ORR_RANGE=${ORR_RANGE:-200}
PRIORITY_DISTANCE=${PRIORITY_DISTANCE:-100}
PRIORITY_CLOSING_SPEED=${PRIORITY_CLOSING_SPEED:-3}
PRIORITY_TTC=${PRIORITY_TTC:-5}

BG_NODES=${BG_NODES:-8}
BG_SIZE=${BG_SIZE:-1500}
BG_INTERVAL_MS=${BG_INTERVAL_MS:-15}
BG_PER_VEHICLE=${BG_PER_VEHICLE:-false}

SWITCH_CBR=${SWITCH_CBR:-0.6}
HYBRID_CBR_MAX=${HYBRID_CBR_MAX:-0.8}
MEC_FORWARD_RANGE=${MEC_FORWARD_RANGE:-220}
MEC_SRS_PERIODICITY=${MEC_SRS_PERIODICITY:-320}

mkdir -p "$OUT_DIR"

RESULT="$OUT_DIR/policy_comparison.csv"
echo "method,policy,object_recognition_rate,high_priority_object_recognition_rate,low_priority_object_recognition_rate,packet_loss_rate,dsrc_tx,mec_tx,mec_tx_per_dsrc,mec_uplink_bytes,mec_forwarded_bytes,mec_uplink_mbps,mec_total_mbps,object_gain_vs_v2v,low_gain_vs_v2v,recovery_efficiency,run_dir" > "$RESULT"

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

run_one() {
  local label=$1
  local port=$2
  local methods=$3
  local policy=$4

  env \
    SIM_TIME="$SIM_TIME" \
    METHODS="$methods" \
    OUT_DIR="$OUT_DIR/$label" \
    SUMO_PORT="$port" \
    ORR_RANGE="$ORR_RANGE" \
    PRIORITY_DISTANCE="$PRIORITY_DISTANCE" \
    PRIORITY_CLOSING_SPEED="$PRIORITY_CLOSING_SPEED" \
    PRIORITY_TTC="$PRIORITY_TTC" \
    BG_NODES="$BG_NODES" \
    BG_SIZE="$BG_SIZE" \
    BG_INTERVAL_MS="$BG_INTERVAL_MS" \
    BG_PER_VEHICLE="$BG_PER_VEHICLE" \
    SWITCH_CBR="$SWITCH_CBR" \
    HYBRID_CBR_MAX="$HYBRID_CBR_MAX" \
    MEC_FORWARD_RANGE="$MEC_FORWARD_RANGE" \
    MEC_SRS_PERIODICITY="$MEC_SRS_PERIODICITY" \
    MEC_RECOVERY_POLICY="$policy" \
    bash "$SCRIPT_DIR/run_hybrid_step5_evaluation.sh"
}

append_row() {
  local method=$1
  local policy=$2
  local run_dir=$3
  local ref_object=${4:-}
  local ref_low=${5:-}

  local summary="$run_dir/summary.csv"
  local observation="$run_dir/$method/observation.csv"
  local object high low loss dsrc_tx mec_tx ratio uplink forwarded uplink_mbps total_mbps object_gain low_gain efficiency
  object=$(csv_value "$summary" "object_recognition_rate")
  high=$(csv_value "$summary" "high_priority_object_recognition_rate")
  low=$(csv_value "$summary" "low_priority_object_recognition_rate")
  loss=$(csv_value "$summary" "packet_loss_rate")
  dsrc_tx=$(csv_value "$summary" "dsrc_tx")
  mec_tx=$(csv_value "$summary" "mec_tx")
  ratio=$(awk -v m="$mec_tx" -v d="$dsrc_tx" 'BEGIN {if (d > 0) print m / d; else print "N/A"}')
  uplink=$(last_observation_value "$observation" "mec_uplink_bytes")
  forwarded=$(last_observation_value "$observation" "mec_forwarded_bytes")
  uplink_mbps=$(awk -v b="$uplink" -v t="$SIM_TIME" 'BEGIN {if (t > 0) print b * 8 / t / 1000000; else print "N/A"}')
  total_mbps=$(awk -v u="$uplink" -v f="$forwarded" -v t="$SIM_TIME" 'BEGIN {if (t > 0) print (u + f) * 8 / t / 1000000; else print "N/A"}')

  if [[ -n "$ref_object" ]]; then
    object_gain=$(awk -v o="$object" -v r="$ref_object" 'BEGIN {print o - r}')
    low_gain=$(awk -v l="$low" -v r="$ref_low" 'BEGIN {print l - r}')
  else
    object_gain="N/A"
    low_gain="N/A"
  fi
  if [[ "$ratio" != "N/A" && "$object_gain" != "N/A" ]]; then
    efficiency=$(awk -v g="$object_gain" -v r="$ratio" 'BEGIN {if (r > 0) print g / r; else print "N/A"}')
  else
    efficiency="N/A"
  fi

  echo "$method,$policy,$object,$high,$low,$loss,$dsrc_tx,$mec_tx,$ratio,$uplink,$forwarded,$uplink_mbps,$total_mbps,$object_gain,$low_gain,$efficiency,$run_dir" >> "$RESULT"
}

run_one "baseline_no_control" "$SUMO_PORT" "no-control" "staged"
append_row "no-control" "none" "$OUT_DIR/baseline_no_control"

run_one "baseline_reactive_rmr" "$((SUMO_PORT + 1))" "reactive-rmr" "staged"
append_row "reactive-rmr" "none" "$OUT_DIR/baseline_reactive_rmr"

run_one "baseline_predictive_v2v" "$((SUMO_PORT + 2))" "predictive-rmr-v2v" "staged"
append_row "predictive-rmr-v2v" "none" "$OUT_DIR/baseline_predictive_v2v"
REF_OBJECT=$(csv_value "$OUT_DIR/baseline_predictive_v2v/summary.csv" "object_recognition_rate")
REF_LOW=$(csv_value "$OUT_DIR/baseline_predictive_v2v/summary.csv" "low_priority_object_recognition_rate")

index=0
for policy in offload-only staged duplicate-always; do
  run_one "v2n2v_${policy}" "$((SUMO_PORT + 10 + index))" "predictive-rmr-v2n2v" "$policy"
  append_row "predictive-rmr-v2n2v" "$policy" "$OUT_DIR/v2n2v_${policy}" "$REF_OBJECT" "$REF_LOW"
  index=$((index + 1))
done

echo "Wrote $RESULT"
