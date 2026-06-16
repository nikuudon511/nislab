#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$ROOT_DIR"

SIM_TIME=${SIM_TIME:-120}
START_PORT=${START_PORT:-8101}
OUT_ROOT=${OUT_ROOT:-/tmp/van3twin_results/final_load_sweep}
METHODS=${METHODS:-"no-control predictive-rmr-v2v v2n2v-all-object v2n2v-high-priority-only v2n2v-adaptive-probability"}

COMMON_ENV=(
  SIM_TIME="$SIM_TIME"
  METHODS="$METHODS"
  MAX_COMMUNICATION_VEHICLES="${MAX_COMMUNICATION_VEHICLES:-100}"
  SENSOR_RANGE="${SENSOR_RANGE:-30}"
  PRIORITY_DISTANCE="${PRIORITY_DISTANCE:-100}"
  ORR_RANGE="${ORR_RANGE:-200}"
  HIGH_PRIORITY_CPM_RECOGNITION_TTL="${HIGH_PRIORITY_CPM_RECOGNITION_TTL:-0.2}"
  LOW_PRIORITY_CPM_RECOGNITION_TTL="${LOW_PRIORITY_CPM_RECOGNITION_TTL:-0.5}"
  SWITCH_CBR="${SWITCH_CBR:-0.6}"
  HYBRID_CBR_MAX="${HYBRID_CBR_MAX:-0.8}"
  RELEASE_CBR="${RELEASE_CBR:-0.5}"
  MEC_FORWARD_RANGE="${MEC_FORWARD_RANGE:-200}"
  MEC_IDEAL_LINK="${MEC_IDEAL_LINK:-true}"
  MEC_IDEAL_LATENCY_MS="${MEC_IDEAL_LATENCY_MS:-75}"
  MEC_IDEAL_LATENCY_STDDEV_MS="${MEC_IDEAL_LATENCY_STDDEV_MS:-25}"
  MEC_IDEAL_LATENCY_MIN_MS="${MEC_IDEAL_LATENCY_MIN_MS:-30}"
  MEC_IDEAL_LATENCY_MAX_MS="${MEC_IDEAL_LATENCY_MAX_MS:-150}"
  MEC_IDEAL_CPM_INTERVAL_MS="${MEC_IDEAL_CPM_INTERVAL_MS:-100}"
  MEC_IDEAL_DL_PDR="${MEC_IDEAL_DL_PDR:-0.97}"
  MEC_ADAPTIVE_LOW_MAX_PROB="${MEC_ADAPTIVE_LOW_MAX_PROB:-0.5}"
  THESIS_EVAL_INTERVAL="${THESIS_EVAL_INTERVAL:-2}"
  OBSERVATION_LOG_INTERVAL="${OBSERVATION_LOG_INTERVAL:-2}"
)

mkdir -p "$OUT_ROOT/nohup_logs"

run_load () {
  local label=$1
  local port=$2
  shift 2
  (
    env "${COMMON_ENV[@]}" "$@" \
      OUT_DIR="$OUT_ROOT/$label" \
      SUMO_PORT="$port" \
      bash src/automotive/examples/run_hybrid_nr_v2n2v_evaluation.sh
  ) > "$OUT_ROOT/nohup_logs/$label.log" 2>&1 &
}

run_load low_nobg "$START_PORT" \
  NR_BG=false

run_load mid_bg300_100 "$((START_PORT + 10))" \
  NR_BG=true NR_BG_PER_VEHICLE=true NR_BG_SIZE=300 NR_BG_INTERVAL_MS=100

run_load high_bg500_80 "$((START_PORT + 20))" \
  NR_BG=true NR_BG_PER_VEHICLE=true NR_BG_SIZE=500 NR_BG_INTERVAL_MS=80

run_load over_bg800_80 "$((START_PORT + 30))" \
  NR_BG=true NR_BG_PER_VEHICLE=true NR_BG_SIZE=800 NR_BG_INTERVAL_MS=80

wait

COMBINED="$OUT_ROOT/combined_summary.csv"
: > "$COMBINED"
header_written=false
for label in low_nobg mid_bg300_100 high_bg500_80 over_bg800_80; do
  summary="$OUT_ROOT/$label/summary.csv"
  if [[ ! -s "$summary" ]]; then
    echo "missing summary: $summary" >&2
    continue
  fi
  if [[ "$header_written" == false ]]; then
    awk -F, 'NR == 1 {print "load," $0}' "$summary" > "$COMBINED"
    header_written=true
  fi
  awk -F, -v label="$label" 'NR > 1 {print label "," $0}' "$summary" >> "$COMBINED"
done

echo "Wrote $COMBINED"
