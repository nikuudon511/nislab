#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$ROOT_DIR"

pow() {
  awk -v x="$1" -v e="$2" 'BEGIN { printf "%.6f", exp(e * log(x)) }'
}

round_int() {
  awk -v x="$1" 'BEGIN { if (x < 1) x = 1; printf "%d", x + 0.5 }'
}

scale_value() {
  awk -v base="$1" -v scale="$2" -v expn="$3" \
    'BEGIN { printf "%.6f", base * exp(expn * log(scale)) }'
}

scale_int() {
  round_int "$(scale_value "$1" "$2" "$3")"
}

SCALE=${SCALE:-0.25}
SIM_TIME=${SIM_TIME:-30}
BASE_VEHICLES=${BASE_VEHICLES:-200}
VEHICLES=${VEHICLES:-$(scale_int "$BASE_VEHICLES" "$SCALE" 1)}
RUN_TAG=${RUN_TAG:-scaled_trend_s${SCALE}_n${VEHICLES}_${SIM_TIME}s_$(date +%Y%m%d_%H%M%S)}
SUMO_PORT_BASE=${SUMO_PORT_BASE:-20340}

BASE_MEC_BANDWIDTH_MHZ=${BASE_MEC_BANDWIDTH_MHZ:-20}
BASE_NR_CHANNEL_RATE_MBPS=${BASE_NR_CHANNEL_RATE_MBPS:-6}
BASE_BG_SIZE=${BASE_BG_SIZE:-500}
BASE_RMR_DELETE_LOW=${BASE_RMR_DELETE_LOW:-10}
BASE_RMR_DELETE_MIDDLE=${BASE_RMR_DELETE_MIDDLE:-20}
BASE_RMR_DELETE_HIGH=${BASE_RMR_DELETE_HIGH:-40}

MEC_CAPACITY_EXPONENT=${MEC_CAPACITY_EXPONENT:-2}
NR_CAPACITY_EXPONENT=${NR_CAPACITY_EXPONENT:-1}
MEC_BG_SIZE_EXPONENT=${MEC_BG_SIZE_EXPONENT:-1}
NR_BG_SIZE_EXPONENT=${NR_BG_SIZE_EXPONENT:-0}
RMR_DELETE_EXPONENT=${RMR_DELETE_EXPONENT:-1}

MEC_BANDWIDTH_MHZ=${MEC_BANDWIDTH_MHZ:-$(scale_value "$BASE_MEC_BANDWIDTH_MHZ" "$SCALE" "$MEC_CAPACITY_EXPONENT")}
NR_CHANNEL_RATE_MBPS=${NR_CHANNEL_RATE_MBPS:-$(scale_value "$BASE_NR_CHANNEL_RATE_MBPS" "$SCALE" "$NR_CAPACITY_EXPONENT")}
MEC_BG_SIZE_SCALED=${MEC_BG_SIZE:-$(scale_int "$BASE_BG_SIZE" "$SCALE" "$MEC_BG_SIZE_EXPONENT")}
NR_BG_SIZE_SCALED=${NR_BG_SIZE:-$(scale_int "$BASE_BG_SIZE" "$SCALE" "$NR_BG_SIZE_EXPONENT")}
RMR_DELETE_LOW_SCALED=${RMR_DELETE_LOW:-$(scale_int "$BASE_RMR_DELETE_LOW" "$SCALE" "$RMR_DELETE_EXPONENT")}
RMR_DELETE_MIDDLE_SCALED=${RMR_DELETE_MIDDLE:-$(scale_int "$BASE_RMR_DELETE_MIDDLE" "$SCALE" "$RMR_DELETE_EXPONENT")}
RMR_DELETE_HIGH_SCALED=${RMR_DELETE_HIGH:-$(scale_int "$BASE_RMR_DELETE_HIGH" "$SCALE" "$RMR_DELETE_EXPONENT")}

if [[ -z "${MOB_TRACE:-}" || -z "${SUMO_CONFIG:-}" ]]; then
  case "$VEHICLES" in
    50)
      MOB_TRACE=${MOB_TRACE:-cars_50_2clusters_nostop_debug.rou.xml}
      SUMO_CONFIG=${SUMO_CONFIG:-src/automotive/examples/sumo_files_highway_straight/map_50_2clusters_nostop_debug.sumo.cfg}
      ;;
    100)
      MOB_TRACE=${MOB_TRACE:-cars_100_2clusters_nostop_debug.rou.xml}
      SUMO_CONFIG=${SUMO_CONFIG:-src/automotive/examples/sumo_files_highway_straight/map_100_2clusters_nostop_debug.sumo.cfg}
      ;;
    200)
      MOB_TRACE=${MOB_TRACE:-cars_200_2clusters_nostop.rou.xml}
      SUMO_CONFIG=${SUMO_CONFIG:-src/automotive/examples/sumo_files_highway_straight/map_200_2clusters_nostop.sumo.cfg}
      ;;
    *)
      echo "No built-in SUMO trace for VEHICLES=$VEHICLES." >&2
      echo "Set MOB_TRACE and SUMO_CONFIG explicitly, or use SCALE=0.25/0.5/1.0." >&2
      exit 2
      ;;
  esac
fi

common_env() {
  export CCACHE_DISABLE=1
  export SIM_TIME
  export SUMO_SEED=${SUMO_SEED:-30}
  export RNG_RUN=${RNG_RUN:-1}
  export MOB_TRACE
  export SUMO_CONFIG
  export SUMO_SYNC_INTERVAL=${SUMO_SYNC_INTERVAL:-0.1}
  export MAX_COMMUNICATION_VEHICLES=$VEHICLES
  export THESIS_EVAL_START_MIN_VEHICLES=$VEHICLES
  export THESIS_EVAL_WARMUP_SECONDS=${THESIS_EVAL_WARMUP_SECONDS:-1}
  export THESIS_EVAL_INTERVAL=${THESIS_EVAL_INTERVAL:-1}
  export OBSERVATION_LOG_INTERVAL=${OBSERVATION_LOG_INTERVAL:-1}

  export HIGH_PRIORITY_CPM_RECOGNITION_TTL=${HIGH_PRIORITY_CPM_RECOGNITION_TTL:-0.2}
  export LOW_PRIORITY_CPM_RECOGNITION_TTL=${LOW_PRIORITY_CPM_RECOGNITION_TTL:-0.2}

  export TRAFFIC_FLOW_ROAD_LENGTH=${TRAFFIC_FLOW_ROAD_LENGTH:-3000}
  export PREDICTION_HORIZON=${PREDICTION_HORIZON:-20}
  export TRAFFIC_FLOW_CHANNEL_RATE_MBPS=$NR_CHANNEL_RATE_MBPS
  export TRAFFIC_FLOW_RSU_PREDICTOR=${TRAFFIC_FLOW_RSU_PREDICTOR:-false}
  export PREDICTIVE_RMR=${PREDICTIVE_RMR:-false}
  export PREDICTOR_CPM_SIZE_WEIGHT=${PREDICTOR_CPM_SIZE_WEIGHT:-0}
  export PREDICTOR_CPM_TX_RATE_WEIGHT=${PREDICTOR_CPM_TX_RATE_WEIGHT:-0}
  export PREDICTOR_ACTIVE_VEHICLE_WEIGHT=${PREDICTOR_ACTIVE_VEHICLE_WEIGHT:-0}

  export RMR_DELETE_LOW=$RMR_DELETE_LOW_SCALED
  export RMR_DELETE_MIDDLE=$RMR_DELETE_MIDDLE_SCALED
  export RMR_DELETE_HIGH=$RMR_DELETE_HIGH_SCALED
  export RMR_GUARD_IMPORTANT=${RMR_GUARD_IMPORTANT:-false}
  export MEC_IDEAL_RMR_MODE=${MEC_IDEAL_RMR_MODE:-object-payload}
  export MEC_IDEAL_RMR_LONGTAIL=${MEC_IDEAL_RMR_LONGTAIL:-false}

  export RSU_I2V_RANGE=${RSU_I2V_RANGE:-300}
  export SENSOR_RANGE=${SENSOR_RANGE:-30}
  export ORR_RANGE=${ORR_RANGE:-200}
  export PRIORITY_DISTANCE=${PRIORITY_DISTANCE:-100}
  export PRIORITY_CLOSING_SPEED=${PRIORITY_CLOSING_SPEED:-3}
  export PRIORITY_TTC=${PRIORITY_TTC:-5}
  export MEC_FORWARD_RANGE=${MEC_FORWARD_RANGE:-200}

  export MEC_IDEAL_LINK=${MEC_IDEAL_LINK:-true}
  export MEC_IDEAL_AOI_FILTER=${MEC_IDEAL_AOI_FILTER:-true}
  export MEC_IDEAL_AOI_FILTER_THRESHOLD_MS=${MEC_IDEAL_AOI_FILTER_THRESHOLD_MS:-200}
  export MEC_IDEAL_DL_PDR=${MEC_IDEAL_DL_PDR:-1.0}
  export MEC_IDEAL_DELAY_MODEL=${MEC_IDEAL_DELAY_MODEL:-empirical}
  export MEC_IDEAL_LATENCY_MS=${MEC_IDEAL_LATENCY_MS:-39.4}
  export MEC_IDEAL_LATENCY_STDDEV_MS=${MEC_IDEAL_LATENCY_STDDEV_MS:-20}
  export MEC_IDEAL_LATENCY_MIN_MS=${MEC_IDEAL_LATENCY_MIN_MS:-6.1}
  export MEC_IDEAL_LATENCY_P90_MS=${MEC_IDEAL_LATENCY_P90_MS:-59.86}
  export MEC_IDEAL_LATENCY_P99_MS=${MEC_IDEAL_LATENCY_P99_MS:-120.33}
  export MEC_IDEAL_LATENCY_MAX_MS=${MEC_IDEAL_LATENCY_MAX_MS:-201}
  export MEC_IDEAL_CAPACITY_MODEL=${MEC_IDEAL_CAPACITY_MODEL:-true}
  export MEC_IDEAL_BANDWIDTH_MHZ=$MEC_BANDWIDTH_MHZ
  export MEC_IDEAL_SPECTRAL_EFFICIENCY_BPSHZ=${MEC_IDEAL_SPECTRAL_EFFICIENCY_BPSHZ:-4.5234}
  export MEC_IDEAL_MAX_QUEUE_DELAY_MS=${MEC_IDEAL_MAX_QUEUE_DELAY_MS:-0}
  export MEC_IDEAL_UL_FIRST_LOSS_RATE=${MEC_IDEAL_UL_FIRST_LOSS_RATE:-0.10}
  export MEC_IDEAL_DL_FIRST_LOSS_RATE=${MEC_IDEAL_DL_FIRST_LOSS_RATE:-0.10}
  export MEC_IDEAL_RETX_SUCCESS_PROB=${MEC_IDEAL_RETX_SUCCESS_PROB:-0.53}
  export MEC_IDEAL_MAX_RETX=${MEC_IDEAL_MAX_RETX:-4}
  export MEC_IDEAL_RETX_DELAY_MS=${MEC_IDEAL_RETX_DELAY_MS:-3}

  export MEC_BG=${MEC_BG:-true}
  export MEC_BG_PER_VEHICLE=${MEC_BG_PER_VEHICLE:-true}
  export MEC_BG_SIZE=$MEC_BG_SIZE_SCALED
  export MEC_BG_INTERVAL_MS=${MEC_BG_INTERVAL_MS:-100}
  export NR_BG=${NR_BG:-true}
  export NR_BG_PER_VEHICLE=${NR_BG_PER_VEHICLE:-true}
  export NR_BG_SIZE=$NR_BG_SIZE_SCALED
  export NR_BG_INTERVAL_MS=${NR_BG_INTERVAL_MS:-100}
}

run_case() {
  local label=$1
  local methods=$2
  local reactive_rmr=$3
  local port=$4

  common_env
  export METHODS=$methods
  export REACTIVE_RMR=$reactive_rmr
  export OUT_DIR="results/${RUN_TAG}_${label}"
  export SUMO_PORT=$port

  echo "[START] $label: $OUT_DIR"
  bash src/automotive/examples/run_hybrid_nr_v2n2v_evaluation.sh
  echo "[DONE] $label: $OUT_DIR"
}

cat <<EOF
Scaled trend profile:
- SCALE=$SCALE
- VEHICLES=$VEHICLES
- SIM_TIME=$SIM_TIME
- MOB_TRACE=$MOB_TRACE
- SUMO_CONFIG=$SUMO_CONFIG
- MEC_BANDWIDTH_MHZ=$MEC_BANDWIDTH_MHZ, exponent=$MEC_CAPACITY_EXPONENT
- NR_CHANNEL_RATE_MBPS=$NR_CHANNEL_RATE_MBPS, exponent=$NR_CAPACITY_EXPONENT
- MEC_BG_SIZE=$MEC_BG_SIZE_SCALED, exponent=$MEC_BG_SIZE_EXPONENT
- NR_BG_SIZE=$NR_BG_SIZE_SCALED, exponent=$NR_BG_SIZE_EXPONENT
- RMR_DELETE=$RMR_DELETE_LOW_SCALED/$RMR_DELETE_MIDDLE_SCALED/$RMR_DELETE_HIGH_SCALED, exponent=$RMR_DELETE_EXPONENT
- RUN_TAG=$RUN_TAG
EOF

case "${1:-all}" in
  all)
    run_case "v2v_only" "no-control" "false" "$SUMO_PORT_BASE"
    run_case "v2n2v_only" "v2n2v-only" "false" "$((SUMO_PORT_BASE + 10))"
    run_case "hybrid_rmr" "v2n2v-adaptive-probability" "true" "$((SUMO_PORT_BASE + 20))"
    ;;
  v2v-only)
    run_case "v2v_only" "no-control" "false" "$SUMO_PORT_BASE"
    ;;
  v2n2v-only)
    run_case "v2n2v_only" "v2n2v-only" "false" "$SUMO_PORT_BASE"
    ;;
  hybrid-rmr)
    run_case "hybrid_rmr" "v2n2v-adaptive-probability" "true" "$SUMO_PORT_BASE"
    ;;
  *)
    echo "Usage: $0 [all|v2v-only|v2n2v-only|hybrid-rmr]" >&2
    exit 2
    ;;
esac

echo "Summary files:"
find "results" -maxdepth 2 -path "results/${RUN_TAG}_*/summary.csv" -print
