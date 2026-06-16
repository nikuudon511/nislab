#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$ROOT_DIR"

SIM_TIME=${SIM_TIME:-120}
METHODS=${METHODS:-"no-control predictive-rmr-v2v v2n2v-all-object v2n2v-high-priority-only v2n2v-adaptive-probability"}
SUMO_FOLDER=${SUMO_FOLDER:-src/automotive/examples/sumo_files_nr_loop_1km_bidirectional/}
MOB_TRACE=${MOB_TRACE:-cars_300_loop_bidirectional.rou.xml}
SUMO_CONFIG=${SUMO_CONFIG:-src/automotive/examples/sumo_files_nr_loop_1km_bidirectional/map_loop_1km_bidirectional.sumo.cfg}
SUMO_PORT=${SUMO_PORT:-3400}
OUT_DIR=${OUT_DIR:-/tmp/van3twin_results/hybrid_nr_v2n2v_eval}

SWITCH_CBR=${SWITCH_CBR:-0.6}
RELEASE_CBR=${RELEASE_CBR:-0.5}
HYBRID_CBR_MAX=${HYBRID_CBR_MAX:-0.8}
HYBRID_LOW_V2V_MIN_PROB=${HYBRID_LOW_V2V_MIN_PROB:-0.2}
HYBRID_LOW_V2V_ALPHA=${HYBRID_LOW_V2V_ALPHA:-1}

RMR_CBR_LOW=${RMR_CBR_LOW:-0.33}
RMR_CBR_HIGH=${RMR_CBR_HIGH:-0.67}
PREDICTION_HORIZON=${PREDICTION_HORIZON:-2}
TRAFFIC_FLOW_ROAD_LENGTH=${TRAFFIC_FLOW_ROAD_LENGTH:-1000}
TRAFFIC_FLOW_MESSAGE_RATE=${TRAFFIC_FLOW_MESSAGE_RATE:-10}
TRAFFIC_FLOW_AVG_PACKET_SIZE=${TRAFFIC_FLOW_AVG_PACKET_SIZE:-500}
PREDICTOR_CPM_SIZE_WEIGHT=${PREDICTOR_CPM_SIZE_WEIGHT:-0}
PREDICTOR_CPM_TX_RATE_WEIGHT=${PREDICTOR_CPM_TX_RATE_WEIGHT:-0}
PREDICTOR_ACTIVE_VEHICLE_WEIGHT=${PREDICTOR_ACTIVE_VEHICLE_WEIGHT:-0}

MEC_BACKHAUL_DELAY_MS=${MEC_BACKHAUL_DELAY_MS:-10}
MEC_PROCESSING_DELAY_MS=${MEC_PROCESSING_DELAY_MS:-0}
MEC_FORWARD_RANGE=${MEC_FORWARD_RANGE:-200}
MEC_RECOVERY_POLICY=${MEC_RECOVERY_POLICY:-staged}
MEC_OBJECT_POLICY=${MEC_OBJECT_POLICY:-all-objects}
MEC_ADAPTIVE_LOW_MAX_PROB=${MEC_ADAPTIVE_LOW_MAX_PROB:-0.5}
MEC_MIN_HOLD_TIME=${MEC_MIN_HOLD_TIME:-5}
MEC_IDEAL_LINK=${MEC_IDEAL_LINK:-true}
MEC_IDEAL_LATENCY_MS=${MEC_IDEAL_LATENCY_MS:-50}
MEC_IDEAL_LATENCY_STDDEV_MS=${MEC_IDEAL_LATENCY_STDDEV_MS:-20}
MEC_IDEAL_LATENCY_MIN_MS=${MEC_IDEAL_LATENCY_MIN_MS:-20}
MEC_IDEAL_LATENCY_MAX_MS=${MEC_IDEAL_LATENCY_MAX_MS:-150}
MEC_IDEAL_CPM_INTERVAL_MS=${MEC_IDEAL_CPM_INTERVAL_MS:-100}
MEC_IDEAL_PACKET_SIZE=${MEC_IDEAL_PACKET_SIZE:-500}
MEC_IDEAL_DL_PDR=${MEC_IDEAL_DL_PDR:-1.0}
MAX_COMMUNICATION_VEHICLES=${MAX_COMMUNICATION_VEHICLES:-100}

PRIORITY_DISTANCE=${PRIORITY_DISTANCE:-100}
PRIORITY_CLOSING_SPEED=${PRIORITY_CLOSING_SPEED:-3}
PRIORITY_TTC=${PRIORITY_TTC:-5}
SENSOR_RANGE=${SENSOR_RANGE:-30}
ORR_RANGE=${ORR_RANGE:-200}
HIGH_PRIORITY_CPM_RECOGNITION_TTL=${HIGH_PRIORITY_CPM_RECOGNITION_TTL:-0.2}
LOW_PRIORITY_CPM_RECOGNITION_TTL=${LOW_PRIORITY_CPM_RECOGNITION_TTL:-0.5}
THESIS_EVAL_INTERVAL=${THESIS_EVAL_INTERVAL:-1}
OBSERVATION_LOG_INTERVAL=${OBSERVATION_LOG_INTERVAL:-1}

NR_BG=${NR_BG:-true}
NR_BG_NODES=${NR_BG_NODES:-6}
NR_BG_PER_VEHICLE=${NR_BG_PER_VEHICLE:-false}
NR_BG_SIZE=${NR_BG_SIZE:-1500}
NR_BG_INTERVAL_MS=${NR_BG_INTERVAL_MS:-20}
NR_BG_TARGET_CBR=${NR_BG_TARGET_CBR:-}
NR_BG_TARGET_ACTIVE_VEHICLES=${NR_BG_TARGET_ACTIVE_VEHICLES:-60}
NR_BG_CBR_BANDWIDTH_MBPS=${NR_BG_CBR_BANDWIDTH_MBPS:-6}
NR_BG_START=${NR_BG_START:-1}
NR_BG_STOP=${NR_BG_STOP:-0}
NR_MCS=${NR_MCS:-14}
NR_SENSING=${NR_SENSING:-false}
NR_CHANNEL_RANDOMNESS=${NR_CHANNEL_RANDOMNESS:-false}

if [[ -n "$NR_BG_TARGET_CBR" ]]; then
  NR_BG_SIZE=$(awk -v target="$NR_BG_TARGET_CBR" \
                    -v mbps="$NR_BG_CBR_BANDWIDTH_MBPS" \
                    -v interval_ms="$NR_BG_INTERVAL_MS" \
                    -v vehicles="$NR_BG_TARGET_ACTIVE_VEHICLES" \
                    'BEGIN { bytes = target * mbps * 1000000 * (interval_ms / 1000.0) / (8.0 * vehicles);
                             if (bytes < 1) bytes = 1;
                             printf("%d", bytes + 0.5); }')
fi

mkdir -p "$OUT_DIR"
SUMMARY="$OUT_DIR/summary.csv"
: > "$SUMMARY"
header_written=false
run_port=$SUMO_PORT

for method in $METHODS; do
  run_method=$method
  run_mec_object_policy=$MEC_OBJECT_POLICY
  case "$method" in
    v2n2v-all-object)
      run_method="predictive-rmr-v2n2v"
      run_mec_object_policy="all-objects"
      ;;
    v2n2v-high-priority-only)
      run_method="predictive-rmr-v2n2v"
      run_mec_object_policy="high-priority-only"
      ;;
    v2n2v-adaptive-probability)
      run_method="predictive-rmr-v2n2v"
      run_mec_object_policy="adaptive-probability"
      ;;
  esac

  RUN_DIR="$OUT_DIR/$method"
  mkdir -p "$RUN_DIR"

  ./ns3 run "v2v-hybrid-nr-v2n2v \
    --method=$run_method \
    --sumo-gui=false \
    --sim-time=$SIM_TIME \
    --sumo-folder=$SUMO_FOLDER \
    --mob-trace=$MOB_TRACE \
    --sumo-config=$SUMO_CONFIG \
    --sumo-port=$run_port \
    --switch-cbr=$SWITCH_CBR \
    --release-cbr=$RELEASE_CBR \
    --hybrid-cbr-max=$HYBRID_CBR_MAX \
    --hybrid-low-v2v-min-prob=$HYBRID_LOW_V2V_MIN_PROB \
    --hybrid-low-v2v-alpha=$HYBRID_LOW_V2V_ALPHA \
    --rmr-cbr-low=$RMR_CBR_LOW \
    --rmr-cbr-high=$RMR_CBR_HIGH \
    --prediction-horizon=$PREDICTION_HORIZON \
    --traffic-flow-road-length=$TRAFFIC_FLOW_ROAD_LENGTH \
    --traffic-flow-message-rate=$TRAFFIC_FLOW_MESSAGE_RATE \
    --traffic-flow-avg-packet-size=$TRAFFIC_FLOW_AVG_PACKET_SIZE \
    --predictor-cpm-size-weight=$PREDICTOR_CPM_SIZE_WEIGHT \
    --predictor-cpm-tx-rate-weight=$PREDICTOR_CPM_TX_RATE_WEIGHT \
    --predictor-active-vehicle-weight=$PREDICTOR_ACTIVE_VEHICLE_WEIGHT \
    --mec-backhaul-delay-ms=$MEC_BACKHAUL_DELAY_MS \
    --mec-processing-delay-ms=$MEC_PROCESSING_DELAY_MS \
    --mec-forward-range=$MEC_FORWARD_RANGE \
    --mec-recovery-policy=$MEC_RECOVERY_POLICY \
    --mec-object-policy=$run_mec_object_policy \
    --mec-adaptive-low-max-prob=$MEC_ADAPTIVE_LOW_MAX_PROB \
    --mec-min-hold-time=$MEC_MIN_HOLD_TIME \
    --mec-ideal-link=$MEC_IDEAL_LINK \
    --mec-ideal-latency-ms=$MEC_IDEAL_LATENCY_MS \
    --mec-ideal-latency-stddev-ms=$MEC_IDEAL_LATENCY_STDDEV_MS \
    --mec-ideal-latency-min-ms=$MEC_IDEAL_LATENCY_MIN_MS \
    --mec-ideal-latency-max-ms=$MEC_IDEAL_LATENCY_MAX_MS \
    --mec-ideal-cpm-interval-ms=$MEC_IDEAL_CPM_INTERVAL_MS \
    --mec-ideal-packet-size=$MEC_IDEAL_PACKET_SIZE \
    --mec-ideal-dl-pdr=$MEC_IDEAL_DL_PDR \
    --max-communication-vehicles=$MAX_COMMUNICATION_VEHICLES \
    --priority-distance=$PRIORITY_DISTANCE \
    --priority-closing-speed=$PRIORITY_CLOSING_SPEED \
    --priority-ttc=$PRIORITY_TTC \
    --sensor-range=$SENSOR_RANGE \
    --orr-range=$ORR_RANGE \
    --high-priority-cpm-recognition-ttl=$HIGH_PRIORITY_CPM_RECOGNITION_TTL \
    --low-priority-cpm-recognition-ttl=$LOW_PRIORITY_CPM_RECOGNITION_TTL \
    --nr-bg=$NR_BG \
    --nr-bg-nodes=$NR_BG_NODES \
    --nr-bg-per-vehicle=$NR_BG_PER_VEHICLE \
    --nr-bg-size=$NR_BG_SIZE \
    --nr-bg-interval-ms=$NR_BG_INTERVAL_MS \
    --nr-bg-start=$NR_BG_START \
    --nr-bg-stop=$NR_BG_STOP \
    --nr-mcs=$NR_MCS \
    --nr-sensing=$NR_SENSING \
    --nr-channel-randomness=$NR_CHANNEL_RANDOMNESS \
    --thesis-eval-interval=$THESIS_EVAL_INTERVAL \
    --observation-log-interval=$OBSERVATION_LOG_INTERVAL \
    --cbr-log=$RUN_DIR/channel.csv \
    --route-log=$RUN_DIR/route.csv \
    --observation-log=$RUN_DIR/observation.csv \
    --summary-csv=$RUN_DIR/summary.csv" > "$RUN_DIR/stdout.txt" 2>&1

  if [[ "$header_written" == false ]]; then
    head -n 1 "$RUN_DIR/summary.csv" > "$SUMMARY"
    header_written=true
  fi
  awk -F, -v OFS=, -v label="$method" 'NR > 1 {$1 = label; print}' "$RUN_DIR/summary.csv" >> "$SUMMARY"
  run_port=$((run_port + 1))
done

echo "Wrote $SUMMARY"
