#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$ROOT_DIR"

SIM_TIME=${SIM_TIME:-120}
METHODS=${METHODS:-"no-control predictive-rmr-v2v v2n2v-all-object v2n2v-high-priority-only v2n2v-adaptive-probability"}
SUMO_FOLDER=${SUMO_FOLDER:-src/automotive/examples/sumo_files_highway_straight/}
MOB_TRACE=${MOB_TRACE:-cars_200_density_wave.rou.xml}
SUMO_CONFIG=${SUMO_CONFIG:-src/automotive/examples/sumo_files_highway_straight/map_200_density_wave.sumo.cfg}
SUMO_PORT=${SUMO_PORT:-3400}
SUMO_SEED=${SUMO_SEED:-10}
RNG_RUN=${RNG_RUN:-1}
OUT_DIR=${OUT_DIR:-/tmp/van3twin_results/hybrid_nr_v2n2v_eval}

SWITCH_CBR=${SWITCH_CBR:-0.6}
RELEASE_CBR=${RELEASE_CBR:-0.5}
HYBRID_CBR_MAX=${HYBRID_CBR_MAX:-0.8}
HYBRID_LOW_V2V_MIN_PROB=${HYBRID_LOW_V2V_MIN_PROB:-0.2}
HYBRID_LOW_V2V_ALPHA=${HYBRID_LOW_V2V_ALPHA:-1}

RMR_CBR_LOW=${RMR_CBR_LOW:-0.33}
RMR_CBR_HIGH=${RMR_CBR_HIGH:-0.67}
PREDICTION_HORIZON=${PREDICTION_HORIZON:-20}
TRAFFIC_FLOW_ROAD_LENGTH=${TRAFFIC_FLOW_ROAD_LENGTH:-3000}
TRAFFIC_FLOW_TOPOLOGY=${TRAFFIC_FLOW_TOPOLOGY:-straight}
TRAFFIC_FLOW_MESSAGE_RATE=${TRAFFIC_FLOW_MESSAGE_RATE:-10}
TRAFFIC_FLOW_AVG_PACKET_SIZE=${TRAFFIC_FLOW_AVG_PACKET_SIZE:-500}
TRAFFIC_FLOW_CHANNEL_RATE_MBPS=${TRAFFIC_FLOW_CHANNEL_RATE_MBPS:-6}
RSU_PASSIVE_PDR_LOW_CBR=${RSU_PASSIVE_PDR_LOW_CBR:-0.95}
RSU_PASSIVE_PDR_MID_CBR=${RSU_PASSIVE_PDR_MID_CBR:-0.90}
RSU_PASSIVE_SATURATION_CBR=${RSU_PASSIVE_SATURATION_CBR:-0.90}
RSU_I2V_RANGE=${RSU_I2V_RANGE:-300}
FIXED_RSU_CBR_MODE=${FIXED_RSU_CBR_MODE:-disabled}
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
MEC_IDEAL_DELAY_MODEL=${MEC_IDEAL_DELAY_MODEL:-empirical}
MEC_IDEAL_LATENCY_MS=${MEC_IDEAL_LATENCY_MS:-39.4}
MEC_IDEAL_LATENCY_STDDEV_MS=${MEC_IDEAL_LATENCY_STDDEV_MS:-20}
MEC_IDEAL_LATENCY_MIN_MS=${MEC_IDEAL_LATENCY_MIN_MS:-6.1}
MEC_IDEAL_LATENCY_P90_MS=${MEC_IDEAL_LATENCY_P90_MS:-59.86}
MEC_IDEAL_LATENCY_P99_MS=${MEC_IDEAL_LATENCY_P99_MS:-120.33}
MEC_IDEAL_LATENCY_MAX_MS=${MEC_IDEAL_LATENCY_MAX_MS:-201}
MEC_IDEAL_CPM_INTERVAL_MS=${MEC_IDEAL_CPM_INTERVAL_MS:-100}
MEC_IDEAL_PACKET_SIZE=${MEC_IDEAL_PACKET_SIZE:-500}
MEC_IDEAL_DL_PDR=${MEC_IDEAL_DL_PDR:-1.0}
MEC_IDEAL_CAPACITY_MODEL=${MEC_IDEAL_CAPACITY_MODEL:-true}
MEC_IDEAL_BANDWIDTH_MHZ=${MEC_IDEAL_BANDWIDTH_MHZ:-20}
MEC_IDEAL_SPECTRAL_EFFICIENCY_BPSHZ=${MEC_IDEAL_SPECTRAL_EFFICIENCY_BPSHZ:-4.5234}
MEC_IDEAL_MAX_QUEUE_DELAY_MS=${MEC_IDEAL_MAX_QUEUE_DELAY_MS:-0}
MEC_IDEAL_UL_FIRST_LOSS_RATE=${MEC_IDEAL_UL_FIRST_LOSS_RATE:-0.10}
MEC_IDEAL_DL_FIRST_LOSS_RATE=${MEC_IDEAL_DL_FIRST_LOSS_RATE:-0.10}
MEC_IDEAL_RETX_SUCCESS_PROB=${MEC_IDEAL_RETX_SUCCESS_PROB:-0.53}
MEC_IDEAL_MAX_RETX=${MEC_IDEAL_MAX_RETX:-4}
MEC_IDEAL_RETX_DELAY_MS=${MEC_IDEAL_RETX_DELAY_MS:-3}
MAX_COMMUNICATION_VEHICLES=${MAX_COMMUNICATION_VEHICLES:-100}

PRIORITY_DISTANCE=${PRIORITY_DISTANCE:-100}
PRIORITY_CLOSING_SPEED=${PRIORITY_CLOSING_SPEED:-3}
PRIORITY_TTC=${PRIORITY_TTC:-5}
SENSOR_RANGE=${SENSOR_RANGE:-30}
ORR_RANGE=${ORR_RANGE:-200}
SENSOR_EXTERNAL_EVENTS=${SENSOR_EXTERNAL_EVENTS:-false}
SENSOR_EXTERNAL_EVENT_PROBABILITY=${SENSOR_EXTERNAL_EVENT_PROBABILITY:-0.20}
SENSOR_EXTERNAL_EVENT_ACTIVATION_RANGE=${SENSOR_EXTERNAL_EVENT_ACTIVATION_RANGE:-400}
HIGH_PRIORITY_CPM_RECOGNITION_TTL=${HIGH_PRIORITY_CPM_RECOGNITION_TTL:-0.2}
LOW_PRIORITY_CPM_RECOGNITION_TTL=${LOW_PRIORITY_CPM_RECOGNITION_TTL:-0.5}
THESIS_EVAL_INTERVAL=${THESIS_EVAL_INTERVAL:-1}
THESIS_EVAL_START_MIN_VEHICLES=${THESIS_EVAL_START_MIN_VEHICLES:-120}
THESIS_EVAL_START_USE_ALL_VEHICLES=${THESIS_EVAL_START_USE_ALL_VEHICLES:-true}
HOLD_TRAFFIC_UNTIL_EVAL_START=${HOLD_TRAFFIC_UNTIL_EVAL_START:-true}
THESIS_EVAL_WARMUP_SECONDS=${THESIS_EVAL_WARMUP_SECONDS:-5}
OBSERVATION_LOG_INTERVAL=${OBSERVATION_LOG_INTERVAL:-1}

NR_BG=${NR_BG:-true}
NR_BG_NODES=${NR_BG_NODES:-6}
NR_BG_PER_VEHICLE=${NR_BG_PER_VEHICLE:-false}
NR_BG_SIZE=${NR_BG_SIZE:-1500}
NR_BG_INTERVAL_MS=${NR_BG_INTERVAL_MS:-100}
NR_BG_TARGET_CBR=${NR_BG_TARGET_CBR:-}
NR_BG_TARGET_ACTIVE_VEHICLES=${NR_BG_TARGET_ACTIVE_VEHICLES:-60}
NR_BG_CBR_BANDWIDTH_MBPS=${NR_BG_CBR_BANDWIDTH_MBPS:-6}
NR_BG_START=${NR_BG_START:-1}
NR_BG_STOP=${NR_BG_STOP:-0}
NR_BG_MOVING_WAVE=${NR_BG_MOVING_WAVE:-false}
NR_BG_FIXED_ZONES=${NR_BG_FIXED_ZONES:-false}
NR_BG_WAVE_PERIOD=${NR_BG_WAVE_PERIOD:-20}
NR_BG_WAVE_AMPLITUDE=${NR_BG_WAVE_AMPLITUDE:-0.8}
NR_BG_WAVE_SPEED=${NR_BG_WAVE_SPEED:-25}
NR_BG_WAVE_MIN_FACTOR=${NR_BG_WAVE_MIN_FACTOR:-0.2}
NR_BG_WAVE_WIDTH=${NR_BG_WAVE_WIDTH:-500}
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
    v2n2v-only)
      run_method="v2n2v-only"
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

  /usr/bin/time -v -o "$RUN_DIR/resource_usage.txt" ./ns3 run "v2v-hybrid-nr-v2n2v \
    --method=$run_method \
    --sumo-gui=${SUMO_GUI:-false} \
    --sim-time=$SIM_TIME \
    --sumo-folder=$SUMO_FOLDER \
    --mob-trace=$MOB_TRACE \
    --sumo-config=$SUMO_CONFIG \
    --sumo-port=$run_port \
    --sumo-seed=$SUMO_SEED \
    --sumo-sync-interval=${SUMO_SYNC_INTERVAL:-0.01} \
    --rng-run=$RNG_RUN \
    --switch-cbr=$SWITCH_CBR \
    --release-cbr=$RELEASE_CBR \
    --hybrid-cbr-max=$HYBRID_CBR_MAX \
    --hybrid-low-v2v-min-prob=$HYBRID_LOW_V2V_MIN_PROB \
    --hybrid-low-v2v-alpha=$HYBRID_LOW_V2V_ALPHA \
    --rmr-cbr-low=$RMR_CBR_LOW \
    --rmr-cbr-high=$RMR_CBR_HIGH \
    --prediction-horizon=$PREDICTION_HORIZON \
    --traffic-flow-road-length=$TRAFFIC_FLOW_ROAD_LENGTH \
    --traffic-flow-topology=$TRAFFIC_FLOW_TOPOLOGY \
    --traffic-flow-message-rate=$TRAFFIC_FLOW_MESSAGE_RATE \
    --traffic-flow-avg-packet-size=$TRAFFIC_FLOW_AVG_PACKET_SIZE \
    --traffic-flow-channel-rate-mbps=$TRAFFIC_FLOW_CHANNEL_RATE_MBPS \
    --rsu-passive-pdr-low-cbr=$RSU_PASSIVE_PDR_LOW_CBR \
    --rsu-passive-pdr-mid-cbr=$RSU_PASSIVE_PDR_MID_CBR \
    --rsu-passive-saturation-cbr=$RSU_PASSIVE_SATURATION_CBR \
    --rsu-i2v-range=$RSU_I2V_RANGE \
    --fixed-rsu-cbr-mode=$FIXED_RSU_CBR_MODE \
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
    --mec-ideal-delay-model=$MEC_IDEAL_DELAY_MODEL \
    --mec-ideal-latency-stddev-ms=$MEC_IDEAL_LATENCY_STDDEV_MS \
    --mec-ideal-latency-min-ms=$MEC_IDEAL_LATENCY_MIN_MS \
    --mec-ideal-latency-p90-ms=$MEC_IDEAL_LATENCY_P90_MS \
    --mec-ideal-latency-p99-ms=$MEC_IDEAL_LATENCY_P99_MS \
    --mec-ideal-latency-max-ms=$MEC_IDEAL_LATENCY_MAX_MS \
    --mec-ideal-cpm-interval-ms=$MEC_IDEAL_CPM_INTERVAL_MS \
    --mec-ideal-packet-size=$MEC_IDEAL_PACKET_SIZE \
    --mec-ideal-dl-pdr=$MEC_IDEAL_DL_PDR \
    --mec-ideal-capacity-model=$MEC_IDEAL_CAPACITY_MODEL \
    --mec-ideal-bandwidth-mhz=$MEC_IDEAL_BANDWIDTH_MHZ \
    --mec-ideal-spectral-efficiency-bpshz=$MEC_IDEAL_SPECTRAL_EFFICIENCY_BPSHZ \
    --mec-ideal-max-queue-delay-ms=$MEC_IDEAL_MAX_QUEUE_DELAY_MS \
    --mec-ideal-ul-first-loss-rate=$MEC_IDEAL_UL_FIRST_LOSS_RATE \
    --mec-ideal-dl-first-loss-rate=$MEC_IDEAL_DL_FIRST_LOSS_RATE \
    --mec-ideal-retx-success-prob=$MEC_IDEAL_RETX_SUCCESS_PROB \
    --mec-ideal-max-retx=$MEC_IDEAL_MAX_RETX \
    --mec-ideal-retx-delay-ms=$MEC_IDEAL_RETX_DELAY_MS \
    --max-communication-vehicles=$MAX_COMMUNICATION_VEHICLES \
    --priority-distance=$PRIORITY_DISTANCE \
    --priority-closing-speed=$PRIORITY_CLOSING_SPEED \
    --priority-ttc=$PRIORITY_TTC \
    --sensor-range=$SENSOR_RANGE \
    --orr-range=$ORR_RANGE \
    --sensor-external-events=$SENSOR_EXTERNAL_EVENTS \
    --sensor-external-event-probability=$SENSOR_EXTERNAL_EVENT_PROBABILITY \
    --sensor-external-event-activation-range=$SENSOR_EXTERNAL_EVENT_ACTIVATION_RANGE \
    --high-priority-cpm-recognition-ttl=$HIGH_PRIORITY_CPM_RECOGNITION_TTL \
    --low-priority-cpm-recognition-ttl=$LOW_PRIORITY_CPM_RECOGNITION_TTL \
    --nr-bg=$NR_BG \
    --nr-bg-nodes=$NR_BG_NODES \
    --nr-bg-per-vehicle=$NR_BG_PER_VEHICLE \
    --nr-bg-size=$NR_BG_SIZE \
    --nr-bg-interval-ms=$NR_BG_INTERVAL_MS \
    --nr-bg-start=$NR_BG_START \
    --nr-bg-stop=$NR_BG_STOP \
    --nr-bg-moving-wave=$NR_BG_MOVING_WAVE \
    --nr-bg-fixed-zones=$NR_BG_FIXED_ZONES \
    --nr-bg-wave-period=$NR_BG_WAVE_PERIOD \
    --nr-bg-wave-amplitude=$NR_BG_WAVE_AMPLITUDE \
    --nr-bg-wave-speed=$NR_BG_WAVE_SPEED \
    --nr-bg-wave-min-factor=$NR_BG_WAVE_MIN_FACTOR \
    --nr-bg-wave-width=$NR_BG_WAVE_WIDTH \
    --nr-mcs=$NR_MCS \
    --nr-sensing=$NR_SENSING \
    --nr-channel-randomness=$NR_CHANNEL_RANDOMNESS \
    --thesis-eval-interval=$THESIS_EVAL_INTERVAL \
    --thesis-eval-start-min-vehicles=$THESIS_EVAL_START_MIN_VEHICLES \
    --thesis-eval-start-use-all-vehicles=$THESIS_EVAL_START_USE_ALL_VEHICLES \
    --hold-traffic-until-eval-start=$HOLD_TRAFFIC_UNTIL_EVAL_START \
    --thesis-eval-warmup-seconds=$THESIS_EVAL_WARMUP_SECONDS \
    --observation-log-interval=$OBSERVATION_LOG_INTERVAL \
    --cbr-log=$RUN_DIR/channel.csv \
    --route-log=$RUN_DIR/route.csv \
    --observation-log=$RUN_DIR/observation.csv \
    --cpm-input-diag-log=$RUN_DIR/cpm_input_diag.csv \
    --rsu-prediction-log=$RUN_DIR/rsu_prediction.csv \
    --summary-csv=$RUN_DIR/summary.csv" > "$RUN_DIR/stdout.txt" 2>&1

  if [[ "$header_written" == false ]]; then
    head -n 1 "$RUN_DIR/summary.csv" > "$SUMMARY"
    header_written=true
  fi
  awk -F, -v OFS=, -v label="$method" 'NR > 1 {$1 = label; print}' "$RUN_DIR/summary.csv" >> "$SUMMARY"
  run_port=$((run_port + 1))
done

echo "Wrote $SUMMARY"
