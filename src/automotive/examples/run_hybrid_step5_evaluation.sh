#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$ROOT_DIR"

SIM_TIME=${SIM_TIME:-120}
METHODS=${METHODS:-"no-control predictive-rmr-v2v predictive-rmr-v2n2v"}
SUMO_FOLDER=${SUMO_FOLDER:-src/automotive/examples/sumo_files_highway_straight/}
MOB_TRACE=${MOB_TRACE:-cars_60_oneway.rou.xml}
SUMO_CONFIG=${SUMO_CONFIG:-src/automotive/examples/sumo_files_highway_straight/map_oneway.sumo.cfg}
SUMO_PORT=${SUMO_PORT:-3400}
OUT_DIR=${OUT_DIR:-results/hybrid_step5_$(date +%Y%m%d_%H%M%S)}

PHY_MODE=${PHY_MODE:-OfdmRate6MbpsBW10MHz}
DCC_BITRATE=${DCC_BITRATE:-6.0}
SWITCH_CBR=${SWITCH_CBR:-0.6}
RELEASE_CBR=${RELEASE_CBR:-0.5}
HYBRID_CBR_MAX=${HYBRID_CBR_MAX:-0.8}
HYBRID_LOW_V2V_MIN_PROB=${HYBRID_LOW_V2V_MIN_PROB:-0.2}
HYBRID_LOW_V2V_ALPHA=${HYBRID_LOW_V2V_ALPHA:-1}
PRIORITY_DISTANCE=${PRIORITY_DISTANCE:-100}
PRIORITY_CLOSING_SPEED=${PRIORITY_CLOSING_SPEED:-3}
PRIORITY_TTC=${PRIORITY_TTC:-5}
SENSOR_RANGE=${SENSOR_RANGE:-100}
ORR_RANGE=${ORR_RANGE:-100}
BG_NODES=${BG_NODES:-6}
BG_SIZE=${BG_SIZE:-1500}
BG_INTERVAL_MS=${BG_INTERVAL_MS:-20}
BG_PER_VEHICLE=${BG_PER_VEHICLE:-false}
RMR_CBR_LOW=${RMR_CBR_LOW:-0.33}
RMR_CBR_HIGH=${RMR_CBR_HIGH:-0.67}
PREDICTION_HORIZON=${PREDICTION_HORIZON:-2}
TRAFFIC_FLOW_ROAD_LENGTH=${TRAFFIC_FLOW_ROAD_LENGTH:-2000}
TRAFFIC_FLOW_MESSAGE_RATE=${TRAFFIC_FLOW_MESSAGE_RATE:-10}
TRAFFIC_FLOW_AVG_PACKET_SIZE=${TRAFFIC_FLOW_AVG_PACKET_SIZE:-500}
PREDICTOR_CPM_SIZE_WEIGHT=${PREDICTOR_CPM_SIZE_WEIGHT:-0}
PREDICTOR_CPM_TX_RATE_WEIGHT=${PREDICTOR_CPM_TX_RATE_WEIGHT:-0}
PREDICTOR_ACTIVE_WEIGHT=${PREDICTOR_ACTIVE_WEIGHT:-0}
MEC_BACKHAUL_DELAY_MS=${MEC_BACKHAUL_DELAY_MS:-10}
MEC_PROCESSING_DELAY_MS=${MEC_PROCESSING_DELAY_MS:-0}
MEC_SRS_PERIODICITY=${MEC_SRS_PERIODICITY:-320}
MEC_DUPLICATE_RECOVERY=${MEC_DUPLICATE_RECOVERY:-false}
MEC_RECOVERY_POLICY=${MEC_RECOVERY_POLICY:-staged}
MEC_MIN_HOLD_TIME=${MEC_MIN_HOLD_TIME:-5}
MEC_FORWARD_RANGE=${MEC_FORWARD_RANGE:-0}

mkdir -p "$OUT_DIR"

SUMMARY="$OUT_DIR/summary.csv"
echo "method,object_recognition_rate,high_priority_object_recognition_rate,low_priority_object_recognition_rate,sensor_object_recognition_rate,sensor_high_priority_object_recognition_rate,sensor_low_priority_object_recognition_rate,cooperative_object_recognition_rate,recognition_rate,packet_loss_rate,ideal_rx,true_rx,orr,high_pdr,low_pdr,high_packet_loss,low_packet_loss,high_ideal_rx,high_true_rx,low_ideal_rx,low_true_rx,dsrc_route_loss,nr_route_loss,mec_route_loss,dsrc_cbr,dsrc_tx,dsrc_rx,mec_tx,mec_rx,mec_latency_ms,traffic_q_in_avg,traffic_q_out_avg,traffic_flow_cbr_avg,traffic_delta_cbr_avg,rsu_count,rsu_local_vehicle_avg,rsu_upstream_vehicle_avg,rsu_future_vehicle_avg,rsu_predicted_cbr_avg,rsu_predicted_cbr_max,interference_tx,interference_drops,interference_bytes,interference_offered_cbr,switch_to_mec,release_to_dsrc,final_dsrc_routes,final_mec_routes,cbr_exceed_seconds" > "$SUMMARY"

extract_after_colon() {
  local label=$1
  local file=$2
  local line
  line=$(grep -F "$label" "$file" | tail -n 1 || true)
  printf '%s' "${line#*: }"
}

extract_txrx() {
  local label=$1
  local file=$2
  extract_after_colon "$label" "$file"
}

for method in $METHODS; do
  RUN_DIR="$OUT_DIR/$method"
  mkdir -p "$RUN_DIR"
  STDOUT="$RUN_DIR/stdout.txt"

  ./ns3 run "v2v-hybrid-cbr-80211p-nrv2x \
    --method=$method \
    --sumo-gui=false \
    --sim-time=$SIM_TIME \
    --sumo-folder=$SUMO_FOLDER \
    --mob-trace=$MOB_TRACE \
    --sumo-config=$SUMO_CONFIG \
    --sumo-port=$SUMO_PORT \
    --phyMode=$PHY_MODE \
    --dsrc-dcc-bitrate-mbps=$DCC_BITRATE \
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
    --predictor-active-vehicle-weight=$PREDICTOR_ACTIVE_WEIGHT \
    --mec-backhaul-delay-ms=$MEC_BACKHAUL_DELAY_MS \
    --mec-processing-delay-ms=$MEC_PROCESSING_DELAY_MS \
    --mec-srs-periodicity=$MEC_SRS_PERIODICITY \
    --mec-duplicate-recovery=$MEC_DUPLICATE_RECOVERY \
    --mec-recovery-policy=$MEC_RECOVERY_POLICY \
    --mec-min-hold-time=$MEC_MIN_HOLD_TIME \
    --mec-forward-range=$MEC_FORWARD_RANGE \
    --priority-distance=$PRIORITY_DISTANCE \
    --priority-closing-speed=$PRIORITY_CLOSING_SPEED \
    --priority-ttc=$PRIORITY_TTC \
    --sensor-range=$SENSOR_RANGE \
    --orr-range=$ORR_RANGE \
    --dsrc-interference=true \
    --dsrc-interference-nodes=$BG_NODES \
    --dsrc-interference-per-vehicle=$BG_PER_VEHICLE \
    --dsrc-interference-size=$BG_SIZE \
    --dsrc-interference-interval-ms=$BG_INTERVAL_MS \
    --cbr-log=$RUN_DIR/cbr.csv \
    --route-log=$RUN_DIR/route.csv \
    --observation-log=$RUN_DIR/observation.csv \
    --summary-csv=$RUN_DIR/summary.csv" > "$STDOUT" 2>&1

  recognition=$(extract_after_colon "Thesis 4.3 recognition rate (%)" "$STDOUT")
  object_recognition=$(extract_after_colon "Thesis 4.3 object recognition rate (%)" "$STDOUT")
  high_priority_object_recognition=$(extract_after_colon "Thesis 4.3 high priority object recognition rate (%)" "$STDOUT")
  low_priority_object_recognition=$(extract_after_colon "Thesis 4.3 low priority object recognition rate (%)" "$STDOUT")
  sensor_object_recognition=$(extract_after_colon "Thesis 4.3 sensor object recognition rate (%)" "$STDOUT")
  sensor_high_priority_object_recognition=$(extract_after_colon "Thesis 4.3 sensor high priority object recognition rate (%)" "$STDOUT")
  sensor_low_priority_object_recognition=$(extract_after_colon "Thesis 4.3 sensor low priority object recognition rate (%)" "$STDOUT")
  cooperative_object_recognition=$(extract_after_colon "Thesis 4.3 cooperative object recognition rate (%)" "$STDOUT")
  orr=$(extract_after_colon "Thesis 4.3 ORR (%)" "$STDOUT")
  packet_loss=$(extract_after_colon "Thesis 4.3 CPM packet loss rate (%)" "$STDOUT")
  ideal_true=$(extract_after_colon "Thesis 4.3 CPM ideal/true RX" "$STDOUT")
  ideal_rx=${ideal_true%/*}
  true_rx=${ideal_true#*/}

  importance_pdr=$(grep -F "Thesis 4.3 importance PDR (%)" "$STDOUT" | tail -n 1)
  high_pdr=$(printf '%s\n' "$importance_pdr" | sed -n 's/.*High=\([^,]*\),.*/\1/p')
  low_pdr=$(printf '%s\n' "$importance_pdr" | sed -n 's/.*Low=\([^ ]*\).*/\1/p')

  importance_loss=$(grep -F "Thesis 4.3 importance packet loss (%)" "$STDOUT" | tail -n 1)
  high_packet_loss=$(printf '%s\n' "$importance_loss" | sed -n 's/.*High=\([^,]*\),.*/\1/p')
  low_packet_loss=$(printf '%s\n' "$importance_loss" | sed -n 's/.*Low=\([^ ]*\).*/\1/p')

  importance_ideal_true=$(grep -F "Thesis 4.3 importance ideal/true RX" "$STDOUT" | tail -n 1)
  high_ideal_true=$(printf '%s\n' "$importance_ideal_true" | sed -n 's/.*High=\([^,]*\),.*/\1/p')
  low_ideal_true=$(printf '%s\n' "$importance_ideal_true" | sed -n 's/.*Low=\([^ ]*\).*/\1/p')
  high_ideal_rx=${high_ideal_true%/*}
  high_true_rx=${high_ideal_true#*/}
  low_ideal_rx=${low_ideal_true%/*}
  low_true_rx=${low_ideal_true#*/}

  route_loss=$(grep -F "Thesis 4.3 CPM packet loss by route" "$STDOUT" | tail -n 1)
  dsrc_route_loss=$(printf '%s\n' "$route_loss" | sed -n 's/.*802\.11p=\([^,]*\),.*/\1/p')
  nr_route_loss=$(printf '%s\n' "$route_loss" | sed -n 's/.*NR-V2X sidelink V2V=\([^,]*\),.*/\1/p')
  mec_route_loss=$(printf '%s\n' "$route_loss" | sed -n 's/.*MEC V2N2V=\([^ ]*\).*/\1/p')

  dsrc_cbr=$(extract_after_colon "802.11p average CBR" "$STDOUT")
  dsrc_txrx=$(extract_txrx "802.11p TX/RX" "$STDOUT")
  dsrc_tx=${dsrc_txrx%/*}
  dsrc_rx=${dsrc_txrx#*/}

  mec_txrx=$(extract_txrx "MEC V2N2V TX/RX" "$STDOUT")
  if [[ "$mec_txrx" == "$STDOUT" || -z "$mec_txrx" ]]; then
    mec_tx=0
    mec_rx=0
  else
    mec_tx=${mec_txrx%/*}
    mec_rx=${mec_txrx#*/}
  fi
  mec_latency=$(extract_after_colon "MEC V2N2V average latency (ms)" "$STDOUT")

  traffic_q_in_avg=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} h["traffic_q_in"] > 0 {sum+=$h["traffic_q_in"]; count++} END {print count ? sum/count : 0}' "$RUN_DIR/observation.csv")
  traffic_q_out_avg=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} h["traffic_q_out"] > 0 {sum+=$h["traffic_q_out"]; count++} END {print count ? sum/count : 0}' "$RUN_DIR/observation.csv")
  traffic_flow_cbr_avg=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} h["traffic_flow_cbr"] > 0 {sum+=$h["traffic_flow_cbr"]; count++} END {print count ? sum/count : 0}' "$RUN_DIR/observation.csv")
  traffic_delta_cbr_avg=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} h["traffic_delta_cbr"] > 0 {sum+=$h["traffic_delta_cbr"]; count++} END {print count ? sum/count : 0}' "$RUN_DIR/observation.csv")
  rsu_count=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} {value=$h["rsu_count"]} END {print value+0}' "$RUN_DIR/observation.csv")
  rsu_local_vehicle_avg=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} h["rsu_local_vehicle_avg"] > 0 {sum+=$h["rsu_local_vehicle_avg"]; count++} END {print count ? sum/count : 0}' "$RUN_DIR/observation.csv")
  rsu_upstream_vehicle_avg=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} h["rsu_upstream_vehicle_avg"] > 0 {sum+=$h["rsu_upstream_vehicle_avg"]; count++} END {print count ? sum/count : 0}' "$RUN_DIR/observation.csv")
  rsu_future_vehicle_avg=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} h["rsu_future_vehicle_avg"] > 0 {sum+=$h["rsu_future_vehicle_avg"]; count++} END {print count ? sum/count : 0}' "$RUN_DIR/observation.csv")
  rsu_predicted_cbr_avg=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} h["rsu_predicted_cbr_avg"] > 0 {sum+=$h["rsu_predicted_cbr_avg"]; count++} END {print count ? sum/count : 0}' "$RUN_DIR/observation.csv")
  rsu_predicted_cbr_max=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} {value=$h["rsu_predicted_cbr_max"]} END {print value+0}' "$RUN_DIR/observation.csv")

  interference=$(extract_after_colon "DSRC interference TX/drops/bytes" "$STDOUT")
  interference_tx=$(printf '%s' "$interference" | cut -d/ -f1)
  interference_drops=$(printf '%s' "$interference" | cut -d/ -f2)
  interference_bytes=$(printf '%s' "$interference" | cut -d/ -f3)
  interference_offered_cbr=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} {value=$h["interference_offered_cbr"]} END {print value+0}' "$RUN_DIR/observation.csv")

  switch_to_mec=$(awk -F, 'NR>1 && $4 ~ /MEC/ {count++} END {print count+0}' "$RUN_DIR/route.csv")
  release_to_dsrc=$(awk -F, 'NR>1 && $4 == "DSRC_V2V" {count++} END {print count+0}' "$RUN_DIR/route.csv")
  final_dsrc_routes=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} {value=$h["dsrc_route_count"]} END {print value+0}' "$RUN_DIR/observation.csv")
  final_mec_routes=$(awk -F, 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} {value=$h["mec_route_count"]} END {print value+0}' "$RUN_DIR/observation.csv")
  cbr_exceed_seconds=$(awk -F, -v threshold="$SWITCH_CBR" 'NR==1 {for (i=1; i<=NF; ++i) h[$i]=i; next} $h["dsrc_cbr_max"] >= threshold {count++} END {print count+0}' "$RUN_DIR/observation.csv")

  echo "$method,$object_recognition,$high_priority_object_recognition,$low_priority_object_recognition,$sensor_object_recognition,$sensor_high_priority_object_recognition,$sensor_low_priority_object_recognition,$cooperative_object_recognition,$recognition,$packet_loss,$ideal_rx,$true_rx,$orr,$high_pdr,$low_pdr,$high_packet_loss,$low_packet_loss,$high_ideal_rx,$high_true_rx,$low_ideal_rx,$low_true_rx,$dsrc_route_loss,$nr_route_loss,$mec_route_loss,$dsrc_cbr,$dsrc_tx,$dsrc_rx,$mec_tx,$mec_rx,$mec_latency,$traffic_q_in_avg,$traffic_q_out_avg,$traffic_flow_cbr_avg,$traffic_delta_cbr_avg,$rsu_count,$rsu_local_vehicle_avg,$rsu_upstream_vehicle_avg,$rsu_future_vehicle_avg,$rsu_predicted_cbr_avg,$rsu_predicted_cbr_max,$interference_tx,$interference_drops,$interference_bytes,$interference_offered_cbr,$switch_to_mec,$release_to_dsrc,$final_dsrc_routes,$final_mec_routes,$cbr_exceed_seconds" >> "$SUMMARY"
done

echo "Wrote $SUMMARY"
