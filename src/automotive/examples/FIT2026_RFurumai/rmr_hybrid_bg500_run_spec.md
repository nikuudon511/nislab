# RMR + Hybrid BG500 Run Specification

Date: 2026-07-16

Purpose:

- Run the existing RMR implementation with Hybrid V2V-V2N2V.
- Do not modify the RMR formula or deletion logic.
- Use this as a baseline to compare against:
  - Hybrid baseline
  - Hybrid + prediction
  - V2N2V-only

## Important

Use exactly this condition on the other PC.

- Method: `v2n2v-adaptive-probability`
- Actual ns-3 method: `hybrid-v2v-v2n2v`
- RMR: existing implementation only
- RMR deletion counts: existing script/default values
- Prediction weights: all disabled
- BG: 500 bytes / 100 ms / per vehicle
- Vehicles: 200 communication vehicles
- Simulation time: 100 s
- Seed: 30
- SUMO sync interval: 0.1 s
- MEC forward range: 200 m
- AoI filter: enabled, 200 ms threshold

## Git Setup On The Other PC

Use the latest branch state:

```bash
git fetch origin
git checkout thesis-simulation-20260618
git pull origin thesis-simulation-20260618
```

Confirm:

```bash
git log --oneline --decorate -n 3
```

The latest commit should include:

```text
47d69751 0716
```

## Run Command

```bash
cd /home/ryose/VaN3Twin/ns-3-dev

mkdir -p results/nohup_logs

CFG="src/automotive/examples/sumo_files_highway_straight/map_200_2clusters_nostop.sumo.cfg"
ROU="cars_200_2clusters_nostop.rou.xml"
OUT="results/bg500_hybrid_rmr_seed30_100s"

nohup env \
  CCACHE_DISABLE=1 \
  SIM_TIME=100 \
  METHODS="v2n2v-adaptive-probability" \
  SUMO_SEED=30 \
  MOB_TRACE="$ROU" \
  SUMO_CONFIG="$CFG" \
  OUT_DIR="$OUT" \
  SUMO_PORT=19180 \
  SUMO_SYNC_INTERVAL=0.1 \
  MAX_COMMUNICATION_VEHICLES=200 \
  THESIS_EVAL_START_MIN_VEHICLES=200 \
  THESIS_EVAL_WARMUP_SECONDS=1 \
  THESIS_EVAL_INTERVAL=1 \
  OBSERVATION_LOG_INTERVAL=1 \
  TRAFFIC_FLOW_ROAD_LENGTH=3000 \
  PREDICTION_HORIZON=20 \
  PREDICTOR_CPM_SIZE_WEIGHT=0 \
  PREDICTOR_CPM_TX_RATE_WEIGHT=0 \
  PREDICTOR_ACTIVE_VEHICLE_WEIGHT=0 \
  RSU_I2V_RANGE=300 \
  SENSOR_RANGE=30 \
  ORR_RANGE=200 \
  PRIORITY_DISTANCE=100 \
  PRIORITY_CLOSING_SPEED=3 \
  PRIORITY_TTC=5 \
  MEC_FORWARD_RANGE=200 \
  MEC_IDEAL_LINK=true \
  MEC_IDEAL_AOI_FILTER=true \
  MEC_IDEAL_AOI_FILTER_THRESHOLD_MS=200 \
  MEC_IDEAL_DL_PDR=1.0 \
  MEC_IDEAL_DELAY_MODEL=empirical \
  MEC_IDEAL_LATENCY_MS=39.4 \
  MEC_IDEAL_LATENCY_STDDEV_MS=20 \
  MEC_IDEAL_LATENCY_MIN_MS=6.1 \
  MEC_IDEAL_LATENCY_P90_MS=59.86 \
  MEC_IDEAL_LATENCY_P99_MS=120.33 \
  MEC_IDEAL_LATENCY_MAX_MS=201 \
  MEC_IDEAL_CAPACITY_MODEL=true \
  MEC_IDEAL_BANDWIDTH_MHZ=20 \
  MEC_IDEAL_SPECTRAL_EFFICIENCY_BPSHZ=4.5234 \
  MEC_IDEAL_MAX_QUEUE_DELAY_MS=0 \
  MEC_IDEAL_UL_FIRST_LOSS_RATE=0.10 \
  MEC_IDEAL_DL_FIRST_LOSS_RATE=0.10 \
  MEC_IDEAL_RETX_SUCCESS_PROB=0.53 \
  MEC_IDEAL_MAX_RETX=4 \
  MEC_IDEAL_RETX_DELAY_MS=3 \
  MEC_BG=true \
  MEC_BG_PER_VEHICLE=true \
  MEC_BG_SIZE=500 \
  MEC_BG_INTERVAL_MS=100 \
  NR_BG=true \
  NR_BG_PER_VEHICLE=true \
  NR_BG_SIZE=500 \
  NR_BG_INTERVAL_MS=100 \
  REACTIVE_RMR=true \
  RMR_DELETE_LOW=10 \
  RMR_DELETE_MIDDLE=20 \
  RMR_DELETE_HIGH=40 \
  bash src/automotive/examples/run_hybrid_nr_v2n2v_evaluation.sh \
  > results/nohup_logs/bg500_hybrid_rmr_seed30_100s.log 2>&1 &
```

## Process Check

```bash
ps -ef | grep -E 'v2v-hybrid-nr-v2n2v|sumo.*19180|ns3 run' | grep -v grep
```

Expected:

- One `ns3-dev-v2v-hybrid-nr-v2n2v-optimized` process
- One `sumo ... --remote-port 19180` process
- Output path includes `results/bg500_hybrid_rmr_seed30_100s/v2n2v-adaptive-probability`

## Progress Check

```bash
python3 -c "import csv,os; p='results/bg500_hybrid_rmr_seed30_100s/v2n2v-adaptive-probability/observation.csv'; rows=list(csv.DictReader(open(p))) if os.path.exists(p) else []; print('rows='+str(len(rows))); print('not logged yet' if not rows else 't='+rows[-1]['time_s']+' ORR='+rows[-1]['orr']+' CBR='+rows[-1]['channel_busy_ratio_avg']+' v2vRadioLoss='+rows[-1]['v2v_radio_loss_rate']+' mecRadioLoss='+rows[-1]['mec_radio_final_loss_rate']+' mecAoi200='+rows[-1]['mec_aoi_violation_rate_200ms']+' nrRmrDeleted='+rows[-1]['nr_rmr_deleted_total'])"
```

## Completion Check

```bash
python3 - <<'PY'
import csv, os
p='results/bg500_hybrid_rmr_seed30_100s/v2n2v-adaptive-probability/summary.csv'
if not os.path.exists(p):
    print('summary missing')
else:
    r=list(csv.DictReader(open(p)))[-1]
    for c in [
        'orr',
        'high_priority_object_recognition_rate',
        'low_priority_object_recognition_rate',
        'channel_busy_ratio',
        'v2v_radio_loss_rate',
        'mec_radio_final_loss_rate',
        'mec_aoi_violation_rate_200ms',
        'ttl_violation_rate',
        'never_received_rate',
        'nr_rmr_deleted_total',
        'nr_cpm_size_bytes_total'
    ]:
        print(f'{c}={r.get(c)}')
PY
```

## Notes

- If the process exits early and `summary.csv` is missing, do not reuse the partial result.
- Re-run with a new output directory, for example:
  - `results/bg500_hybrid_rmr_seed30_100s_retry`
- Do not compare partial `observation.csv` only runs as final results.
