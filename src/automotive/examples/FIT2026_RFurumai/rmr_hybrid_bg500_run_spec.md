# RMR + Hybrid BG500 Run Specification

Date: 2026-07-17 10:35 JST

Purpose:

- Run the existing RMR implementation with Hybrid V2V-V2N2V.
- Do not modify the RMR formula or deletion logic.
- Use this as a baseline to compare against:
  - Hybrid baseline
  - Hybrid + prediction
  - V2N2V-only

## Result Reference

Use these result paths for the current completed comparison:

- Baseline Hybrid:
  - `results/bg500_hybrid_fixed_seed30_100s_retry/v2n2v-adaptive-probability/summary.csv`
  - `results/bg500_hybrid_fixed_seed30_100s_retry/v2n2v-adaptive-probability/observation.csv`
- RMR + Hybrid:
  - `results/bg500_hybrid_rmr_seed30_100s/v2n2v-adaptive-probability/summary.csv`
  - `results/bg500_hybrid_rmr_seed30_100s/v2n2v-adaptive-probability/observation.csv`

Summary comparison:

| Metric | Baseline Hybrid | RMR + Hybrid | Interpretation |
|---|---:|---:|---|
| ORR | 69.7406 | 98.6314 | Large improvement |
| High-priority ORR | 70.5270 | 98.7799 | Large improvement |
| Low-priority ORR | 68.4500 | 98.3786 | Large improvement |
| Channel busy ratio | 0.802389 | 0.802023 | Almost unchanged |
| V2V radio loss rate | 63.8187 | 55.7377 | Improved |
| MEC radio final loss rate | 0.482234 | 0.482234 | Unchanged |
| MEC update failure rate | 79.4684 | 14.4283 | Large improvement |
| MEC valid update success rate | 20.5316 | 85.5717 | Large improvement |
| MEC AoI violation rate, 200 ms | 78.0518 | 22.3451 | Large improvement |
| MEC AoI violation rate, 500 ms | 68.5561 | 0 | Large improvement |
| TTL violation rate | 29.1034 | 0.436093 | Large improvement |
| MEC AoI p99 | 25579 ms | 249 ms | Large improvement |
| MEC DL busy p99 | 1.0 | 0.911616 | Improved |

Observation final-row comparison:

| Metric | Baseline Hybrid, t=99 | RMR + Hybrid, t=99 | Interpretation |
|---|---:|---:|---|
| ORR | 70.0809 | 98.6187 | Large improvement |
| CBR | 0.825138 | 0.824873 | Almost unchanged |
| V2V radio loss rate | 63.9561 | 55.9219 | Improved |
| MEC radio final loss rate | 0.48233 | 0.48233 | Unchanged |
| MEC AoI violation rate, 200 ms | 77.63 | 22.441 | Large improvement |
| MEC update failure rate | 79.3431 | 14.5044 | Large improvement |
| NR CPM size bytes total | 12919837 | 4467518 | 65.42% reduction |
| NR RMR deleted total | 0 | 187294 | RMR active |

Key interpretation:

- RMR with a maximum deletion count of 40 objects was effective in this BG500 Hybrid scenario.
- The main benefit is not a change in MEC radio loss. `mec_radio_final_loss_rate` stayed almost identical.
- The benefit comes from reducing CPM payload volume, which improves MEC freshness and valid update success.
- The 65.42% reduction is for `nr_cpm_size_bytes_total`, i.e., cumulative NR sidelink CPM payload size. It should not be described as a 65.42% reduction of all communication traffic.

## RMR Direction After This Result

This result shows that the existing delete count set `10/20/40` is already strong enough to improve Hybrid performance.

Do not immediately double or triple the deletion counts as the next step:

- It may further reduce channel load, but it can remove useful object information and hurt ORR.
- The result already achieves ORR near 98.6%, so aggressive deletion has limited upside and larger risk.
- A safer next step is to understand which deleted objects were redundant and which were harmful to delete.

Lower-risk improvement ideas without a complex new formula:

- Sweep the existing delete-count set around the current values, for example `5/10/20`, `10/20/40`, and `15/30/60`.
- Add a cap based on priority so high-priority objects are deleted less aggressively.
- Add a minimum redundancy guard, such as keeping at least two recent observations for important objects where possible.
- Log receiver-side redundancy before changing the RMR formula.

Prediction should not be the immediate next main result:

- The current prediction-only Hybrid run made `predicted_channel_busy_ratio_avg` valid, but it did not improve routing because baseline Hybrid was already in full dual transmission early.
- Prediction may help only after the control policy avoids over-triggering dual transmission or after RMR reduces payload size.
- If tested, use it after the RMR baseline is stable and compare `RMR + Hybrid` versus `Prediction + RMR + Hybrid`.

## Receiver-Side Redundancy Metrics

The simulator now logs receiver-side redundancy metrics in `observation.csv` and `summary.csv`.

Goal:

- Measure how many independent or recent observations each receiver actually has for the same object.
- Determine whether RMR removes waste while preserving enough redundancy to tolerate packet loss.

Definition:

- For each receiver and expected object, count fresh independent observations from:
  - sensor recognition
  - NR sidelink CPM
  - MEC/V2N2V CPM
- `receiver_fresh_redundancy_200ms_mean`: mean fresh observation count under the 200 ms AoI threshold.
- `receiver_fresh_redundancy_500ms_mean`: mean fresh observation count under the 500 ms AoI threshold.
- `receiver_fresh_redundancy_ge2_200ms_rate`: percentage of receiver-object pairs with at least two fresh observations under 200 ms.
- `receiver_fresh_redundancy_ge2_500ms_rate`: same under 500 ms.
- `receiver_high_fresh_redundancy_200ms_mean`: 200 ms fresh redundancy mean for high-priority expected objects.
- `receiver_low_fresh_redundancy_200ms_mean`: 200 ms fresh redundancy mean for low-priority expected objects.
- `receiver_rv_200ms_score`: receiver-side RV-like score, defined as `min(fresh_redundancy_200ms / 2, 1)` averaged over receiver-object pairs.

Interpretation target:

- Redundancy of 0 means the object was never received.
- Redundancy of 1 means recognition depends on a single update and is fragile.
- Redundancy of 2 or more suggests packet-loss tolerance through duplicated object knowledge.
- Excessively high redundancy suggests waste and is a candidate for RMR deletion.

RMR deletion diagnostics:

- `rmr_deleted_eval_distance_mean_m`: mean receiver-object distance for expected objects deleted by RMR.
- `rmr_deleted_eval_fresh_redundancy_ge2_200ms_rate`: among expected deleted objects, percentage that still had at least two fresh observations under 200 ms.
- `rmr_deleted_eval_fresh_redundancy_ge2_500ms_rate`: same under 500 ms.
- `rmr_deleted_eval_closing_speed_ge_threshold_rate`: percentage of expected deleted objects whose closing speed exceeded the high-priority threshold.
- `rmr_deleted_eval_ttc_le_threshold_rate`: percentage of expected deleted objects whose TTC was within the high-priority threshold.
- `rmr_deleted_sender_eval_total`: total deleted objects counted once from the sender side.
- `rmr_deleted_sender_eval_high`: deleted objects that were high-priority from the sender side.
- `rmr_deleted_sender_eval_low`: deleted objects that were low-priority from the sender side.
- `rmr_deleted_sender_eval_distance_mean_m`: mean sender-object distance for deleted objects.
- `rmr_deleted_sender_eval_closing_speed_ge_threshold_rate`: percentage of deleted objects whose sender-side closing speed exceeded the high-priority threshold.
- `rmr_deleted_sender_eval_ttc_le_threshold_rate`: percentage of deleted objects whose sender-side TTC was within the high-priority threshold.

Use the receiver-side `rmr_deleted_eval_*` fields to explain whether deleted objects were still redundant at receivers. Use the sender-side `rmr_deleted_sender_eval_*` fields to explain what kinds of objects RMR selected for deletion in the first place.

RMR formula feature diagnostics in `observation.csv`:

- `rmr_deleted_feature_count`: route-agnostic cumulative number of RMR-deleted objects whose selection features were logged up to this observation time.
- `rmr_deleted_distance_mean_m`: route-agnostic mean RMR candidate distance for deleted objects.
- `rmr_deleted_frequency_mean`: route-agnostic mean recent observation frequency `n` for deleted objects.
- `rmr_deleted_position_change_mean_m`: route-agnostic mean position change `l` for deleted objects.
- `rmr_deleted_speed_change_mean_mps`: route-agnostic mean speed change `v` for deleted objects.
- `rmr_deleted_score_mean`: route-agnostic mean RMR score `n / (distance * min(l, v))` for deleted objects.
- `nr_rmr_deleted_feature_count`: cumulative number of NR RMR-deleted objects whose selection features were logged up to this observation time.
- `nr_rmr_deleted_distance_mean_m`: mean RMR candidate distance for deleted objects.
- `nr_rmr_deleted_frequency_mean`: mean recent observation frequency `n` for deleted objects.
- `nr_rmr_deleted_position_change_mean_m`: mean position change `l` for deleted objects.
- `nr_rmr_deleted_speed_change_mean_mps`: mean speed change `v` for deleted objects.
- `nr_rmr_deleted_score_mean`: mean RMR score `n / (distance * min(l, v))` for deleted objects.

The route-agnostic `rmr_deleted_feature_*` fields are the primary diagnostics because RMR primary/offload sharing can make NR-side deleted counts differ from the container that computed the selection features. These fields do not depend on converting RMR object IDs back to active SUMO vehicle IDs.

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

The latest commits should include:

```text
Enable hybrid traffic-flow prediction without RMR
Enable reactive RMR for hybrid evaluation
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
  SUMO_PORT=19200 \
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
  TRAFFIC_FLOW_RSU_PREDICTOR=false \
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
ps -ef | grep -E 'v2v-hybrid-nr-v2n2v|sumo.*19200|ns3 run' | grep -v grep
```

Expected:

- One `ns3-dev-v2v-hybrid-nr-v2n2v-optimized` process
- One `sumo ... --remote-port 19200` process
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
