# RMR + Hybrid BG500 Run Specification

Date: 2026-07-17 10:35 JST
Last updated: 2026-07-19 00:20 JST

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

## Completed V2V-Only RMR Check

Use this result to answer whether V2V-only with RMR is enough:

- V2V only:
  - `results/bg500_base3_seed30_100s_retry/no-control/summary.csv`
- V2V only + RMR:
  - `results/bg500_v2v_only_rmr_fixed_seed30_100s/no-control/summary.csv`

Summary:

| Metric | V2V only | V2V only + RMR | Interpretation |
|---|---:|---:|---|
| ORR | 61.3192 | 54.8371 | Worse with RMR |
| High-priority ORR | 62.3321 | 54.3017 | Worse with RMR |
| Low-priority ORR | 59.5805 | 54.9982 | Worse with RMR |
| Channel busy ratio | 0.802389 | 0.802023 | Almost unchanged |
| Packet/radio loss rate | 63.8187 | 55.7377 | Improved |
| V2V update failure rate | N/A | 80.0029 | High after RMR |
| Receiver RL mean | N/A | 0.495277 | Low redundancy |
| Receiver RV mean | N/A | 0.093639 | Low redundancy value |

Interpretation:

- V2V-only RMR reduced packet/radio loss, but ORR decreased by about 6.5 points.
- In V2V-only, object deletion directly removes reception opportunities and can reduce useful redundancy.
- This supports the claim that V2V-only + RMR is not sufficient under the current BG500 condition.
- The Hybrid + RMR gain should therefore be explained as a V2N2V/MEC freshness improvement, not as a simple V2V CBR reduction.

Remaining required check:

- `V2N2V-only + RMR / BG500 / seed30 / 100 s` is still not available as a completed result.
- Existing `results/bg500_v2n2v_only_rmr_seed30_100s*` summary files are empty and must not be used as final results.
- This run is needed to separate:
  - RMR effect on the V2N2V path itself.
  - Hybrid complementarity between V2V and V2N2V.

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

## One-Week Progress Strategy

Do not introduce a new RL/RV-optimizing RMR formula before the next progress report.

Reason:

- The current RMR + Hybrid result already gives a strong main result.
- New control logic would require implementation, smoke validation, 100 s evaluation, and failure analysis.
- With one week left, the safer path is to finish comparable results and use RL/RV as an analysis axis.

Priority before the next report:

1. Finish `RMR + Hybrid + Delooz RL/RV / BG500 / 100 s`.
2. Finish `Baseline Hybrid + Delooz RL/RV / BG500 / 100 s`.
3. Compare ORR, AoI, update failure, packet loss, RL, and RV.
4. Use the result to show:
   - RMR improves ORR and MEC freshness.
   - Delooz RL/RV reveals whether enough receiver-side redundancy remains.
   - Remaining issue is not simply deletion volume, but deciding which objects should keep redundancy.

Lower-priority runs if time remains:

- `V2N2V-only + Delooz RL/RV / BG500 / 100 s`
- `V2V-only + Delooz RL/RV / BG500 / 100 s`
- `RMR + Hybrid / BG300 / 100 s`

Defer:

- New RL/RV-controlled RMR formula.
- Prediction-Deviation object selection.
- Lane-change scenario.
- Large RMR delete-count sweep.
- MEC forward range 500 m sensitivity.

## Prediction Positioning

The FIT title includes a bandwidth-prediction concept, so prediction should not be removed from the research story.

However, prediction is not the current main effect. The main result is still:

- Hybrid V2V/V2N2V combines complementary paths.
- RMR reduces CPM payload.
- Reduced payload improves MEC freshness, update success, and ORR.

Current concern:

- The previous prediction-only Hybrid run did not improve results.
- The likely reason is not that prediction is useless, but that prediction was used too directly to trigger stronger route use or full dual transmission.
- In the current straight-road, no-lane-change scenario, prediction-deviation based object value is also expected to have limited effect because vehicle motion is highly predictable.

Preferred prediction role:

- Use bandwidth prediction as an early RMR-strength control signal.
- Do not use prediction mainly to increase dual transmission.
- If predicted future CBR is high, reduce CPM payload earlier by increasing RMR strength.
- Keep high-priority, near-distance, and dangerous-TTC objects protected from deletion.

Expected effect:

- Prediction may not give a dramatic improvement over static RMR.
- Its effect should be larger when RMR deletion strength is allowed to vary with predicted congestion.
- Static `10/20/40` RMR already performs well, so prediction should be presented as an incremental extension, not the main contribution.

Suggested wording:

> The bandwidth prediction component is used to anticipate future congestion and adjust redundancy mitigation before the channel becomes saturated. In the current scenario, static RMR already provides a large improvement, so prediction is expected to provide incremental rather than dramatic gains. Its main role is to make RMR adaptive to future bandwidth pressure while RL/RV monitors whether sufficient receiver-side redundancy remains.

## Time-Varying Bandwidth Stress Scenario For Prediction

The current BG500 scenario has essentially one major congestion phase. This makes static RMR look strong, because a fixed aggressive deletion policy can handle the single peak.

Prediction is expected to be more meaningful when bandwidth stress changes over time.

Avoid spatial high-load zones for this validation. They make the interpretation depend on vehicle position, traffic clustering, and zone crossing behavior. Instead, use a time-varying background load that applies globally and changes sinusoidally over time.

Preferred load model:

```text
BG_factor(t) = clamp(1 + A * sin(pi * t / 10), min_factor, max_factor)
```

With this setting:

- `period = 20 s`
- `t = 5 s` is a high-load peak
- `t = 15 s` is a low-load valley
- `t = 25 s` is the next high-load peak

This isolates the bandwidth-prediction problem from traffic-flow and lane-change effects.

Hypothesis:

- Without prediction, RMR reacts to current CBR.
  - It may delete too late before a congestion peak.
  - It may keep deleting too strongly after the peak has passed.
  - ORR may remain high, but receiver-side RL/RV may be unnecessarily reduced during non-peak periods.
- With bandwidth prediction, RMR can increase deletion before the peak and relax deletion after the peak.
  - ORR and AoI should be maintained.
  - RL/RV may improve because useful redundancy is preserved outside the predicted congestion peak.

The prediction contribution should therefore be framed as:

> Prediction does not necessarily raise ORR dramatically when static RMR is already strong. Its value is adaptive control: keeping enough receiver-side redundancy while responding earlier to time-varying bandwidth pressure.

Recommended comparison:

1. `RMR + Hybrid`
   - current-CBR-based RMR
   - periodic BG load
2. `Prediction + RMR + Hybrid`
   - predicted-CBR-based RMR
   - same periodic BG load

Important metrics:

- `orr`
- `mec_aoi_violation_rate_200ms`
- `mec_update_failure_rate`
- `channel_busy_ratio_avg`
- `nr_cpm_size_bytes`
- `nr_rmr_deleted_total`
- `receiver_rl_mean`
- `receiver_rl_ge1_rate`
- `receiver_rl_ge2_rate`
- `receiver_rv_delooz_mean`

Use time-series plots rather than only summary averages. The expected difference is temporal adaptation, not necessarily a large final average ORR gap.

Lightweight validation design:

- Avoid a full 200-vehicle, 100 s run first.
- Use a shorter run such as 30 s or 40 s.
- Use 50 or 100 communication vehicles.
- Increase periodic BG intensity to create clear CBR waves.
- Prefer time-only BG controls instead of spatial high-load zones:
  - `NR_BG_TIME_WAVE=true`
  - `NR_BG_TIME_WAVE_PERIOD=20`
  - `NR_BG_TIME_WAVE_AMPLITUDE=0.8`
  - `NR_BG_TIME_WAVE_MIN_FACTOR=0.2`
  - `NR_BG_TIME_WAVE_MAX_FACTOR=1.8`
- Keep the vehicle route file and road layout unchanged if possible.

Implementation preference:

- Vary effective BG packet size, not packet interval.
- This is easier to implement because scheduled traffic events do not need to be rescheduled.
- Example:

```text
effective_bg_size = base_bg_size * BG_factor(t)
```

Start with NR BG only. Add MEC BG time variation only if NR-only variation does not produce clear CBR waves.

Possible lightweight settings:

- Option A: 50 vehicles, 40 s
  - Fastest validation.
  - May need stronger BG to reproduce congestion.
- Option B: 100 vehicles, 40 s
  - Better balance between runtime and congestion realism.
  - More likely to show RMR/RL/RV differences.
- Option C: 200 vehicles, 40 s
  - Keeps density close to main evaluation.
  - Still much shorter than 100 s, but runtime may remain high.

Preferred first trial:

- `SIM_TIME=40`
- `MAX_COMMUNICATION_VEHICLES=100`
- `THESIS_EVAL_START_MIN_VEHICLES=100`
- time-varying BG enabled
- compare only two methods:
  - static RMR
  - prediction-aware RMR

Do not introduce lane changes for this validation. Lane changes would make prediction-deviation evaluation more meaningful, but they would also expand the scenario design and move away from the current thesis evaluation path.

If the lightweight time-varying-BG experiment shows that prediction improves `receiver_rv_delooz_mean` or `receiver_rl_ge1_rate` without hurting ORR/AoI, then it is a strong supporting result for the FIT title.

## Next Research Candidate: RL-Aware RMR

The current RMR result shows a clear tradeoff:

- ORR increases over time.
- Delooz-style RL/RV decreases over time.

Observed example from the running RMR + Hybrid + Delooz evaluation:

| Time | ORR | RL mean | RL >= 1 rate | RL >= 2 rate | RV mean | NR RMR deleted total |
|---:|---:|---:|---:|---:|---:|---:|
| 18 s | 93.0744 | 1.54368 | 25.102% | 12.9548% | 0.194473 | 22646 |
| 27 s | 95.5676 | 1.15356 | 19.0588% | 9.7656% | 0.149577 | 24755 |
| 44 s | 97.2790 | 0.809916 | 13.853% | 6.80985% | 0.111575 | 45134 |

Interpretation:

- Current RMR successfully reduces CPM payload and improves recognition.
- However, it also removes redundant updates that would increase packet-loss tolerance.
- ORR answers whether an object was recognized at least once.
- RL/RV answers whether the receiver still has enough update redundancy for robustness.

This motivates an RL-aware RMR direction.

Ideal rule:

```text
If RL(receiver, object) < 1:
  do not delete
If 1 <= RL(receiver, object) < 2:
  delete only low-priority / low-risk objects
If RL(receiver, object) >= 2:
  deletion is safer
```

Main difficulty:

- True Delooz RL is receiver-side information.
- A sender does not directly know how many recent updates each receiver already has for each object.
- Therefore, exact RL-aware RMR cannot be implemented by the sender alone without additional information exchange or estimation.

Possible implementation paths:

1. Offline counterfactual analysis
   - Use logs to check how many RMR-deleted objects had `RL < 1` or `RL < 2`.
   - This is the safest first step.
   - It can show whether current RMR deletes objects that were not yet sufficiently redundant.

2. Sender-side estimated RL
   - Estimate redundancy from local observation frequency, object dynamics, distance, and priority.
   - Easier to implement because it does not require protocol changes.
   - Less accurate than receiver-side RL.

3. Receiver-advertised RL
   - Receivers periodically advertise compact object redundancy state.
   - Example fields: `object_id`, `local_rl_bin`, `timestamp`.
   - This could be carried by an extended CAM/CPM or a lightweight custom beacon.
   - It may allow senders to avoid transmitting objects already redundant at receivers.
   - Risk: the redundancy advertisement itself adds overhead and freshness issues.

Research value:

- RL-aware RMR directly addresses the weakness revealed by Delooz RL/RV.
- It is more principled than simply increasing or decreasing the RMR deletion count.
- It can be positioned as the next step after showing that current RMR improves ORR but reduces redundancy.

## Can Current Bandwidth Prediction Improve RL/RV?

Existing bandwidth prediction may improve RL/RV, but only if it changes RMR timing or strength in the right direction.

Likely improvement condition:

- Prediction is used to avoid over-deletion outside congestion periods.
- Prediction makes RMR strong only before or during predicted congestion.
- Prediction relaxes RMR when future bandwidth pressure is low.

In that case:

- ORR and AoI may stay close to static RMR.
- `receiver_rl_mean`, `receiver_rl_ge1_rate`, and `receiver_rv_delooz_mean` may improve during non-peak periods.

Likely failure condition:

- Prediction only pushes CBR toward 1.0.
- Prediction triggers stronger route use or always-strong RMR.
- Then RL/RV may not improve and may even degrade.

Conclusion:

- Bandwidth prediction alone is unlikely to dramatically improve ORR because static RMR is already strong.
- Its best role is adaptive RMR strength control that preserves RL/RV outside predicted congestion peaks.
- For a clearer prediction benefit, use a time-varying BG scenario rather than the current single-stress scenario.

Presentation wording:

> RMR greatly improves Hybrid ORR and AoI. However, Delooz-style RL/RV shows that high ORR does not necessarily mean sufficient receiver-side redundancy. The next step is to preserve redundancy for important or fragile objects while continuing to suppress redundant CPM payloads.

## SUMO Sync Interval Decision

Use `SUMO_SYNC_INTERVAL=0.1` for the current evaluation set.

Reason:

- The current scenario is straight-road, no lane changing, and smooth vehicle motion.
- CPM generation and thesis evaluation intervals are at 100 ms or longer.
- All compared methods use the same sync interval, so the comparison is fair.
- Returning to 0.01 s would greatly increase runtime and make 100 s evaluation less practical.

If future scenarios include lane changes, sudden braking, intersections, or prediction-deviation evaluation, re-check sensitivity with `SUMO_SYNC_INTERVAL=0.05` or `0.01`.

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

These `receiver_fresh_redundancy_*` fields are auxiliary diagnostics only. They count fresh source categories and should not be presented as the main redundancy metric.

Delooz-style receiver RL/RV diagnostics:

- `receiver_rl_mean`: mean Redundancy Level over receiver-object pairs.
- `receiver_rl_median`: median Redundancy Level per receiver, then averaged over receiver samples.
- `receiver_rl_lt1_rate`: percentage of receiver-object pairs with fewer updates than required.
- `receiver_rl_ge1_rate`: percentage of receiver-object pairs with at least the required updates.
- `receiver_rl_ge2_rate`: percentage of receiver-object pairs with at least twice the required updates.
- `receiver_rv_delooz_mean`: mean Redundancy Valuation using Delooz's Gompertz function.
- `receiver_rv_delooz_median`: median Delooz RV per receiver, then averaged over receiver samples.
- `receiver_high_rl_mean`: RL mean for high-priority expected objects.
- `receiver_low_rl_mean`: RL mean for low-priority expected objects.

Implementation note:

- `n_rec` counts CPM object updates received in the last 1 s, independent of route. A local sensor observation contributes one update when the object is inside sensor range.
- `n_req = ceil(max(distance_change / 4 m, speed_change / 0.5 m/s, 1))`.
- Heading variation is not included in the current lightweight implementation because the simulator-side receiver history does not yet keep robust per-object heading history.
- `RL = n_rec / n_req`.
- `RV = exp(-7 * exp(-2.31337 * RL))`.
- This follows Delooz et al. for the evaluation concept, while keeping the current RMR deletion logic unchanged.

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
