# Metric Definitions

Last updated: 2026-07-26 JST

This file defines the metrics used for comparison, presentation, and paper
drafting.  Use it together with:

- `src/automotive/examples/FIT2026_RFurumai/V2X_specification.md`
- `docs/research/experiment_index.md`
- `docs/research/claim_evidence_index.md`

## Metric Groups

| Group | Purpose | Main metrics |
|---|---|---|
| Recognition KPI | Measures final application-level recognition performance | ORR, high/low ORR |
| MEC bottleneck KPI | Explains whether V2N2V can deliver useful fresh updates | MEC required DL load, MEC update failure, MEC AoI p99 |
| V2V load KPI | Explains V2V sidelink demand in Mbps in addition to CBR | V2V sidelink required Mbps, V2V required/capacity ratio |
| Radio reliability | Separates physical/abstract radio losses from application failures | V2V radio loss, MEC UL/DL final radio loss |
| Freshness | Measures whether received updates are fresh enough | AoI violation, TTL violation |
| Diagnostics | Helps explain why RMR helped or hurt | CPM payload, RMR deleted counts, RL/RV |
| Route control | Separates hybrid path-policy effects from RMR effects | dual route count, high dual TX setting, MEC forwarding probability |

## Primary KPI

These are the main metrics to use in headline comparisons.

| Metric | CSV name | Meaning | Preferred use | Important caution |
|---|---|---|---|---|
| Object Recognition Rate | `object_recognition_rate`, `orr` | Fraction of expected objects recognized within the evaluation logic | Main performance KPI | Not equal to packet delivery ratio |
| High-priority ORR | `high_priority_object_recognition_rate` | ORR for high-priority objects | Safety-critical recognition KPI | Depends on priority definition |
| Low-priority ORR | `low_priority_object_recognition_rate` | ORR for low-priority objects | Coverage/secondary recognition KPI | Must use TTL 0.2 s in final evaluation |
| MEC required DL load p99 | `mec_dl_required_mbps_p99` | p99 downlink load required by MEC forwarding before clipping | Main MEC bottleneck KPI | For old runs, compute from `observation.csv` |
| MEC required/capacity p99 | `mec_dl_required_load_ratio_p99` | p99 required DL load divided by DL capacity, not clipped at 1 | Main overload KPI | Values above 1 mean requested load exceeds capacity |
| MEC update failure rate | `mec_update_failure_rate` | Expected MEC updates that did not become valid fresh recognition updates | Main V2N2V usefulness KPI | Not a physical radio packet-loss rate |
| MEC AoI p99 | `mec_aoi_p99_ms` | p99 generation-time-based AoI for MEC updates | Main freshness-tail KPI | High values indicate stale updates even if packets arrived |
| MEC AoI violation rate | `mec_aoi_violation_rate_200ms` | Received MEC updates whose AoI exceeds the threshold | Freshness KPI | Not a radio loss metric |
| MEC valid update success rate | `mec_valid_update_success_rate` | Complement of MEC valid-update failure | Positive form of MEC usefulness | Keep separate from radio PDR |
| V2V sidelink required Mbps p99 | `v2v_sidelink_required_mbps_p99` | p99 NR/V2V CPM plus NR background offered load converted to Mbps | V2V-side load KPI alongside CBR | Capacity denominator is the configured proxy |
| V2V required/capacity p99 | `v2v_sidelink_required_load_ratio_p99` | `v2v_sidelink_required_mbps / v2v_channel_capacity_mbps` | Shows V2V load pressure in bandwidth terms | Not a replacement for measured CBR |
| High-priority fresh redundancy >=2 rate | `receiver_high_fresh_redundancy_ge2_200ms_rate` | High-priority expected objects with at least two fresh recognition sources within 200 ms | Main high-priority redundancy KPI | Complements High ORR; not required for low-priority objects |
| High-priority RL >=1 / >=2 rate | `receiver_high_rl_ge1_rate`, `receiver_high_rl_ge2_rate` | High-priority receiver-side redundancy-level achievement | High-priority redundancy diagnostic | RL/RV remain supporting metrics, not primary ORR |

Recommended headline table:

| Method | ORR | High ORR | Low ORR | MEC update failure | MEC AoI p99 ms | MEC required DL p99 Mbps | MEC required/capacity p99 |
|---|---:|---:|---:|---:|---:|---:|---:|
| V2V only | | | | N/A | N/A | N/A | N/A |
| V2N2V only | | | | | | | |
| Hybrid | | | | | | | |
| Hybrid + RMR | | | | | | | |

Recommended high-priority safety table:

| Method | High ORR | High fresh redundancy >=2 200 ms | High RL mean | High RL >=1 | High RL >=2 | High RV mean |
|---|---:|---:|---:|---:|---:|---:|
| V2V only | | | | | | |
| V2N2V only | | | | | | |
| Hybrid | | | | | | |
| Hybrid + RMR | | | | | | |

Interpretation rule:

- Low-priority objects are mainly evaluated by ORR with TTL 0.2 s.
- High-priority objects should be evaluated by both High ORR and fresh
  redundancy achievement, because some redundancy is valuable for important
  information.
- Do not require all low-priority objects to have redundancy >=2; that would
  conflict with load reduction.

## MEC Load Metrics

| Metric | CSV name | Definition | Use | Do not say |
|---|---|---|---|---|
| MEC DL required Mbps | `mec_dl_required_mbps`, `mec_dl_required_mbps_*` | Required MEC downlink bytes in the observation window converted to Mbps | Shows actual demanded downlink load | Do not call it measured 5G throughput |
| MEC DL required load ratio | `mec_dl_required_load_ratio`, `mec_dl_required_load_ratio_*` | `mec_dl_required_mbps / mec_dl_capacity_mbps` | Shows overload magnitude above capacity | Do not clip it to 1 |
| MEC DL busy ratio / DLBR | `mec_dl_busy_ratio`, `mec_dl_busy_ratio_*` | Clipped downlink utilization proxy | Useful as utilization diagnostic | Do not use alone to explain overload magnitude |
| MEC forwarded packets | `mec_forwarded_packets` | Number of MEC downlink receiver-wise forwarding transmissions | Fanout/load accounting | One uplink CPM can produce many downlink transmissions |
| MEC forwarded bytes | `mec_forwarded_bytes` | Downlink bytes after MEC fanout and packet-size accounting | Explains capacity pressure | Interpret with packet size/RMR mode |
| MEC DL capacity | `mec_dl_capacity_mbps` | Configured lightweight MEC downlink service-rate proxy | Denominator for load ratio | Not a measured highway operator throughput |

Preferred wording:

- "MEC required downlink load exceeded the configured service-rate proxy."
- "Hybrid+RMR reduced the MEC required DL p99 load."
- "DLBR is saturated/clipped, so required Mbps is needed to show overload size."

Avoid:

- "MEC DL packet loss was 100%" when the metric is update failure or AoI failure.
- "DLBR increased by 600%" because DLBR is a clipped ratio.

## V2V Load Metrics

CBR remains the primary V2V congestion metric.  The V2V required-load metrics
are bandwidth-style diagnostics comparable to the MEC required DL load, not
claims about measured NR-V2X PC5 physical capacity.  The default 6 Mbps
denominator is a conservative reference value for normalization; conclusions
about V2V congestion should be based on CBR, loss/update failure, and ORR.

| Metric | CSV name | Definition | Use | Caution |
|---|---|---|---|---|
| V2V channel capacity proxy | `v2v_channel_capacity_mbps` | Configured `traffic-flow-channel-rate-mbps`; default 6 Mbps | Denominator for V2V load ratio | Diagnostic normalization only; not a measured NR-V2X PHY capacity |
| V2V sidelink required Mbps | `v2v_sidelink_required_mbps`, `v2v_sidelink_required_mbps_*` | NR CPM bytes plus NR background bytes in the observation window converted to Mbps | Explains V2V offered load in bandwidth terms | Use alongside CBR, not instead of CBR |
| V2V sidelink CPM Mbps | `v2v_sidelink_cpm_mbps` | CPM-only part of V2V offered load | Shows RMR effect on V2V payload | Observation-only |
| V2V sidelink BG Mbps | `v2v_sidelink_bg_mbps` | NR background part of V2V offered load | Separates artificial background from CPM load | Observation-only |
| V2V required/capacity ratio | `v2v_sidelink_required_load_ratio`, `v2v_sidelink_required_load_ratio_*` | Required V2V Mbps divided by the channel capacity proxy | Bandwidth-style V2V load pressure | CBR still captures channel occupancy/interference effects |

## Recognition and Freshness Metrics

| Metric | CSV name | Meaning | Main use | Caution |
|---|---|---|---|---|
| AoI | `mec_aoi_*`, `v2v_aoi_*` | `now - message generation time` | Freshness measurement | Must be generation-time based |
| AoI <= threshold rate | `mec_aoi_le_200_ms_rate` etc. | Fraction of received updates satisfying threshold | Fresh update success among received samples | Threshold-dependent |
| AoI violation rate | `mec_aoi_violation_rate_200ms` etc. | Complement of AoI achievement among received updates | Explains stale received updates | Not radio loss |
| TTL violation rate | `ttl_violation_rate` | Recognition state not refreshed within TTL | Application-level recognition continuity | Not packet lifetime |
| Never received rate | `never_received_rate` | Expected object never observed by the evaluating vehicle | Diagnostic for missing recognition | May mix range, route, policy, and radio effects |

Final TTL policy:

- High-priority recognition TTL: 0.2 s.
- Low-priority recognition TTL: 0.2 s.
- Older results with low-priority TTL 0.5 s are diagnostic, not final headline results.

## Radio Reliability Metrics

| Metric | CSV name | Meaning | Use | Caution |
|---|---|---|---|---|
| V2V radio loss rate | `v2v_radio_loss_rate` | Sidelink packet-level loss proxy | Shows V2V channel difficulty | Does not directly equal ORR |
| MEC UL final radio loss | `mec_ul_final_loss_rate` | Uplink packets still lost after abstract retransmission recovery | Uu radio reliability diagnostic | Lightweight abstraction, not full 3GPP PHY/MAC |
| MEC DL final radio loss | `mec_dl_final_loss_rate` | Downlink packets still lost after abstract retransmission recovery | Uu radio reliability diagnostic | Separate from AoI/update failure |
| MEC radio final loss | `mec_radio_final_loss_rate` | Combined final Uu radio loss diagnostic | Radio reliability summary | Usually much smaller than update failure |
| Packet loss rate | `packet_loss_rate` | Application/evaluation packet success aggregate | Broad comparison diagnostic | Can mix route/application expectations; define context before using |

Preferred wording:

- "MEC valid-update failure is dominated by freshness/load, not final radio loss."
- "V2V radio loss remains high, but Hybrid can recover recognition through fresh MEC updates."

## RMR and Payload Metrics

| Metric | CSV name | Meaning | Use | Caution |
|---|---|---|---|---|
| NR RMR deleted total | `nr_rmr_deleted_total` | Objects removed from NR sidelink CPM path | Measures V2V CPM object RMR | Can reduce useful redundancy |
| NR CPM size total | `nr_cpm_size_bytes_total` | Total serialized CPM bytes on NR path | Payload reduction evidence | Compare under same run duration/vehicle count |
| MEC RMR deleted total | `mec_rmr_deleted_total` | Legacy combined MEC RMR deletion count | Backward-compatible diagnostic | Ambiguous in old results |
| MEC payload deleted total | `mec_rmr_payload_deleted_total` | Objects accounted as removed from MEC payload | Separates payload RMR from fanout RMR | New results only |
| MEC payload deleted high/low | `mec_rmr_payload_deleted_high_total`, `mec_rmr_payload_deleted_low_total` | Importance-class split of MEC object-payload RMR accounting | Confirms whether payload reduction is mostly low-priority | Accounting model, not full ID-level deletion |
| MEC fanout deleted total | `mec_rmr_fanout_deleted_total` | Receiver-wise MEC forwarding targets suppressed | Separates fanout suppression from payload RMR | New results only |
| MEC CPM size total | `mec_cpm_size_bytes_total` | Total MEC CPM bytes after accounting | MEC payload/load explanation | Interpret with MEC RMR mode |
| MEC cross-priority delivery rate | `mec_receiver_low_from_source_high_rate` | Share of low-priority MEC deliveries whose sender CPM also had at least one high-priority receiver | Tests whether high-priority MEC forwarding can also improve wider low-priority awareness | `source_high` is batch-level, not sender-vehicle subjective priority |
| MEC cross-priority delivery counts | `mec_receiver_{high,low}_from_source_{high,low}_*` | Receiver-priority x sender-CPM-has-high-priority-receiver cross table | Supports discussion of V2N2V wide-area complement behavior | New results only |

Rule:

- For new results, do not use `mec_rmr_deleted_total` alone to claim payload
  reduction or fanout suppression.  Use the split payload/fanout counters.

## Route-Control Diagnostics

Hybrid+RMR results include both RMR and route-control behavior.  Treat these as
separate explanatory axes.

| Metric/config | Source | Meaning | Use | Caution |
|---|---|---|---|---|
| Dual route count | `dual_route_count` in `observation.csv` or route logs | Number of vehicles currently using both NR/V2V and MEC/V2N2V routes | Confirms hybrid activation | Does not by itself prove useful recognition |
| High dual TX setting | `hybrid-high-dual-tx`, `stdout.txt`, `run_config.txt` | Whether high-priority congested CPMs may use both routes | Sensitivity knob for high-priority ORR/RL | Can also increase MEC DL load |
| MEC recovery policy | `mec-recovery-policy` | Whether MEC is used as staged complement or offload path | Explains route role | Compare only under same RMR/TTL |
| Low-priority MEC forwarding cap | `mec-adaptive-low-max-prob` | Upper bound of low-priority V2N2V forwarding probability | Sensitivity knob for load/coverage tradeoff | Lower values may reduce low-priority ORR |
| CBR thresholds | `switch-cbr`, `release-cbr`, `hybrid-cbr-max` | Trigger points for route activation/deactivation | Sensitivity knob for when MEC complement starts | Changes both load and recognition opportunity |

Interpretation rule:

- Hybrid is not simple V2V+V2N2V bandwidth addition.  It combines V2V broadcast,
  MEC-assisted complement, high-priority route diversity, and load control.
- When arguing that RMR helped, also state whether route-control settings were
  held constant or varied.

## Auxiliary Diagnostics: RL and RV

RL and RV are useful, but they are not headline KPIs yet.

| Metric | CSV name | Meaning | Current role | Caution |
|---|---|---|---|---|
| Receiver RL mean/median/p90/p99 | `receiver_rl_*` | Number of valid recognition updates in the applicable TTL window | Explains recognition level and redundancy | Prefer distribution over mean alone |
| RL >= 1 / RL >= 2 rate | `receiver_rl_ge1_rate`, `receiver_rl_ge2_rate` | `RL>=1` is recognized; `RL>=2` is redundant recognition | RMR side-effect diagnosis | Compare with ORR only using the same priority class |
| High RL >= 1 / >= 2 rate | `receiver_high_rl_ge1_rate`, `receiver_high_rl_ge2_rate` | High-priority recognition and redundant-recognition rates | Main high-priority redundancy diagnostic | `RL>=1` should track high-priority ORR |
| High RL histogram | `receiver_high_rl_eq0_rate`, `receiver_high_rl_eq1_rate`, `receiver_high_rl_eq2_rate`, `receiver_high_rl_ge3_rate` | High-priority distribution of missed, single, double, and 3+ recognition updates | Main high-priority redundancy diagnostic | Best way to explain RL shape |
| High fresh redundancy >=2 rate | `receiver_high_fresh_redundancy_ge2_200ms_rate` | High-priority objects with at least two fresh sources within 200 ms | Main high-priority redundancy KPI | Use alongside High ORR |
| RV | `receiver_rv_delooz_*` | Delooz-style redundancy value diagnostic | Helps discuss redundancy tradeoff | Use as supporting explanation only |
| High RV | `receiver_high_rv_delooz_mean` | High-priority-only RV mean | High-priority redundancy diagnostic | New results only |

Current interpretation:

- `RL=0` means not recognized, `RL=1` means one valid recognition update within
  TTL, `RL=2` means two valid updates, and `RL>=3` means three or more valid
  updates.
- `receiver_high_rl_ge1_rate` is the RL-form of high-priority ORR when the
  same denominator is used.  `receiver_high_rl_ge2_rate` and the histogram show
  redundancy beyond basic recognition.
- RL/RV are useful for future RL-aware RMR, but not the main result KPI.
- For the current thesis story, high-priority RL/RV and high fresh redundancy
  are more important than all-object RL/RV, because low-priority objects do not
  need the same redundancy target.

## Old vs New Result Rules

Old results before 2026-07-25:

- May use low-priority TTL 0.5 s.
- Do not include `run_config.txt`.
- Do not include split MEC payload/fanout RMR counters.
- Do not include summary-level `mec_dl_required_mbps_*`.

New results:

- Must use TTL high/low 0.2/0.2 for final headline comparisons.
- Must include `run_config.txt`.
- Should report `mec_dl_required_mbps_p99` and `mec_dl_required_load_ratio_p99`.
- Should use split RMR counters when explaining MEC RMR behavior.

## Quick Interpretation Checklist

Before using a metric in slides or paper text:

1. Is it a recognition KPI, radio metric, freshness metric, load metric, or diagnostic?
2. What is the denominator?
3. Is the value from `summary.csv` or computed from `observation.csv`?
4. Did the run use TTL 0.2/0.2?
5. Is MEC RMR mode explicit?
6. Is the result registered in `experiment_index.md`?
7. Is the claim registered in `claim_evidence_index.md`?

## Downlink Load Terminology

Use `DL PRB utilization`, `DL PRB usage ratio`, or `DL radio resource
utilization` in paper/slide text instead of the internal shorthand `DLBR`.

Mapping:

- `mec_dl_busy_ratio`: simulation proxy for actual downlink resource usage.
  Treat as the closest metric to gNB-side DL PRB utilization.
- `mec_dl_required_load_ratio`: required downlink traffic divided by configured
  downlink capacity.  This is useful for overload diagnosis, but it is closer to
  a demand/capacity estimate than to a directly observed vehicle-side metric.
- `mec_dl_predicted_load_ratio_for_rmr`: optional short-horizon prediction of
  MEC DL required/capacity pressure used by MEC-side RMR.  It is a control
  diagnostic, not a measured KPI.
- `mec_dl_offered_mbps`: offered MEC downlink traffic volume.  Use as a
  supporting traffic-volume metric.

Reality note:

- A vehicle cannot directly observe whole-cell downlink PRB utilization.
- A gNB/OAM/MEC-side controller can observe or obtain downlink PRB usage from
  scheduler/OAM counters and use it as a V2N2V-side load signal.
- 3GPP TS 28.552 defines DL Total PRB Usage as the percentage of downlink PRBs
  used during a measurement period; TS 28.554 also defines high-load KPIs based
  on PRB usage distribution.  Therefore, the research term `DL busy ratio`
  should be presented externally as a DL PRB utilization/load metric.
