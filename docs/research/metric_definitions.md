# Metric Definitions

Last updated: 2026-07-25 JST

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
| Radio reliability | Separates physical/abstract radio losses from application failures | V2V radio loss, MEC UL/DL final radio loss |
| Freshness | Measures whether received updates are fresh enough | AoI violation, TTL violation |
| Diagnostics | Helps explain why RMR helped or hurt | CPM payload, RMR deleted counts, RL/RV |

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
| MEC fanout deleted total | `mec_rmr_fanout_deleted_total` | Receiver-wise MEC forwarding targets suppressed | Separates fanout suppression from payload RMR | New results only |
| MEC CPM size total | `mec_cpm_size_bytes_total` | Total MEC CPM bytes after accounting | MEC payload/load explanation | Interpret with MEC RMR mode |

Rule:

- For new results, do not use `mec_rmr_deleted_total` alone to claim payload
  reduction or fanout suppression.  Use the split payload/fanout counters.

## Auxiliary Diagnostics: RL and RV

RL and RV are useful, but they are not headline KPIs yet.

| Metric | CSV name | Meaning | Current role | Caution |
|---|---|---|---|---|
| Receiver RL mean/median | `receiver_rl_*` | Receiver-side redundancy level diagnostic | Explains whether recognition has redundant support | Not yet a final optimization objective |
| RL >= 1 / RL >= 2 rate | `receiver_rl_ge1_rate`, `receiver_rl_ge2_rate` | Fraction of samples with enough redundancy | RMR side-effect diagnosis | Lower RL can still coexist with high ORR |
| High RL >= 1 / >= 2 rate | `receiver_high_rl_ge1_rate`, `receiver_high_rl_ge2_rate` | High-priority-only RL achievement | Main high-priority redundancy diagnostic | More relevant than all-object RL for safety-critical discussion |
| High fresh redundancy >=2 rate | `receiver_high_fresh_redundancy_ge2_200ms_rate` | High-priority objects with at least two fresh sources within 200 ms | Main high-priority redundancy KPI | Use alongside High ORR |
| RV | `receiver_rv_delooz_*` | Delooz-style redundancy value diagnostic | Helps discuss redundancy tradeoff | Use as supporting explanation only |
| High RV | `receiver_high_rv_delooz_mean` | High-priority-only RV mean | High-priority redundancy diagnostic | New results only |

Current interpretation:

- Hybrid+RMR can have high ORR even when RL/RV is lower, because ORR measures
  whether an object was recognized within TTL, while RL/RV measure redundancy.
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
