# Experiment Index

Last updated: 2026-07-25 JST

This file is the shared experiment index for human review and Codex sessions.
Use it as the first lookup point before scanning `results/`.

Status meanings:

- `main`: current headline result.
- `diagnostic`: useful for interpretation, but not a clean headline result.
- `obsolete`: superseded by a cleaner run or metric definition.
- `invalid`: known configuration, logging, or implementation problem.
- `pending`: planned or needs rerun.

## Main Experiment Index

| ID | Date | Label | Result dir | Summary | Observation | Config | Key result abstract | Status |
|---|---|---|---|---|---|---|---|---|
| EXP-001 | 2026-07-11 | V2V only | `results/bg500_base3_seed30_100s_retry/no-control` | [`summary.csv`](../../results/bg500_base3_seed30_100s_retry/no-control/summary.csv) | [`observation.csv`](../../results/bg500_base3_seed30_100s_retry/no-control/observation.csv) | old/no config | ORR about 61.3%. Sidelink-only baseline; packet/radio loss remains high. | diagnostic |
| EXP-002 | 2026-07-16 | V2N2V only | `results/bg500_v2n2v_only_fixed_seed30_100s_retry/v2n2v-only` | [`summary.csv`](../../results/bg500_v2n2v_only_fixed_seed30_100s_retry/v2n2v-only/summary.csv) | [`observation.csv`](../../results/bg500_v2n2v_only_fixed_seed30_100s_retry/v2n2v-only/observation.csv) | old/no config | ORR about 13.9%. MEC update failure about 100%, AoI p99 about 84 s, required DL p99 about 667 Mbps from observation. | diagnostic |
| EXP-003 | 2026-07-13 | Hybrid baseline | `results/bg500_hybrid_fixed_seed30_100s_retry/v2n2v-adaptive-probability` | [`summary.csv`](../../results/bg500_hybrid_fixed_seed30_100s_retry/v2n2v-adaptive-probability/summary.csv) | [`observation.csv`](../../results/bg500_hybrid_fixed_seed30_100s_retry/v2n2v-adaptive-probability/observation.csv) | old/no config | ORR about 69.7%. V2N2V improves over V2V only, but MEC update failure and AoI tail remain severe. | diagnostic |
| EXP-004 | 2026-07-18 | Hybrid + fixed RMR 10/20/40 | `results/bg500_hybrid_rmr_delooz_seed30_100s/v2n2v-adaptive-probability` | [`summary.csv`](../../results/bg500_hybrid_rmr_delooz_seed30_100s/v2n2v-adaptive-probability/summary.csv) | [`observation.csv`](../../results/bg500_hybrid_rmr_delooz_seed30_100s/v2n2v-adaptive-probability/observation.csv) | old/no config | Old best result: ORR about 98.6%, MEC update failure about 14.4%, AoI p99 about 249 ms, required DL p99 about 82.4 Mbps from observation. TTL low was 0.5 s. | diagnostic |
| EXP-005 | 2026-07-18 | V2V only + RMR | `results/bg500_v2v_only_rmr_fixed_seed30_100s/no-control` | [`summary.csv`](../../results/bg500_v2v_only_rmr_fixed_seed30_100s/no-control/summary.csv) | [`observation.csv`](../../results/bg500_v2v_only_rmr_fixed_seed30_100s/no-control/observation.csv) | old/no config | ORR about 54.8%, worse than V2V only. Shows RMR on V2V alone does not explain Hybrid+RMR gain. | diagnostic |
| EXP-006 | 2026-07-20 | Hybrid + longtail MEC RMR | `results/bg500_hybrid_light_mec_rmr_longtail_seed30_100s/v2n2v-adaptive-probability` | [`summary.csv`](../../results/bg500_hybrid_light_mec_rmr_longtail_seed30_100s/v2n2v-adaptive-probability/summary.csv) | [`observation.csv`](../../results/bg500_hybrid_light_mec_rmr_longtail_seed30_100s/v2n2v-adaptive-probability/observation.csv) | old/no config | ORR about 80.9%. MEC AoI improves, but update failure and low-priority ORR remain worse than old fixed RMR. | diagnostic |
| EXP-007 | 2026-07-21 | Hybrid + fixed RMR strict TTL | `results/bg500_hybrid_fixed_rmr_strictttl_seed30_100s/v2n2v-adaptive-probability` | [`summary.csv`](../../results/bg500_hybrid_fixed_rmr_strictttl_seed30_100s/v2n2v-adaptive-probability/summary.csv) | [`observation.csv`](../../results/bg500_hybrid_fixed_rmr_strictttl_seed30_100s/v2n2v-adaptive-probability/observation.csv) | old/no config | ORR about 70.0%, high ORR about 90.0%, low ORR about 45.3%. Shows low-priority ORR is TTL-sensitive. | diagnostic |
| EXP-008 | 2026-07-22 | Hybrid + hard guarded RMR | `results/bg500_hybrid_guarded_rmr_seed30_100s/v2n2v-adaptive-probability` | [`summary.csv`](../../results/bg500_hybrid_guarded_rmr_seed30_100s/v2n2v-adaptive-probability/summary.csv) | [`observation.csv`](../../results/bg500_hybrid_guarded_rmr_seed30_100s/v2n2v-adaptive-probability/observation.csv) | old/no config | ORR about 73.1%. Hard guard protected too much, increased MEC bottleneck, and degraded AoI/ORR. | diagnostic |
| EXP-009 | 2026-07-24 | Hybrid + RMR 5/10/20 | `results/bg500_hybrid_rmr_5_10_20_seed30_100s/v2n2v-adaptive-probability` | [`summary.csv`](../../results/bg500_hybrid_rmr_5_10_20_seed30_100s/v2n2v-adaptive-probability/summary.csv) | [`observation.csv`](../../results/bg500_hybrid_rmr_5_10_20_seed30_100s/v2n2v-adaptive-probability/observation.csv) | old/no config | ORR about 90.4%. Lower required DL p99 than old fixed RMR, but low-priority ORR drops strongly. | diagnostic |
| EXP-010 | 2026-07-25 | Hybrid object RMR 5/10/20 rerun | `results/bg500_hybrid_object_rmr_5_10_20_seed30_100s/v2n2v-adaptive-probability` | [`summary.csv`](../../results/bg500_hybrid_object_rmr_5_10_20_seed30_100s/v2n2v-adaptive-probability/summary.csv) | [`observation.csv`](../../results/bg500_hybrid_object_rmr_5_10_20_seed30_100s/v2n2v-adaptive-probability/observation.csv) | old/no config | Same summary as EXP-009. Useful as a check, but not a separate method claim. | diagnostic |
| EXP-011 | TBD | TTL 0.2/0.2 main comparison rerun | `results/...` | TBD | TBD | `run_config.txt` required | Needed because old headline results used low-priority TTL 0.5 s or ambiguous settings. | pending |

## Figure Index

| Figure ID | Figure / CSV | Source experiments | Message | Safe to use? |
|---|---|---|---|---|
| FIG-001 | [`slide_01_main_comparison.png`](../../results/slide_progress_figures/slide_01_main_comparison.png) | EXP-001 to EXP-004 | Main ORR comparison across V2V, V2N2V, Hybrid, Hybrid+RMR. | diagnostic until TTL rerun |
| FIG-002 | [`slide_02_orr_timeseries.png`](../../results/slide_progress_figures/slide_02_orr_timeseries.png) | EXP-001 to EXP-004 | ORR time-series comparison. | diagnostic |
| FIG-003 | [`slide_06_orr_cbr_mec_bottleneck.png`](../../results/slide_progress_figures/slide_06_orr_cbr_mec_bottleneck.png) | EXP-003, EXP-004 | Improvement is tied to MEC bottleneck relief, not mean CBR reduction. | diagnostic |
| FIG-004 | [`slide_08_mec_side_bottleneck_rmr.png`](../../results/slide_progress_figures/slide_08_mec_side_bottleneck_rmr.png) | EXP-003, EXP-004 | Hybrid+RMR improves MEC AoI/update failure versus Hybrid baseline. | diagnostic |
| FIG-005 | [`slide_09_v2v_rmr_hybrid_rmr_comparison.png`](../../results/slide_progress_figures/slide_09_v2v_rmr_hybrid_rmr_comparison.png) | EXP-001, EXP-004, EXP-005 | V2V+RMR does not reproduce Hybrid+RMR benefit. | diagnostic |
| FIG-006 | [`slide_13_rmr_ablation_metrics.csv`](../../results/slide_progress_figures/slide_13_rmr_ablation_metrics.csv), [`slide_13_rmr_design_ablation.png`](../../results/slide_progress_figures/slide_13_rmr_design_ablation.png) | EXP-004, EXP-007, EXP-006, EXP-008 | RMR design ablation across fixed, strict TTL, longtail, and hard guard. | diagnostic |
| FIG-007 | [`slide_17a_main_methods_detail_table.csv`](../../results/slide_progress_figures/slide_17a_main_methods_detail_table.csv), [`slide_17a_main_methods_detail_table.png`](../../results/slide_progress_figures/slide_17a_main_methods_detail_table.png) | main old methods | Detailed old main-method metrics. | diagnostic |
| FIG-008 | [`slide_17a_main_methods_detail_table_compact_orr_offered_load.csv`](../../results/slide_progress_figures/slide_17a_main_methods_detail_table_compact_orr_offered_load.csv), [`slide_17a_main_methods_detail_table_compact_orr_offered_load.png`](../../results/slide_progress_figures/slide_17a_main_methods_detail_table_compact_orr_offered_load.png) | main old methods | Compact ORR and offered-load view. | diagnostic |
| FIG-009 | [`slide_18_receiver_rl_rv_timeseries.png`](../../results/slide_progress_figures/slide_18_receiver_rl_rv_timeseries.png), [`slide_18_rl_rv_summary_table.csv`](../../results/slide_progress_figures/slide_18_rl_rv_summary_table.csv) | RMR diagnostics | RL/RV redundancy diagnostics. | diagnostic |
| FIG-010 | [`slide_21_hybrid_fixed_rmr_orr_timeseries.png`](../../results/slide_progress_figures/slide_21_hybrid_fixed_rmr_orr_timeseries.png) | EXP-004 | Old best fixed-RMR ORR time-series. | diagnostic |

## Metric Notes

- Prefer `mec_dl_required_mbps_p99` for MEC downlink load in new summaries.
- Treat `mec_dl_busy_ratio_*` as clipped DLBR/utilization; it hides overload magnitude once load exceeds capacity.
- Do not treat `mec_update_failure_rate` as physical radio packet loss.
- Keep radio final loss, AoI violation, TTL violation, and never received as separate failure categories.
- For old results, `mec_dl_required_mbps_p99` may need to be computed from `observation.csv` because the summary column was added later.

## Known Ambiguities

- Old results before 2026-07-25 do not include `run_config.txt`.
- Old results may have low-priority TTL 0.5 s even if newer code defaults to 0.2 s.
- Old `mec_rmr_deleted_total` can be ambiguous. New results should use split columns:
  - `mec_rmr_payload_deleted_total`
  - `mec_rmr_fanout_deleted_total`
- EXP-009 and EXP-010 have identical summary results and should not be presented as independent evidence.

## Maintenance Rules

- Add every result used in a slide, paper draft, or supervisor discussion.
- Use one row per result directory.
- Link `summary.csv`, `observation.csv`, and `run_config.txt` when available.
- Add a one-sentence result abstract for every experiment.
- Mark old or ambiguous results as `diagnostic`, not `main`.
- Promote a result to `main` only when TTL, RMR mode, and metric definitions are traceable.
