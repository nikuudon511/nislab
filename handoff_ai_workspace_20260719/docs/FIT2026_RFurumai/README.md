# FIT2026_RFurumai Document Index

Last updated: 2026-07-19 00:20 JST

This directory stores the research notes, evaluation specifications, and run
records for the V2V/V2N2V hybrid cooperative perception study.

## Start Here

| Purpose | File |
|---|---|
| Current progress-report draft and main results | [0727進捗.md](../../../../0727進捗.md) |
| Main simulation and communication specification | [V2X_specification.md](V2X_specification.md) |
| V2N/V2N2V parameter literature survey | [V2X_V2N通信パラメータの文献調査.md](V2X_V2N通信パラメータの文献調査.md) |
| RMR + Hybrid BG500 run record and interpretation | [rmr_hybrid_bg500_run_spec.md](rmr_hybrid_bg500_run_spec.md) |

## Result Summaries

| Topic | File |
|---|---|
| V2N2V-only vs Hybrid preliminary report | [v2n2v_only_vs_hybrid_prelim_report.md](v2n2v_only_vs_hybrid_prelim_report.md) |
| RMR findings before the current BG500 evaluation | [2026-07-05_rmr_findings.md](2026-07-05_rmr_findings.md) |
| Dense V2N2V smoke progress | [2026-07-08_v2n2v_dense_smoke_progress.md](2026-07-08_v2n2v_dense_smoke_progress.md) |

## Planning And Review Notes

| Topic | File |
|---|---|
| Execution plan | [execution_plan.md](execution_plan.md) |
| Todo list | [ToDo.md](ToDo.md) |
| Codex revision items | [FIT2026_codex_revision_items.md](FIT2026_codex_revision_items.md) |
| Feedback notes | [feedback.md](feedback.md) |
| Deep research notes | [DeepResearch.md](DeepResearch.md) |

## Current Result Paths

The main 2026-07 progress comparison uses:

| Case | Summary path |
|---|---|
| V2V only | `results/bg500_base3_seed30_100s_retry/no-control/summary.csv` |
| V2V only + RMR | `results/bg500_v2v_only_rmr_fixed_seed30_100s/no-control/summary.csv` |
| V2N2V only | `results/bg500_v2n2v_only_fixed_seed30_100s_retry/v2n2v-only/summary.csv` |
| Hybrid baseline | `results/bg500_hybrid_baseline_delooz_seed30_100s/v2n2v-adaptive-probability/summary.csv` |
| Hybrid + RMR | `results/bg500_hybrid_rmr_delooz_seed30_100s/v2n2v-adaptive-probability/summary.csv` |

Slide figures are under:

```text
results/slide_progress_figures/
```

Important figures:

| Figure | Use |
|---|---|
| `slide_12_orr_v2v_cbr_v2n2v_dlbusy.png` | ORR with V2V CBR and V2N2V MEC DL busy |
| `slide_10_v2v_rmr_vs_hybrid_rmr_key.png` | V2V+RMR vs Hybrid+RMR comparison |
| `slide_08_mec_side_bottleneck_rmr.png` | Hybrid baseline vs Hybrid+RMR MEC bottleneck |
| `slide_04_rl_rv_interpretation.png` | RL/RV time-series interpretation |

## Current Open Checks

- `V2N2V only + RMR / BG500 / seed30 / 100 s` is still needed.
- `Hybrid + RMR / seed31 / BG500 / 100 s` is useful for seed sensitivity.
- Prediction should be treated as a later extension unless the static RMR story is complete.
