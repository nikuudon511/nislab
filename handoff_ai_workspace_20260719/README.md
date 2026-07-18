# Handoff AI Workspace

Last updated: 2026-07-19 00:20 JST

This bundle is a lightweight explanation workspace for another person or AI.
It is not intended to rebuild or rerun ns-3/Van3Twin.  It contains only the
source files, notes, selected summaries, selected time-series metrics, and
figures needed to explain the current V2V/V2N2V Hybrid + RMR study.

## What To Read First

1. `0727進捗.md`
   - Main progress-report memo.
   - Includes the current story, key results, and unresolved checks.
2. `docs/FIT2026_RFurumai/README.md`
   - Index for the research notes and specifications.
3. `docs/FIT2026_RFurumai/V2X_specification.md`
   - Communication model and parameter definitions.
4. `docs/FIT2026_RFurumai/rmr_hybrid_bg500_run_spec.md`
   - RMR + Hybrid run record, interpretation, and next-run priorities.

## Main Result Files

Summary CSVs:

```text
results_selected/summaries/
```

Selected observation time-series:

```text
results_selected/observations_selected/
```

Slide figures:

```text
results_selected/plots/
```

Important plots:

| Plot | Meaning |
|---|---|
| `slide_12_orr_v2v_cbr_v2n2v_dlbusy.png` | ORR comparison with V2V CBR and V2N2V MEC downlink busy |
| `slide_10_v2v_rmr_vs_hybrid_rmr_key.png` | V2V+RMR vs Hybrid+RMR |
| `slide_08_mec_side_bottleneck_rmr.png` | MEC bottleneck improvement by RMR |
| `slide_04_rl_rv_interpretation.png` | RL/RV redundancy diagnostics |

## Main Source Files

```text
src/automotive/examples/v2v-hybrid-nr-v2n2v.cc
src/automotive/examples/run_hybrid_nr_v2n2v_evaluation.sh
```

These files are included for explanation of the implemented metrics and run
configuration.  This bundle does not include the full build tree.

## Current Main Findings

- V2V only reached ORR 61.3% under BG500.
- V2N2V only reached ORR 13.9% because MEC downlink fanout caused AoI/update failure.
- Hybrid baseline improved ORR to 69.7%, but MEC update failure remained high.
- Hybrid + RMR improved ORR to 98.6%.
- V2V only + RMR reduced packet/radio loss but lowered ORR to 54.8%.
- Therefore, in the current condition, RMR is not enough on V2V alone; it is most useful when it makes the V2N2V/MEC path fresh enough to complement V2V.

## Open Checks

- `V2N2V only + RMR / BG500 / seed30 / 100 s` is still needed.
- `Hybrid + RMR / seed31 / BG500 / 100 s` is useful for seed sensitivity.
- Prediction is currently positioned as a later extension for adaptive RMR strength, not the main result.
