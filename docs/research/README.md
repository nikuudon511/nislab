# Research Onboarding for Codex

## Purpose

This directory is the relational knowledge base for the V2V/V2N2V Hybrid+RMR
research.  When a Codex session is reset, read this file first to recover the
current research state.

## Read Order

1. `docs/research/progress_2026-07-21_2026-07-27.md`
2. `docs/research/progress_2026-07-28.md`
3. `src/automotive/examples/FIT2026_RFurumai/V2X_specification.md`
4. `docs/research/metric_definitions.md`
5. `docs/research/log_schema_index.md`
6. `docs/research/simulation_run_policy.md`
7. `docs/research/claim_evidence_index.md`
8. `docs/research/experiment_index.md`
9. `docs/research/resume_outline_hybrid_rmr_2026-07-26.md`
10. `docs/research/backup_plan.md`

## Current Main Claim

Hybrid V2V/V2N2V needs RMR to make the V2N2V path useful under dense highway
traffic.

Working explanation:

- V2V only suffers from sidelink loss/congestion.
- V2N2V only suffers from MEC downlink fanout load, tail delay, and AoI
  violations.
- Hybrid+RMR can reduce CPM payload/MEC downlink load, making MEC updates fresh
  enough to complement V2V.
- Current Hybrid+RMR also includes route control: high-priority information may
  be dual-transmitted over V2V and V2N2V, while low-priority V2N2V forwarding is
  probability-controlled.

## Current Caveat

Old headline results are diagnostic, not final.

Reasons:

- Some old runs used low-priority TTL `0.5 s`.
- Final headline runs must use TTL high/low `0.2/0.2`.
- Old runs may lack `run_config.txt`.
- Old `mec_rmr_deleted_total` may not distinguish payload deletion from fanout
  suppression.
- Hybrid+RMR should not be described as an RMR-only effect unless route-control
  settings are fixed or ablated.

## Clean-Run Requirements

A result can be used as main evidence only if:

- `run_config.txt` exists.
- TTL high/low is `0.2/0.2`.
- `MEC_IDEAL_RMR_MODE` is explicit.
- `summary.csv` includes MEC required DL load metrics.
- Logs include split MEC RMR counters.
- High-priority redundancy metrics are available.
- The result is registered in `experiment_index.md`.
- The related claim is registered in `claim_evidence_index.md`.

## Simulation Run Policy

Do not default every additional experiment to a long final-evaluation run.

- Use Smoke runs for command/log validation.
- Use Trend runs for parameter direction and bottleneck screening.
- Use Candidate runs before promoting a setting to final evidence.
- Use Final runs only for paper/slide headline evidence.

For current Trend screening, preserve the main load geometry whenever possible:
200 vehicles, BG500-style load, TTL `0.2/0.2`, `ORR_RANGE=200`, and
`MEC_FORWARD_RANGE=200`; shorten `SIM_TIME` before reducing vehicle count.

Default trend reproduction script:

```bash
bash src/automotive/examples/run_hybrid_nr_v2n2v_trend_reproduce.sh
```

Scaled trend script:

```bash
bash src/automotive/examples/run_hybrid_nr_v2n2v_scaled_trend.sh
```

The scaled profile defaults to `SCALE=0.25`, 50 vehicles, MEC capacity scaled
by `SCALE^2`, NR capacity proxy scaled by `SCALE`, and RMR delete budgets scaled
by `SCALE`.

## Next Runs

Priority order:

1. Hybrid + RMR
2. V2N2V only + RMR
3. V2V only + RMR
4. V2N2V only no RMR
5. Hybrid baseline no RMR
6. V2V only no RMR

All should use:

- TTL high/low `0.2/0.2`
- `ORR_RANGE=200`
- `MEC_FORWARD_RANGE=200`
- explicit `MEC_IDEAL_RMR_MODE`

## Preferred Metrics

Use these for headline comparisons:

- `object_recognition_rate`
- `high_priority_object_recognition_rate`
- `low_priority_object_recognition_rate`
- `mec_update_failure_rate`
- `mec_aoi_p99_ms`
- `mec_dl_required_mbps_p99`
- `mec_dl_required_load_ratio_p99`
- `receiver_high_fresh_redundancy_ge2_200ms_rate`
- `receiver_high_rl_ge1_rate`
- `receiver_high_rl_ge2_rate`
- `receiver_high_rl_eq0_rate`, `receiver_high_rl_eq1_rate`,
  `receiver_high_rl_eq2_rate`, `receiver_high_rl_ge3_rate`
- `dual_route_count` and `route.csv` for route-control diagnostics

Do not use clipped `mec_dl_busy_ratio` alone to explain overload magnitude.

## Before Making Any Claim

Check:

1. Which `EXP-*` supports it.
2. Whether the run is `main`, `diagnostic`, or `pending`.
3. Whether TTL is `0.2/0.2`.
4. Whether MEC RMR mode is explicit.
5. Whether the metric denominator is clear.
6. Whether counter evidence or caveats are recorded.
7. Whether route-control settings such as `hybrid-high-dual-tx` were fixed or
   are part of the comparison.

## Important Files

- Main implementation: `src/automotive/examples/v2v-hybrid-nr-v2n2v.cc`
- Main wrapper: `src/automotive/examples/run_hybrid_nr_v2n2v_evaluation.sh`
- Load sweep wrapper: `src/automotive/examples/run_hybrid_nr_v2n2v_load_sweep.sh`
- Stable assumptions: `src/automotive/examples/FIT2026_RFurumai/V2X_specification.md`
