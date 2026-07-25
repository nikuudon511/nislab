# Weekly Research Progress Template

Use this file as the weekly research note template.  Copy it to a dated file,
for example:

`docs/research/progress_2026-07-27_2026-08-02.md`

Keep one file per week.  Do not use one long rolling progress file.

Update `docs/research/experiment_index.md` whenever a result is used in slides,
discussion, or a paper draft.

Use `docs/research/metric_definitions.md` for KPI names, denominators, and
allowed wording.

Use `docs/research/log_schema_index.md` for CSV/log columns and clean-run
requirements.

## 1. Week Scope

| Item | Value |
|---|---|
| Week | YYYY-MM-DD to YYYY-MM-DD |
| Main objective | |
| Primary code file | `src/automotive/examples/v2v-hybrid-nr-v2n2v.cc` |
| Specification file | `src/automotive/examples/FIT2026_RFurumai/V2X_specification.md` |
| Previous progress note | |
| Experiment index | `docs/research/experiment_index.md` |
| Metric definitions | `docs/research/metric_definitions.md` |
| Log schema index | `docs/research/log_schema_index.md` |

## 2. Current Research Claim

Write the claim that this week's work supports or challenges.

- Claim:
- Why it matters:
- Current confidence: low / medium / high

## 3. Implementation Changes

List only changes that affect interpretation, metrics, routing, RMR, TTL, AoI,
packet accounting, or simulation behavior.

| Date | File | Change | Why it matters | Verification |
|---|---|---|---|---|
| YYYY-MM-DD | | | | |

Required notes:

- If a wrapper default changes, record it.
- If a C++ default changes, record it.
- If a metric name changes or a new metric is added, define its denominator.
- If a previous result becomes hard to compare, state why.

## 4. Experiment Registry

Every result directory used in slides or discussion should appear here.

| Run directory | Method label | Seed | TTL high/low | NR RMR | MEC RMR mode | MEC RMR delete | Main use | Status |
|---|---|---:|---|---|---|---|---|---|
| `results/...` | | | `0.2/0.2` | none / object | none / object-payload / fanout | `10/20/40` | main / diagnostic / obsolete | complete / partial / invalid |

Status rules:

- `main`: suitable for headline comparison.
- `diagnostic`: useful for explanation, not headline.
- `obsolete`: superseded by a cleaner run or changed metric definition.
- `invalid`: known configuration, logging, or implementation problem.

## 5. Key Metrics

Prefer metrics that directly support the claim.  Keep the table short.

| Method | ORR | High ORR | Low ORR | MEC update failure | MEC AoI p99 ms | MEC required DL p99 Mbps | MEC DL capacity Mbps | Notes |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| V2V only | | | | | | | | |
| V2N2V only | | | | | | | | |
| Hybrid | | | | | | | | |
| Hybrid + RMR | | | | | | | | |

Important metric rules:

- Use `mec_dl_required_mbps_*` for required MEC downlink load.
- Use `mec_dl_busy_ratio_*` only as a clipped utilization/DLBR diagnostic.
- Do not call `mec_update_failure_rate` a physical radio packet loss rate.
- Keep AoI violation, TTL violation, radio final loss, and never received separate.

## 6. Configuration Snapshot

Copy the relevant `run_config.txt` values here for headline runs.

```text
SIM_TIME=
SUMO_SEED=
MAX_COMMUNICATION_VEHICLES=
MOB_TRACE=
SUMO_CONFIG=
HIGH_PRIORITY_CPM_RECOGNITION_TTL=
LOW_PRIORITY_CPM_RECOGNITION_TTL=
REACTIVE_RMR=
RMR_DELETE=
MEC_IDEAL_RMR_MODE=
MEC_IDEAL_RMR_LONGTAIL=
MEC_FORWARD_RANGE=
MEC_IDEAL_AOI_FILTER_THRESHOLD_MS=
MEC_IDEAL_BANDWIDTH_MHZ=
MEC_IDEAL_SPECTRAL_EFFICIENCY_BPSHZ=
```

## 7. Result Interpretation

Write conclusions as evidence-backed bullets.

- What improved:
- What got worse:
- Main bottleneck:
- Evidence against alternative explanations:
- Remaining uncertainty:

## 8. Slide Usage

| Figure/table | Source result | Message | Safe to use? |
|---|---|---|---|
| `results/slide_progress_figures/...` | | | yes / no |

Rules:

- Record the source result directory for every slide figure.
- Mark figures based on old TTL or ambiguous RMR mode as diagnostic or obsolete.
- Do not mix summary metrics and observation final-row metrics without saying so.

## 9. Next Actions

Keep this to the next 3-5 concrete actions.

1. 
2. 
3. 

## 10. Codex Handoff

This section is for the next new Codex session.

Current state:

- 

Do not assume:

- 

Most relevant files:

- `src/automotive/examples/v2v-hybrid-nr-v2n2v.cc`
- `src/automotive/examples/run_hybrid_nr_v2n2v_evaluation.sh`
- `src/automotive/examples/FIT2026_RFurumai/V2X_specification.md`

Before suggesting a claim, Codex should check:

- The run's `run_config.txt`
- `summary.csv`
- `observation.csv` if p90/p99/time-series metrics are needed
- Whether the result used old TTL, ambiguous MEC RMR mode, or obsolete metric names
