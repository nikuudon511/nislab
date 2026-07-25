# Claim Evidence Index

Last updated: 2026-07-25 JST

This file links research claims to experiments, metrics, counter evidence, and
paper-writing status.  Use `experiment_index.md` for result files and figures;
use this file for what those results support.

Status meanings:

- `main`: usable as a current paper/report claim.
- `diagnostic`: useful, but depends on old settings or needs cleaner rerun.
- `pending`: plausible claim, but evidence is incomplete.
- `rejected`: tested and not supported.

Confidence meanings:

- `high`: supported by clean runs and stable across key sensitivity checks.
- `medium`: supported by current results but has known caveats.
- `low`: early indication only.

## Claim Table

| Claim ID | Claim | Evidence experiments | Key evidence | Counter evidence / caveat | Confidence | Paper section | Status |
|---|---|---|---|---|---|---|---|
| CLM-001 | Hybrid+RMR improves ORR over Hybrid baseline by making MEC updates fresh enough to complement V2V. | EXP-003, EXP-004 | ORR about 69.7 -> 98.6; MEC update failure about 79.5 -> 14.4; MEC AoI p99 about 25579 ms -> 249 ms. | Old best result used low-priority TTL 0.5 s; rerun with TTL 0.2/0.2 is required before treating this as final. | medium | Evaluation main result | diagnostic |
| CLM-002 | The Hybrid+RMR gain should not be explained as mean CBR reduction. | EXP-003, EXP-004 | Mean V2V CBR is almost unchanged, while MEC AoI/update success and MEC DL load improve. | Need regenerated summary with `mec_dl_required_mbps_p99` for clean final wording. | medium | Evaluation / Discussion | diagnostic |
| CLM-003 | V2V+RMR alone is not sufficient under the current dense highway setting. | EXP-001, EXP-005 | V2V only ORR about 61.3; V2V+RMR ORR about 54.8, despite lower packet/radio loss. | Only seed30 old run; should not overgeneralize beyond this setting. | medium | Ablation / Discussion | diagnostic |
| CLM-004 | V2N2V only is limited by MEC downlink fanout/required DL load and tail AoI. | EXP-002 | ORR about 13.9; MEC update failure about 100; AoI p99 about 84 s; required DL p99 about 667 Mbps vs about 90.5 Mbps capacity. | Old result lacks `run_config.txt`; new run should record required DL load directly in summary. | medium | Baseline evaluation | diagnostic |
| CLM-005 | MEC downlink required load is more informative than clipped DLBR for explaining overload magnitude. | EXP-002, EXP-004, EXP-009 | V2N2V only required DL p99 is about 667 Mbps, while Hybrid+RMR old best is about 82.4 Mbps; DLBR clips overload to 1.0. | New metric columns were added after these old runs, so old values are computed from `observation.csv`. | high | Metrics / Evaluation | main |
| CLM-006 | Hard Guarded RMR can degrade performance if it protects too many high-priority MEC fanout pairs. | EXP-004, EXP-008 | Hard guard ORR about 73.1 vs fixed RMR 98.6; MEC AoI p99 worsens to about 12909 ms. | Old TTL caveat remains; still useful as a design warning. | medium | RMR design discussion | diagnostic |
| CLM-007 | Low-priority TTL 0.5 s can make low-priority ORR too permissive for highway-speed evaluation. | EXP-004, EXP-007 | Strict TTL run reduces low ORR strongly; 0.5 s can exceed 10 m position error at highway speed. | Strict TTL run used 0.1/0.2, while final policy is 0.2/0.2; needs clean rerun. | medium | Metric definition / Sensitivity | diagnostic |
| CLM-008 | Future headline results must use unified high/low recognition TTL 0.2/0.2. | Specification, EXP-011 pending | Specification and code defaults now use 0.2/0.2; wrappers updated to avoid low=0.5 override. | Existing headline figures are old/diagnostic until rerun. | high | Evaluation setup | main |
| CLM-009 | Old `mec_rmr_deleted_total` is insufficient to distinguish payload reduction from fanout suppression. | EXP-009, EXP-010, code review | New logs split `mec_rmr_payload_deleted_total` and `mec_rmr_fanout_deleted_total`. | Requires new runs to populate the split columns. | high | Implementation / Threats to validity | main |
| CLM-010 | Low-priority objects can be evaluated mainly by ORR, while high-priority objects should also satisfy fresh redundancy targets. | Metric design, new logs pending | New metrics track `receiver_high_fresh_redundancy_ge2_200ms_rate`, high RL>=1/2, and high RV. | Requires new reruns to populate high-priority redundancy columns. | medium | Metrics / Evaluation | pending |

## Claim Details

### CLM-001: Hybrid+RMR Makes MEC Updates Fresh

Working explanation:

- Hybrid baseline adds V2N2V, but many MEC updates miss AoI/TTL freshness.
- Fixed RMR reduces CPM payload and MEC downlink bytes, which reduces capacity delay and AoI tail.
- Once MEC updates are fresh enough, they complement V2V losses and raise ORR.

Finalization requirement:

- Rerun main comparison with TTL 0.2/0.2.
- Confirm the improvement still appears using `mec_dl_required_mbps_p99`.

### CLM-002: Not Mean CBR

Working explanation:

- Average V2V CBR stays almost the same between Hybrid baseline and Hybrid+RMR.
- MEC-side freshness metrics change strongly.
- Therefore the claim should focus on MEC required DL load, capacity delay, AoI, and valid update success.

### CLM-003: V2V+RMR Is Not Enough

Working explanation:

- V2V+RMR reduces packet/radio loss but can also remove redundancy needed for object recognition.
- In the current dense highway run, ORR decreases instead of improving.
- This supports the need for Hybrid complementarity rather than V2V-only RMR.

### CLM-004: V2N2V Only Bottleneck

Working explanation:

- V2N2V only sends many receiver-wise downlink transmissions through MEC.
- Required DL load can exceed the assumed 90.468 Mbps capacity by several times.
- Queueing and AoI tail make many received updates stale or unusable.

### CLM-005: Required DL Load Metric

Working explanation:

- DLBR is clipped to 1.0 when offered load exceeds capacity.
- `mec_dl_required_mbps` preserves overload magnitude.
- `mec_dl_required_load_ratio` is required load divided by capacity and is not clipped.

Use in paper:

- Prefer p99 required DL Mbps and p99 required/capacity ratio when explaining MEC bottlenecks.
- Use DLBR only as a utilization diagnostic.

### CLM-006: Hard Guard Can Be Harmful

Working explanation:

- Protecting every high-priority item preserves candidates but can overload downlink.
- Overload makes updates stale, so overall recognition can degrade.
- The guard should be soft: absolute protection only for very near or TTC-critical items.

### CLM-007: TTL Must Be Strict Enough

Working explanation:

- Highway-speed motion makes long recognition TTL risky.
- A 0.5 s stale state can imply more than 10 m of position error.
- A unified 0.2 s TTL tolerates roughly one missed update or moderate delay but rejects older state.

### CLM-008: Unified TTL Required

Working explanation:

- Different TTLs by priority bias low-priority ORR.
- The specification, C++ defaults, and wrapper defaults were aligned to 0.2/0.2 on 2026-07-25.
- Old low=0.5 results should remain diagnostic until rerun.

### CLM-009: Split MEC RMR Counters

Working explanation:

- A single `mec_rmr_deleted_total` can hide whether the model deleted payload objects or suppressed fanout receivers.
- New columns separate these effects.
- This prevents future mismatch between implementation, plots, and claims.

### CLM-010: Priority-Aware Redundancy Evaluation

Working explanation:

- Low-priority objects do not necessarily need multiple fresh redundant updates;
  TTL-bounded ORR is usually sufficient for the current evaluation.
- High-priority objects should be evaluated more strictly: they should have high
  ORR and, when possible, at least two fresh recognition sources within 200 ms.
- This matches the design goal of reducing communication load while preserving
  redundancy for valuable or safety-relevant information.

Finalization requirement:

- Rerun main methods and compare:
  - `high_priority_object_recognition_rate`
  - `receiver_high_fresh_redundancy_ge2_200ms_rate`
  - `receiver_high_rl_ge1_rate`
  - `receiver_high_rl_ge2_rate`
  - `receiver_high_rv_delooz_mean`

## Paper-Writing Use

When drafting the paper:

1. Start from the claims marked `main`.
2. Use `diagnostic` claims for explanation or motivation only.
3. Promote a claim to `main` only after the linked experiments have clean TTL, explicit RMR mode, and traceable metric definitions.
4. Link every figure back to an `EXP-*` row in `experiment_index.md`.
5. For every claim, include at least one supporting metric and one caveat or sensitivity result.
