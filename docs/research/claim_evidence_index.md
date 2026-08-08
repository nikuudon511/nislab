# Claim Evidence Index

Last updated: 2026-08-03 JST

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
| CLM-001 | Hybrid+RMR improves ORR over Hybrid baseline by making MEC updates fresh enough to complement V2V. | EXP-016, clean Hybrid+RMR run | Clean TTL 0.2/0.2 comparison: Hybrid no RMR ORR 63.3%, MEC fail 82.5%, AoI p99 25.6 s, DL req/cap p99 2.34; Hybrid+RMR ORR 97.8%, MEC fail 25.5%, AoI p99 248 ms, DL req/cap p99 0.90. | Seed30 single scenario; still needs seed sensitivity before final paper wording. | high | Evaluation main result | main |
| CLM-002 | The Hybrid+RMR gain should not be explained as mean CBR reduction. | EXP-016, clean Hybrid+RMR run | Mean V2V CBR is almost unchanged, while MEC AoI/update success and MEC DL load improve strongly. | This does not mean sidelink is unimportant; it means the observed Hybrid+RMR gain is mainly explained by MEC freshness/load relief. | high | Evaluation / Discussion | main |
| CLM-003 | V2V+RMR alone is not sufficient under the current dense highway setting. | EXP-012, EXP-017 | Clean TTL 0.2/0.2 V2V no RMR gives ORR 52.6%, while V2V+RMR gives ORR 44.3%. Both keep CBR around 0.802 and V2V req/cap p99 around 2.13. | Seed30 single scenario; V2V required/capacity ratio uses a diagnostic 6 Mbps denominator, so CBR/loss/ORR should carry the claim. | high | Ablation / Discussion | main |
| CLM-004 | V2N2V only is limited by MEC downlink fanout/required DL load and tail AoI. | EXP-011, EXP-013, EXP-014, EXP-018 | Clean TTL 0.2/0.2 V2N2V no RMR gives ORR 13.9%, MEC fail 100%, AoI p99 84.4 s, and DL req/cap p99 7.38. Object-payload RMR reduces DL req/cap p99 to 4.73 but ORR remains 13.9%. Object-limit 10 keeps DL req/cap p99 near capacity at 0.97 but ORR is only 27.7%. | Seed30 single scenario; still an idealized MEC/Uu model. Object-limit results are diagnostic ablations, not proposed final controls. | high | Baseline evaluation | main |
| CLM-005 | MEC downlink required load is more informative than clipped DLBR for explaining overload magnitude. | EXP-018, EXP-011, clean Hybrid+RMR run | V2N2V no RMR required DL p99 is 667.3 Mbps, 7.38x capacity; V2N2V+RMR is 427.9 Mbps, 4.73x; clean Hybrid+RMR is below capacity at req/cap p99 0.90. DLBR alone would clip the overload severity. | Seed30 single scenario; use this as metric justification, not as a channel-capacity generalization. | high | Metrics / Evaluation | main |
| CLM-006 | Hard Guarded RMR can degrade performance if it protects too many high-priority MEC fanout pairs. | EXP-004, EXP-008 | Hard guard ORR about 73.1 vs fixed RMR 98.6; MEC AoI p99 worsens to about 12909 ms. | Old TTL caveat remains; still useful as a design warning. | medium | RMR design discussion | diagnostic |
| CLM-007 | Low-priority TTL 0.5 s can make low-priority ORR too permissive for highway-speed evaluation. | EXP-004, EXP-007 | Strict TTL run reduces low ORR strongly; 0.5 s can exceed 10 m position error at highway speed. | Strict TTL run used 0.1/0.2, while final policy is 0.2/0.2; needs clean rerun. | medium | Metric definition / Sensitivity | diagnostic |
| CLM-008 | Future headline results must use unified high/low recognition TTL 0.2/0.2. | Specification, EXP-011 pending | Specification and code defaults now use 0.2/0.2; wrappers updated to avoid low=0.5 override. | Existing headline figures are old/diagnostic until rerun. | high | Evaluation setup | main |
| CLM-009 | Old `mec_rmr_deleted_total` is insufficient to distinguish payload reduction from fanout suppression. | EXP-009, EXP-010, code review | New logs split `mec_rmr_payload_deleted_total` and `mec_rmr_fanout_deleted_total`. | Requires new runs to populate the split columns. | high | Implementation / Threats to validity | main |
| CLM-010 | Low-priority objects can be evaluated mainly by ORR, while high-priority objects should also satisfy fresh redundancy targets. | Metric design, new logs pending | New metrics track `receiver_high_fresh_redundancy_ge2_200ms_rate`, high RL>=1/2, and high RV. | Requires new reruns to populate high-priority redundancy columns. | medium | Metrics / Evaluation | pending |
| CLM-011 | High-priority dual transmission is an important part of the current Hybrid+RMR mechanism. | EXP-015, clean Hybrid+RMR run | Disabling high-priority dual TX drops ORR 97.78 -> 87.23, high ORR 98.78 -> 90.45, high RL>=2 9.16 -> 2.13, and worsens MEC AoI p99 248 ms -> 4.79 s. | Seed30 single scenario; this supports the current design but does not prove dual TX is globally optimal. | medium | Ablation / Discussion | main |

## Claim Details

### CLM-001: Hybrid+RMR Makes MEC Updates Fresh

Working explanation:

- Hybrid baseline adds V2N2V, but many MEC updates miss AoI/TTL freshness.
- Fixed RMR reduces CPM payload and MEC downlink bytes, which reduces capacity delay and AoI tail.
- Once MEC updates are fresh enough, they complement V2V losses and raise ORR.
- Clean TTL 0.2/0.2 Hybrid no RMR is a stronger baseline than the old Hybrid run:
  it still drops to ORR 63.30% because DL req/cap p99 reaches 2.34 and MEC AoI
  p99 reaches 25.6 s.
- Clean Hybrid+RMR keeps DL req/cap p99 below capacity at 0.90 and AoI p99 at
  248 ms, raising ORR to 97.78%.
- High RL>=2 is higher without RMR (25.32%) than with RMR (9.16%), but the
  updates are stale. This shows repeated update count alone is not sufficient;
  freshness and capacity feasibility are required.

Finalization requirement:

- Add seed sensitivity if this becomes a final paper claim.

### CLM-002: Not Mean CBR

Working explanation:

- Average V2V CBR stays almost the same between Hybrid baseline and Hybrid+RMR.
- MEC-side freshness metrics change strongly.
- Therefore the claim should focus on MEC required DL load, capacity delay, AoI, and valid update success.
- In the clean no-RMR baseline, mean CBR remains about 0.802, close to
  Hybrid+RMR, while MEC fail and AoI tail are much worse. The main improvement
  is therefore not a mean-CBR reduction story.

### CLM-003: V2V+RMR Is Not Enough

Working explanation:

- V2V+RMR reduces packet/radio loss but can also remove redundancy needed for object recognition.
- In the current dense highway run, ORR decreases instead of improving.
- This supports the need for Hybrid complementarity rather than V2V-only RMR.
- Clean V2V no RMR gives ORR 52.64%, high ORR 62.33%, low ORR 39.36%.
  Clean V2V+RMR gives ORR 44.25%, high ORR 54.30%, low ORR 30.43%.
- Mean CBR is almost unchanged (0.802 vs 0.802) and required-load p99 ratio is
  the same (2.13). V2V-side object deletion therefore does not remove the
  core contention bottleneck in this condition.
- Radio loss decreases with RMR, but update failure and ORR worsen. The likely
  interpretation is that RMR deletes useful CPM content/redundancy while the
  sidelink channel remains saturated.

### CLM-004: V2N2V Only Bottleneck

Working explanation:

- V2N2V only sends many receiver-wise downlink transmissions through MEC.
- Required DL load can exceed the assumed 90.468 Mbps capacity by several times.
- Queueing and AoI tail make many received updates stale or unusable.
- Clean no-RMR V2N2V-only gives DL req/cap p99 7.38, MEC update failure 100%,
  AoI p99 84.4 s, and ORR 13.9%. This replaces the old/no-config V2N2V-only
  baseline for the main bottleneck claim.
- Object-payload RMR reduces DL req/cap p99 from 7.38 to 4.73, but ORR remains
  13.9% because the MEC path is still far above capacity and stale.
- Fixed per-CPM object caps do not remove this limitation cleanly. With a cap
  of 10, DL req/cap p99 falls to 0.97, but ORR remains 27.7% because useful
  objects are removed. With a cap of 20, DL req/cap p99 rises to 1.86 and the
  method again becomes stale, with MEC update failure 100% and AoI p99 45.5 s.

### CLM-005: Required DL Load Metric

Working explanation:

- DLBR is clipped to 1.0 when offered load exceeds capacity.
- `mec_dl_required_mbps` preserves overload magnitude.
- `mec_dl_required_load_ratio` is required load divided by capacity and is not clipped.
- Clean V2N2V-only examples show why this matters: no RMR requires 667.3 Mbps
  p99 (7.38x capacity), while object-payload RMR still requires 427.9 Mbps p99
  (4.73x capacity). Both would look saturated if only clipped DLBR were shown.

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

### CLM-011: High-Priority Dual Transmission

Working explanation:

- The current Hybrid+RMR design sends high-priority information over both V2V
  and V2N2V when route control selects Hybrid forwarding.
- Disabling this dual transmission reduces CBR slightly, but it also reduces
  high-priority ORR, high-priority RL>=1/2, and total ORR.
- The ablation also worsens MEC freshness, suggesting that the route-control
  balance changes unfavorably when high-priority complementarity is removed.

Use in discussion:

- Describe high-priority dual TX as a current design component, not as a free
  bandwidth addition.
- Keep future work open: dual TX may still need adaptive gating based on
  RL/RV, CBR, and MEC DL required load.

## Paper-Writing Use

When drafting the paper:

1. Start from the claims marked `main`.
2. Use `diagnostic` claims for explanation or motivation only.
3. Promote a claim to `main` only after the linked experiments have clean TTL, explicit RMR mode, and traceable metric definitions.
4. Link every figure back to an `EXP-*` row in `experiment_index.md`.
5. For every claim, include at least one supporting metric and one caveat or sensitivity result.
