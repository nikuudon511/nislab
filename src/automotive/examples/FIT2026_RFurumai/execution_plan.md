# Execution Plan

## 1. Main Direction

FIT preprint results are treated as preliminary results based on an idealized
V2N2V link.  For the final presentation and later thesis work, the evaluation
will move to a stricter NR-only V2X setting.

Main policy:

- Use `v2v-hybrid-nr-v2n2v.cc` as the implementation base for now.
- Do not remove 802.11p/legacy code yet, because existing metrics and route
  control still depend on legacy `dsrc` names.
- Update `V2X_specification.md` whenever assumptions or model definitions
  change.
- Prefer lightweight approximate models over Sionna/5G-LENA full evaluation,
  because the target scenario uses 100+ communication vehicles.

## 2. Target Evaluation Scenario

The new main scenario should avoid artificial background load as much as
possible.

Target setting:

- Communication penetration rate: 100%.
- No artificial NR background traffic in the main evaluation.
- No moving high-load zone in the main evaluation.
- Dense vehicle clusters instead of sparse vehicle placement.
- Opposing vehicle clusters should meet/cross at a controlled point.
- Congestion should be generated mainly by CPM/CAM traffic from real vehicles.
- V2N2V should be evaluated under fanout, capacity, queueing delay, and AoI
  constraints.

Rationale:

- 100% communication vehicles are easier to compare with other studies.
- Removing background traffic makes the congestion mechanism easier to explain.
- Dense cluster crossing naturally increases CPM object count and CPM size.
- V2N2V-only must be evaluated to answer why V2N2V alone is not sufficient.

## 3. Implementation Phases

### Phase 1: Specification and Logging

Goal: make the current model measurable before changing behavior heavily.

Tasks:

- Add generation-time-based V2N2V AoI logs.
- Log V2N2V E2E delay percentiles.
- Log V2N2V fanout statistics.
- Log uplink/downlink bytes separately.
- Log delivered-after-deadline count.
- Log drop reasons separately.

Required new metrics:

- `mec_aoi_mean_ms`
- `mec_aoi_p50_ms`
- `mec_aoi_p90_ms`
- `mec_aoi_p95_ms`
- `mec_aoi_p99_ms`
- `mec_aoi_achieve_50ms_rate`
- `mec_aoi_achieve_100ms_rate`
- `mec_aoi_achieve_200ms_rate`
- `mec_aoi_achieve_500ms_rate`
- `mec_high_aoi_achieve_50ms_rate`
- `mec_high_aoi_achieve_100ms_rate`
- `mec_high_aoi_achieve_200ms_rate`
- `mec_high_aoi_achieve_500ms_rate`
- `mec_low_aoi_achieve_50ms_rate`
- `mec_low_aoi_achieve_100ms_rate`
- `mec_low_aoi_achieve_200ms_rate`
- `mec_low_aoi_achieve_500ms_rate`
- `mec_fanout_mean`
- `mec_fanout_p90`
- `mec_fanout_p95`
- `mec_dl_load_ratio`
- `mec_ul_load_ratio`
- `mec_queue_delay_mean_ms`
- `mec_queue_overflow_drops`

### Phase 2: V2N2V Approximate Path Model

Goal: replace the ideal fixed-delay V2N2V link with a realistic lightweight
model.

Tasks:

- Replace fixed packet size with actual CPM-size or object-count-based payload.
- Decompose V2N2V delay into UL, MEC processing, DL, and queue delay.
- Use heavy-tail delay distribution rather than clipped normal delay.
- Add DL capacity constraint.
- Add queue length and queue delay.
- Add queue overflow or AoI-expired drop.

Initial model:

- Mean E2E delay around 40 ms.
- p90 around 60 ms.
- p99 around 120 ms.
- UL effective capacity: 10 Mbps.
- DL effective capacity: 50 Mbps.
- DL load increases with receiver-wise unicast fanout.
- Queue delay increases as load ratio approaches 1.

### Phase 3: Natural Dense-Traffic Scenario

Goal: generate high CBR without artificial background load.

Tasks:

- Create or modify SUMO route files to form dense vehicle clusters.
- Make opposing clusters cross at a controlled time and location.
- Increase communication vehicles to 100%.
- Test whether CPM alone can raise CBR sufficiently.
- Add CAM only if CPM alone cannot create stable congestion.
- Avoid DENM until CPM/CAM behavior is understood.

Expected behavior:

- Cluster crossing increases perceived object count.
- CPM size increases.
- V2V CBR rises due to real message load.
- V2N2V-only suffers from downlink fanout and AoI violations.
- Hybrid uses V2V for low-latency local sharing and V2N2V for selected recovery.

### Phase 4: Comparison Evaluation

Required comparison methods:

- V2V only.
- V2N2V only.
- Hybrid high-priority V2N2V.
- Hybrid adaptive V2N2V.

Key metrics:

- ORR.
- High-priority ORR.
- Low-priority ORR.
- CBR.
- Packet loss.
- V2N2V AoI violation rate.
- V2N2V E2E delay p50/p90/p99.
- V2N2V DL fanout.
- V2N2V queue delay.
- V2V/V2N2V traffic volume.

## 4. NLOS Policy

NLOS should not be part of the first main evaluation if communication
penetration is 100%.

Reason:

- 100% communication vehicles allow cooperative perception to cover many
  sensor blind spots.
- NLOS adds extra computation and model complexity.
- The first goal is to isolate route-control and V2N2V path constraints.

Future extension:

- Add simple geometric NLOS if penetration rate is reduced below 100%.
- Use Sionna only for small-scale validation or representative examples.

## 5. Expected Simulation Cost

Current baseline:

- One method, 120 s simulation: about 1 hour.

Estimated cost:

- V2N2V logging only: 1.05x to 1.2x.
- V2N2V capacity/queue model: 1.1x to 1.3x.
- Dense 100% communication vehicles: depends on vehicle count, likely 1.5x or
  more.
- SUMO/ns-3 sync interval 0.1 s instead of 0.01 s: about 0.1x for TraCI
  synchronization steps, but total runtime reduction must be measured because
  CPM/V2N2V processing still dominates.
- CAM addition: +10% to +50%.
- Simple NLOS: +20% to +100%, depending on frequency and implementation.
- Full Sionna/5G-LENA main evaluation: not recommended for 100+ vehicles.

Note: use `SUMO_SYNC_INTERVAL=0.1` only for GUI checks, smoke tests, and
preliminary sweeps unless validated.  Main evaluation should either use the
historical `0.01 s` setting or include a same-seed comparison showing that
`0.1 s` does not materially change ORR, CBR, packet loss, MEC delay, or AoI.

## 6. Open Questions

- How many vehicles are needed to reach CBR 0.8 without background traffic?
- Can CPM alone create enough congestion, or is CAM also required?
- Which AoI threshold should be emphasized in the main result: 50, 100, 200,
  or 500 ms?
- Is UL 10 Mbps / DL 50 Mbps sufficiently defensible as a conservative V2N
  effective capacity?
- Should V2N2V delay distribution be log-normal, empirical percentile based, or
  queue-only?
- How should V2N2V-only choose its forwarding targets?
- Should non-communication vehicles be introduced after the 100% penetration
  case is stable?

## 7. Immediate Next Steps

1. Add V2N2V AoI/fanout/delay/drop logs.
2. Run the current scenario once to quantify ideal-link AoI behavior.
3. Implement payload-size-aware V2N2V accounting.
4. Implement DL capacity and queue delay.
5. Build a dense-cluster SUMO scenario without background traffic.
6. Run a short 30 s smoke test for CBR behavior.
7. Run 120 s comparison for V2V-only, V2N2V-only, and hybrid methods.
