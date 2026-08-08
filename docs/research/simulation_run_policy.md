# Simulation Run Policy

Last updated: 2026-07-26 JST

This file defines how to choose simulation length and scenario fidelity.  The
goal is to avoid spending final-evaluation time on runs that are only needed to
check direction, implementation behavior, or rough parameter sensitivity.

## Run Classes

| Class | Typical duration | Scenario fidelity | Purpose | Claim status |
|---|---:|---|---|---|
| Smoke | 5-10 s | Can use debug/small scenario | Start-up, crash check, log-column check, command validation | No research claim |
| Trend | 20-30 s | Prefer same vehicle count/load/TTL as final | Early direction, overload check, RMR/route sensitivity screening | Diagnostic only |
| Candidate | 100 s, seed 30 | Same as final single-seed condition | Check whether a setting is worth promoting | Candidate evidence |
| Final | 100 s or longer, multiple seeds | Final condition | Paper/slide headline results | Main evidence |

## Default Decision Rule

- Use Smoke when checking whether code, logs, or commands work.
- Use Trend when the question is "which parameter direction looks promising?"
- Use Candidate when the question is "can this setting reproduce the main trend?"
- Use Final only after the setting has passed Trend/Candidate screening.

Do not use Final runs for every additional experiment.  A 7-hour simulation is
justified only when the result may become paper/slide headline evidence.

## Trend Profile for the Current Main Comparison

For this research, Trend runs should normally keep the main load geometry rather
than shrinking vehicle count:

- 200 vehicles.
- `cars_200_2clusters_nostop.rou.xml`.
- `map_200_2clusters_nostop.sumo.cfg`.
- TTL high/low `0.2/0.2`.
- `ORR_RANGE=200`.
- `MEC_FORWARD_RANGE=200`.
- BG500 style load: `NR_BG=true`, `MEC_BG=true`,
  `NR_BG_PER_VEHICLE=true`, `MEC_BG_PER_VEHICLE=true`,
  packet size `500`, interval `100 ms`.
- `SUMO_SYNC_INTERVAL=0.1`.
- `THESIS_EVAL_START_MIN_VEHICLES=200`.
- `THESIS_EVAL_WARMUP_SECONDS=1`.

Rationale:

- V2N2V bottleneck depends strongly on receiver-wise MEC downlink fanout.
- Reducing vehicle count can hide the MEC fanout overload and produce a
  different problem.
- Shortening `SIM_TIME` preserves the load structure while saving time.

## Target Trend to Reproduce

The trend profile is calibrated against the current major diagnostic results:

| Method | Target ORR direction | Supporting old result |
|---|---:|---|
| V2V only | about 60% | Old V2V-only result was about 61.3% ORR |
| V2N2V only | about 10-20% | Old V2N2V-only result was about 13.9% ORR |
| Hybrid + RMR 10/20/40 | high, roughly 90-98% once stabilized | Old Hybrid+RMR was about 98.6%; TTL-unified interim reached about 94% at `t=23 s` |

Trend runs are not expected to match final 100 s averages exactly.  They are
useful if they preserve the ordering and bottleneck interpretation:

```text
V2N2V only: MEC DL overload / high AoI failure / low ORR
V2V only: sidelink-limited mid ORR
Hybrid+RMR: high ORR with lower MEC freshness failure than V2N2V-only
```

## Promotion Criteria

A Trend run can be promoted to a Candidate run only if:

- The command writes `run_config.txt`.
- `stdout.txt` confirms TTL `0.2/0.2`.
- MEC RMR mode is explicit when RMR is enabled.
- Split RMR counters show whether payload or fanout deletion occurred.
- Required MEC DL load is available.
- The method ordering is plausible against the target trend.

If a Trend run is already clearly overloaded or clearly worse, stop early and
adjust the parameter instead of waiting for a long final run.

## Script

Use this profile for short trend reproduction:

```bash
bash src/automotive/examples/run_hybrid_nr_v2n2v_trend_reproduce.sh
```

Default duration is `30 s`.  Override with:

```bash
SIM_TIME=20 bash src/automotive/examples/run_hybrid_nr_v2n2v_trend_reproduce.sh
```

The script runs serially:

1. V2V only.
2. V2N2V only.
3. Hybrid + object-payload RMR 10/20/40.

Use the resulting logs for screening only.  Do not cite them as final evidence
without rerunning the selected setting as Candidate or Final.

## Scaled Trend Profile

When a 200-vehicle Trend run is still too expensive, use a scaled Trend profile.
This is a computational shortcut, not a final evaluation condition.

Default script:

```bash
bash src/automotive/examples/run_hybrid_nr_v2n2v_scaled_trend.sh
```

Default scale:

| Item | Full condition | Scaled default | Scaling rule |
|---|---:|---:|---|
| Vehicles | 200 | 50 | `N * SCALE` |
| MEC bandwidth proxy | 20 MHz | 1.25 MHz | `bandwidth * SCALE^2` |
| MEC DL capacity proxy | 90.468 Mbps | about 5.654 Mbps | `capacity * SCALE^2` |
| NR channel-rate proxy | 6 Mbps | 1.5 Mbps | `rate * SCALE` |
| NR background packet | 500 bytes | 500 bytes | per-vehicle traffic scales with vehicle count |
| MEC background packet | 500 bytes | 125 bytes | per-vehicle count plus packet size gives about `SCALE^2` total |
| RMR delete 10/20/40 | 10/20/40 | 3/5/10 | delete budget scales with expected objects per CPM |

Rationale:

- V2V channel occupancy is closer to sender-count scaling, so `SCALE^1` is the
  first approximation.
- V2N2V downlink forwarding is receiver-wise fanout.  With fixed road length
  and forwarding range, total MEC downlink object traffic is closer to
  `senders * receivers`, so `SCALE^2` is the first approximation.
- RMR delete count is a per-CPM object budget, so it should scale closer to
  local object count, i.e. `SCALE^1`, not `SCALE^2`.

The exponents are deliberately configurable:

```bash
SCALE=0.25 MEC_CAPACITY_EXPONENT=2 NR_CAPACITY_EXPONENT=1 \
  bash src/automotive/examples/run_hybrid_nr_v2n2v_scaled_trend.sh
```

If V2N2V-only becomes too good, the MEC capacity scaling is too generous; reduce
`MEC_BANDWIDTH_MHZ` or increase MEC load.  If V2N2V-only is completely dead from
the first few seconds, the scaled capacity may be too harsh.  The target is not
exact equality, but preserving the major diagnostic ordering and approximate
ORR classes.
