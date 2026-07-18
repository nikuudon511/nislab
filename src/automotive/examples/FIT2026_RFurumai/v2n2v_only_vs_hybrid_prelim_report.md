# V2N2V-only vs Hybrid preliminary comparison

## Purpose

Professor feedback: if V2N2V performs well, the paper must explain why V2N2V-only is not sufficient.

This note summarizes preliminary dense-scenario checks using the lightweight V2N2V model. The point is not final performance ranking yet; it is to identify whether V2N2V-only creates a load or tail-delay disadvantage.

## Current findings

- In a 50-vehicle completed 10 s run, V2N2V-only achieved ORR 100%, while Hybrid forced achieved ORR 82.92%.
- However, V2N2V-only used more downlink forwarding:
  - V2N2V-only: 110083 MEC RX packets in summary.
  - Hybrid forced: 64366 MEC RX packets in summary.
- Therefore, ORR alone is not enough to evaluate V2N2V-only. V2N2V downlink fanout and tail delay must be reported together.

## 100-vehicle stress observation

The 100-vehicle runs were stopped early due runtime, so these are preliminary t=2 observations, not final summary results.

Latest sandbox-outside rerun at t=2:

- V2N2V-only:
  - MEC routes: 100
  - Downlink packets: 68826
  - AoI p99: 839 ms
  - Capacity delay p99: 1267 ms
- Adaptive normal:
  - MEC routes: 0
  - Downlink packets: 0
  - AoI p99: 0 ms
  - Capacity delay p99: 0 ms
  - Note: normal threshold did not activate V2N2V by t=2.
- Hybrid forced:
  - MEC routes: 64
  - Downlink packets: 16862
  - AoI p99: 206 ms
  - Capacity delay p99: 182 ms

Relative to Hybrid forced, V2N2V-only had:

- 4.08x downlink packets.
- 4.07x AoI p99.
- 6.96x capacity-delay p99.

This supports the argument that V2N2V-only can overload the downlink fanout path in dense conditions, while Hybrid keeps V2N2V supplemental.

## Caveats

- The 100-vehicle comparison is early-stop observation data only.
- The 50-vehicle Hybrid forced setting is not yet optimized for ORR.
- Final claims need a stable medium/high-density condition that completes, or a clearly defined short stress-window evaluation.
- SUMO/TraCI simulations must be run outside the command sandbox because local socket creation is blocked there.

## Next steps

- Find a 60-80 vehicle condition that completes within practical runtime.
- Compare V2N2V-only, Hybrid forced, and normal adaptive under the same condition.
- Report ORR, MEC downlink packets, AoI p90/p99, capacity-delay p90/p99, and final UL/DL loss counts together.
