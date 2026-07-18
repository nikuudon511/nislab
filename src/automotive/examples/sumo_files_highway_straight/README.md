# Straight Highway SUMO Scenario

This scenario is an isolated highway baseline for V2V/V2N2V congestion-control experiments.

- Highway length: 5000 m
- Lanes: 3 per direction, 6 total
- Lane width: 4 m
- Speed limit: 27.78 m/s, approximately 100 km/h
- Directions: eastbound and westbound carriageways
- RSU POIs: 10 stations in the median, spaced every 500 m
- Vehicles: 60 explicit vehicles named `veh1` through `veh60`

The lane count and lane width follow the common 3GPP V2X highway evaluation shape: 3 lanes in each direction with 4 m lanes. Vehicle IDs are explicit because VaN3Twin examples often derive ns-3 node setup from SUMO route-file vehicle entries.

After SUMO coordinate normalization, the eastbound lanes are at y = -4, 0, 4 and the westbound lanes are at y = 12, 16, 20. The RSU POIs are placed in the median at y = 8.

Use `map_oneway.sumo.cfg` with `cars_60_oneway.rou.xml` for the first simple one-direction evaluation. Use `map.sumo.cfg` with `cars_60.rou.xml` for the bidirectional follow-up evaluation.
