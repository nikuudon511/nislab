#!/usr/bin/env bash
set -euo pipefail

out=${1:-cars_200_density_wave.rou.xml}
awk 'BEGIN {
  srand(20260618)
  print "<routes>"
  print "  <vType id=\"cavDedicated80\" accel=\"2.6\" decel=\"4.5\" sigma=\"0\" length=\"5\" minGap=\"2.5\" maxSpeed=\"22.22\" color=\"0,80,255\"/>"
  print "  <vType id=\"cavGeneral90\" accel=\"2.6\" decel=\"4.5\" sigma=\"0.3\" length=\"5\" minGap=\"2.5\" maxSpeed=\"25.00\" color=\"0,140,255\"/>"
  print "  <vType id=\"noncav100\" accel=\"2.0\" decel=\"4.5\" sigma=\"0.5\" length=\"5\" minGap=\"2.5\" maxSpeed=\"27.78\" color=\"150,150,150\"/>"
  print "  <route id=\"eastbound\" edges=\"highway_eb\"/>"
  print "  <route id=\"westbound\" edges=\"highway_wb\"/>"

  # Place 20 mixed initial platoons: 1 CAV + 5 non-CAVs each.
  for (direction = 0; direction < 2; direction++)
    for (lane = 0; lane < 3; lane++)
      lastPos[direction, lane] = 500

  cavId = 81
  noncavId = 101
  for (platoon = 0; platoon < 20; platoon++) {
    direction = platoon % 2
    cols = 1 + int(rand() * 3)
    gap = 50 + int(rand() * 101)
    spacing = 12 + int(rand() * 14)
    base = 500
    for (lane = 0; lane < 3; lane++)
      if (lastPos[direction, lane] + gap > base)
        base = lastPos[direction, lane] + gap

    for (lane = 0; lane < 3; lane++)
      laneRow[lane] = 0
    for (n = 0; n < 6; n++) {
      route = direction ? "eastbound" : "westbound"
      cav = n == platoon % 6
      lane = cav ? 0 : 1 + (n % 2)
      pos = base + laneRow[lane]++ * spacing
      id = cav ? cavId++ : noncavId++
      type = cav ? "cavDedicated80" : "noncav100"
      count++
      departures[count] = 0
      vehicles[count] = sprintf("  <vehicle id=\"veh%d\" type=\"%s\" route=\"%s\" depart=\"0.0\" departLane=\"%d\" departPos=\"%.1f\" departSpeed=\"0\"/>", id, type, route, lane, pos)
      lastPos[direction, lane] = pos
    }
  }

  # Introduce 20 mixed platoons: 80 CAVs + 50 non-CAVs in total.
  cavId = 1
  noncavId = 201
  for (platoon = 0; platoon < 20; platoon++) {
    direction = platoon % 2
    cols = 1 + int(rand() * 3)
    base = 10 + (platoon / 19) * 35
    platoonSize = platoon < 10 ? 7 : 6
    for (lane = 0; lane < 3; lane++)
      laneRow[lane] = 0
    for (n = 0; n < platoonSize; n++) {
      route = direction ? "eastbound" : "westbound"
      cav = n < 4
      lane = cav ? (n == 0 ? 0 : 1 + ((n - 1) % 2)) : 1 + (n % 2)
      depart = base + laneRow[lane]++ * 0.8
      id = cav ? cavId++ : noncavId++
      type = cav ? (lane == 0 ? "cavDedicated80" : "cavGeneral90") : "noncav100"
      count++
      departures[count] = depart
      vehicles[count] = sprintf("  <vehicle id=\"veh%d\" type=\"%s\" route=\"%s\" depart=\"%.1f\" departLane=\"%d\" departSpeed=\"max\"/>", id, type, route, depart, lane)
    }
  }
  for (i = 1; i <= count; i++) {
    best = i
    for (j = i + 1; j <= count; j++)
      if (departures[j] < departures[best])
        best = j
    tmp = departures[i]; departures[i] = departures[best]; departures[best] = tmp
    tmp = vehicles[i]; vehicles[i] = vehicles[best]; vehicles[best] = tmp
    print vehicles[i]
  }
  print "</routes>"
}' > "$out"
