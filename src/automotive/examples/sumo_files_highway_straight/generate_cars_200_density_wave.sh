#!/usr/bin/env bash
set -euo pipefail

out=${1:-cars_250_density_wave.rou.xml}
total=${2:-250}
cfg=${3:-}

if (( total <= 0 || total % 5 != 0 )); then
  echo "total vehicles must be a positive multiple of 5" >&2
  exit 2
fi

awk -v total="$total" 'BEGIN {
  srand(20260620)
  print "<routes>"
  print "  <vType id=\"cavDedicated80\" accel=\"2.6\" decel=\"4.5\" sigma=\"0\" length=\"5\" minGap=\"2.5\" maxSpeed=\"22.22\" color=\"0,80,255\"/>"
  print "  <vType id=\"cavGeneral90\" accel=\"2.6\" decel=\"4.5\" sigma=\"0.3\" length=\"5\" minGap=\"2.5\" maxSpeed=\"25.00\" color=\"0,140,255\"/>"
  print "  <vType id=\"noncav100\" accel=\"2.0\" decel=\"4.5\" sigma=\"0.5\" length=\"5\" minGap=\"2.5\" maxSpeed=\"27.78\" color=\"150,150,150\"/>"
  print "  <route id=\"eastbound\" edges=\"highway_eb\"/>"
  print "  <route id=\"westbound\" edges=\"highway_wb\"/>"

  groups = total / 5
  initialGroups = int(groups * 0.6)
  for (direction = 0; direction < 2; direction++)
    for (lane = 0; lane < 3; lane++)
      lastPos[direction, lane] = 150

  count = 0
  for (group = 0; group < groups; group++) {
    direction = group % 2
    route = direction ? "eastbound" : "westbound"
    generalCavLane = 1 + (group % 2)

    if (group < initialGroups) {
      gap = 30 + int(rand() * 41)
      spacing = 10 + int(rand() * 8)
      base = 150
      for (lane = 0; lane < 3; lane++)
        if (lastPos[direction, lane] + gap > base)
          base = lastPos[direction, lane] + gap
      for (lane = 0; lane < 3; lane++)
        laneRow[lane] = 0
    } else {
      departBase = 10 + (group - initialGroups) * 1.5
      for (lane = 0; lane < 3; lane++)
        laneRow[lane] = 0
    }

    for (n = 0; n < 5; n++) {
      cav = n < 2
      lane = n == 0 ? 0 : (n == 1 ? generalCavLane : 1 + ((n + group) % 2))
      type = n == 0 ? "cavDedicated80" : (n == 1 ? "cavGeneral90" : "noncav100")
      id = ++count

      if (group < initialGroups) {
        pos = base + laneRow[lane]++ * spacing
        depart = 0
        vehicles[count] = sprintf("  <vehicle id=\"veh%d\" type=\"%s\" route=\"%s\" depart=\"0.0\" departLane=\"%d\" departPos=\"%.1f\" departSpeed=\"0\"/>", id, type, route, lane, pos)
        for (updateLane = 0; updateLane < 3; updateLane++)
          if (lastPos[direction, updateLane] < base)
            lastPos[direction, updateLane] = base
        lastPos[direction, lane] = pos
      } else {
        depart = departBase + laneRow[lane]++ * 0.8
        vehicles[count] = sprintf("  <vehicle id=\"veh%d\" type=\"%s\" route=\"%s\" depart=\"%.1f\" departLane=\"%d\" departSpeed=\"max\"/>", id, type, route, depart, lane)
      }
      departures[count] = depart
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

if [[ -n "$cfg" ]]; then
  route_name=$(basename "$out")
  cat > "$cfg" <<EOF
<configuration>
  <input>
    <net-file value="highway.net.xml"/>
    <route-files value="$route_name"/>
    <additional-files value="stations.xml"/>
  </input>
  <time>
    <begin value="0"/>
    <end value="180"/>
    <step-length value="0.1"/>
  </time>
</configuration>
EOF
fi
