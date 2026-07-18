#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path


parser = argparse.ArgumentParser(
    description="Select the minimum-V2N method satisfying ORR constraints per load."
)
parser.add_argument("inputs", nargs="+", help="LOAD=summary.csv")
parser.add_argument("--target-orr", type=float, required=True)
parser.add_argument("--target-high", type=float, required=True)
parser.add_argument("--output", default="selected_min_v2n.csv")
args = parser.parse_args()

results = []
for item in args.inputs:
    load, separator, filename = item.partition("=")
    if not separator:
        parser.error(f"invalid input '{item}'; expected LOAD=summary.csv")
    with Path(filename).open(newline="") as stream:
        for row in csv.DictReader(stream):
            try:
                orr = float(row["object_recognition_rate"])
                high = float(row["high_priority_object_recognition_rate"])
                mec_tx = float(row["mec_tx"])
            except (KeyError, TypeError, ValueError) as error:
                parser.error(f"{filename}: invalid summary row: {error}")
            feasible = orr >= args.target_orr and high >= args.target_high
            results.append({
                "load": load,
                "method": row["method"],
                "orr": orr,
                "high_orr": high,
                "low_orr": row["low_priority_object_recognition_rate"],
                "mec_tx": mec_tx,
                "feasible": feasible,
                "selected": False,
            })

for load in {row["load"] for row in results}:
    feasible = [row for row in results if row["load"] == load and row["feasible"]]
    if feasible:
        min(feasible, key=lambda row: (row["mec_tx"], -row["orr"], row["method"]))[
            "selected"
        ] = True

fields = ["load", "method", "orr", "high_orr", "low_orr", "mec_tx", "feasible", "selected"]
with Path(args.output).open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    writer.writerows(results)
print(f"Wrote {args.output}")
