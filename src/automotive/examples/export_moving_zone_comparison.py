#!/usr/bin/env python3
import csv
from pathlib import Path


base = Path("/tmp/van3twin_results")
runs = [
    ("no-control", "moving_zone120_bgfix_noctrl"),
    ("predictive-rmr-v2v", "moving_zone120_bgfix_v2v"),
    ("v2n2v-high-priority-only", "moving_zone120_bgfix_highonly"),
    ("v2n2v-adaptive-probability", "moving_zone120_bgfix_adaptive"),
]
fields = [
    "method",
    "orr_percent",
    "high_orr_percent",
    "low_orr_percent",
    "average_cbr",
    "packet_loss_percent",
    "mec_tx_packets",
    "mec_rx_packets",
    "v2n_uplink_bytes",
    "mec_forwarded_bytes",
    "total_v2n2v_bytes",
    "ttl_violation_percent",
    "never_received_percent",
]
output = base / "moving_zone120_bgfix_comparison.csv"

with output.open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    for method, directory in runs:
        with (base / directory / "summary.csv").open(newline="") as source:
            summary = list(csv.DictReader(source))[-1]
        with (base / directory / method / "observation.csv").open(newline="") as source:
            observation = list(csv.DictReader(source))[-1]
        uplink = int(observation["mec_uplink_bytes"])
        forwarded = int(observation["mec_forwarded_bytes"])
        writer.writerow({
            "method": method,
            "orr_percent": summary["object_recognition_rate"],
            "high_orr_percent": summary["high_priority_object_recognition_rate"],
            "low_orr_percent": summary["low_priority_object_recognition_rate"],
            "average_cbr": summary["channel_busy_ratio"],
            "packet_loss_percent": summary["packet_loss_rate"],
            "mec_tx_packets": summary["mec_tx"],
            "mec_rx_packets": summary["mec_rx"],
            "v2n_uplink_bytes": uplink,
            "mec_forwarded_bytes": forwarded,
            "total_v2n2v_bytes": uplink + forwarded,
            "ttl_violation_percent": summary["ttl_violation_rate"],
            "never_received_percent": summary["never_received_rate"],
        })

descriptions = {
    "method": "手法名",
    "orr_percent": "全体の物標認識率 [%]",
    "high_orr_percent": "高重要度物標認識率 [%]",
    "low_orr_percent": "低重要度物標認識率 [%]",
    "average_cbr": "平均チャネル占有率",
    "packet_loss_percent": "CPMパケット損失率 [%]",
    "mec_tx_packets": "MECアップリンク送信パケット数",
    "mec_rx_packets": "MEC経由受信・転送パケット数",
    "v2n_uplink_bytes": "車両からMECへの送信量 [bytes]",
    "mec_forwarded_bytes": "MECから車両群への転送量 [bytes]",
    "total_v2n2v_bytes": "V2N2V総通信量 [bytes]",
    "ttl_violation_percent": "AoI期限超過率 [%]",
    "never_received_percent": "一度も更新されなかった割合 [%]",
}
description_output = base / "moving_zone120_bgfix_columns.csv"
with description_output.open("w", newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(["column", "description"])
    writer.writerows(descriptions.items())

print(output)
print(description_output)
