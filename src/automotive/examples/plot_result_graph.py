import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update(
    {
        "font.size": 15,
        "axes.titlesize": 16,
        "axes.labelsize": 15,
        "xtick.labelsize": 13,
        "ytick.labelsize": 13,
        "legend.fontsize": 12,
    }
)
plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK JP",
    "IPAexGothic",
    "IPAGothic",
    "Yu Gothic",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False

methods = ["無制御", "予測\nRMR", "高重要度\nV2N2V", "適応型\nV2N2V"]
orr_all = [52.76, 52.76, 74.97, 91.67]
orr_high = [83.76, 83.76, 99.35, 99.54]
orr_low = [38.85, 38.85, 64.41, 87.53]
cbr = [71.9, 53.1, 53.1, 53.1]

x = np.arange(len(methods))
width = 0.23

fig, ax = plt.subplots(figsize=(5.2, 5.0))

ax.bar(x - width, orr_all, width, label="全体ORR", color="#1f77b4")
ax.bar(
    x,
    orr_high,
    width,
    label="高重要度ORR",
    color="#aec7e8",
)
ax.bar(
    x + width,
    orr_low,
    width,
    label="低重要度ORR",
    color="#ffbb78",
)
ax.plot(
    x,
    cbr,
    label="平均CBR",
    color="#d62728",
    marker="o",
    linewidth=2,
    markersize=5,
    zorder=3,
)

ax.set_ylabel("割合（%）", fontsize=12)
ax.set_xticks(x)
ax.set_xticklabels(methods, fontsize=10)
ax.set_ylim(0, 110)
ax.grid(axis="y", linestyle="--", alpha=0.7)
ax.legend(
    loc="upper center",
    bbox_to_anchor=(0.5, -0.16),
    ncol=2,
    frameon=False,
    columnspacing=1.0,
    handlelength=1.4,
)

fig.tight_layout()
fig.savefig("result_graph.pdf", format="pdf", bbox_inches="tight")
print("Wrote result_graph.pdf")
