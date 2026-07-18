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

methods = ["無制御", "反応型\nRMR", "予測\nRMR", "高重要度\nV2N2V", "適応型\nV2N2V"]
orr_all = [52.55, 53.18, 53.18, 75.63, 91.59]
orr_high = [83.51, 83.51, 83.51, 99.61, 99.68]
orr_low = [38.74, 39.57, 39.57, 65.32, 87.47]
cbr = [82.3, 52.3, 52.3, 52.3, 52.3]

x = np.arange(len(methods))
width = 0.23

fig, ax = plt.subplots(figsize=(6.0, 4.6))

bars_all = ax.bar(x - width, orr_all, width, label="全体ORR", color="#1f77b4")
bars_high = ax.bar(
    x,
    orr_high,
    width,
    label="高重要度ORR",
    color="#aec7e8",
)
bars_low = ax.bar(
    x + width,
    orr_low,
    width,
    label="低重要度ORR",
    color="#ffbb78",
)

for bars in (bars_all, bars_high):
    for bar in bars:
        height = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            height + 1.0,
            f"{height:.2f}",
            ha="center",
            va="bottom",
            fontsize=12,
        )

for bar in bars_low:
    height = bar.get_height()
    ax.text(
        bar.get_x() + bar.get_width() / 2,
        height - 2.0,
        f"{height:.2f}",
        ha="center",
        va="top",
        fontsize=12,
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
