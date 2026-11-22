"""Charts for the deck, at slide size: large fonts, the key finding marked. Run: uv run --with matplotlib python docs/presentation/charts.py"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

NAVY, BLUE, GREY, LIGHT, GOOD, BAD = "#1F4E79", "#2E75B6", "#595959", "#D9D9D9", "#0ca30c", "#d03b3b"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 17, "axes.edgecolor": LIGHT, "axes.labelcolor": GREY, "xtick.color": GREY,
                     "ytick.color": GREY, "axes.spines.top": False, "axes.spines.right": False, "figure.facecolor": "white", "axes.facecolor": "white"})
OUT = "docs/presentation/"

# hand-over attempts in sequence
runs = [0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1]
fig, ax = plt.subplots(figsize=(8.6, 4.6))
for i, ok in enumerate(runs, 1):
    ax.scatter(i, ok, s=260, marker="o" if ok else "X", color=GOOD if ok else BAD, edgecolors="white", linewidths=2, zorder=3)
ax.axvspan(11.5, 17.6, color="#EAF1F8", zorder=0)
ax.text(11.7, 1.42, "replay of the one\ndemonstration", fontsize=15, color=NAVY, va="top")
for a, b, t in ((1, 8, "first 8: 1 worked"), (10, 17, "last 8: 5 worked")):
    ax.plot([a - 0.3, b + 0.3], [-0.55, -0.55], color=GREY, lw=2, clip_on=False)
    ax.text((a + b) / 2, -0.62, t, ha="center", va="top", fontsize=17, color=NAVY, weight="bold", clip_on=False)
ax.set_yticks([0, 1]); ax.set_yticklabels(["missed", "took the cube"]); ax.set_ylim(-0.35, 1.5); ax.set_xlim(0.3, 17.7)
ax.set_xticks([]); ax.spines["bottom"].set_visible(False)
ax.tick_params(length=0); ax.spines["left"].set_visible(False)
fig.subplots_adjust(left=0.2, right=0.98, top=0.95, bottom=0.33); fig.savefig(OUT + "deck_attempts.png", dpi=200); plt.close(fig)

# error budget
items = [("SO-101 stops short,\nbefore the correction", 18), ("ElRobot off the shown\npose, worst case", 17), ("cube position in\nthe fingers", 10), ("fit of the SO-101\nposition", 8), ("SO-101 stops short,\nafter the correction", 5)]
fig, ax = plt.subplots(figsize=(8.6, 4.8))
for i, (n, mm) in enumerate(items[::-1]):
    ax.barh(i, mm, height=0.55, color=BAD if mm > 11 else BLUE, zorder=3)
    ax.text(mm + 0.3, i, f"{mm} mm", va="center", fontsize=16, color=GREY)
ax.set_yticks(range(len(items))); ax.set_yticklabels([n for n, _ in items[::-1]], fontsize=14)
ax.axvline(11, color=NAVY, lw=2.5, ls="--", zorder=4)
ax.text(11.3, 0.35, "room each side\nof the cube: 11 mm", color=NAVY, fontsize=15, weight="bold", va="center")
ax.set_xlim(0, 22); ax.set_xlabel("largest error seen, mm", fontsize=15); ax.tick_params(length=0); ax.spines["left"].set_visible(False)
ax.grid(axis="x", color=LIGHT, zorder=0)
fig.subplots_adjust(left=0.33, right=0.98, top=0.97, bottom=0.15); fig.savefig(OUT + "deck_error_budget.png", dpi=200); plt.close(fig)

# ablation: height correction, and pose source
bars = [("computed,\nno height fix", 0, 3), ("computed,\nheight fix", 2, 2), ("one pose\nadjusted", 1, 2), ("shown with\nboth arms", 4, 6)]
fig, ax = plt.subplots(figsize=(8.6, 4.6))
for i, (n, ok, tot) in enumerate(bars):
    ax.bar(i, ok / tot, width=0.55, color=NAVY if i == 1 else BLUE, zorder=3)
    ax.text(i, ok / tot + 0.04, f"{ok} of {tot}", ha="center", va="bottom", fontsize=18, color=GREY)
ax.set_xticks(range(len(bars))); ax.set_xticklabels([b[0] for b in bars], fontsize=15)
ax.set_ylim(0, 1.25); ax.set_yticks([0, 0.5, 1]); ax.set_yticklabels(["0", "half", "all"]); ax.grid(axis="y", color=LIGHT, zorder=0)
ax.tick_params(length=0); ax.spines["left"].set_visible(False); ax.set_ylabel("attempts that worked", fontsize=15)
fig.subplots_adjust(left=0.13, right=0.98, top=0.95, bottom=0.2); fig.savefig(OUT + "deck_ablation.png", dpi=200); plt.close(fig)
print("ok")
