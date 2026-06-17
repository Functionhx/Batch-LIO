#!/usr/bin/env python3
"""batch-LIO result figures (standalone, screenshottable). PNG @200dpi.
All numbers are verbatim from docs/results/2026-06-17-batch-lio-ab.md.
Run: python3 figures/make_figs.py  ->  figures/fig1..fig4.png
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = os.path.dirname(os.path.abspath(__file__))

# ---- clean academic style (light, professional, no chart junk) ----
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans"],
    "font.size": 12,
    "axes.titlesize": 14,
    "axes.titleweight": "bold",
    "axes.labelsize": 12,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.axisbelow": True,
    "figure.dpi": 200,
    "savefig.dpi": 200,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.15,
})
BLUE, GREEN, RED, AMBER, GREY = "#4C72B0", "#55A868", "#C44E52", "#DD8452", "#555555"


def style_axes(ax):
    ax.yaxis.grid(True, color="#cccccc", linewidth=0.7, alpha=0.7)
    ax.set_axisbelow(True)
    ax.tick_params(length=0)


def save(fig, name):
    p = os.path.join(OUT, name)
    fig.savefig(p)
    plt.close(fig)
    print("saved", p)


# ===== Figure 1: speedup comparison (grouped bars) =====
def fig1():
    groups = ["quick-shack", "HKU_MB"]
    baseline = [7.69, 16.21]
    batch = [3.35, 4.56]
    speed = ["2.3×", "3.6×"]
    x = np.arange(len(groups)); w = 0.34
    fig, ax = plt.subplots(figsize=(6.4, 4.4))
    b1 = ax.bar(x - w/2, baseline, w, label="Point-LIO", color=BLUE)
    b2 = ax.bar(x + w/2, batch, w, label="batch-LIO (1 ms + OMP)", color=GREEN)
    for bars in (b1, b2):
        for r in bars:
            ax.text(r.get_x()+r.get_width()/2, r.get_height()+0.25,
                    f"{r.get_height():.2f}", ha="center", va="bottom", fontsize=11)
    ymax = max(baseline) * 1.26
    for i, s in enumerate(speed):
        top = max(baseline[i], batch[i])
        ax.annotate(s, xy=(i, top), xytext=(i, top + ymax*0.085), ha="center",
                    fontsize=14, fontweight="bold", color=GREY)
    ax.set_xticks(x); ax.set_xticklabels(groups)
    ax.set_ylabel("Compute per frame (ms)")
    ax.set_ylim(0, ymax)
    ax.set_title("batch-LIO vs Point-LIO: per-frame compute")
    ax.legend(frameon=False, loc="upper left")
    style_axes(ax)
    save(fig, "fig1_speedup.png")


# ===== Figure 2: OMP causality (3 bars + baseline reference) =====
def fig2():
    labels = ["point-wise\n+ OMP", "batch 1 ms\n(single thread)", "batch 1 ms\n+ OMP"]
    vals = [10.4, 6.72, 3.35]
    val_lbl = ["10.4", "6.72", "3.35"]
    colors = [RED, AMBER, GREEN]
    fig, ax = plt.subplots(figsize=(6.6, 4.4))
    bars = ax.bar(labels, vals, color=colors, width=0.6)
    for r, t in zip(bars, val_lbl):
        ax.text(r.get_x()+r.get_width()/2, r.get_height()+0.18, t,
                ha="center", va="bottom", fontsize=11)
    ax.axhline(7.69, color=BLUE, linestyle="--", linewidth=1.4)
    ax.text(2.42, 7.69+0.12, "Point-LIO baseline 7.69", color=BLUE,
            ha="right", va="bottom", fontsize=10)
    ax.annotate("OMP alone: slower\nthan baseline", xy=(0, 10.4), xytext=(0, 11.4),
                ha="center", fontsize=10, color=RED)
    ax.set_ylabel("Compute per frame (ms)")
    ax.set_ylim(0, 12.4)
    ax.set_title("Batch enables parallelism (quick-shack)")
    style_axes(ax)
    save(fig, "fig2_omp_causality.png")


# ===== Figure 3: de-skew validation (2 bars) =====
def fig3():
    labels = ["de-skew OFF", "de-skew ON"]
    vals = [7.6, 0.5]
    fig, ax = plt.subplots(figsize=(5.4, 4.4))
    bars = ax.bar(labels, vals, color=[RED, GREEN], width=0.5)
    for r in bars:
        ax.text(r.get_x()+r.get_width()/2, r.get_height()+0.15,
                f"{r.get_height():g} m", ha="center", va="bottom", fontsize=12)
    ax.annotate("", xy=(1, 0.5), xytext=(1, 7.6),
                arrowprops=dict(arrowstyle="<->", color=GREY, lw=1.3))
    ax.text(1.12, 3.9, "8× lower", color=GREY, fontsize=12, fontweight="bold", va="center")
    ax.set_ylabel("Trajectory drift (m)")
    ax.set_ylim(0, 8.6)
    ax.set_title("De-skew validation (20 ms window probe)")
    style_axes(ax)
    save(fig, "fig3_deskew.png")


# ===== Figure 4: batch_dt sweep (measured: compute + accuracy, dual axis) =====
def fig4():
    dt = [0.5, 1, 2, 5, 10, 20]
    compute = [4.47, 3.44, 2.72, 2.22, 1.99, 1.85]      # ms (measured)
    err = [0.056, 0.059, 0.059, 0.104, 1.81, 0.360]      # mean |dpos| vs baseline (m)
    x = np.arange(len(dt))
    fig, ax = plt.subplots(figsize=(6.8, 4.4))
    # sweet-spot shading (1-2 ms)
    ax.axvspan(0.7, 2.3, color=GREEN, alpha=0.12)
    ax.text(1.5, 1.65, "sweet spot\n1–2 ms", ha="center", va="center",
            fontsize=11, color="#2f6b3f", fontweight="bold")
    # left axis = compute (blue), right axis = accuracy error (red); axes are colour-coded
    ax.plot(x, compute, "-o", color=BLUE, lw=2)
    ax.text(0.05, 4.78, "compute / frame", color=BLUE, fontsize=10, va="top")
    ax.set_xlabel("batch_dt (ms)")
    ax.set_ylabel("Compute per frame (ms)", color=BLUE)
    ax.tick_params(axis="y", colors=BLUE)
    ax.set_xticks(x); ax.set_xticklabels([str(d) for d in dt])
    ax.set_ylim(0, 5.0)
    ax2 = ax.twinx()
    ax2.spines["top"].set_visible(False)
    ax2.plot(x, err, "--s", color=RED, lw=2)
    ax2.text(0.05, 0.17, "traj error vs baseline", color=RED, fontsize=10, va="bottom")
    ax2.set_ylabel("Mean trajectory error vs baseline (m)", color=RED)
    ax2.tick_params(axis="y", colors=RED)
    ax2.set_ylim(0, 2.0)
    ax2.annotate("10 ms:\nnon-monotonic\ndegradation", xy=(4, 1.81), xytext=(2.75, 1.42),
                 fontsize=9, color=RED, ha="center",
                 arrowprops=dict(arrowstyle="->", color=RED, lw=1.0))
    ax.set_title("batch_dt sweep: faster with larger windows,\naccuracy degrades past ~2 ms (quick-shack)")
    ax.yaxis.grid(True, color="#cccccc", linewidth=0.7, alpha=0.7)
    ax.set_axisbelow(True); ax.tick_params(length=0); ax2.tick_params(length=0)
    save(fig, "fig4_batchdt_sweep.png")


if __name__ == "__main__":
    fig1(); fig2(); fig3(); fig4()
    print("done")
