#!/usr/bin/env python3
"""README artwork for Batch-LIO.

    python3 docs/assets/make_assets.py

Writes into this directory (standard library only):
  hero.svg                       animated keynote-style banner (theme-independent)
  stats-{zh,en}.svg              key-numbers banner (theme-independent)
  {speedup,omp,sweep,pipeline}-{zh,en}-{light,dark}.svg

Every number is taken verbatim from docs/RESULTS.md and docs/RESULTS_ROS2.md.
Chart colours: blue = Batch-LIO, orange = Point-LIO (validated for CVD separation
and >= 3:1 contrast on both GitHub surfaces).
"""
import math
import os
import random

OUT = os.path.dirname(os.path.abspath(__file__))

SANS = ("'Inter','SF Pro Display',-apple-system,BlinkMacSystemFont,'Segoe UI',"
        "'PingFang SC','Hiragino Sans GB','Microsoft YaHei','Noto Sans CJK SC',"
        "'Noto Sans SC',Helvetica,Arial,sans-serif")
MONO = "ui-monospace,SFMono-Regular,'SF Mono',Menlo,Consolas,'Liberation Mono',monospace"

# ---------------------------------------------------------------- palettes
STAGE = dict(bg0="#04060c", bg1="#0a0f1e", text="#f5f7fb", sub="#a3acbf", faint="#5b6478",
             line="#1c2233", blue="#4ea1ff", cyan="#38d3f5", violet="#9b8cff", orange="#ff7a45")

THEME = {
    "light": dict(surface="#ffffff", border="#d1d9e0", text="#1f2328", sub="#59636e",
                  muted="#818b98", grid="#eef1f4", axis="#c8d1da",
                  blue="#2a78d6", orange="#eb6834", wash="#2a78d6"),
    "dark": dict(surface="#0d1117", border="#30363d", text="#f0f6fc", sub="#9198a1",
                 muted="#6e7681", grid="#1b2129", axis="#30363d",
                 blue="#3987e5", orange="#d95926", wash="#3987e5"),
}


def esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def T(x, y, s, size, fill, weight=400, anchor="start", family=SANS, extra=""):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-family="{family}" font-size="{size}" '
            f'font-weight="{weight}" fill="{fill}" text-anchor="{anchor}" {extra}>{esc(s)}</text>')


def svg_open(w, h, label):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
            f'viewBox="0 0 {w} {h}" role="img" aria-label="{esc(label)}">')


def hbar(x, y, w, h, fill, r=4):
    """Horizontal bar: square at the baseline (left), 4px rounded data-end (right)."""
    if w <= 0:
        return ""
    r = min(r, w, h / 2)
    return (f'<path d="M{x:.1f},{y:.1f} H{x + w - r:.1f} Q{x + w:.1f},{y:.1f} {x + w:.1f},{y + r:.1f} '
            f'V{y + h - r:.1f} Q{x + w:.1f},{y + h:.1f} {x + w - r:.1f},{y + h:.1f} H{x:.1f} Z" fill="{fill}"/>')


def card(w, h, t):
    return f'<rect x="0.5" y="0.5" width="{w - 1}" height="{h - 1}" rx="16" fill="{t["surface"]}" stroke="{t["border"]}"/>'


def legend(x, y, items, t):
    s = []
    for label, color in items:
        s.append(f'<rect x="{x}" y="{y - 10}" width="12" height="12" rx="3" fill="{color}"/>')
        s.append(T(x + 18, y, label, 14, t["sub"], 500))
        x += 18 + 8.2 * len(label) + 28
    return "".join(s)


# ============================================================ hero (animated)
def hero():
    W, H = 1280, 600
    CYCLE = 7.0                # seconds per loop
    SWEEP = 5.0                # playhead sweep duration
    x0, x1 = 330, 1190         # timeline
    n_win = 5
    px = lambda ms: x0 + (x1 - x0) * ms / n_win
    frac = lambda ms: (ms / n_win) * (SWEEP / CYCLE)       # time fraction in the loop

    rng = random.Random(11)
    pts = []
    for w in range(n_win):
        for _ in range(10 + rng.randint(-1, 2)):
            pts.append(round(w + rng.uniform(0.05, 0.95), 3))
    pts.sort()
    stamps = sorted(set(round(p, 2) for p in pts))

    s = [svg_open(W, H, "Batch-LIO: LiDAR-inertial odometry, one millisecond at a time"), "<defs>",
         f'<linearGradient id="bg" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="{STAGE["bg1"]}"/>'
         f'<stop offset="1" stop-color="{STAGE["bg0"]}"/></linearGradient>',
         '<radialGradient id="glow" cx="0.5" cy="0" r="0.75"><stop offset="0" stop-color="#2b5cff" stop-opacity="0.45"/>'
         '<stop offset="0.55" stop-color="#2b5cff" stop-opacity="0.06"/><stop offset="1" stop-color="#2b5cff" stop-opacity="0"/></radialGradient>',
         f'<linearGradient id="brand" x1="0" y1="0" x2="1" y2="0"><stop offset="0" stop-color="{STAGE["cyan"]}"/>'
         f'<stop offset="0.5" stop-color="{STAGE["blue"]}"/><stop offset="1" stop-color="{STAGE["violet"]}"/></linearGradient>',
         '<linearGradient id="head" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#ffffff" stop-opacity="0"/>'
         '<stop offset="0.5" stop-color="#ffffff" stop-opacity="0.9"/><stop offset="1" stop-color="#ffffff" stop-opacity="0"/></linearGradient>',
         '<pattern id="dots" width="28" height="28" patternUnits="userSpaceOnUse">'
         '<circle cx="1.5" cy="1.5" r="1.1" fill="#ffffff" fill-opacity="0.05"/></pattern>',
         '<filter id="soft" x="-50%" y="-50%" width="200%" height="200%"><feGaussianBlur stdDeviation="6"/></filter>',
         "</defs>",
         f'<rect width="{W}" height="{H}" rx="24" fill="url(#bg)"/>',
         f'<rect width="{W}" height="{H}" rx="24" fill="url(#dots)"/>',
         f'<rect width="{W}" height="{H}" rx="24" fill="url(#glow)"/>']

    # wordmark
    s.append(T(W / 2, 92, "INTRODUCING", 18, STAGE["sub"], 600, "middle", extra='letter-spacing="6"'))
    s.append(T(W / 2, 186, "Batch‑LIO", 104, "url(#brand)", 800, "middle", extra='letter-spacing="-3"'))
    s.append(T(W / 2, 236, "LiDAR‑inertial odometry, one millisecond at a time.", 30, STAGE["text"], 400, "middle",
               extra='fill-opacity="0.86"'))

    # stage panel
    py0 = 292
    s.append(f'<rect x="60" y="{py0}" width="{W - 120}" height="262" rx="18" fill="#ffffff" fill-opacity="0.025" '
             f'stroke="#ffffff" stroke-opacity="0.08"/>')

    def appear(ms, lo=0.18):
        f = frac(ms)
        return (f'<animate attributeName="opacity" dur="{CYCLE}s" repeatCount="indefinite" '
                f'values="{lo};{lo};1;1;{lo}" keyTimes="0;{f:.4f};{f + 0.004:.4f};0.94;1"/>')

    # lane A: Point-LIO
    ya = py0 + 82
    s.append(T(96, ya - 6, "Point‑LIO", 27, STAGE["text"], 700))
    s.append(T(96, ya + 22, f"{len(stamps)} updates / 5 ms", 18, STAGE["sub"], 500))
    s.append(f'<line x1="{x0}" y1="{ya}" x2="{x1}" y2="{ya}" stroke="#ffffff" stroke-opacity="0.12" stroke-width="1.5"/>')
    for u in stamps:
        f = frac(u)
        s.append(f'<rect x="{px(u) - 1.2:.1f}" y="{ya - 30}" width="2.4" height="16" rx="1.2" fill="{STAGE["orange"]}" opacity="0.15">'
                 f'<animate attributeName="opacity" dur="{CYCLE}s" repeatCount="indefinite" '
                 f'values="0.15;0.15;1;0.45;0.45;0.15" keyTimes="0;{f:.4f};{f + 0.004:.4f};{f + 0.05:.4f};0.94;1"/></rect>')
    for p in pts:
        s.append(f'<circle cx="{px(p):.1f}" cy="{ya}" r="4" fill="#c7cede" opacity="0.18">{appear(p)}</circle>')

    # lane B: Batch-LIO
    yb = py0 + 196
    s.append(T(96, yb - 6, "Batch‑LIO", 27, STAGE["text"], 700))
    s.append(T(96, yb + 22, f"{n_win} updates / 5 ms", 18, STAGE["sub"], 500))
    for w in range(n_win):
        a, b = px(w) + 5, px(w + 1) - 5
        fe = frac(w + 1)
        fs = frac(w)
        s.append(f'<rect x="{a:.1f}" y="{yb - 24}" width="{b - a:.1f}" height="48" rx="24" fill="{STAGE["blue"]}" '
                 f'fill-opacity="0.04" stroke="{STAGE["blue"]}" stroke-opacity="0.35" stroke-width="1.2">'
                 f'<animate attributeName="fill-opacity" dur="{CYCLE}s" repeatCount="indefinite" '
                 f'values="0.04;0.04;0.10;0.26;0.14;0.14;0.04" '
                 f'keyTimes="0;{fs:.4f};{fs + 0.004:.4f};{fe:.4f};{fe + 0.06:.4f};0.94;1"/>'
                 f'<animate attributeName="stroke-opacity" dur="{CYCLE}s" repeatCount="indefinite" '
                 f'values="0.35;0.35;1;0.6;0.6;0.35" keyTimes="0;{fe:.4f};{fe + 0.004:.4f};{fe + 0.06:.4f};0.94;1"/></rect>')
        # one update per window: a pulse at the window end
        cx = b
        s.append(f'<circle cx="{cx:.1f}" cy="{yb - 38}" r="6" fill="none" stroke="{STAGE["cyan"]}" stroke-width="2" opacity="0">'
                 f'<animate attributeName="r" dur="{CYCLE}s" repeatCount="indefinite" values="6;6;22;22" '
                 f'keyTimes="0;{fe:.4f};{fe + 0.07:.4f};1"/>'
                 f'<animate attributeName="opacity" dur="{CYCLE}s" repeatCount="indefinite" values="0;0;0.9;0;0" '
                 f'keyTimes="0;{fe:.4f};{fe + 0.004:.4f};{fe + 0.07:.4f};1"/></circle>')
        s.append(f'<path d="M{cx - 7:.1f},{yb - 44} L{cx + 7:.1f},{yb - 44} L{cx:.1f},{yb - 32} Z" fill="{STAGE["cyan"]}" opacity="0.2">'
                 f'<animate attributeName="opacity" dur="{CYCLE}s" repeatCount="indefinite" values="0.2;0.2;1;1;0.2" '
                 f'keyTimes="0;{fe:.4f};{fe + 0.004:.4f};0.94;1"/></path>')
    for p in pts:
        s.append(f'<circle cx="{px(p):.1f}" cy="{yb}" r="4" fill="{STAGE["blue"]}" opacity="0.2">{appear(p, 0.2)}</circle>')

    # time axis
    yt = py0 + 244
    for w in range(n_win + 1):
        s.append(T(px(w), yt, f"{w} ms", 16, STAGE["faint"], 500, "middle"))

    # playhead
    fend = SWEEP / CYCLE
    s.append(f'<g opacity="0"><rect x="-1" y="{py0 + 34}" width="2" height="196" fill="url(#head)"/>'
             f'<rect x="-6" y="{py0 + 34}" width="12" height="196" fill="url(#head)" opacity="0.35" filter="url(#soft)"/>'
             f'<animateTransform attributeName="transform" type="translate" dur="{CYCLE}s" repeatCount="indefinite" '
             f'values="{x0} 0;{x1} 0;{x1} 0" keyTimes="0;{fend:.4f};1"/>'
             f'<animate attributeName="opacity" dur="{CYCLE}s" repeatCount="indefinite" values="0;1;1;0;0" '
             f'keyTimes="0;0.02;{fend:.4f};{fend + 0.04:.4f};1"/></g>')
    s.append("</svg>")
    return "\n".join(s)


# ============================================================ stats banner
STATS = {
    "zh": [("4.7×", "每帧算力最高降低", "100 Hz 剧烈运动序列"),
           ("0.03%", "与基线的轨迹偏差", "103 m 穿行，平均 3.1 cm"),
           ("3.6×", "闭环误差更小", "outdoor_run：7.3 → 2.0 cm"),
           ("100%", "可回退到原版", "batch_dt = 0 即原版")],
    "en": [("4.7×", "less compute per frame", "100 Hz high‑dynamic run"),
           ("0.03%", "deviation from baseline", "103 m traverse, 3.1 cm mean"),
           ("3.6×", "lower loop‑closure error", "outdoor_run: 7.3 → 2.0 cm"),
           ("100%", "falls back to the original", "batch_dt = 0 ≡ Point‑LIO")],
}


def stats(lang):
    W, H = 1280, 260
    s = [svg_open(W, H, " · ".join(f"{a} {b}" for a, b, _ in STATS[lang])), "<defs>",
         f'<linearGradient id="bg" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="{STAGE["bg1"]}"/>'
         f'<stop offset="1" stop-color="{STAGE["bg0"]}"/></linearGradient>',
         f'<linearGradient id="brand" x1="0" y1="0" x2="1" y2="0"><stop offset="0" stop-color="{STAGE["cyan"]}"/>'
         f'<stop offset="0.5" stop-color="{STAGE["blue"]}"/><stop offset="1" stop-color="{STAGE["violet"]}"/></linearGradient>',
         '<radialGradient id="glow" cx="0.5" cy="1.2" r="0.9"><stop offset="0" stop-color="#2b5cff" stop-opacity="0.28"/>'
         '<stop offset="1" stop-color="#2b5cff" stop-opacity="0"/></radialGradient>',
         "</defs>",
         f'<rect width="{W}" height="{H}" rx="24" fill="url(#bg)"/>',
         f'<rect width="{W}" height="{H}" rx="24" fill="url(#glow)"/>']
    cw = W / 4
    for i, (big, label, sub) in enumerate(STATS[lang]):
        cx = cw * i + cw / 2
        if i:
            s.append(f'<line x1="{cw * i:.1f}" y1="56" x2="{cw * i:.1f}" y2="{H - 56}" stroke="#ffffff" stroke-opacity="0.1"/>')
        s.append(T(cx, 120, big, 76, "url(#brand)", 800, "middle", extra='letter-spacing="-2"'))
        s.append(T(cx, 166, label, 24, STAGE["text"], 600, "middle"))
        s.append(T(cx, 198, sub, 17, STAGE["sub"], 500, "middle"))
    s.append("</svg>")
    return "\n".join(s)


# ============================================================ chart: speedup
L = {
    "zh": dict(pl="Point‑LIO", bl="Batch‑LIO",
               sp_title="每帧算力，相对 Point‑LIO", sp_sub="每帧平均耗时，越短越好 · 纯 CPU，32 核 x86_64",
               seqs=[("outdoor_run", "100 Hz 高动态户外回环"), ("HKU_MB", "260 s 楼宇穿行，103 m"),
                     ("quick‑shack", "手持室内回环")],
               omp_title="分批，让并行成为可能", omp_sub="quick‑shack · 每帧平均耗时（ms），越短越好",
               omp_rows=["逐点", "逐点 + OpenMP", "分批 1 ms", "分批 1 ms + OpenMP"],
               omp_note1="OpenMP 反而慢 36%", omp_note2="OpenMP 再快 2 倍",
               sw_title="窗口长度：1–2 ms 最佳", sw_sub="quick‑shack · 开启 OpenMP 与去畸变 · 横轴为 batch_dt",
               sw_a="每帧耗时（ms）", sw_b="首尾漂移（m，对数刻度）", sw_band="最佳区间",
               ref="Point‑LIO"),
    "en": dict(pl="Point‑LIO", bl="Batch‑LIO",
               sp_title="Per‑frame compute, relative to Point‑LIO", sp_sub="Average time per frame, lower is better · CPU only, 32‑core x86_64",
               seqs=[("outdoor_run", "100 Hz high‑dynamic outdoor loop"), ("HKU_MB", "260 s building traverse, 103 m"),
                     ("quick‑shack", "handheld indoor loop")],
               omp_title="Batching makes parallelism pay off", omp_sub="quick‑shack · average time per frame (ms), lower is better",
               omp_rows=["point‑wise", "point‑wise + OpenMP", "batch 1 ms", "batch 1 ms + OpenMP"],
               omp_note1="OpenMP: 36% slower", omp_note2="OpenMP: 2× faster",
               sw_title="Window length: 1–2 ms is the sweet spot", sw_sub="quick‑shack · OpenMP and de‑skew on · x axis is batch_dt",
               sw_a="time per frame (ms)", sw_b="start‑to‑end drift (m, log scale)", sw_band="sweet spot",
               ref="Point‑LIO"),
}


def speedup(lang, theme):
    t, l = THEME[theme], L[lang]
    data = [(2.56, 0.54, "4.7×"), (16.21, 4.56, "3.6×"), (12.42, 3.51, "3.5×")]
    W, H = 960, 420
    gx, gw = 290, 430           # plot area
    s = [svg_open(W, H, l["sp_title"]), card(W, H, t)]
    s.append(T(40, 58, l["sp_title"], 24, t["text"], 700))
    s.append(T(40, 86, l["sp_sub"], 15, t["sub"]))
    s.append(legend(W - 250, 60, [(l["pl"], t["orange"]), (l["bl"], t["blue"])], t))
    top, bh, gap_in, gap_out = 128, 22, 6, 40
    y_end = top + 3 * (2 * bh + gap_in) + 2 * gap_out
    for k in range(5):
        x = gx + gw * k / 4
        s.append(f'<line x1="{x:.1f}" y1="{top - 14}" x2="{x:.1f}" y2="{y_end + 8}" stroke="{t["grid"]}" stroke-width="1"/>')
        s.append(T(x, y_end + 30, f"{25 * k}%", 13, t["muted"], 500, "middle"))
    s.append(f'<line x1="{gx}" y1="{top - 14}" x2="{gx}" y2="{y_end + 8}" stroke="{t["axis"]}" stroke-width="1"/>')
    y = top
    for (name, scene), (base, ours, sp) in zip(l["seqs"], data):
        s.append(T(40, y + 20, name, 18, t["text"], 700))
        s.append(T(40, y + 44, scene, 14, t["sub"]))
        s.append(hbar(gx, y, gw, bh, t["orange"]))
        s.append(T(gx + gw + 10, y + 16, f"{base:.2f} ms", 14, t["sub"], 500))
        wo = gw * ours / base
        s.append(hbar(gx, y + bh + gap_in, wo, bh, t["blue"]))
        s.append(T(gx + wo + 10, y + bh + gap_in + 16, f"{ours:.2f} ms", 14, t["text"], 600))
        s.append(T(W - 36, y + bh + 12, sp, 38, t["text"], 800, "end", extra='letter-spacing="-1"'))
        y += 2 * bh + gap_in + gap_out
    s.append("</svg>")
    return "\n".join(s)


# ============================================================ chart: OpenMP causality
def omp(lang, theme):
    t, l = THEME[theme], L[lang]
    vals = [7.69, 10.43, 6.72, 3.35]
    cols = [t["orange"], t["orange"], t["blue"], t["blue"]]
    W, H = 960, 396
    gx, gw, vmax = 250, 440, 12.0
    s = [svg_open(W, H, l["omp_title"]), card(W, H, t)]
    s.append(T(40, 58, l["omp_title"], 24, t["text"], 700))
    s.append(T(40, 86, l["omp_sub"], 15, t["sub"]))
    s.append(legend(W - 250, 60, [(l["pl"], t["orange"]), (l["bl"], t["blue"])], t))
    top, bh, step = 130, 24, 56
    y_end = top + 3 * step + bh
    for k in range(0, 13, 3):
        x = gx + gw * k / vmax
        s.append(f'<line x1="{x:.1f}" y1="{top - 14}" x2="{x:.1f}" y2="{y_end + 12}" stroke="{t["grid"]}"/>')
        s.append(T(x, y_end + 34, f"{k} ms", 13, t["muted"], 500, "middle"))
    s.append(f'<line x1="{gx}" y1="{top - 14}" x2="{gx}" y2="{y_end + 12}" stroke="{t["axis"]}"/>')
    for i, (name, v, c) in enumerate(zip(l["omp_rows"], vals, cols)):
        y = top + i * step
        s.append(T(gx - 16, y + 17, name, 16, t["text"], 600, "end"))
        w = gw * v / vmax
        s.append(hbar(gx, y, w, bh, c))
        s.append(T(gx + w + 10, y + 17, f"{v:.2f}", 15, t["text"], 600))
    # annotations (text tokens, bracket in axis colour)
    for (i, j), note in (((0, 1), l["omp_note1"]), ((2, 3), l["omp_note2"])):
        ya, yb = top + i * step + bh / 2, top + j * step + bh / 2
        xb = gx + gw + 62
        s.append(f'<path d="M{xb - 8},{ya} H{xb} V{yb} H{xb - 8}" fill="none" stroke="{t["axis"]}" stroke-width="1.5"/>')
        s.append(T(xb + 14, (ya + yb) / 2 + 6, note, 16, t["text"], 700))
    s.append("</svg>")
    return "\n".join(s)


# ============================================================ chart: batch_dt sweep (small multiples)
def sweep(lang, theme):
    t, l = THEME[theme], L[lang]
    dts = [0.5, 1, 2, 5, 10, 20]
    ms = [4.47, 3.44, 2.72, 2.22, 1.99, 1.85]
    drift = [0.092, 0.053, 0.066, 0.143, 3.67, 0.509]
    W, H = 960, 470
    s = [svg_open(W, H, l["sw_title"]), card(W, H, t)]
    s.append(T(40, 58, l["sw_title"], 24, t["text"], 700))
    s.append(T(40, 86, l["sw_sub"], 15, t["sub"]))
    s.append(legend(W - 250, 60, [(l["pl"], t["orange"]), (l["bl"], t["blue"])], t))

    def panel(x0, title, ys, fy, ticks, ref, ref_lbl, fmt, labels):
        pw, top, ph = 350, 150, 230
        xs = [x0 + pw * k / (len(dts) - 1) for k in range(len(dts))]
        out = [T(x0, 128, title, 16, t["text"], 700)]
        # sweet-spot band between 1 and 2 ms
        out.append(f'<rect x="{xs[1] - 18:.1f}" y="{top}" width="{xs[2] - xs[1] + 36:.1f}" height="{ph}" rx="8" '
                   f'fill="{t["wash"]}" fill-opacity="0.08"/>')
        out.append(T((xs[1] + xs[2]) / 2, top + 20, l["sw_band"], 13, t["sub"], 600, "middle"))
        for v, lab in ticks:
            y = top + ph - ph * fy(v)
            out.append(f'<line x1="{x0}" y1="{y:.1f}" x2="{x0 + pw}" y2="{y:.1f}" stroke="{t["grid"]}"/>')
            out.append(T(x0 - 10, y + 4, lab, 12, t["muted"], 500, "end"))
        out.append(f'<line x1="{x0}" y1="{top + ph}" x2="{x0 + pw}" y2="{top + ph}" stroke="{t["axis"]}"/>')
        for x, d in zip(xs, dts):
            out.append(T(x, top + ph + 24, f"{d:g} ms", 12, t["muted"], 500, "middle"))
        # reference: Point-LIO
        yr = top + ph - ph * fy(ref)
        out.append(f'<line x1="{x0}" y1="{yr:.1f}" x2="{x0 + pw}" y2="{yr:.1f}" stroke="{t["orange"]}" stroke-width="2"/>')
        out.append(T(x0 + pw, yr - 8, f'{l["ref"]} {fmt(ref)}', 12, t["sub"], 600, "end"))
        pts = [(x, top + ph - ph * fy(v)) for x, v in zip(xs, ys)]
        out.append('<polyline fill="none" stroke="{}" stroke-width="2.5" stroke-linejoin="round" stroke-linecap="round" '
                   'points="{}"/>'.format(t["blue"], " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)))
        for k, (x, y) in enumerate(pts):
            out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="{t["blue"]}" stroke="{t["surface"]}" stroke-width="2"/>')
            if k in labels:
                dy = labels[k]
                out.append(T(x, y + dy, fmt(ys[k]), 13, t["text"], 700, "middle"))
        return "".join(out)

    s.append(panel(76, l["sw_a"], ms, lambda v: v / 8.0,
                   [(0, "0"), (2, "2"), (4, "4"), (6, "6"), (8, "8")], 7.69, "", lambda v: f"{v:.2f} ms",
                   {0: -14, 1: -14, 2: -14, 5: -14}))
    lo, hi = math.log10(0.01), math.log10(10)
    fy = lambda v: (math.log10(v) - lo) / (hi - lo)
    s.append(panel(546, l["sw_b"], drift, fy,
                   [(0.01, "0.01"), (0.1, "0.1"), (1, "1"), (10, "10")], 0.072, "", lambda v: f"{v:g} m",
                   {1: 22, 2: 22, 4: -14}))
    s.append("</svg>")
    return "\n".join(s)


# ============================================================ pipeline
PIPE = {
    "zh": dict(boxes=[("LiDAR 点云", ["逐点时间戳"], False),
                      ("1 ms 时间窗", ["按时间分组", "而非按时间戳"], True),
                      ("窗内去畸变", ["p′ = Exp(ωΔt)·p", "+ R_Iᵀ·v·Δt"], True),
                      ("KNN + 平面拟合", ["逐点匹配", "OpenMP 并行"], True),
                      ("一次 IEKF 更新", ["残差行堆叠", "每窗口一次"], True),
                      ("iVox 地图", ["插入新点"], False)],
               state=("EKF 状态（IMU 传播）", "窗口末的 ω, v, R_I"), nn="最近邻", tag="改动",
               legend="相对 Point‑LIO 的改动", title="每个 1 ms 窗口的处理流程"),
    "en": dict(boxes=[("LiDAR scan", ["per‑point", "timestamps"], False),
                      ("1 ms windows", ["group by time,", "not by timestamp"], True),
                      ("In‑window de‑skew", ["p′ = Exp(ωΔt)·p", "+ R_Iᵀ·v·Δt"], True),
                      ("KNN + plane fit", ["per point,", "OpenMP‑parallel"], True),
                      ("One IEKF update", ["stacked residuals,", "once per window"], True),
                      ("iVox map", ["insert points"], False)],
               state=("EKF state (IMU‑propagated)", "ω, v, R_I at window end"), nn="nearest neighbours", tag="NEW",
               legend="changed vs Point‑LIO", title="What happens in every 1 ms window"),
}


def pipeline(lang, theme):
    t, p = THEME[theme], PIPE[lang]
    W, H = 1280, 440
    s = [svg_open(W, H, p["title"]), "<defs>",
         f'<marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
         f'<path d="M0,0 L10,5 L0,10 Z" fill="{t["muted"]}"/></marker>',
         f'<marker id="arb" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
         f'<path d="M0,0 L10,5 L0,10 Z" fill="{t["blue"]}"/></marker>',
         "</defs>", card(W, H, t)]
    s.append(T(40, 60, p["title"], 28, t["text"], 700))
    n, bw, gap, x0 = 6, 170, 36, 40
    by, bh = 210, 132
    centers = []
    for i, (title, lines, new) in enumerate(p["boxes"]):
        x = x0 + i * (bw + gap)
        centers.append(x + bw / 2)
        if new:
            s.append(f'<rect x="{x}" y="{by}" width="{bw}" height="{bh}" rx="14" fill="{t["blue"]}" fill-opacity="0.07" '
                     f'stroke="{t["blue"]}" stroke-width="1.5"/>')
            s.append(f'<rect x="{x + bw - 52}" y="{by - 11}" width="44" height="22" rx="11" fill="{t["blue"]}"/>')
            s.append(T(x + bw - 30, by + 4, p["tag"], 12, "#ffffff", 700, "middle"))
        else:
            s.append(f'<rect x="{x}" y="{by}" width="{bw}" height="{bh}" rx="14" fill="{t["surface"]}" stroke="{t["border"]}" stroke-width="1.5"/>')
        s.append(T(x + bw / 2, by + 46, title, 19, t["text"], 700, "middle"))
        for k, ln in enumerate(lines):
            fam = MONO if ("=" in ln or ln.startswith("+ R")) else SANS
            s.append(T(x + bw / 2, by + 78 + 24 * k, ln, 17 if fam == SANS else 15, t["sub"], 500, "middle", fam))
        if i < n - 1:
            s.append(f'<line x1="{x + bw + 5}" y1="{by + bh / 2}" x2="{x + bw + gap - 5}" y2="{by + bh / 2}" '
                     f'stroke="{t["muted"]}" stroke-width="1.6" marker-end="url(#ar)"/>')
    # state box feeding de-skew, fed by the update
    sw, sh = 330, 70
    sx, sy = centers[2] - sw / 2 + 40, 92
    s.append(f'<rect x="{sx}" y="{sy}" width="{sw}" height="{sh}" rx="14" fill="{t["surface"]}" stroke="{t["border"]}" stroke-width="1.5"/>')
    s.append(T(sx + sw / 2, sy + 28, p["state"][0], 19, t["text"], 700, "middle"))
    s.append(T(sx + sw / 2, sy + 52, p["state"][1], 16, t["sub"], 500, "middle", MONO if p["state"][1].isascii() else SANS))
    s.append(f'<line x1="{centers[2]}" y1="{sy + sh}" x2="{centers[2]}" y2="{by - 14}" stroke="{t["blue"]}" '
             f'stroke-width="1.8" marker-end="url(#arb)"/>')
    s.append(f'<path d="M{centers[4]},{by - 14} V{sy + sh / 2} H{sx + sw + 5}" fill="none" stroke="{t["muted"]}" '
             f'stroke-width="1.6" marker-end="url(#ar)"/>')
    # map -> KNN
    yb = by + bh + 30
    s.append(f'<path d="M{centers[5]},{by + bh + 5} V{yb} H{centers[3]} V{by + bh + 6}" fill="none" stroke="{t["muted"]}" '
             f'stroke-width="1.6" marker-end="url(#ar)"/>')
    s.append(T((centers[3] + centers[5]) / 2, yb + 24, p["nn"], 16, t["sub"], 500, "middle"))
    s.append(f'<rect x="40" y="{H - 44}" width="14" height="14" rx="4" fill="{t["blue"]}" fill-opacity="0.15" stroke="{t["blue"]}"/>')
    s.append(T(62, H - 31, p["legend"], 16, t["sub"], 500))
    s.append("</svg>")
    return "\n".join(s)


def main():
    files = {"hero.svg": hero()}
    for lang in ("zh", "en"):
        files[f"stats-{lang}.svg"] = stats(lang)
        for theme in ("light", "dark"):
            for stem, fn in (("speedup", speedup), ("omp", omp), ("sweep", sweep), ("pipeline", pipeline)):
                files[f"{stem}-{lang}-{theme}.svg"] = fn(lang, theme)
    for name, body in files.items():
        with open(os.path.join(OUT, name), "w", encoding="utf-8") as f:
            f.write(body + "\n")
    print(f"wrote {len(files)} files to {OUT}")


if __name__ == "__main__":
    main()
