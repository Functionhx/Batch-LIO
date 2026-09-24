#!/usr/bin/env python3
"""README artwork for Batch-LIO: hero banner + pipeline diagram, light and dark variants.

Pure standard library; writes SVGs next to this file:
    python3 docs/assets/make_assets.py
The README picks the variant with <picture> + prefers-color-scheme.
"""
import os
import random

OUT = os.path.dirname(os.path.abspath(__file__))
FONT = "-apple-system,BlinkMacSystemFont,'Segoe UI','Noto Sans',Helvetica,Arial,sans-serif"
MONO = "ui-monospace,SFMono-Regular,'SF Mono',Menlo,Consolas,monospace"

THEMES = {
    "light": dict(bg0="#ffffff", bg1="#f3f6fa", text="#1f2328", muted="#59636e",
                  line="#d0d7de", grid="#e6eaef", base="#8c959f", accent="#0969da",
                  accent_fill="#ddf4ff", update="#bc4c00", box="#ffffff", box_line="#d0d7de"),
    "dark": dict(bg0="#0d1117", bg1="#161b22", text="#e6edf3", muted="#9198a1",
                 line="#30363d", grid="#21262d", base="#6e7681", accent="#4493f8",
                 accent_fill="#102a52", update="#f0883e", box="#161b22", box_line="#3d444d"),
}


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def text(x, y, s, size, fill, weight=400, anchor="start", family=FONT, extra=""):
    return (f'<text x="{x}" y="{y}" font-family="{family}" font-size="{size}" '
            f'font-weight="{weight}" fill="{fill}" text-anchor="{anchor}" {extra}>{esc(s)}</text>')


# ------------------------------------------------------------------ hero
def point_times(n_windows, per_window, seed=7):
    """Deterministic, irregular LiDAR point timestamps (ms), a few shared timestamps."""
    rng = random.Random(seed)
    ts = []
    for w in range(n_windows):
        k = per_window + rng.randint(-2, 2)
        base = sorted(round(w + rng.uniform(0.04, 0.96), 2) for _ in range(k))
        ts.extend(base)
    return ts


def hero(t):
    W, H = 1280, 500
    x0, x1 = 150, 1200          # timeline extent
    n_win = 5
    px = lambda ms: x0 + (x1 - x0) * ms / n_win
    ts = point_times(n_win, 11)
    uniq = sorted(set(ts))
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
         f'role="img" aria-label="Batch-LIO: one EKF update per 1 ms window instead of per timestamp">',
         '<defs><linearGradient id="bg" x1="0" y1="0" x2="0" y2="1">'
         f'<stop offset="0" stop-color="{t["bg0"]}"/><stop offset="1" stop-color="{t["bg1"]}"/>'
         '</linearGradient></defs>',
         f'<rect width="{W}" height="{H}" rx="18" fill="url(#bg)" stroke="{t["line"]}"/>']

    # title block
    s.append(text(64, 92, "Batch‑LIO", 60, t["text"], 800, extra='letter-spacing="-1"'))
    s.append(text(64, 132, "Point‑LIO, re‑batched: one EKF update per 1 ms window.",
                  21, t["muted"]))
    # stat chips (right)
    chips = [("2.3–4.7×", "less compute / frame"), ("bit‑exact", "Point‑LIO at batch_dt = 0")]
    cx = W - 64
    for big, small in reversed(chips):
        w = 214
        cx -= w
        s.append(f'<rect x="{cx}" y="46" width="{w}" height="86" rx="12" fill="{t["box"]}" stroke="{t["box_line"]}"/>')
        s.append(text(cx + w / 2, 88, big, 30, t["accent"], 800, "middle"))
        s.append(text(cx + w / 2, 116, small, 14, t["muted"], 500, "middle"))
        cx -= 14

    # --- row 1: Point-LIO
    y1 = 238
    s.append(text(64, y1 - 36, "Point‑LIO", 17, t["text"], 700))
    s.append(text(170, y1 - 36, f"one EKF update per distinct timestamp  ·  {len(uniq)} updates", 15, t["muted"]))
    s.append(f'<line x1="{x0}" y1="{y1}" x2="{x1}" y2="{y1}" stroke="{t["line"]}" stroke-width="2"/>')
    for u in uniq:
        s.append(f'<line x1="{px(u):.1f}" y1="{y1 - 20}" x2="{px(u):.1f}" y2="{y1 - 8}" '
                 f'stroke="{t["update"]}" stroke-width="2.2" stroke-linecap="round"/>')
    for v in ts:
        s.append(f'<circle cx="{px(v):.1f}" cy="{y1}" r="4.2" fill="{t["base"]}"/>')

    # --- row 2: Batch-LIO
    y2 = 360
    s.append(text(64, y2 - 50, "Batch‑LIO", 17, t["text"], 700))
    s.append(text(170, y2 - 50, f"de‑skew to window end, stack residuals, one update per window  ·  {n_win} updates",
                  15, t["muted"]))
    for w in range(n_win):
        a, b = px(w) + 4, px(w + 1) - 4
        s.append(f'<rect x="{a:.1f}" y="{y2 - 22}" width="{b - a:.1f}" height="44" rx="10" '
                 f'fill="{t["accent_fill"]}" stroke="{t["accent"]}" stroke-width="1.4"/>')
    s.append(f'<line x1="{x0}" y1="{y2}" x2="{x1}" y2="{y2}" stroke="{t["line"]}" stroke-width="2"/>')
    for v in ts:
        s.append(f'<circle cx="{px(v):.1f}" cy="{y2}" r="4.2" fill="{t["accent"]}"/>')
    for w in range(n_win):
        xe = px(w + 1) - 4
        s.append(f'<path d="M{xe - 7:.1f},{y2 - 38} L{xe + 7:.1f},{y2 - 38} L{xe:.1f},{y2 - 26} Z" fill="{t["update"]}"/>')

    # time axis
    ya = 412
    s.append(f'<line x1="{x0}" y1="{ya}" x2="{x1}" y2="{ya}" stroke="{t["line"]}" stroke-width="1.2"/>')
    for w in range(n_win + 1):
        s.append(f'<line x1="{px(w):.1f}" y1="{ya}" x2="{px(w):.1f}" y2="{ya + 6}" stroke="{t["line"]}"/>')
        s.append(text(px(w), ya + 24, f"{w} ms", 13, t["muted"], 500, "middle"))
    # legend
    lx, ly = x0, ya + 58
    s.append(f'<line x1="{lx}" y1="{ly - 11}" x2="{lx}" y2="{ly + 1}" stroke="{t["update"]}" stroke-width="2.2" stroke-linecap="round"/>')
    s.append(text(lx + 10, ly, "EKF update", 13, t["muted"]))
    s.append(f'<circle cx="{lx + 104}" cy="{ly - 5}" r="4.2" fill="{t["base"]}"/>')
    s.append(text(lx + 114, ly, "LiDAR point (Point‑LIO)", 13, t["muted"]))
    s.append(f'<circle cx="{lx + 290}" cy="{ly - 5}" r="4.2" fill="{t["accent"]}"/>')
    s.append(text(lx + 300, ly, "LiDAR point in a 1 ms window (Batch‑LIO)", 13, t["muted"]))
    s.append("</svg>")
    return "\n".join(s)


# -------------------------------------------------------------- pipeline
def pipeline(t):
    W, H = 1280, 330
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
         f'role="img" aria-label="Batch-LIO per-window pipeline">',
         '<defs>'
         f'<marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse">'
         f'<path d="M0,0 L10,5 L0,10 Z" fill="{t["muted"]}"/></marker>'
         f'<marker id="ara" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse">'
         f'<path d="M0,0 L10,5 L0,10 Z" fill="{t["accent"]}"/></marker>'
         '</defs>',
         f'<rect width="{W}" height="{H}" rx="18" fill="{t["bg0"]}" stroke="{t["line"]}"/>']

    by, bh, bw, gap = 150, 104, 180, 36
    x = 40
    boxes = [
        ("LiDAR scan", "irregular per‑point", "timestamps", False),
        ("1 ms windows", "group by time,", "not by timestamp", True),
        ("In‑window de‑skew", "p′ = Exp(ωΔt)·p", "+ R_Iᵀ·v·Δt", True),
        ("KNN + plane fit", "per point, OpenMP", "(batch_omp)", True),
        ("One IEKF update", "row‑stacked residuals", "per window", True),
    ]
    centers = []
    for title, l1, l2, new in boxes:
        stroke = t["accent"] if new else t["box_line"]
        fill = t["accent_fill"] if new else t["box"]
        s.append(f'<rect x="{x}" y="{by}" width="{bw}" height="{bh}" rx="12" fill="{fill}" stroke="{stroke}" stroke-width="1.5"/>')
        s.append(text(x + bw / 2, by + 36, title, 17, t["text"], 700, "middle"))
        fam = MONO if "=" in l1 else FONT
        s.append(text(x + bw / 2, by + 64, l1, 14, t["muted"], 500, "middle", fam))
        s.append(text(x + bw / 2, by + 85, l2, 14, t["muted"], 500, "middle", MONO if "(" in l2 or "·" in l2 else FONT))
        centers.append(x + bw / 2)
        x += bw + gap
    # arrows between boxes
    for i in range(len(boxes) - 1):
        xa = 40 + (i + 1) * bw + i * gap + 4
        s.append(f'<line x1="{xa}" y1="{by + bh / 2}" x2="{xa + gap - 8}" y2="{by + bh / 2}" '
                 f'stroke="{t["muted"]}" stroke-width="1.8" marker-end="url(#ar)"/>')
    # map box (right end)
    mx = 40 + 5 * (bw + gap) - gap + 30
    xu = 40 + 5 * bw + 4 * gap + 4
    s.append(f'<line x1="{xu}" y1="{by + bh / 2}" x2="{mx - 4}" y2="{by + bh / 2}" '
             f'stroke="{t["muted"]}" stroke-width="1.8" marker-end="url(#ar)"/>')
    s.append(f'<rect x="{mx}" y="{by}" width="{W - 40 - mx}" height="{bh}" rx="12" fill="{t["box"]}" stroke="{t["box_line"]}" stroke-width="1.5"/>')
    s.append(text((mx + W - 40) / 2, by + 44, "iVox map", 17, t["text"], 700, "middle"))
    s.append(text((mx + W - 40) / 2, by + 70, "insert", 14, t["muted"], 500, "middle"))
    # state box on top feeding de-skew
    sx, sy, sw, sh = centers[2] - 150, 30, 300, 62
    s.append(f'<rect x="{sx}" y="{sy}" width="{sw}" height="{sh}" rx="12" fill="{t["box"]}" stroke="{t["box_line"]}" stroke-width="1.5"/>')
    s.append(text(sx + sw / 2, sy + 27, "EKF state (IMU‑propagated)", 16, t["text"], 700, "middle"))
    s.append(text(sx + sw / 2, sy + 49, "ω, v, R_I at window end", 14, t["muted"], 500, "middle", MONO))
    s.append(f'<line x1="{centers[2]}" y1="{sy + sh}" x2="{centers[2]}" y2="{by - 4}" stroke="{t["accent"]}" '
             f'stroke-width="1.8" stroke-dasharray="5 4" marker-end="url(#ara)"/>')
    # update -> state feedback
    s.append(f'<path d="M{centers[4]},{by} L{centers[4]},{sy + sh / 2} L{sx + sw + 4},{sy + sh / 2}" fill="none" '
             f'stroke="{t["muted"]}" stroke-width="1.6" marker-end="url(#ar)"/>')
    # map -> KNN feedback (below)
    yb = by + bh + 34
    s.append(f'<path d="M{(mx + W - 40) / 2},{by + bh} L{(mx + W - 40) / 2},{yb} L{centers[3]},{yb} L{centers[3]},{by + bh + 4}" '
             f'fill="none" stroke="{t["muted"]}" stroke-width="1.6" marker-end="url(#ar)"/>')
    s.append(text((centers[3] + (mx + W - 40) / 2) / 2, yb + 22, "nearest neighbours", 13, t["muted"], 500, "middle"))
    # legend
    s.append(f'<rect x="40" y="{H - 34}" width="16" height="16" rx="4" fill="{t["accent_fill"]}" stroke="{t["accent"]}"/>')
    s.append(text(64, H - 21, "changed or added vs Point‑LIO", 13, t["muted"]))
    s.append("</svg>")
    return "\n".join(s)


def main():
    for name, th in THEMES.items():
        for stem, fn in (("hero", hero), ("pipeline", pipeline)):
            p = os.path.join(OUT, f"{stem}-{name}.svg")
            with open(p, "w", encoding="utf-8") as f:
                f.write(fn(th) + "\n")
            print("wrote", p)


if __name__ == "__main__":
    main()
