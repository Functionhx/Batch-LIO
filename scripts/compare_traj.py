#!/usr/bin/env python3
"""batch-LIO A/B analysis.

Reads Point-LIO / batch-LIO pos_log.txt (per-frame state dump) and the node.log
(per-frame "[ mapping ]" timing prints), and reports efficiency + trajectory
metrics. Optionally diffs a run's trajectory against a baseline run.

pos_log.txt columns (use_imu_as_input=0 / kf_output model, see
laserMapping.cpp dump_lio_state_to_log):
  1:t  2-4:roll,pitch,yaw  5-7:px,py,pz  8-10:omega(0)  11-13:vx,vy,vz  ...

Usage:
  compare_traj.py LABEL POS_LOG NODE_LOG [BASELINE_POS_LOG]
"""
import sys
import re
import math


def read_pos(path):
    """Return list of (t, (px,py,pz)) from a pos_log.txt."""
    rows = []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) < 7:
                continue
            try:
                t = float(p[0])
                pos = (float(p[4]), float(p[5]), float(p[6]))
            except ValueError:
                continue
            rows.append((t, pos))
    return rows


def traj_metrics(rows):
    """Trajectory length and start->end drift (return-to-origin error)."""
    if len(rows) < 2:
        return 0.0, 0.0
    length = 0.0
    for (_, a), (_, b) in zip(rows[:-1], rows[1:]):
        length += math.dist(a, b)
    drift = math.dist(rows[0][1], rows[-1][1])
    return length, drift


def last_timing(node_log):
    """Parse the last complete '[ mapping ]' timing line -> (total, icp, propag) seconds."""
    pat = re.compile(
        r"ave total:\s*([0-9.]+).*?icp:\s*([0-9.]+).*?propogate:\s*([0-9.]+)")
    best = None
    with open(node_log, errors="ignore") as f:
        for line in f:
            line = re.sub(r"\x1b\[[0-9;]*m", "", line)  # strip ANSI
            m = pat.search(line)
            if m:
                best = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    return best


def diff_vs_base(rows, base_rows):
    """Per-frame |Δpos|, aligned by the bag timestamp rather than row index."""
    base_by_time = {round(t, 6): pos for t, pos in base_rows}
    deltas = [math.dist(pos, base_by_time[round(t, 6)])
              for t, pos in rows if round(t, 6) in base_by_time]
    if not deltas:
        return None
    ordered = sorted(deltas)
    p95 = ordered[int(0.95 * (len(ordered) - 1))]
    return sum(deltas) / len(deltas), p95, max(deltas), len(deltas)


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(1)
    label, pos_log, node_log = sys.argv[1], sys.argv[2], sys.argv[3]
    base = sys.argv[4] if len(sys.argv) > 4 else None

    rows = read_pos(pos_log)
    length, drift = traj_metrics(rows)
    timing = last_timing(node_log)

    print(f"==== {label} ====")
    print(f"  frames            : {len(rows)}")
    print(f"  trajectory length : {length:.3f} m")
    print(f"  start->end drift  : {drift:.4f} m")
    if timing:
        tot, icp, prop = (x * 1000 for x in timing)
        print(f"  avg total / icp / propag : {tot:.3f} / {icp:.3f} / {prop:.3f} ms")
    else:
        print("  timing            : (not found)")
    if base:
        d = diff_vs_base(rows, read_pos(base))
        if d:
            print(f"  vs baseline       : mean|Δpos|={d[0]:.4f} m  "
                  f"p95={d[1]:.4f} m  max={d[2]:.4f} m  "
                  f"(matched timestamps={d[3]})")
    print()


if __name__ == "__main__":
    main()
