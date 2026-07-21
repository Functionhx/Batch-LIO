#!/bin/bash
# Headless batch-LIO ROS2 run harness (port of scripts/run_lio.sh).
# Usage: run_lio.sh <config.yaml> <bag_dir> <out_dir> [rate] [pkg_src_dir] ["-p overrides..."]
#   config.yaml   : absolute path to a ROS2 params-file (config/*.yaml)
#   bag_dir       : directory of a converted ROS2 mcap bag
#   out_dir       : where to put node.log, play.log, odom/, pos_log.txt
#   rate          : ros2 bag play rate (default 1.0)
#   pkg_src_dir   : package source dir whose Log/pos_log.txt to harvest (creates Log/)
#   overrides     : extra "-p key:=value" params, e.g. "-p batch_dt:=0.001 -p batch_omp:=true"
# Type-safe param sweeps use `ros2 run ... --ros-args --params-file <cfg> -p key:=val`
# (the -p form infers types; declare_parameter in the node uses dynamic_typing + coercion).

CFG="$1"; BAG="$2"; OUTDIR="$3"; RATE="${4:-1.0}"; LOGSRC="${5:-}"; PARAMS="${6:-}"
mkdir -p "$OUTDIR"

source /opt/ros/humble/setup.bash
source "${HOME}/batch_lio_ws/install/setup.bash"

# ensure Log dir exists (the node writes Log/pos_log.txt under ROOT_DIR == pkg_src_dir)
if [ -n "$LOGSRC" ]; then mkdir -p "$LOGSRC/Log"; fi

# clean leftovers
pkill -f batchlio_mapping 2>/dev/null || true
sleep 1

# start node (config from file, batch params overridden via -p for type-safe sweeps)
# shellcheck disable=SC2086
ros2 run batch_lio batchlio_mapping --ros-args --params-file "$CFG" $PARAMS \
    > "$OUTDIR/node.log" 2>&1 &
NODE_PID=$!
for _ in $(seq 1 60); do ros2 node list 2>/dev/null | grep -q '/laserMapping' && break; sleep 0.3; done
sleep 1

# record odometry output
ros2 bag record -o "$OUTDIR/odom" /aft_mapped_to_init > "$OUTDIR/record.log" 2>&1 &
REC_PID=$!
sleep 1

# play the bag (foreground; waits until finished)
echo "[run_lio] playing $BAG at rate $RATE (params: $PARAMS)"
ros2 bag play -r "$RATE" "$BAG" > "$OUTDIR/play.log" 2>&1

# drain + clean shutdown
sleep 3
kill "$REC_PID" 2>/dev/null || true
sleep 1
kill "$NODE_PID" 2>/dev/null || true
pkill -f batchlio_mapping 2>/dev/null || true
sleep 1

# harvest runtime log
if [ -n "$LOGSRC" ] && [ -f "$LOGSRC/Log/pos_log.txt" ]; then
  cp "$LOGSRC/Log/pos_log.txt" "$OUTDIR/pos_log.txt"
fi

echo "[run_lio] DONE -> $OUTDIR"
