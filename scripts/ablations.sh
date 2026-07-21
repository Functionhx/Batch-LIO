#!/bin/bash
# batch-LIO ROS2 ablations: output bandwidth, batch_dt sweep, aggressive-bag deskew.
# Bags must be converted first (scripts/convert_bag.py) into ~/batch_lio_ws/bags/.
# The in-ROS2 A/B compares batch_dt=0.0 (point-wise, == Point-LIO behavior) against
# batch_dt>0 (batch). runtime_pos_log_enable:=true emits pos_log.txt + [ mapping ] timings
# that scripts/compare_traj.py parses.
PKG=/home/as/vllm/Batch-LIO
CFG="$PKG/config/avia.yaml"
RL="$PKG/scripts/run_lio.sh"
OUT="$PKG/run/out"
BAGS="${HOME}/batch_lio_ws/bags"
QS="$BAGS/quick-shack"
AGG="$BAGS/outdoor_run"
RUNLOG="-p runtime_pos_log_enable:=true"

mkdir -p "$OUT"

echo "===== ROUND A: per-frame cost (quick-shack) — point-wise vs 1ms batch ====="
$RL "$CFG" "$QS" "$OUT/bw_pointwise" 1.0 "$PKG" "$RUNLOG -p batch_dt:=0.0 -p batch_omp:=true"
$RL "$CFG" "$QS" "$OUT/bw_1ms"       1.0 "$PKG" "$RUNLOG -p batch_dt:=0.001 -p batch_omp:=true"

echo "===== ROUND B: batch_dt sweep (quick-shack, omp on, deskew on) ====="
for dt in 0.0005 0.001 0.002 0.005 0.01 0.02; do
  $RL "$CFG" "$QS" "$OUT/sweep_${dt}" 1.0 "$PKG" "$RUNLOG -p batch_dt:=$dt -p batch_omp:=true -p batch_deskew:=true"
done

echo "===== ROUND C: aggressive-bag deskew (outdoor_run) ====="
$RL "$CFG" "$AGG" "$OUT/agg_pointwise"   1.0 "$PKG" "$RUNLOG -p batch_dt:=0.0   -p batch_omp:=true"
$RL "$CFG" "$AGG" "$OUT/agg_1ms_deskew"  1.0 "$PKG" "$RUNLOG -p batch_dt:=0.001 -p batch_omp:=true -p batch_deskew:=true"
$RL "$CFG" "$AGG" "$OUT/agg_1ms_noskew"  1.0 "$PKG" "$RUNLOG -p batch_dt:=0.001 -p batch_omp:=true -p batch_deskew:=false"
echo ABLATIONS_DONE
