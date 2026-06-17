#!/bin/bash
# batch-LIO ablations: output bandwidth, batch_dt sweep, aggressive bag.
B=/root/batch-lio
QS=$B/bags/2020-09-16-quick-shack.bag
AGG=$B/bags/outdoor_run_100Hz_2020-12-27-17-12-19.bag
RL=$B/run/run_lio.sh
LB=$B/run/avia_batch.launch
BL=$B/run/avia_baseline.launch

echo "===== ROUND A: output bandwidth ====="
# point-wise (batch_dt=0) publishing per group => Point-LIO native high rate (kHz, noisy)
$RL $LB $QS $B/run/out/bw_pointwise_hifreq 1.0 $B/Batch-LIO "batch_dt:=0.0   pub_hifreq:=1 batch_omp:=1"
# 1ms batch publishing per window => ~1000 Hz
$RL $LB $QS $B/run/out/bw_1ms_hifreq       1.0 $B/Batch-LIO "batch_dt:=0.001 pub_hifreq:=1 batch_omp:=1"

echo "===== ROUND B: batch_dt sweep (omp on, deskew on) ====="
for dt in 0.0005 0.001 0.002 0.005 0.01 0.02; do
  $RL $LB $QS $B/run/out/sweep_${dt} 1.0 $B/Batch-LIO "batch_dt:=$dt batch_omp:=1 batch_deskew:=1"
done

echo "===== ROUND C: aggressive bag (outdoor_run_100Hz, 63s) ====="
$RL $BL $AGG $B/run/out/agg_baseline   1.0 $B/Point-LIO
$RL $LB $AGG $B/run/out/agg_1ms_deskew 1.0 $B/Batch-LIO "batch_dt:=0.001 batch_omp:=1 batch_deskew:=1"
$RL $LB $AGG $B/run/out/agg_1ms_noskew 1.0 $B/Batch-LIO "batch_dt:=0.001 batch_omp:=1 batch_deskew:=0"
echo ABLATIONS_DONE
