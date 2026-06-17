#!/bin/bash
# Headless Point-LIO / batch-LIO run harness.
# Usage: run_lio.sh <launch_abs_path> <bag_abs_path> <out_dir> [rate] [log_src_dir]
#   launch_abs_path : absolute path to a .launch file
#   bag_abs_path    : absolute path to a rosbag
#   out_dir         : where to put node.log, play.log, odom.bag, pos_log.txt
#   rate            : rosbag play rate (default 1.0)
#   log_src_dir     : package dir whose Log/pos_log.txt to harvest (optional)

LAUNCH="$1"; BAG="$2"; OUTDIR="$3"; RATE="${4:-1.0}"; LOGSRC="$5"; LAUNCH_ARGS="$6"
mkdir -p "$OUTDIR"

source /opt/ros/noetic/setup.bash
source /root/batch-lio/catkin_ws/devel/setup.bash

# clean any leftovers
pkill -f lio_mapping 2>/dev/null
pkill -f rosmaster 2>/dev/null
pkill -f roscore 2>/dev/null
sleep 1

roscore > "$OUTDIR/roscore.log" 2>&1 &
until rostopic list >/dev/null 2>&1; do sleep 0.3; done

# start node
roslaunch "$LAUNCH" $LAUNCH_ARGS > "$OUTDIR/node.log" 2>&1 &
# wait for node registration (max ~20s)
for i in $(seq 1 60); do rosnode list 2>/dev/null | grep -q laserMapping && break; sleep 0.3; done
sleep 1

# record odometry output
rosbag record -O "$OUTDIR/odom.bag" /aft_mapped_to_init __name:=odomrec > "$OUTDIR/record.log" 2>&1 &
sleep 1

# play the bag (foreground; waits until finished)
echo "[run_lio] playing $BAG at rate $RATE"
rosbag play -r "$RATE" "$BAG" > "$OUTDIR/play.log" 2>&1

# drain + clean shutdown
sleep 3
rosnode kill /odomrec 2>/dev/null
sleep 2
rosnode kill /laserMapping 2>/dev/null
sleep 2
pkill -f lio_mapping 2>/dev/null
pkill -f rosmaster 2>/dev/null
pkill -f roscore 2>/dev/null
sleep 1

# harvest runtime log
if [ -n "$LOGSRC" ] && [ -f "$LOGSRC/Log/pos_log.txt" ]; then
  cp "$LOGSRC/Log/pos_log.txt" "$OUTDIR/pos_log.txt"
fi

echo "[run_lio] DONE -> $OUTDIR"
