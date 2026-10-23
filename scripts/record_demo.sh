#!/bin/bash
# Record the scripted demo day: RViz -> ~/sun_ws/docs/demo_rviz.mp4, log -> docs/demo_run.csv
#   bash ~/sun_ws/src/sun_bot/scripts/record_demo.sh            (extra args go to the launch)
# Then:  ros2 run sun_bot sun_replay ~/sun_ws/docs/demo_run.csv   (top-down replay video)
DOCS=~/sun_ws/docs
LOG=/tmp/sun_demo_launch.log
source /opt/ros/humble/setup.bash
source ~/sun_ws/install/setup.bash
rm -f $DOCS/demo_run.csv

# a background job of a non-interactive shell ignores Ctrl+C: restore SIGINT before exec
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  ros2 launch sun_bot sun_day.launch.py demo:=true "$@" > $LOG 2>&1 &
LAUNCH=$!
echo "demo launched (log: $LOG); waiting for Nav2 + collision monitor ..."
for i in $(seq 1 120); do
  [ "$(grep -c 'Managed nodes are active' $LOG)" -ge 3 ] && break
  sleep 1
done
if [ "$(grep -c 'Managed nodes are active' $LOG)" -lt 3 ]; then
  echo "start-up hung, stopping (just run the script again)"; kill -INT $LAUNCH; exit 1
fi
# run the recorder as its own process (not under `ros2 run`), so Ctrl+C reaches it directly
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  python3 -m sun_bot.record_window --title 'demo.rviz' --out $DOCS/demo_rviz.mp4 --fps 10 --speedup 5 &
REC=$!
echo "recording RViz as a 5x time-lapse; the day runs 07:00 -> 17:45 (~11 min) ..."
for i in $(seq 1 1500); do grep -q 'rows written' $LOG && break; sleep 1; done
sleep 3
kill -INT $REC; wait $REC
kill -INT $LAUNCH
for i in $(seq 1 40); do kill -0 $LAUNCH 2>/dev/null || break; sleep 1; done
echo "done: $DOCS/demo_rviz.mp4 and $DOCS/demo_run.csv"
