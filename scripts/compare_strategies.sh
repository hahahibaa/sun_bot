#!/bin/bash
# Run the three strategies (parked, greedy, brain) for several cloud seeds, headless.
#   bash ~/sun_ws/src/sun_bot/scripts/compare_strategies.sh 1 2 3 4 7
# Each day: 07:00-17:45 at 1 day-min per sim-s (~13 min). Logs go to ~/sun_ws/docs/seeds/
# as run_<strategy>_s<seed>.csv; runs whose CSV already exists are skipped (resumable).
# Then: ros2 run sun_bot sun_report --compare-seeds ~/sun_ws/docs/seeds
OUT=~/sun_ws/docs/seeds
mkdir -p $OUT
source /opt/ros/humble/setup.bash
source ~/sun_ws/install/setup.bash

stop_all() {
  local lp left
  lp=$(ps -eo pid,args | awk '$0 ~ /ros2 launch sun_bot/ && $0 !~ /awk/ {print $1}')
  [ -n "$lp" ] && kill -INT $lp
  for i in $(seq 1 40); do
    ps -eo args | grep -qE '^ign gazebo|^/opt/ros/humble/lib/|lib/sun_bot/' || break
    sleep 1
  done
  left=$(ps -eo pid,args | awk '$2 ~ /^(ign|\/opt\/ros\/humble\/lib\/)/ || $3 ~ /lib\/sun_bot\// {print $1}')
  [ -n "$left" ] && { kill -TERM $left; sleep 5; kill -KILL $left 2>/dev/null; }
  timeout 15 ros2 daemon stop >/dev/null 2>&1
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
}

run_one() {   # strategy seed
  local csv=$OUT/run_$1_s$2.csv log=$OUT/run_$1_s$2.log
  [ -s "$csv" ] && grep -q . "$csv" && [ "$(wc -l < $csv)" -gt 6000 ] && {
    echo "[$1 seed $2] exists, skipped"; return 0; }
  for attempt in 1 2 3; do
    stop_all
    rm -f "$csv"
    # a background job of a non-interactive shell ignores Ctrl+C: restore SIGINT before exec
    python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
      ros2 launch sun_bot sun_day.launch.py gui:=false rviz:=false clouds:=true seed:=$2 \
      strategy:=$1 log:=$csv > $log 2>&1 &
    for i in $(seq 1 90); do
      [ "$(grep -c 'Managed nodes are active' $log)" -ge 3 ] && break
      sleep 1
    done
    if [ "$(grep -c 'Managed nodes are active' $log)" -ge 3 ]; then
      for i in $(seq 1 1500); do grep -q 'rows written' $log && break; sleep 1; done
      echo "[$1 seed $2] $(grep -o '[0-9]* rows written' $log) ($(date +%H:%M))"
      stop_all
      return 0
    fi
    echo "[$1 seed $2] attempt $attempt: start-up hung, retrying"
  done
  stop_all
  return 1
}

SEEDS="${*:-1 2 3 4 7}"
for seed in $SEEDS; do
  for strategy in parked greedy brain; do
    run_one $strategy $seed
  done
done
