#!/usr/bin/env bash
# 上电启动离线 KWS：等 ROS + 录音设备，再开麦监听
# 注意：不要 set -u，catkin setup.bash 会触发 unbound variable
set +u

LOG_DIR=/home/nvidia/moon/logs
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/moon_kws_boot.log"
echo "[moon-kws] $(date) boot wrapper enter" >>"$LOG_FILE"

source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash

export HOME=/home/nvidia
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export PATH="/home/nvidia/.local/bin:${PATH}"
export PYTHONPATH="/home/nvidia/.local/lib/python3.8/site-packages:${PYTHONPATH:-}"

exec >>"$LOG_FILE" 2>&1
echo "[moon-kws] $(date) starting URI=${ROS_MASTER_URI}"
echo "[moon-kws] waiting for ROS master ..."
for i in $(seq 1 90); do
  if rostopic list >/dev/null 2>&1; then
    echo "[moon-kws] ROS master ready"
    break
  fi
  sleep 2
  if [[ "$i" -eq 90 ]]; then
    echo "[moon-kws] WARN: ROS master not up after 180s, starting anyway"
  fi
done

exec python3 -u /home/nvidia/moon/voice/kws_node.py "$@"
