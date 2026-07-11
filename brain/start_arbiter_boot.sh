#!/usr/bin/env bash
# 上电启动中央决策 mode_arbiter
set +u

LOG_DIR=/home/nvidia/moon/logs
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/moon_arbiter_boot.log"
echo "[moon-arbiter] $(date) boot wrapper enter" >>"$LOG_FILE"

# Prefer ~/sim2real (symlink ok). Override: export SIM2REAL_WS=...
# shellcheck source=/dev/null
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../scripts/sim2real_env.sh" 2>/dev/null \
  || source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../../scripts/sim2real_env.sh"
moon_source_sim2real

export HOME=/home/nvidia
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export PATH="/home/nvidia/.local/bin:${PATH}"
export PYTHONPATH="/home/nvidia/.local/lib/python3.8/site-packages:${PYTHONPATH:-}"

exec >>"$LOG_FILE" 2>&1
echo "[moon-arbiter] $(date) starting"
echo "[moon-arbiter] waiting for ROS master ..."
for i in $(seq 1 90); do
  if rostopic list >/dev/null 2>&1; then
    echo "[moon-arbiter] ROS master ready"
    break
  fi
  sleep 2
done

exec python3 -u /home/nvidia/moon/brain/mode_arbiter.py "$@"
