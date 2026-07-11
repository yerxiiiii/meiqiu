#!/usr/bin/env bash
# 干跑验收：arbiter --dry-run + 模拟口令（不控腿）
set -euo pipefail
source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash

AID=""
cleanup() {
  if [[ -n "${AID}" ]]; then
    kill -INT "${AID}" 2>/dev/null || true
    sleep 0.5
    kill -9 "${AID}" 2>/dev/null || true
  fi
  pkill -f 'moon/brain/mode_arbiter.py' 2>/dev/null || true
}
trap cleanup EXIT

if ! timeout 2 rostopic list >/dev/null 2>&1; then
  echo "=== start temporary roscore ==="
  roscore >/tmp/moon_roscore_dryrun.log 2>&1 &
  for _ in $(seq 1 20); do
    if timeout 1 rostopic list >/dev/null 2>&1; then
      break
    fi
    sleep 0.5
  done
fi

echo "=== 1) unit tests ==="
python3 /home/nvidia/moon/brain/test_brain_unit.py

echo ""
echo "=== 2) start arbiter dry-run ==="
python3 -u /home/nvidia/moon/brain/mode_arbiter.py --dry-run --no-camera-manage &
AID=$!
sleep 2
if ! kill -0 "${AID}" 2>/dev/null; then
  echo "arbiter failed to start"
  exit 1
fi

echo "=== 3) simulate voice cmds ==="
python3 /home/nvidia/moon/voice/voice_sim.py --once face_look
sleep 0.8
python3 /home/nvidia/moon/voice/voice_sim.py --once uwb_follow
sleep 0.8
python3 /home/nvidia/moon/voice/voice_sim.py --once stop
sleep 0.8

echo "=== 4) /moon/mode ==="
timeout 3 rostopic echo -n 1 /moon/mode || true

echo ""
echo "dry-run ladder OK (no leg control)"
