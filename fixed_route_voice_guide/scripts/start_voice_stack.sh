#!/usr/bin/env bash
# 语音跟随一键启动 + 状态日志（开多个终端时用）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$ROOT/logs"
mkdir -p "$LOG_DIR"

source /home/nvidia/sim2real/install/setup.bash

pkill -f kws_trigger_node.py 2>/dev/null || true
pkill -f guide_demo_node.py 2>/dev/null || true
pkill -f mode_arbiter.py 2>/dev/null || true
sleep 0.5

MODE="${1:-run}"

case "$MODE" in
  run)
    echo "=== 启动 KWS（前台）==="
    echo "另开终端看状态日志："
    echo "  bash $ROOT/scripts/start_voice_stack.sh logs"
    echo "  bash $ROOT/scripts/start_voice_stack.sh watch"
    cd "$ROOT"
    exec python3 -u scripts/kws_trigger_node.py
    ;;
  logs)
    echo "=== tail 状态日志（Ctrl+C 退出）==="
    touch "$LOG_DIR/mode_arbiter_from_kws.log" "$LOG_DIR/guide_demo_from_kws.log"
    tail -n 30 -F "$LOG_DIR/mode_arbiter_from_kws.log" "$LOG_DIR/guide_demo_from_kws.log"
    ;;
  watch)
    echo "=== ROS 状态（Ctrl+C 退出）==="
    watch -n 1 "echo '--- /moon/mode ---'; timeout 0.5 rostopic echo /moon/mode -n 1 2>/dev/null; echo; echo '--- /fsm_state ---'; timeout 0.5 rostopic echo /fsm_state -n 1 2>/dev/null; echo; echo '--- cmd_vel pubs ---'; rostopic info /cmd_vel 2>/dev/null | sed -n '/Publishers:/,/Subscribers:/p'"
    ;;
  *)
    echo "用法: $0 [run|logs|watch]"
    exit 1
    ;;
esac
