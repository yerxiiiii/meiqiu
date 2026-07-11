#!/usr/bin/env bash
# 实机带路：请先确认 RUNNING，并停掉 uwb-follow
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MOON_ROOT="$(cd "$ROOT/.." && pwd)"
# Prefer ~/sim2real (symlink ok). Override: export SIM2REAL_WS=...
# shellcheck source=/dev/null
source "${MOON_ROOT}/scripts/sim2real_env.sh"
moon_source_sim2real

if systemctl is-active --quiet uwb-follow.service 2>/dev/null; then
  echo "[WARN] uwb-follow.service 仍在运行，建议: sudo systemctl stop uwb-follow.service"
fi

TEXT="${1:-}"
EXTRA=()
if [[ -n "$TEXT" ]]; then
  EXTRA+=(--text "$TEXT" --once)
fi

# 可选环境变量: GUIDE_ENTER_RUNNING=1 GUIDE_OBSTACLE=1 GUIDE_UWB=1
[[ "${GUIDE_ENTER_RUNNING:-0}" == "1" ]] && EXTRA+=(--enter-running)
[[ "${GUIDE_OBSTACLE:-0}" == "1" ]] && EXTRA+=(--enable-obstacle)
[[ "${GUIDE_UWB:-0}" == "1" ]] && EXTRA+=(--enable-uwb)

exec python3 "$ROOT/guide_demo_node.py" "${EXTRA[@]}"
