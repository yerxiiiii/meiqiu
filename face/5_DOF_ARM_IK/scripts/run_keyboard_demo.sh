#!/usr/bin/env bash
# 键盘末端 IK + 实机（默认 --robot；仅算 IK 用 --sim-only）
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
export PYTHONPATH="${ROOT}:${PYTHONPATH:-}"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/noetic/setup.bash
fi
if [ -f "${HOME}/sim2real/devel/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "${HOME}/sim2real/devel/setup.bash"
elif [ -f "${HOME}/sim2real/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "${HOME}/sim2real/install/setup.bash"
fi

EXTRA=()
HAS_MODE=0
for arg in "$@"; do
  case "${arg}" in
    --sim-only|--robot|--dry-run) HAS_MODE=1 ;;
  esac
done
if [[ "${HAS_MODE}" -eq 0 ]]; then
  EXTRA=(--robot)
fi

exec python3 "${ROOT}/scripts/keyboard_teleop_demo.py" "${EXTRA[@]}" "$@"
