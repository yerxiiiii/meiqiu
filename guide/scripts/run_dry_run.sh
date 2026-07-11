#!/usr/bin/env bash
# dry-run 冒烟：不控机器人
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MOON_ROOT="$(cd "$ROOT/.." && pwd)"
# Prefer ~/sim2real (symlink ok). Override: export SIM2REAL_WS=...
# shellcheck source=/dev/null
source "${MOON_ROOT}/scripts/sim2real_env.sh"
moon_source_sim2real
TEXT="${1:-小派带我去炳胜餐厅}"
exec python3 "$ROOT/guide_demo_node.py" --dry-run --once --text "$TEXT"
