#!/usr/bin/env bash
# 启动离线 KWS → /moon/voice_cmd
set -euo pipefail
# Prefer ~/sim2real (symlink ok). Override: export SIM2REAL_WS=...
# shellcheck source=/dev/null
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../scripts/sim2real_env.sh" 2>/dev/null \
  || source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../../scripts/sim2real_env.sh"
moon_source_sim2real
exec python3 /home/nvidia/moon/voice/kws_node.py "$@"
