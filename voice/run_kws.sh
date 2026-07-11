#!/usr/bin/env bash
# 启动离线 KWS → /moon/voice_cmd
set -euo pipefail
source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash
exec python3 /home/nvidia/moon/voice/kws_node.py "$@"
