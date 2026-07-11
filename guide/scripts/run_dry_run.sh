#!/usr/bin/env bash
# dry-run 冒烟：不控机器人
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash
TEXT="${1:-小派带我去炳胜餐厅}"
exec python3 "$ROOT/guide_demo_node.py" --dry-run --once --text "$TEXT"
