#!/usr/bin/env bash
# Install amp_right_hold configs from moon/sim2real into the local sim2real workspace.
#
# Usage:
#   ./scripts/install_amp_right_hold.sh
#   SIM2REAL_WS=/path/to/ws ./scripts/install_amp_right_hold.sh
#
# Copies into:
#   $SIM2REAL_WS/install/share/sim2real/...
# and (if present) src tree:
#   $SIM2REAL_WS/src/sim2real/...

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MOON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/sim2real_env.sh"

SRC_CFG="${MOON_ROOT}/sim2real/config"
SHARE="${SIM2REAL_WS}/install/share/sim2real"

if [ ! -d "${SRC_CFG}" ]; then
  echo "[ERR] missing snapshot: ${SRC_CFG}" >&2
  exit 1
fi

if [ ! -d "${SHARE}" ]; then
  echo "[ERR] sim2real share not found: ${SHARE}" >&2
  echo "      Set SIM2REAL_WS to your workspace (must contain install/share/sim2real)." >&2
  exit 1
fi

echo "[INFO] SIM2REAL_WS=${SIM2REAL_WS}"
echo "[INFO] installing amp_right_hold configs → ${SHARE}"

mkdir -p "${SHARE}/config/walk" "${SHARE}/config/pi_plus_22dof_config"

cp -v "${SRC_CFG}/walk/amp_pi_plus_20dof_right_hold.yaml" "${SHARE}/config/walk/"
cp -v "${SRC_CFG}/walk/amp_pi_plus_20dof.yaml" "${SHARE}/config/walk/"
cp -v "${SRC_CFG}/walk/lr.yaml" "${SHARE}/config/walk/"
cp -v "${SRC_CFG}/walk/README_amp_right_hold.md" "${SHARE}/config/walk/" 2>/dev/null || true
cp -v "${SRC_CFG}/pi_plus_22dof_config/pi_plus_22dof_rl_config.yaml" \
  "${SHARE}/config/pi_plus_22dof_config/"
cp -v "${SRC_CFG}/pi_plus_22dof_config/pi_plus_22dof_pd_config.yaml" \
  "${SHARE}/config/pi_plus_22dof_config/"

# This robot (and many Orin images) use production_type=custom → also install custom registry
if [ -f "${SRC_CFG}/pi_plus_22dof_config/pi_plus_22dof_rl_config_custom.yaml" ]; then
  cp -v "${SRC_CFG}/pi_plus_22dof_config/pi_plus_22dof_rl_config_custom.yaml" \
    "${SHARE}/config/pi_plus_22dof_config/"
fi

# Optional: also sync into catkin src if present
for SRC_ROOT in \
  "${SIM2REAL_WS}/src/sim2real" \
  "${SIM2REAL_WS}/../src/sim2real"
do
  if [ -d "${SRC_ROOT}/config" ]; then
    echo "[INFO] also syncing → ${SRC_ROOT}/config"
    mkdir -p "${SRC_ROOT}/config/walk" "${SRC_ROOT}/config/pi_plus_22dof_config"
    cp -v "${SRC_CFG}/walk/amp_pi_plus_20dof_right_hold.yaml" "${SRC_ROOT}/config/walk/"
    cp -v "${SRC_CFG}/walk/amp_pi_plus_20dof.yaml" "${SRC_ROOT}/config/walk/"
    cp -v "${SRC_CFG}/walk/lr.yaml" "${SRC_ROOT}/config/walk/"
    cp -v "${SRC_CFG}/pi_plus_22dof_config/pi_plus_22dof_rl_config.yaml" \
      "${SRC_ROOT}/config/pi_plus_22dof_config/"
    cp -v "${SRC_CFG}/pi_plus_22dof_config/pi_plus_22dof_pd_config.yaml" \
      "${SRC_ROOT}/config/pi_plus_22dof_config/"
  fi
done

DOCS_DST="${SIM2REAL_WS}/docs"
mkdir -p "${DOCS_DST}"
if [ -f "${MOON_ROOT}/docs/AMP_RIGHT_HOLD.md" ]; then
  cp -v "${MOON_ROOT}/docs/AMP_RIGHT_HOLD.md" "${DOCS_DST}/"
fi
if [ -f "${MOON_ROOT}/sim2real/README_amp_right_hold.md" ]; then
  cp -v "${MOON_ROOT}/sim2real/README_amp_right_hold.md" "${SIM2REAL_WS}/"
fi

echo
echo "[OK] amp_right_hold installed."
echo "     Restart sim2real_master, then look for:"
echo "       rl group resgester: [amp_right_hold] [all-amp-rhold]"
echo
echo "Tip: keep a stable path:"
echo "  ln -sfn \"${SIM2REAL_WS}\" \"\${HOME}/sim2real\""
echo "  # or: export SIM2REAL_WS=${SIM2REAL_WS}"
