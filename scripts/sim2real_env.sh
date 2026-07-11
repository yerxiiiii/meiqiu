#!/usr/bin/env bash
# Resolve sim2real workspace + setup.bash (portable).
# Usage:  source /path/to/moon/scripts/sim2real_env.sh
#
# Override:  export SIM2REAL_WS=/path/to/your/sim2real

_sim2real_pick_ws() {
  local c
  for c in \
    "${SIM2REAL_WS:-}" \
    "${HOME}/sim2real" \
    "${HOME}/sim2real_master-feature-master_and_slave" \
    "/home/nvidia/sim2real" \
    "/home/nvidia/sim2real_master-feature-master_and_slave"
  do
    [ -n "$c" ] || continue
    if [ -f "$c/install/setup.bash" ] || [ -f "$c/devel/setup.bash" ]; then
      echo "$c"
      return 0
    fi
  done
  echo "${HOME}/sim2real"
  return 1
}

SIM2REAL_WS="$(_sim2real_pick_ws)"
export SIM2REAL_WS

if [ -f "${SIM2REAL_WS}/install/setup.bash" ]; then
  SIM2REAL_SETUP="${SIM2REAL_WS}/install/setup.bash"
elif [ -f "${SIM2REAL_WS}/devel/setup.bash" ]; then
  SIM2REAL_SETUP="${SIM2REAL_WS}/devel/setup.bash"
else
  SIM2REAL_SETUP="${SIM2REAL_WS}/install/setup.bash"
fi
export SIM2REAL_SETUP

moon_source_sim2real() {
  # shellcheck source=/dev/null
  source "${SIM2REAL_SETUP}"
}
