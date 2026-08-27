#!/usr/bin/env zsh

set -eo pipefail

script_dir=${0:A:h}
source "${script_dir}/setup_dev_env.zsh" >/dev/null
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}

if (( $# < 1 )); then
  print -u2 "Usage: $0 <1|2> [--new] [extra capture options]"
  print -u2 "  1: closing side A"
  print -u2 "  2: closing side B"
  exit 2
fi

exec python3 "${script_dir}/d1_manipulation/scripts/capture_cube_safe_boundary.py" \
  "$@" \
  --object-class zucchini \
  --estimate-service /arm/perception/estimate_zucchini \
  --overlay-topic /arm/perception/debug/zucchini_geometry \
  --estimator-debug-directory /tmp/d1_zucchini_debug_latest \
  --output-root "${script_dir}/validation/real_pregrasp_alignment/zucchini_safe_region_calibration"
