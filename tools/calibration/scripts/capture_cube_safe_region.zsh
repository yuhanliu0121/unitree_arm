#!/usr/bin/env zsh

set -eo pipefail

script_dir=${0:A:h}
workspace=${script_dir:h:h:h}
source "${workspace}/scripts/setup_dev_env.zsh" >/dev/null
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}

if (( $# < 1 )); then
  print -u2 "Usage: $0 <1|2|3|4> [--new] [extra capture options]"
  print -u2 "  1: left/lower; 2: left/upper; 3: right/lower; 4: right/upper"
  exit 2
fi

exec python3 "${script_dir}/d1_manipulation/scripts/capture_cube_safe_boundary.py" \
  "$@" \
  --output-root "${script_dir}/validation/real_pregrasp_alignment/safe_region_calibration"
