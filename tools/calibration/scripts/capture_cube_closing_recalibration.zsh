#!/usr/bin/env zsh

set -eo pipefail

script_dir=${0:A:h}
workspace=${script_dir:h:h:h}
source "${workspace}/scripts/setup_dev_env.zsh" >/dev/null
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}

if (( $# < 1 )); then
  print -u2 "Usage: $0 <1|2|3|4|5|6> [--new|--replace] [extra capture options]"
  print -u2 "  1-3: closing side A (same physical boundary, three valid repeats)"
  print -u2 "  4-6: closing side B (opposite boundary, three valid repeats)"
  exit 2
fi

exec python3 "${script_dir}/d1_manipulation/scripts/capture_cube_safe_boundary.py" \
  "$@" \
  --scheme repeated_closing \
  --runtime-config "${script_dir}/d1_manipulation/config/observe_target.yaml" \
  --closing-margin-mm 1.0 \
  --output-root \
  "${script_dir}/validation/real_pregrasp_alignment/cube_closing_recalibration"
