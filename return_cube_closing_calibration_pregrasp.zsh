#!/usr/bin/env zsh

set -eo pipefail

script_dir=${0:A:h}
source "${script_dir}/setup_dev_env.zsh" >/dev/null
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}

exec python3 \
  "${script_dir}/d1_manipulation/scripts/return_zucchini_calibration_pregrasp.py" \
  --config \
  "${script_dir}/d1_manipulation/config/cube_closing_calibration_pregrasp.yaml" \
  --session-root \
  "${script_dir}/validation/real_pregrasp_alignment/cube_closing_recalibration" \
  "$@"
