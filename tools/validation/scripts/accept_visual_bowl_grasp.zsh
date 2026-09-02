#!/usr/bin/env zsh

set -e
script_dir=${0:A:h}
exec ${script_dir}/accept_visual_zucchini_grasp.zsh --object bowl "$@"
