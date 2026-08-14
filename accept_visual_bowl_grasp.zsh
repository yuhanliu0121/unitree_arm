#!/usr/bin/env zsh

set -e
workspace=${0:A:h}
exec ${workspace}/accept_visual_zucchini_grasp.zsh --object bowl "$@"
