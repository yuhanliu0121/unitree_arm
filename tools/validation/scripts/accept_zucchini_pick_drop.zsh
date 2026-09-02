#!/usr/bin/env zsh

set -e
script_dir=${0:A:h}
exec ${script_dir}/accept_cube_pick_drop.zsh --object zucchini "$@"
