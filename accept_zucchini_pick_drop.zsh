#!/usr/bin/env zsh

set -e
workspace=${0:A:h}
exec ${workspace}/accept_cube_pick_drop.zsh --object zucchini "$@"
