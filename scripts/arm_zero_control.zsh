#!/usr/bin/env zsh
set -eo pipefail

SCRIPT_DIR=${0:A:h}
exec "${SCRIPT_DIR}/arm_maintenance.zsh" zero "$@"
