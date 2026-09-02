#!/usr/bin/env zsh
set -euo pipefail

script_dir=${0:A:h}
exec conda run --no-capture-output -n trash_collection \
  python "${script_dir:h}/sim_perception_domain/validate.py" "$@"
