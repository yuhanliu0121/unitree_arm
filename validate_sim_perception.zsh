#!/usr/bin/env zsh
set -euo pipefail

repo_root=${0:A:h}
exec conda run --no-capture-output -n trash_collection python "$repo_root/validation/sim_perception_domain/validate.py" "$@"
