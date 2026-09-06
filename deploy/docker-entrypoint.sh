#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/conda/etc/profile.d/conda.sh
conda activate "${D1_CONDA_ENV:-trash_collection}"

export D1_WORKSPACE="${D1_WORKSPACE:-/workspace}"
export PYTHONPATH="${D1_WORKSPACE}/simulation/d1_mujoco_sim/src:${D1_WORKSPACE}/runtime/perception_runtime_v1${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

if [[ -r "${D1_WORKSPACE}/install/setup.bash" ]]; then
  source "${D1_WORKSPACE}/install/setup.bash"
fi

exec "$@"
