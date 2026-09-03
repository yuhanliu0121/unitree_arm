#!/usr/bin/env zsh

# Source this file from an interactive zsh:
#   source ./scripts/setup_dev_env.zsh
#
# It intentionally does not set ROS_DOMAIN_ID. Acceptance scripts and the
# future real-machine launcher must choose their DDS domain explicitly.

if [[ -z ${ZSH_VERSION:-} ]]; then
  print -u2 'setup_dev_env.zsh must be sourced from zsh.'
  return 1 2>/dev/null || exit 1
fi

if [[ ${ZSH_EVAL_CONTEXT:-} != *:file ]]; then
  print -u2 'Do not execute this file. Use: source ./scripts/setup_dev_env.zsh'
  exit 1
fi

typeset d1_env_script=${(%):-%x}
typeset d1_workspace=${d1_env_script:A:h:h}
typeset d1_environment_helpers=${d1_workspace}/scripts/lib/environment.zsh
typeset d1_workspace_setup=${d1_workspace}/install/setup.zsh
typeset d1_conda_env=${D1_CONDA_ENV:-trash_collection}
typeset d1_conda_label='skipped'

if [[ ! -r ${d1_environment_helpers} ]]; then
  print -u2 "Environment discovery helpers not found: ${d1_environment_helpers}"
  return 1
fi
source ${d1_environment_helpers} || return 1

d1_resolve_ros_setup || return 1
typeset d1_ros_setup=${REPLY}
source ${d1_ros_setup} || return 1

if [[ ${D1_SKIP_CONDA:-0} != 1 ]]; then
  if [[ ${CONDA_DEFAULT_ENV:-} != ${d1_conda_env} ]]; then
    if (( ! ${+functions[conda]} )); then
      d1_resolve_conda_setup || return 1
      typeset d1_conda_setup=${REPLY}
      source ${d1_conda_setup} || return 1
    fi
    conda activate ${d1_conda_env} || return 1
  fi
  d1_conda_label=${CONDA_DEFAULT_ENV:-${d1_conda_env}}
fi

if [[ -r ${d1_workspace_setup} ]]; then
  source ${d1_workspace_setup} || return 1
else
  print -u2 'Workspace overlay is not built yet; ROS 2 and Conda are ready.'
fi

export D1_WORKSPACE=${d1_workspace}
export PYTHONPATH="${d1_workspace}/simulation/d1_mujoco_sim/src:${d1_workspace}/runtime/perception_runtime_v1${PYTHONPATH:+:${PYTHONPATH}}"
cd ${d1_workspace} || return 1

print "D1 development environment ready:"
print "  workspace: ${D1_WORKSPACE}"
print "  conda:    ${d1_conda_label}"
print "  ROS:      ${ROS_DISTRO:-unknown}"
print "  DDS:      ${ROS_DOMAIN_ID:-not set (selected by launcher)}"

unset d1_env_script d1_workspace d1_environment_helpers d1_ros_setup
unset d1_conda_setup d1_conda_env d1_conda_label d1_workspace_setup
