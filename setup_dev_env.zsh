#!/usr/bin/env zsh

# Source this file from an interactive zsh:
#   source ./setup_dev_env.zsh
#
# It intentionally does not set ROS_DOMAIN_ID. Acceptance scripts and the
# future real-machine launcher must choose their DDS domain explicitly.

if [[ -z ${ZSH_VERSION:-} ]]; then
  print -u2 'setup_dev_env.zsh must be sourced from zsh.'
  return 1 2>/dev/null || exit 1
fi

if [[ ${ZSH_EVAL_CONTEXT:-} != *:file ]]; then
  print -u2 'Do not execute this file. Use: source ./setup_dev_env.zsh'
  exit 1
fi

typeset d1_env_script=${(%):-%x}
typeset d1_workspace=${d1_env_script:A:h}
typeset d1_ros_setup=/opt/ros/humble/setup.zsh
typeset d1_conda_setup=/home/tony/miniconda3/etc/profile.d/conda.sh
typeset d1_workspace_setup=${d1_workspace}/install/setup.zsh

if [[ ! -r ${d1_ros_setup} ]]; then
  print -u2 "ROS 2 Humble setup not found: ${d1_ros_setup}"
  return 1
fi

if [[ ! -r ${d1_conda_setup} ]]; then
  print -u2 "Conda setup not found: ${d1_conda_setup}"
  return 1
fi

source ${d1_ros_setup} || return 1
source ${d1_conda_setup} || return 1
conda activate trash_collection || return 1

if [[ -r ${d1_workspace_setup} ]]; then
  source ${d1_workspace_setup} || return 1
else
  print -u2 'Workspace overlay is not built yet; ROS 2 and Conda are ready.'
fi

export D1_WORKSPACE=${d1_workspace}
cd ${d1_workspace} || return 1

print "D1 development environment ready:"
print "  workspace: ${D1_WORKSPACE}"
print "  conda:    ${CONDA_DEFAULT_ENV}"
print "  ROS:      ${ROS_DISTRO:-unknown}"
print "  DDS:      ${ROS_DOMAIN_ID:-not set (selected by launcher)}"

unset d1_env_script d1_workspace d1_ros_setup d1_conda_setup d1_workspace_setup
