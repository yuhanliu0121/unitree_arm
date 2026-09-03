#!/usr/bin/env zsh

# Shared environment discovery for scripts that must also work after this
# workspace is moved or embedded under another repository.

d1_resolve_ros_setup() {
  local candidate=${D1_ROS_SETUP:-/opt/ros/humble/setup.zsh}
  if [[ ! -r ${candidate} ]]; then
    print -u2 "ROS 2 setup not found: ${candidate}"
    print -u2 'Set D1_ROS_SETUP to the setup.zsh provided by the deployment.'
    return 1
  fi
  REPLY=${candidate:A}
}

d1_resolve_conda_executable() {
  local candidate
  local -a candidates

  if [[ -n ${D1_CONDA_EXE:-} ]]; then
    candidate=${D1_CONDA_EXE}
    if [[ ! -x ${candidate} ]]; then
      print -u2 "Configured Conda executable is not executable: ${candidate}"
      return 1
    fi
    REPLY=${candidate:A}
    return 0
  fi

  if [[ -n ${CONDA_EXE:-} && -x ${CONDA_EXE} ]]; then
    REPLY=${CONDA_EXE:A}
    return 0
  fi
  if (( ${+commands[conda]} )); then
    REPLY=${commands[conda]:A}
    return 0
  fi

  candidates=(
    ${HOME}/miniconda3/bin/conda
    ${HOME}/anaconda3/bin/conda
    /opt/conda/bin/conda
  )
  for candidate in ${candidates}; do
    if [[ -x ${candidate} ]]; then
      REPLY=${candidate:A}
      return 0
    fi
  done

  print -u2 'Conda executable was not found.'
  print -u2 'Set D1_CONDA_EXE, install Conda in a standard location, or add it to PATH.'
  return 1
}

d1_resolve_conda_setup() {
  local candidate
  local conda_executable=''
  local -a candidates

  if [[ -n ${D1_CONDA_SETUP:-} ]]; then
    candidate=${D1_CONDA_SETUP}
    if [[ ! -r ${candidate} ]]; then
      print -u2 "Configured Conda setup is not readable: ${candidate}"
      return 1
    fi
    REPLY=${candidate:A}
    return 0
  fi

  if [[ -n ${D1_CONDA_EXE:-} ]]; then
    d1_resolve_conda_executable || return 1
    conda_executable=${REPLY}
  elif d1_resolve_conda_executable 2>/dev/null; then
    conda_executable=${REPLY}
  fi
  candidates=(
    ${conda_executable:+${conda_executable:h:h}/etc/profile.d/conda.sh}
    ${HOME}/miniconda3/etc/profile.d/conda.sh
    ${HOME}/anaconda3/etc/profile.d/conda.sh
    /opt/conda/etc/profile.d/conda.sh
  )
  for candidate in ${candidates}; do
    if [[ -r ${candidate} ]]; then
      REPLY=${candidate:A}
      return 0
    fi
  done

  print -u2 'Conda shell setup was not found.'
  print -u2 'Set D1_CONDA_SETUP to the deployment conda.sh path.'
  return 1
}
