#!/usr/bin/env zsh
set -eo pipefail

SCRIPT_DIR=${0:A:h}
WORKSPACE=${SCRIPT_DIR:h}
CONFIG_PATH="${WORKSPACE}/src/d1_bringup/config/real_machine.yaml"
SDK_BUILD_DIR="${WORKSPACE:h}/D1-SDK/build-linux"
OPERATION=""
CONFIRM=""

usage() {
  print "Usage:"
  print "  ./scripts/arm_stowed_control.zsh --confirm STOWED_MOVE [--config PATH]"
  print "  ./scripts/arm_zero_control.zsh --confirm ZERO_MOVE [--config PATH]"
  print "  ./scripts/all_joints_unload.zsh --confirm UNLOAD_ALL [--config PATH]"
  print ""
  print "The wrapper automatically detects the local real-machine control stack."
  print "STOWED uses the active ROS controller when available and otherwise uses"
  print "the direct SDK tool. ZERO and UNLOAD stop a local stack before direct SDK access."
}

if (( $# == 0 )); then
  usage >&2
  exit 2
fi

OPERATION=$1
shift
while (( $# > 0 )); do
  case "$1" in
    --confirm)
      (( $# >= 2 )) || { print -u2 "--confirm requires a value"; exit 2; }
      CONFIRM=$2
      shift 2
      ;;
    --config)
      (( $# >= 2 )) || { print -u2 "--config requires a path"; exit 2; }
      CONFIG_PATH=${2:A}
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      print -u2 "Unknown argument: $1"
      usage >&2
      exit 2
      ;;
  esac
done

case "${OPERATION}" in
  stowed)
    EXPECTED_CONFIRM="STOWED_MOVE"
    SDK_EXECUTABLE="arm_stowed_control"
    ;;
  zero)
    EXPECTED_CONFIRM="ZERO_MOVE"
    SDK_EXECUTABLE="arm_zero_control"
    ;;
  unload)
    EXPECTED_CONFIRM="UNLOAD_ALL"
    SDK_EXECUTABLE="all_joints_unload"
    ;;
  *)
    print -u2 "Unknown maintenance operation: ${OPERATION}"
    usage >&2
    exit 2
    ;;
esac

if [[ "${CONFIRM}" != "${EXPECTED_CONFIRM}" ]]; then
  print -u2 "Physical operation requires --confirm ${EXPECTED_CONFIRM}"
  exit 2
fi
if [[ ! -f "${CONFIG_PATH}" ]]; then
  print -u2 "Real-machine config not found: ${CONFIG_PATH}"
  exit 2
fi

source "${SCRIPT_DIR}/setup_dev_env.zsh" >/dev/null

CONFIG_VALUES=$(python3 -c 'import sys,yaml; c=yaml.safe_load(open(sys.argv[1], encoding="utf-8")); print(c["deployment"]["ros_domain_id"], c["arm"]["network_interface"])' "${CONFIG_PATH}")
read CONFIG_ROS_DOMAIN NETWORK_INTERFACE <<< "${CONFIG_VALUES}"
export ROS_DOMAIN_ID=${CONFIG_ROS_DOMAIN}

local_launch_pids() {
  pgrep -f '/opt/ros/humble/bin/ros2 launch d1_bringup real_system.launch.py' 2>/dev/null || true
}

controller_available() {
  local actions
  actions=$(timeout 3 ros2 action list -t 2>/dev/null) || return 1
  print -r -- "${actions}" | grep -Fq '/arm_controller/follow_joint_trajectory [control_msgs/action/FollowJointTrajectory]'
}

stop_local_stack() {
  local pids pid pgid attempt
  pids=$(local_launch_pids)
  [[ -n "${pids}" ]] || return 0

  print "Stopping the active local real-machine control stack before direct SDK access..."
  for pid in ${(f)pids}; do
    pgid=$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')
    if [[ -n "${pgid}" && "${pgid}" == "${pid}" ]]; then
      kill -INT -- "-${pgid}" 2>/dev/null || true
    else
      # A manually launched process may share its terminal's process group.
      # Never signal that whole group because it could include the caller's shell.
      kill -INT -- "${pid}" 2>/dev/null || true
    fi
  done

  attempt=0
  while [[ -n "$(local_launch_pids)" ]] && (( attempt < 100 )); do
    sleep 0.1
    (( ++attempt ))
  done
  pids=$(local_launch_pids)
  if [[ -n "${pids}" ]]; then
    print -u2 "Control stack did not stop after SIGINT; sending SIGTERM."
    for pid in ${(f)pids}; do
      pgid=$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')
      if [[ -n "${pgid}" && "${pgid}" == "${pid}" ]]; then
        kill -TERM -- "-${pgid}" 2>/dev/null || true
      else
        kill -TERM -- "${pid}" 2>/dev/null || true
      fi
    done
    attempt=0
    while [[ -n "$(local_launch_pids)" ]] && (( attempt < 50 )); do
      sleep 0.1
      (( ++attempt ))
    done
  fi
  if [[ -n "$(local_launch_pids)" ]]; then
    print -u2 "ERROR: local control stack did not release D1 command ownership."
    print -u2 "Direct SDK command refused to prevent concurrent publishers."
    return 1
  fi

  # Let ROS processes and the onboard loopback transport release ownership.
  sleep 1
  if controller_available; then
    print -u2 "ERROR: an arm controller is still visible after stopping the local stack."
    print -u2 "Direct SDK command refused to prevent concurrent publishers."
    return 1
  fi
}

run_direct_sdk() {
  local executable="${SDK_BUILD_DIR}/${SDK_EXECUTABLE}"
  if [[ ! -x "${executable}" ]]; then
    print -u2 "Direct SDK utility is unavailable: ${executable}"
    return 2
  fi

  print "No active local controller owns the D1 command channel; using direct SDK."
  case "${OPERATION}" in
    stowed)
      exec "${executable}" --interface "${NETWORK_INTERFACE}" --confirm STOWED_MOVE
      ;;
    zero)
      exec "${executable}" --interface "${NETWORK_INTERFACE}" --confirm ZERO_MOVE
      ;;
    unload)
      exec "${executable}" UNLOAD_ALL "${NETWORK_INTERFACE}"
      ;;
  esac
}

LOCAL_PIDS=$(local_launch_pids)
if [[ "${OPERATION}" == "stowed" ]] && controller_available; then
  print "Active D1 controller detected; recovering through the ROS control stack."
  exec ros2 run d1_bringup recover_stowed --confirm STOWED_MOVE
fi

if [[ "${OPERATION}" != "stowed" && -z "${LOCAL_PIDS}" ]] && controller_available; then
  print -u2 "ERROR: an arm controller is active, but it is not owned by the standard local launch."
  print -u2 "Cannot stop it safely; direct SDK command refused."
  exit 3
fi

if [[ -n "${LOCAL_PIDS}" ]]; then
  if [[ "${OPERATION}" == "stowed" ]]; then
    print -u2 "Local control stack exists but its arm action server is unavailable."
  fi
  stop_local_stack
fi

run_direct_sdk
