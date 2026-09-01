#!/usr/bin/env zsh
set -eo pipefail

SCRIPT_DIR=${0:A:h}
CONFIG_PATH="${SCRIPT_DIR}/d1_bringup/config/real_machine.yaml"

usage() {
  print "Usage: ./arm_stowed_control.zsh --confirm STOWED_MOVE [--config PATH]"
  print ""
  print "Uses the already-running real_bringup arm controller."
  print "It does not start, stop, or switch either onboard controller service."
}

CONFIRM=""
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

if [[ "${CONFIRM}" != "STOWED_MOVE" ]]; then
  print -u2 "Physical motion requires --confirm STOWED_MOVE"
  exit 2
fi
if [[ ! -f "${CONFIG_PATH}" ]]; then
  print -u2 "Real-machine config not found: ${CONFIG_PATH}"
  exit 2
fi

source "${SCRIPT_DIR}/setup_dev_env.zsh"
export ROS_DOMAIN_ID=$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["deployment"]["ros_domain_id"])' "${CONFIG_PATH}")

exec ros2 run d1_bringup recover_stowed --confirm STOWED_MOVE
