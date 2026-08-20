#!/usr/bin/env zsh
set -eo pipefail

SCRIPT_DIR=${0:A:h}
CONFIG_PATH="${SCRIPT_DIR}/d1_bringup/config/real_machine.yaml"
ARM_SERIAL=""
LAUNCH_RVIZ=false

usage() {
  print "Usage: ./real_bringup.zsh [--rviz] [--config PATH] [--arm-serial SERIAL]"
  print ""
  print "Runs the motionless real-machine preflight first. Controllers, MoveIt,"
  print "camera perception, and task Actions start only after the preflight passes."
}

while (( $# > 0 )); do
  case "$1" in
    --rviz)
      LAUNCH_RVIZ=true
      shift
      ;;
    --config)
      if (( $# < 2 )); then
        print -u2 "--config requires a path"
        exit 2
      fi
      CONFIG_PATH=${2:A}
      shift 2
      ;;
    --arm-serial)
      if (( $# < 2 )); then
        print -u2 "--arm-serial requires a value"
        exit 2
      fi
      ARM_SERIAL=$2
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

if [[ ! -f "${CONFIG_PATH}" ]]; then
  print -u2 "Real-machine config not found: ${CONFIG_PATH}"
  exit 2
fi

if [[ -f "${SCRIPT_DIR}/setup_dev_env.zsh" ]]; then
  source "${SCRIPT_DIR}/setup_dev_env.zsh"
else
  print -u2 "Development environment entry point is missing: ${SCRIPT_DIR}/setup_dev_env.zsh"
  exit 2
fi

CONFIG_VALUES=$(python3 -c 'import sys,yaml; c=yaml.safe_load(open(sys.argv[1], encoding="utf-8")); print(c["deployment"]["backend"], c["deployment"]["ros_domain_id"], c["arm"].get("serial_no", ""))' "${CONFIG_PATH}")
read CONFIG_BACKEND CONFIG_ROS_DOMAIN CONFIG_ARM_SERIAL <<< "${CONFIG_VALUES}"
if [[ "${CONFIG_BACKEND}" != "real" ]]; then
  print -u2 "Refusing to start physical hardware with deployment.backend=${CONFIG_BACKEND}"
  exit 2
fi
if [[ -z "${ARM_SERIAL}" ]]; then
  ARM_SERIAL=${CONFIG_ARM_SERIAL}
fi
if [[ -z "${ARM_SERIAL}" ]]; then
  print -u2 "Physical arm serial must be set in YAML or with --arm-serial"
  exit 2
fi

export ROS_DOMAIN_ID=${CONFIG_ROS_DOMAIN}
export ROS_LOG_DIR=/tmp/d1_ros_logs
export D1_REAL_MACHINE_CONFIG=${CONFIG_PATH}
mkdir -p "${ROS_LOG_DIR}"

GRAVITY_CALIBRATION=$(mktemp --suffix=.yaml /tmp/d1_gravity_calibration.XXXXXX)
trap 'rm -f "${GRAVITY_CALIBRATION}"' EXIT

print "[1/2] Running motionless preflight for physical D1 ${ARM_SERIAL}..."
"${SCRIPT_DIR}/real_preflight.zsh" \
  --arm-serial "${ARM_SERIAL}" \
  "config:=${CONFIG_PATH}" \
  "gravity_output_path:=${GRAVITY_CALIBRATION}"

print "Waiting 2 seconds for the RealSense USB streams to be released..."
sleep 2

print "[2/2] Preflight passed; starting physical control, MoveIt, perception, and task Actions..."
print "Press Ctrl+C to stop the stack. No pick/drop motion starts automatically."
ros2 launch d1_bringup real_system.launch.py \
  "config:=${CONFIG_PATH}" \
  "arm_serial:=${ARM_SERIAL}" \
  "gravity_calibration:=${GRAVITY_CALIBRATION}" \
  "launch_rviz:=${LAUNCH_RVIZ}"
