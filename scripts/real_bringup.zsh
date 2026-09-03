#!/usr/bin/env zsh
set -eo pipefail

SCRIPT_DIR=${0:A:h}
WORKSPACE=${SCRIPT_DIR:h}
CONFIG_PATH="${WORKSPACE}/src/d1_bringup/config/real_machine.yaml"
ARM_SERIAL=""
LAUNCH_RVIZ=false
COMMAND_RATE_HZ=""
COMMAND_DURATION_MS=""
JOINT_SPEED_DEG_S=""
GRAVITY_CALIBRATION=""
CAMERA_DRIVER_LOCATION=""
JOINT6_BYPASS=false
LAUNCH_PID=""

launch_process_group_alive() {
  [[ -n "${LAUNCH_PID}" ]] || return 1
  kill -0 -- "-${LAUNCH_PID}" 2>/dev/null
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM HUP

  if launch_process_group_alive; then
    print "Stopping physical D1 launch process group ${LAUNCH_PID}..."
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    local attempt=0
    while launch_process_group_alive && (( attempt < 100 )); do
      sleep 0.1
      (( ++attempt ))
    done
    if launch_process_group_alive; then
      print -u2 "ROS launch did not stop after SIGINT; sending SIGTERM."
      kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
      attempt=0
      while launch_process_group_alive && (( attempt < 50 )); do
        sleep 0.1
        (( ++attempt ))
      done
    fi
    if launch_process_group_alive; then
      print -u2 "ROS launch did not stop after SIGTERM; sending SIGKILL."
      kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null || true
    fi
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi

  [[ -z "${GRAVITY_CALIBRATION}" ]] || rm -f "${GRAVITY_CALIBRATION}"
  exit "${exit_code}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

usage() {
  print "Usage: ./scripts/real_bringup.zsh [--rviz] [--joint6-bypass] [--config PATH] [--arm-serial SERIAL] [--camera-driver-location local|remote] [--joint-speed-deg-s DEG_S] [--command-rate-hz HZ] [--command-duration-ms MS]"
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
    --joint6-bypass)
      JOINT6_BYPASS=true
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
    --camera-driver-location)
      if (( $# < 2 )); then
        print -u2 "--camera-driver-location requires local or remote"
        exit 2
      fi
      CAMERA_DRIVER_LOCATION=$2
      shift 2
      ;;
    --command-rate-hz)
      if (( $# < 2 )); then
        print -u2 "--command-rate-hz requires a value"
        exit 2
      fi
      COMMAND_RATE_HZ=$2
      shift 2
      ;;
    --joint-speed-deg-s)
      if (( $# < 2 )); then
        print -u2 "--joint-speed-deg-s requires a value"
        exit 2
      fi
      JOINT_SPEED_DEG_S=$2
      shift 2
      ;;
    --command-duration-ms)
      if (( $# < 2 )); then
        print -u2 "--command-duration-ms requires a value"
        exit 2
      fi
      COMMAND_DURATION_MS=$2
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

EXISTING_LAUNCH=$(pgrep -f '/opt/ros/humble/bin/ros2 launch d1_bringup real_system.launch.py' || true)
if [[ -n "${EXISTING_LAUNCH}" ]]; then
  print -u2 "A physical D1 ROS launch is already running: PID(s) ${EXISTING_LAUNCH}."
  print -u2 "Stop the existing launch cleanly before starting another one."
  exit 3
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
if [[ -n "${CAMERA_DRIVER_LOCATION}" ]] && [[ "${CAMERA_DRIVER_LOCATION}" != "local" && "${CAMERA_DRIVER_LOCATION}" != "remote" ]]; then
  print -u2 "--camera-driver-location must be local or remote"
  exit 2
fi
if [[ -n "${COMMAND_RATE_HZ}" ]] && ! python3 -c 'import math,sys; value=float(sys.argv[1]); raise SystemExit(0 if math.isfinite(value) and value > 0.0 else 1)' "${COMMAND_RATE_HZ}"; then
  print -u2 "--command-rate-hz must be a finite positive number"
  exit 2
fi
if [[ -n "${JOINT_SPEED_DEG_S}" ]] && ! python3 -c 'import math,sys; value=float(sys.argv[1]); raise SystemExit(0 if math.isfinite(value) and value > 0.0 else 1)' "${JOINT_SPEED_DEG_S}"; then
  print -u2 "--joint-speed-deg-s must be a finite positive number"
  exit 2
fi
if [[ -n "${COMMAND_DURATION_MS}" ]] && ! python3 -c 'import sys; value=sys.argv[1]; raise SystemExit(0 if value.isdigit() and 1 <= int(value) <= 32767 else 1)' "${COMMAND_DURATION_MS}"; then
  print -u2 "--command-duration-ms must be an integer in [1, 32767]"
  exit 2
fi

export ROS_DOMAIN_ID=${CONFIG_ROS_DOMAIN}
export ROS_LOG_DIR=/tmp/d1_ros_logs
export D1_REAL_MACHINE_CONFIG=${CONFIG_PATH}
mkdir -p "${ROS_LOG_DIR}"

GRAVITY_CALIBRATION=$(mktemp --suffix=.yaml /tmp/d1_gravity_calibration.XXXXXX)

PREFLIGHT_ARGS=(
  --arm-serial "${ARM_SERIAL}"
  "config:=${CONFIG_PATH}"
  "gravity_output_path:=${GRAVITY_CALIBRATION}"
)
[[ -z "${CAMERA_DRIVER_LOCATION}" ]] || PREFLIGHT_ARGS+=("camera_driver_location:=${CAMERA_DRIVER_LOCATION}")

print "[1/2] Running motionless preflight for physical D1 ${ARM_SERIAL}..."
"${SCRIPT_DIR}/real_preflight.zsh" "${PREFLIGHT_ARGS[@]}"

EFFECTIVE_CAMERA_DRIVER_LOCATION=${CAMERA_DRIVER_LOCATION}
if [[ -z "${EFFECTIVE_CAMERA_DRIVER_LOCATION}" ]]; then
  EFFECTIVE_CAMERA_DRIVER_LOCATION=$(python3 -c 'import sys,yaml; c=yaml.safe_load(open(sys.argv[1], encoding="utf-8")); print(str(c["wrist_camera"].get("driver_location", "local")).strip().lower())' "${CONFIG_PATH}")
fi
if [[ "${EFFECTIVE_CAMERA_DRIVER_LOCATION}" == "local" ]]; then
  print "Waiting 2 seconds for the local RealSense USB streams to be released..."
  sleep 2
else
  print "Remote RealSense selected; keeping the external camera publisher running."
fi

print "[2/2] Preflight passed; starting physical control, MoveIt, perception, and task Actions..."
print "Press Ctrl+C to stop the stack and all of its child processes."
print "No pick/drop motion starts automatically."
LAUNCH_ARGS=(
  "config:=${CONFIG_PATH}"
  "arm_serial:=${ARM_SERIAL}"
  "gravity_calibration:=${GRAVITY_CALIBRATION}"
  "launch_rviz:=${LAUNCH_RVIZ}"
  "camera_driver_location:=${EFFECTIVE_CAMERA_DRIVER_LOCATION}"
  "joint6_bypass:=${JOINT6_BYPASS}"
)
if [[ "${JOINT6_BYPASS}" == true ]]; then
  print -u2 "*** JOINT6 BYPASS REQUESTED: gripper motion and retention will be simulated ***"
  print -u2 "*** EMPTY-WORKSPACE COMMISSIONING ONLY; this is not a physical grasp test ***"
fi
[[ -z "${COMMAND_RATE_HZ}" ]] || LAUNCH_ARGS+=("real_command_rate_hz:=${COMMAND_RATE_HZ}")
[[ -z "${COMMAND_DURATION_MS}" ]] || LAUNCH_ARGS+=("real_command_duration_ms:=${COMMAND_DURATION_MS}")
[[ -z "${JOINT_SPEED_DEG_S}" ]] || LAUNCH_ARGS+=("native_joint_speed_deg_s:=${JOINT_SPEED_DEG_S}")

setsid ros2 launch d1_bringup real_system.launch.py "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!
if wait "${LAUNCH_PID}"; then
  LAUNCH_STATUS=0
else
  LAUNCH_STATUS=$?
fi
exit "${LAUNCH_STATUS}"
