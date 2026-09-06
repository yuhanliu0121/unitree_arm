#!/usr/bin/env zsh
set -eo pipefail

SCRIPT_DIR=${0:A:h}
WORKSPACE=${SCRIPT_DIR:h}
CONFIG_PATH="${WORKSPACE}/src/d1_bringup/config/real_machine.yaml"
LOCAL_CONFIG_PATH="${WORKSPACE}/src/d1_bringup/config/real_machine.local.yaml"
EFFECTIVE_CONFIG_PATH="${WORKSPACE}/src/d1_bringup/config/real_machine.effective.yaml"
CONFIG_TOOL="${WORKSPACE}/deploy/lib/deployment_config_tool.py"
EXPLICIT_CONFIG=false
source "${SCRIPT_DIR}/setup_dev_env.zsh"
set -u

LAUNCH_ARGS=()
while (( $# > 0 )); do
  case "$1" in
    --arm-serial)
      if (( $# < 2 )); then
        print -u2 "--arm-serial requires a value"
        exit 2
      fi
      LAUNCH_ARGS+=("arm_serial:=$2")
      shift 2
      ;;
    --config)
      if (( $# < 2 )); then
        print -u2 "--config requires a path"
        exit 2
      fi
      CONFIG_PATH=${2:A}
      EXPLICIT_CONFIG=true
      LAUNCH_ARGS+=("config:=${CONFIG_PATH}")
      shift 2
      ;;
    config:=*)
      CONFIG_PATH=${${1#config:=}:A}
      EXPLICIT_CONFIG=true
      LAUNCH_ARGS+=("config:=${CONFIG_PATH}")
      shift
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ "${EXPLICIT_CONFIG}" == false && -f "${LOCAL_CONFIG_PATH}" ]]; then
  python3 "${CONFIG_TOOL}" --materialize "${EFFECTIVE_CONFIG_PATH}" >/dev/null
  CONFIG_PATH="${EFFECTIVE_CONFIG_PATH}"
  LAUNCH_ARGS+=("config:=${CONFIG_PATH}")
fi
if [[ ! -f "${CONFIG_PATH}" ]]; then
  print -u2 "Real-machine config not found: ${CONFIG_PATH}"
  exit 2
fi

# Keep the preflight isolated from simulation and unrelated ROS graphs by using
# the domain selected in the validated effective configuration.
export ROS_DOMAIN_ID=$(python3 -c 'import sys,yaml; c=yaml.safe_load(open(sys.argv[1], encoding="utf-8")); print(c["deployment"]["ros_domain_id"])' "${CONFIG_PATH}")
export ROS_LOG_DIR=/tmp/d1_ros_logs
export D1_REAL_MACHINE_CONFIG=${CONFIG_PATH}
mkdir -p "${ROS_LOG_DIR}"

OUTPUT_FILE=$(mktemp /tmp/d1_real_preflight.XXXXXX)
trap 'rm -f "${OUTPUT_FILE}"' EXIT

set +e
ros2 launch d1_bringup real_preflight.launch.py "${LAUNCH_ARGS[@]}" 2>&1 | tee "${OUTPUT_FILE}"
LAUNCH_STATUS=${pipestatus[1]}
set -e

if grep -q "PREFLIGHT PASSED" "${OUTPUT_FILE}"; then
  exit 0
fi
if (( LAUNCH_STATUS != 0 )); then
  exit ${LAUNCH_STATUS}
fi
exit 2
