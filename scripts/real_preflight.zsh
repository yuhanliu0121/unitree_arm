#!/usr/bin/env zsh
set -eo pipefail

SCRIPT_DIR=${0:A:h}
WORKSPACE=${SCRIPT_DIR:h}
source "${SCRIPT_DIR}/setup_dev_env.zsh"
set -u

# Must match src/d1_bringup/config/real_machine.yaml. Keeping this explicit avoids
# accidentally joining a simulation ROS graph or another team's live graph.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}
export ROS_LOG_DIR=/tmp/d1_ros_logs
mkdir -p "${ROS_LOG_DIR}"

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
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

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
