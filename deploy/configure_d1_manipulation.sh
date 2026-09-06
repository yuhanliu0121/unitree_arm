#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
source "${SCRIPT_DIR}/lib/common.sh"

# A deployment host intentionally does not need a native ROS installation.
# If librealsense utilities are unavailable on the host, reuse the already
# built runtime image to obtain the ASIC serial expected by the ROS driver.
if ! command -v rs-enumerate-devices >/dev/null 2>&1 && \
   [[ ! -x /opt/ros/humble/bin/rs-enumerate-devices ]]; then
  ARCH=$(d1_native_arch)
  IMAGE=$(d1_image_name "${ARCH}")
  if d1_select_docker && "${D1_DOCKER[@]}" image inspect "${IMAGE}" >/dev/null 2>&1; then
    D1_REALSENSE_DEVICE_LIST=$(
      "${D1_DOCKER[@]}" run --rm --privileged \
        --volume /dev:/dev \
        --volume /run/udev:/run/udev:ro \
        "${IMAGE}" /opt/ros/humble/bin/rs-enumerate-devices -s 2>/dev/null || true
    )
    export D1_REALSENSE_DEVICE_LIST
  fi
fi

exec python3 "${SCRIPT_DIR}/lib/deployment_config_tool.py" "$@"
