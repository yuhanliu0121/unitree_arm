#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
source "${SCRIPT_DIR}/lib/common.sh"

# A deployment host intentionally does not need a native ROS installation.
# Prefer a working host-side librealsense utility, but fall back to the runtime
# image when the utility is absent *or present but unable to enumerate*.  The
# latter occurs on hosts with an incomplete native librealsense installation.
REALSENSE_DEVICE_LIST=
REALSENSE_ENUMERATOR=$(command -v rs-enumerate-devices 2>/dev/null || true)
if [[ -z "${REALSENSE_ENUMERATOR}" && -x /opt/ros/humble/bin/rs-enumerate-devices ]]; then
  REALSENSE_ENUMERATOR=/opt/ros/humble/bin/rs-enumerate-devices
fi
if [[ -n "${REALSENSE_ENUMERATOR}" ]]; then
  REALSENSE_DEVICE_LIST=$("${REALSENSE_ENUMERATOR}" -s 2>/dev/null || true)
fi

if ! grep -qi 'realsense' <<<"${REALSENSE_DEVICE_LIST}"; then
  ARCH=$(d1_native_arch)
  IMAGE=$(d1_image_name "${ARCH}")
  if d1_select_docker && "${D1_DOCKER[@]}" image inspect "${IMAGE}" >/dev/null 2>&1; then
    REALSENSE_DEVICE_LIST=$(
      "${D1_DOCKER[@]}" run --rm --privileged \
        --volume /dev:/dev \
        --volume /run/udev:/run/udev:ro \
        "${IMAGE}" /opt/ros/humble/bin/rs-enumerate-devices -s 2>/dev/null || true
    )
  fi
fi

if [[ -n "${REALSENSE_DEVICE_LIST}" ]]; then
  export D1_REALSENSE_DEVICE_LIST="${REALSENSE_DEVICE_LIST}"
fi

exec python3 "${SCRIPT_DIR}/lib/deployment_config_tool.py" "$@"
