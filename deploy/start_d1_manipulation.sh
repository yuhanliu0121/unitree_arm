#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
source "${SCRIPT_DIR}/lib/common.sh"

WORKSPACE=$(d1_workspace_root)
ARCH=$(d1_native_arch)
IMAGE=$(d1_image_name "${ARCH}")
CONTAINER_NAME="unitree-d1-manipulation-${ARCH}"
REAL_ARGS=()
WANTS_RVIZ=false
EXPLICIT_CONFIG=false
LOCAL_CONFIG="${WORKSPACE}/src/d1_bringup/config/real_machine.local.yaml"
EFFECTIVE_CONFIG="${WORKSPACE}/src/d1_bringup/config/real_machine.effective.yaml"
CONFIG_TOOL="${WORKSPACE}/deploy/lib/deployment_config_tool.py"

usage() {
  cat <<'EOF'
Usage: ./deploy/start_d1_manipulation.sh [real_bringup options]

Examples:
  ./deploy/start_d1_manipulation.sh
  ./deploy/start_d1_manipulation.sh --rviz
  ./deploy/start_d1_manipulation.sh --rviz --joint6-bypass

The repository is mounted at /workspace. Its host-side build/, install/, and
log/ directories are reused directly. Press Ctrl+C for coordinated shutdown.
EOF
}

for argument in "$@"; do
  case "${argument}" in
    -h|--help) usage; exit 0 ;;
    --rviz) WANTS_RVIZ=true ;;
    --config) EXPLICIT_CONFIG=true ;;
  esac
  REAL_ARGS+=("${argument}")
done

if [[ "${EXPLICIT_CONFIG}" == false && -f "${LOCAL_CONFIG}" ]]; then
  python3 "${CONFIG_TOOL}" --materialize "${EFFECTIVE_CONFIG}"
  REAL_ARGS+=(--config /workspace/src/d1_bringup/config/real_machine.effective.yaml)
  printf 'Using validated local deployment configuration: %s\n' "${LOCAL_CONFIG}"
elif [[ "${EXPLICIT_CONFIG}" == false ]]; then
  printf 'No local deployment overlay found; using checked-in development defaults.\n'
  printf 'Run ./deploy/configure_d1_manipulation.sh before a new-machine deployment.\n'
fi

d1_select_docker || d1_die "Docker daemon is unavailable; run install_d1_runtime_env.sh first"
"${D1_DOCKER[@]}" image inspect "${IMAGE}" >/dev/null 2>&1 || \
  d1_die "runtime image ${IMAGE} is missing; run install_d1_runtime_env.sh"
[[ -r "${WORKSPACE}/install/setup.bash" ]] || \
  d1_die "workspace is not built; run install_d1_runtime_env.sh"
mkdir -p "${WORKSPACE}/log/runtime"

if "${D1_DOCKER[@]}" container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  d1_die "container ${CONTAINER_NAME} already exists; stop it before starting another stack"
fi

DOCKER_ARGS=(
  run --rm --name "${CONTAINER_NAME}"
  --privileged
  --network host
  --ipc host
  --init
  --env HOME=/tmp/d1-container-home
  --env ROS_HOME=/tmp/d1-ros-home
  --env YOLO_CONFIG_DIR=/tmp/d1-yolo
  --volume "${WORKSPACE}:/workspace"
  --volume "${WORKSPACE}/log/runtime:/tmp/d1_ros_logs"
)

if [[ "${WANTS_RVIZ}" == true ]]; then
  [[ -n "${DISPLAY:-}" ]] || d1_die "--rviz requires DISPLAY on the host"
  DOCKER_ARGS+=(--env "DISPLAY=${DISPLAY}" --volume /tmp/.X11-unix:/tmp/.X11-unix:rw)
  if [[ -n "${XAUTHORITY:-}" && -r "${XAUTHORITY}" ]]; then
    DOCKER_ARGS+=(--env XAUTHORITY=/tmp/d1-xauthority --volume "${XAUTHORITY}:/tmp/d1-xauthority:ro")
  fi
fi

printf 'Starting D1 manipulation service from %s...\n' "${IMAGE}"
printf 'Workspace and colcon products remain on the host at %s.\n' "${WORKSPACE}"
exec "${D1_DOCKER[@]}" "${DOCKER_ARGS[@]}" "${IMAGE}" \
  zsh /workspace/scripts/real_bringup.zsh "${REAL_ARGS[@]}"
