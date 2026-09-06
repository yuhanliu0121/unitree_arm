#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
source "${SCRIPT_DIR}/lib/common.sh"

WORKSPACE=$(d1_workspace_root)
ARCH=$(d1_native_arch)
RUN_TESTS=true
NO_CACHE=false

usage() {
  cat <<'EOF'
Usage: ./deploy/install_d1_runtime_env.sh [--skip-tests] [--no-cache]

Builds the native amd64/arm64 runtime image and compiles the bind-mounted ROS 2
workspace. On amd64 it creates a persistent development container with the
commissioning tools enabled. On arm64 it uses an ephemeral build container.
colcon writes build/, install/, and log/ beside the business source on the host.
EOF
}

while (($#)); do
  case "$1" in
    --skip-tests) RUN_TESTS=false ;;
    --no-cache) NO_CACHE=true ;;
    -h|--help) usage; exit 0 ;;
    *) d1_die "unknown argument: $1" ;;
  esac
  shift
done

d1_install_docker_if_needed
d1_select_docker || d1_die "Docker daemon is unavailable; start Docker or grant this user access"

IMAGE=$(d1_image_name "${ARCH}")
BUILD_ARGS=(
  build
  --build-arg "TARGETARCH=${ARCH}"
  --target "${ARCH}"
  --tag "${IMAGE}"
  --file "${SCRIPT_DIR}/Dockerfile"
)
if [[ "${NO_CACHE}" == true ]]; then
  BUILD_ARGS+=(--no-cache)
fi
BUILD_ARGS+=("${WORKSPACE}")

printf 'Building %s from the environment-only Dockerfile...\n' "${IMAGE}"
"${D1_DOCKER[@]}" "${BUILD_ARGS[@]}"

BUILD_COMMAND='set -e; mkdir -p "$HOME"; export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin; unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER VIRTUAL_ENV; hash -r; colcon build --symlink-install --cmake-clean-cache'
if [[ "${ARCH}" == amd64 ]]; then
  BUILD_COMMAND+=' --cmake-args -DD1_BUILD_COMMISSIONING_TOOLS=ON'
fi
if [[ "${RUN_TESTS}" == true ]]; then
  BUILD_COMMAND+='; colcon test --event-handlers console_direct+; colcon test-result --verbose'
fi

printf 'Building the mounted workspace at %s...\n' "${WORKSPACE}"
if [[ "${ARCH}" == amd64 ]]; then
  DEV_CONTAINER=unitree-d1-manipulation-dev-amd64
  if "${D1_DOCKER[@]}" container inspect "${DEV_CONTAINER}" >/dev/null 2>&1; then
    if [[ "$("${D1_DOCKER[@]}" inspect --format '{{.State.Running}}' "${DEV_CONTAINER}")" == true ]]; then
      d1_die "development container ${DEV_CONTAINER} is running; stop it before rebuilding the environment"
    fi
    "${D1_DOCKER[@]}" container rm "${DEV_CONTAINER}" >/dev/null
  fi

  DEV_ARGS=(
    run --detach --name "${DEV_CONTAINER}"
    --privileged
    --network host
    --ipc host
    --init
    --env HOME=/root
    --env ROS_HOME=/tmp/d1-ros-home
    --env YOLO_CONFIG_DIR=/tmp/d1-yolo
    --volume "${WORKSPACE}:/workspace"
  )
  if [[ -n "${DISPLAY:-}" ]]; then
    DEV_ARGS+=(
      --env "DISPLAY=${DISPLAY}"
      --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
    )
  else
    printf 'WARNING: DISPLAY is empty; MuJoCo and RViz windows will not be available.\n' >&2
  fi

  "${D1_DOCKER[@]}" "${DEV_ARGS[@]}" "${IMAGE}" sleep infinity >/dev/null
  if ! "${D1_DOCKER[@]}" exec "${DEV_CONTAINER}" \
      /usr/local/bin/d1-docker-entrypoint bash -lc "${BUILD_COMMAND}"; then
    "${D1_DOCKER[@]}" container rm --force "${DEV_CONTAINER}" >/dev/null 2>&1 || true
    d1_die "workspace build failed; removed incomplete development container ${DEV_CONTAINER}"
  fi
else
  "${D1_DOCKER[@]}" run --rm \
    --privileged \
    --network host \
    --ipc host \
    --env HOME=/root \
    --volume "${WORKSPACE}:/workspace" \
    "${IMAGE}" bash -lc "${BUILD_COMMAND}"
fi

printf '\nD1 runtime environment is ready.\n'
printf '  architecture: %s\n' "${ARCH}"
printf '  image:        %s\n' "${IMAGE}"
printf '  workspace:    %s\n' "${WORKSPACE}"
if [[ "${ARCH}" == amd64 ]]; then
  printf '  dev container: %s (running)\n' "${DEV_CONTAINER}"
  printf '\nEnter the development environment with:\n'
  printf '  %s exec -it %s /usr/local/bin/d1-docker-entrypoint bash\n' \
    "${D1_DOCKER[*]}" "${DEV_CONTAINER}"
fi
