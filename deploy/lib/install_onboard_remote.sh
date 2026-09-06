#!/usr/bin/env bash
set -euo pipefail

BUNDLE=${1:-}
[[ -r "${BUNDLE}" ]] || { echo "missing deployment bundle: ${BUNDLE}" >&2; exit 2; }

INSTALL_ROOT=/home/ubuntu/marm_code/build
EXECUTABLE=${INSTALL_ROOT}/d1_control_node
MANIFEST=${INSTALL_ROOT}/d1-control-release.env
UNIT=/etc/systemd/system/d1-control.service
STAMP=$(date +%Y%m%d-%H%M%S)
WORK_DIR=$(mktemp -d /tmp/d1-control-build.XXXXXX)
BACKUP_DIR=/home/ubuntu/d1-control-backups/${STAMP}
INSTALL_STARTED=false

sudo_run() {
  printf '%s\n' "${D1_SUDO_PASSWORD:-}" | sudo -S -p '' "$@"
}

cleanup() {
  rm -rf "${WORK_DIR}" "${BUNDLE}" "$0"
}

rollback() {
  local status=$?
  trap - ERR
  if [[ "${INSTALL_STARTED}" == true ]]; then
    echo "Installation failed; restoring the previous onboard deployment..." >&2
    sudo_run systemctl stop d1-control.service || true
    [[ ! -f "${BACKUP_DIR}/d1_control_node" ]] || \
      sudo_run install -m 0755 "${BACKUP_DIR}/d1_control_node" "${EXECUTABLE}"
    [[ ! -f "${BACKUP_DIR}/d1-control-release.env" ]] || \
      sudo_run install -m 0644 "${BACKUP_DIR}/d1-control-release.env" "${MANIFEST}"
    [[ ! -f "${BACKUP_DIR}/d1-control.service" ]] || \
      sudo_run install -m 0644 "${BACKUP_DIR}/d1-control.service" "${UNIT}"
    sudo_run systemctl daemon-reload || true
    sudo_run systemctl enable --now d1-control.service || true
  fi
  cleanup
  exit "${status}"
}

trap rollback ERR
trap cleanup EXIT

tar -C "${WORK_DIR}" -xzf "${BUNDLE}"
SOURCE=${WORK_DIR}/unitree-d1-control
OFFICIAL_SDK=${WORK_DIR}/D1_SDK

# Factory D1 computers may have an unsynchronised clock.  Source timestamps
# inherited from the deployment host can then appear to be years in the
# future, which makes Make report clock skew and can invalidate its dependency
# decisions.  This is a fresh build, so normalise the extracted tree to the
# board's current clock before configuring it.
find "${SOURCE}" "${OFFICIAL_SDK}" -exec touch {} +

cmake -S "${SOURCE}" -B "${WORK_DIR}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DD1_CONTROL_BUILD_HOST=OFF \
  -DD1_CONTROL_BUILD_ONBOARD=ON \
  -DD1_CONTROL_BUILD_TESTS=ON \
  -DD1_OFFICIAL_SDK_ROOT="${OFFICIAL_SDK}"
cmake --build "${WORK_DIR}/build" -j"$(nproc)" \
  --target d1_control_node local_protocol_test
(
  cd "${WORK_DIR}/build"
  grep -q 'add_test(local_protocol_test' CTestTestfile.cmake
  ctest --output-on-failure
)
"${WORK_DIR}/build/d1_control_node" --version

mkdir -p "${BACKUP_DIR}"
[[ ! -f "${EXECUTABLE}" ]] || cp -a "${EXECUTABLE}" "${BACKUP_DIR}/"
[[ ! -f "${MANIFEST}" ]] || cp -a "${MANIFEST}" "${BACKUP_DIR}/"
[[ ! -f "${UNIT}" ]] || sudo_run cp -a "${UNIT}" "${BACKUP_DIR}/"
INSTALL_STARTED=true

sudo_run systemctl stop d1-control.service || true
sudo_run systemctl stop marm_controller.service || true
sudo_run systemctl stop d1-streaming-controller.service || true
sudo_run install -d -m 0755 "${INSTALL_ROOT}"
sudo_run install -m 0755 "${WORK_DIR}/build/d1_control_node" "${EXECUTABLE}.new"
sudo_run mv -f "${EXECUTABLE}.new" "${EXECUTABLE}"
sudo_run install -m 0644 "${SOURCE}/deploy/d1-control-release.env" "${MANIFEST}.new"
sudo_run mv -f "${MANIFEST}.new" "${MANIFEST}"
sudo_run install -m 0644 "${SOURCE}/deploy/systemd/d1-control.service" "${UNIT}"
sudo_run systemctl daemon-reload
sudo_run systemctl disable marm_controller.service d1-streaming-controller.service || true
sudo_run systemctl enable --now d1-control.service
sudo_run systemctl is-active --quiet d1-control.service
grep -q '^D1_CONTROL_VERSION=' "${MANIFEST}"
grep -q '^D1_CONTROL_PROTOCOL=' "${MANIFEST}"

INSTALL_STARTED=false
echo "Onboard executor installed; backup retained at ${BACKUP_DIR}"
