#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
source "${SCRIPT_DIR}/lib/common.sh"

WORKSPACE=$(d1_workspace_root)
BOARD_HOST=192.168.123.100
BOARD_USER=ubuntu
BOARD_PASSWORD=123
CONFIRMATION=

usage() {
  cat <<'EOF'
Usage: ./deploy/update_d1_onboard_sdk.sh [options] --confirm UPDATE_D1_ONBOARD_SDK

Options:
  --host HOST          D1 board address (default: 192.168.123.100)
  --user USER          SSH user (default: ubuntu)
  --password PASSWORD  SSH/sudo password (default: 123)
  --confirm TEXT       Required safety acknowledgement

Before running, connect this computer directly to the D1 and manually set its
wired interface to the 192.168.123.0/24 subnet. This script never changes host
network settings and never sends a motion command.
EOF
}

while (($#)); do
  case "$1" in
    --host) [[ $# -ge 2 ]] || d1_die "--host requires a value"; BOARD_HOST=$2; shift ;;
    --user) [[ $# -ge 2 ]] || d1_die "--user requires a value"; BOARD_USER=$2; shift ;;
    --password) [[ $# -ge 2 ]] || d1_die "--password requires a value"; BOARD_PASSWORD=$2; shift ;;
    --confirm) [[ $# -ge 2 ]] || d1_die "--confirm requires a value"; CONFIRMATION=$2; shift ;;
    -h|--help) usage; exit 0 ;;
    *) d1_die "unknown argument: $1" ;;
  esac
  shift
done

[[ "${CONFIRMATION}" == UPDATE_D1_ONBOARD_SDK ]] || {
  usage >&2
  d1_die "refusing board update without --confirm UPDATE_D1_ONBOARD_SDK"
}
command -v ssh >/dev/null 2>&1 || d1_die "ssh is not installed"
command -v scp >/dev/null 2>&1 || d1_die "scp is not installed"
command -v tar >/dev/null 2>&1 || d1_die "tar is not installed"

SSH_OPTIONS=(-o ConnectTimeout=5 -o ConnectionAttempts=1 -o StrictHostKeyChecking=accept-new)
REMOTE="${BOARD_USER}@${BOARD_HOST}"
if ssh "${SSH_OPTIONS[@]}" -o BatchMode=yes "${REMOTE}" true >/dev/null 2>&1; then
  SSH_COMMAND=(ssh "${SSH_OPTIONS[@]}" -o BatchMode=yes)
  SCP_COMMAND=(scp "${SSH_OPTIONS[@]}" -o BatchMode=yes)
else
  command -v sshpass >/dev/null 2>&1 || \
    d1_die "SSH key login failed and sshpass is unavailable (install package sshpass)"
  export SSHPASS=${BOARD_PASSWORD}
  SSH_COMMAND=(sshpass -e ssh "${SSH_OPTIONS[@]}" -o BatchMode=no)
  SCP_COMMAND=(sshpass -e scp "${SSH_OPTIONS[@]}" -o BatchMode=no)
fi

TEMP_DIR=$(mktemp -d /tmp/d1-onboard-deploy.XXXXXX)
trap 'rm -rf "${TEMP_DIR}"' EXIT
BUNDLE="${TEMP_DIR}/d1-onboard-source.tar.gz"
REMOTE_BUNDLE=/tmp/d1-onboard-source.tar.gz
REMOTE_INSTALLER=/tmp/install-d1-onboard.sh

tar -C "${WORKSPACE}/third_party" \
  --exclude='D1_SDK/build' \
  --exclude='unitree-d1-control/build' \
  -czf "${BUNDLE}" D1_SDK unitree-d1-control

printf 'Uploading versioned D1 onboard source to %s...\n' "${REMOTE}"
"${SCP_COMMAND[@]}" "${BUNDLE}" "${REMOTE}:${REMOTE_BUNDLE}"
"${SCP_COMMAND[@]}" "${SCRIPT_DIR}/lib/install_onboard_remote.sh" \
  "${REMOTE}:${REMOTE_INSTALLER}"

printf 'Building, testing, and atomically installing on the D1 board...\n'
"${SSH_COMMAND[@]}" "${REMOTE}" \
  "D1_SUDO_PASSWORD=$(printf %q "${BOARD_PASSWORD}") bash ${REMOTE_INSTALLER} ${REMOTE_BUNDLE}"

printf '\nD1 onboard SDK update passed. The d1-control.service is enabled and active.\n'
