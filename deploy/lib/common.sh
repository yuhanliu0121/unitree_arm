#!/usr/bin/env bash

d1_die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

d1_workspace_root() {
  local source_path
  source_path=${BASH_SOURCE[0]}
  (cd "$(dirname "${source_path}")/../.." && pwd -P)
}

d1_native_arch() {
  case "$(uname -m)" in
    x86_64) printf '%s\n' amd64 ;;
    aarch64|arm64) printf '%s\n' arm64 ;;
    *) d1_die "unsupported host architecture: $(uname -m)" ;;
  esac
}

d1_select_docker() {
  command -v docker >/dev/null 2>&1 || return 1
  if docker info >/dev/null 2>&1; then
    D1_DOCKER=(docker)
  elif command -v sudo >/dev/null 2>&1 && sudo docker info >/dev/null 2>&1; then
    D1_DOCKER=(sudo docker)
  else
    return 1
  fi
}

d1_install_docker_if_needed() {
  if command -v docker >/dev/null 2>&1; then
    return 0
  fi
  command -v sudo >/dev/null 2>&1 || d1_die "Docker is missing and sudo is unavailable"
  printf 'Docker is not installed; installing the Ubuntu docker.io package...\n'
  sudo apt-get update
  sudo apt-get install -y docker.io
  sudo systemctl enable --now docker
}

d1_image_name() {
  printf 'unitree-d1-manipulation:%s\n' "$1"
}
