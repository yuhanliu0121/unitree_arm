#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PYTHON_EXE=${PYTHON_EXE:-python3}
PROFILE=${1:-dog}
export YOLO_CONFIG_DIR="$ROOT/.runtime_cache/ultralytics"
cd "$ROOT"
exec "$PYTHON_EXE" -m perception_runtime.app \
  --config "$ROOT/configs/runtime.yaml" \
  --profile "$PROFILE" \
  --view split \
  --output "$ROOT/captures"
