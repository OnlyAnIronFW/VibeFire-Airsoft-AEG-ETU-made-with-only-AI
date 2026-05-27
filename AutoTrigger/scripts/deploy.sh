#!/bin/bash
#
# deploy.sh — Cross-compile AutoTrigger for Radxa Zero 3E (RK3566) and deploy via scp.
#
# Usage: ./scripts/deploy.sh [target_host] [target_path]
#   target_host  - SSH host alias (default: radxa)
#   target_path  - Remote install directory (default: /home/radxa/autotrigger)

set -euo pipefail

TARGET_HOST="${1:-radxa}"
TARGET_PATH="${2:-/home/radxa/autotrigger}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== Cross-building for aarch64 ==="
mkdir -p "${PROJECT_DIR}/build_aarch64"
cmake -B "${PROJECT_DIR}/build_aarch64" \
    -DCMAKE_TOOLCHAIN_FILE="${PROJECT_DIR}/cmake/aarch64-linux-gnu.cmake" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF
cmake --build "${PROJECT_DIR}/build_aarch64" -j"$(nproc)"

echo ""
echo "=== Deploying to ${TARGET_HOST}:${TARGET_PATH} ==="
ssh "${TARGET_HOST}" "mkdir -p ${TARGET_PATH}/models ${TARGET_PATH}/tables"

scp "${PROJECT_DIR}/build_aarch64/src/autotrigger" \
    "${TARGET_HOST}:${TARGET_PATH}/"

if ls "${PROJECT_DIR}/models/"*.rknn >/dev/null 2>&1; then
    scp "${PROJECT_DIR}/models/"*.rknn "${TARGET_HOST}:${TARGET_PATH}/models/"
fi

if [ -f "${PROJECT_DIR}/tables/drop_table.bin" ]; then
    scp "${PROJECT_DIR}/tables/drop_table.bin" "${TARGET_HOST}:${TARGET_PATH}/tables/"
fi

echo ""
echo "=== Done. Deployed to ${TARGET_HOST}:${TARGET_PATH} ==="
