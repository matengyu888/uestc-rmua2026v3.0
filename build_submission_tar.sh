#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ROOT_DIR}/basic_dev"
IMAGE_TAG="${1:-basic_dev}"
EXPORT_DIR="${ROOT_DIR}"
TMP_EXPORT_PATH="${EXPORT_DIR}/test_building.tar"

mkdir -p "${EXPORT_DIR}"

echo "[submission] Building image ${IMAGE_TAG} in ${WS_DIR}"
docker build -t "${IMAGE_TAG}" "${WS_DIR}"

echo "[submission] Inspecting image size"
docker image inspect "${IMAGE_TAG}" --format '{{.Size}}' | awk '{printf "[submission] Image size: %.2f GB\n", $1/1024/1024/1024}'

echo "[submission] Saving image to standard docker archive"
rm -f "${TMP_EXPORT_PATH}" "${TMP_EXPORT_PATH}.sha256"
docker save "${IMAGE_TAG}" -o "${TMP_EXPORT_PATH}"

STAMP="$(date +%Y%m%d_%H%M%S)"
EXPORT_NAME="test_${STAMP}.tar"
EXPORT_PATH="${EXPORT_DIR}/${EXPORT_NAME}"
mv "${TMP_EXPORT_PATH}" "${EXPORT_PATH}"
sha256sum "${EXPORT_PATH}" > "${EXPORT_PATH}.sha256"

echo "[submission] Done"
echo "[submission] TAR: ${EXPORT_PATH}"
echo "[submission] SHA256: ${EXPORT_PATH}.sha256"
