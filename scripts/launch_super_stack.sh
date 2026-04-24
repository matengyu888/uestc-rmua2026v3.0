#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/basic_dev"
SETUP_FILE="${WS_DIR}/devel/setup.bash"
CLEANUP_SCRIPT="${ROOT_DIR}/scripts/cleanup_super_env.sh"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "Catkin setup file not found: ${SETUP_FILE}" >&2
  echo "Please build the workspace first." >&2
  exit 1
fi

if [[ -x "${CLEANUP_SCRIPT}" ]]; then
  KEEP_SIM=1 "${CLEANUP_SCRIPT}"
fi

source /opt/ros/noetic/setup.bash
source "${SETUP_FILE}"

cd "${WS_DIR}"
exec roslaunch controller super_rmua.launch "$@"
