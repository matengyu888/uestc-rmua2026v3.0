#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <bag_path> [--rviz]" >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/basic_dev"
SETUP_FILE="${WS_DIR}/devel/setup.bash"
CLEANUP_SCRIPT="${ROOT_DIR}/scripts/cleanup_super_env.sh"
BAG_PATH="$1"
ENABLE_RVIZ="false"

if [[ "${2:-}" == "--rviz" ]]; then
  ENABLE_RVIZ="true"
fi

if [[ ! -f "${BAG_PATH}" ]]; then
  echo "Bag not found: ${BAG_PATH}" >&2
  exit 1
fi

if [[ -x "${CLEANUP_SCRIPT}" ]]; then
  "${CLEANUP_SCRIPT}"
fi

source /opt/ros/noetic/setup.bash
source "${SETUP_FILE}"

LOG_DIR="${ROOT_DIR}/tmp"
mkdir -p "${LOG_DIR}"
STACK_LOG="${LOG_DIR}/super_replay_stack.log"

roscore >/dev/null 2>&1 &
ROSCORE_PID=$!
sleep 2

roslaunch controller super_rmua.launch enable_takeoff:=false enable_rviz:=${ENABLE_RVIZ} >"${STACK_LOG}" 2>&1 &
STACK_PID=$!
sleep 5

cleanup() {
  kill "${STACK_PID}" >/dev/null 2>&1 || true
  kill "${ROSCORE_PID}" >/dev/null 2>&1 || true
  wait "${STACK_PID}" 2>/dev/null || true
  wait "${ROSCORE_PID}" 2>/dev/null || true
}
trap cleanup EXIT

rosbag play "${BAG_PATH}" --topics /airsim_node/drone_1/debug/pose_gt /airsim_node/drone_1/lidar

sleep 2
echo "Replay finished. Stack log: ${STACK_LOG}"
