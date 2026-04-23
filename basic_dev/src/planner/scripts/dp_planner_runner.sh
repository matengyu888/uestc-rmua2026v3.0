#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATKIN_WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="${CATKIN_WS_DIR}/devel/setup.bash"
WS_PYTHONPATH="${CATKIN_WS_DIR}/devel/lib/python3/dist-packages"
DEFAULT_PYTHON="/home/uestc/software/miniconda3/envs/xzc_learn_flight/bin/python3"
FALLBACK_PYTHON="$(command -v python3 || true)"
DP_PLANNER_PYTHON="${DP_PLANNER_PYTHON:-}"

if [[ -n "${DP_PLANNER_PYTHON}" ]]; then
  if [[ ! -x "${DP_PLANNER_PYTHON}" ]]; then
    echo "[dp_planner_runner] 指定的 Python 不可执行: ${DP_PLANNER_PYTHON}" >&2
    exit 1
  fi
elif [[ -x "${DEFAULT_PYTHON}" ]]; then
  DP_PLANNER_PYTHON="${DEFAULT_PYTHON}"
elif [[ -n "${FALLBACK_PYTHON}" ]]; then
  DP_PLANNER_PYTHON="${FALLBACK_PYTHON}"
else
  echo "[dp_planner_runner] 未找到可执行 Python，既没有 conda Python，也没有 python3" >&2
  exit 1
fi

if [[ -f "${ROS_SETUP}" ]]; then
  # Load ROS environment before switching to the conda Python runtime.
  # This makes rospy and generated message packages visible.
  source "${ROS_SETUP}"
fi

if [[ -f "${WS_SETUP}" ]]; then
  source "${WS_SETUP}"
elif [[ -d "${WS_PYTHONPATH}" ]]; then
  export PYTHONPATH="${WS_PYTHONPATH}:${PYTHONPATH:-}"
fi

echo "[dp_planner_runner] using python: ${DP_PLANNER_PYTHON}" >&2
exec "${DP_PLANNER_PYTHON}" "${SCRIPT_DIR}/dp_planner_node.py" "$@"
