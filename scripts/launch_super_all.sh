#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cleanup_script="${ROOT_DIR}/scripts/cleanup_super_env.sh"
SIM_SCRIPT="${ROOT_DIR}/scripts/launch_super_simulator.sh"
STACK_SCRIPT="${ROOT_DIR}/scripts/launch_super_stack.sh"
LOG_DIR="${ROOT_DIR}/verification/logs"
SEED="${1:-1}"
STARTUP_WAIT="${STARTUP_WAIT:-8}"
WAIT_TIMEOUT="${WAIT_TIMEOUT:-120}"

mkdir -p "${LOG_DIR}"
if [[ -x "${cleanup_script}" ]]; then
  "${cleanup_script}"
fi
source /opt/ros/noetic/setup.bash

sim_topics_ready() {
  rostopic list 2>/dev/null | grep -q '^/airsim_node/drone_1/lidar$' &&
  rostopic list 2>/dev/null | grep -q '^/airsim_node/drone_1/debug/pose_gt$'
}

stop_stale_simulator() {
  mapfile -t sim_pids < <(pgrep -f "RMUA-Linux-Shipping" || true)
  if (( ${#sim_pids[@]} == 0 )); then
    return 0
  fi

  echo "Stopping stale simulator process: ${sim_pids[*]}"
  kill "${sim_pids[@]}" 2>/dev/null || true
  sleep 3

  mapfile -t sim_pids < <(pgrep -f "RMUA-Linux-Shipping" || true)
  if (( ${#sim_pids[@]} > 0 )); then
    echo "Force killing remaining simulator process: ${sim_pids[*]}"
    kill -9 "${sim_pids[@]}" 2>/dev/null || true
    sleep 2
  fi
}

if pgrep -f "RMUA-Linux-Shipping" >/dev/null 2>&1; then
  if sim_topics_ready; then
    echo "Simulator already running and AirSim ROS topics are ready."
  else
    echo "Simulator process exists, but AirSim ROS topics are not ready."
    stop_stale_simulator
    nohup "${SIM_SCRIPT}" "${SEED}" > "${LOG_DIR}/simulator.log" 2>&1 &
    echo "Simulator restarted in background. Log: ${LOG_DIR}/simulator.log"
    sleep "${STARTUP_WAIT}"
  fi
else
  nohup "${SIM_SCRIPT}" "${SEED}" > "${LOG_DIR}/simulator.log" 2>&1 &
  echo "Simulator starting in background. Log: ${LOG_DIR}/simulator.log"
  sleep "${STARTUP_WAIT}"
fi

echo "Waiting for AirSim ROS topics..."
deadline=$((SECONDS + WAIT_TIMEOUT))
while (( SECONDS < deadline )); do
  if sim_topics_ready; then
    echo "AirSim ROS topics are ready."
    break
  fi
  sleep 2
done

if ! rostopic list 2>/dev/null | grep -q '^/airsim_node/drone_1/lidar$'; then
  echo "Timed out waiting for /airsim_node/drone_1/lidar" >&2
  echo "Check simulator state and ${LOG_DIR}/simulator.log" >&2
  exit 1
fi

if ! rostopic list 2>/dev/null | grep -q '^/airsim_node/drone_1/debug/pose_gt$'; then
  echo "Timed out waiting for /airsim_node/drone_1/debug/pose_gt" >&2
  echo "Check simulator state and ${LOG_DIR}/simulator.log" >&2
  exit 1
fi

echo "Waiting for takeoff service registration..."
deadline=$((SECONDS + WAIT_TIMEOUT))
while (( SECONDS < deadline )); do
  if rosservice list 2>/dev/null | grep -q '^/airsim_node/drone_1/takeoff$'; then
    echo "Takeoff service is ready."
    break
  fi
  sleep 1
done

if ! rosservice list 2>/dev/null | grep -q '^/airsim_node/drone_1/takeoff$'; then
  echo "Warning: /airsim_node/drone_1/takeoff is not ready yet. Launching stack anyway." >&2
fi

exec "${STACK_SCRIPT}"
