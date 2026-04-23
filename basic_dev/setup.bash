#!/bin/bash
set -eo pipefail

WORKSPACE_DIR="/basic_dev"
TAKEOFF_SERVICE="/airsim_node/drone_1/takeoff"
TAKEOFF_DEADLINE_SEC="${RMUA_TAKEOFF_DEADLINE_SEC:-28}"

cd "${WORKSPACE_DIR}"
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_DIR}/devel/setup.bash"

attempt_takeoff() {
  local deadline_ts
  deadline_ts=$(( $(date +%s) + TAKEOFF_DEADLINE_SEC ))

  echo "[rmua-entrypoint] start takeoff retry loop (${TAKEOFF_DEADLINE_SEC}s)"
  while [[ "$(date +%s)" -lt "${deadline_ts}" ]]; do
    if rosservice call "${TAKEOFF_SERVICE}" "{}" >/tmp/rmua_takeoff.log 2>&1; then
      echo "[rmua-entrypoint] takeoff service call succeeded"
      return 0
    fi
    sleep 1
  done

  echo "[rmua-entrypoint] takeoff service call timed out" >&2
  if [[ -f /tmp/rmua_takeoff.log ]]; then
    tail -n 20 /tmp/rmua_takeoff.log >&2 || true
  fi
  return 1
}

echo "[rmua-entrypoint] launching controller stack"
roslaunch --wait controller hybrid_dp_control.launch &
LAUNCH_PID=$!
attempt_takeoff &
TAKEOFF_PID=$!

cleanup() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" || true
  fi
  if kill -0 "${TAKEOFF_PID}" 2>/dev/null; then
    kill -TERM "${TAKEOFF_PID}" 2>/dev/null || true
    wait "${TAKEOFF_PID}" || true
  fi
}

trap cleanup EXIT INT TERM

if ! wait "${TAKEOFF_PID}"; then
  echo "[rmua-entrypoint] takeoff stage failed" >&2
  exit 1
fi

wait "${LAUNCH_PID}"
