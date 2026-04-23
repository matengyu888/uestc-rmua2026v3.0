#!/usr/bin/env bash

set -u
set -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="${ROOT_DIR}/simulator12.0.0.3"
SIM_SCRIPT="${SIM_DIR}/run_simulator.sh"
WS_DIR="${ROOT_DIR}/basic_dev"
ROS_SETUP="/opt/ros/noetic/setup.bash"
CATKIN_SETUP="${WS_DIR}/devel/setup.bash"
LAUNCH_FILE="${ROOT_DIR}/basic_dev/src/controller/launch/pos_control.launch"
LOG_DIR="${ROOT_DIR}/logs"

SIM_LOG="${LOG_DIR}/simulator.log"
LAUNCH_LOG="${LOG_DIR}/controller.log"
TAKEOFF_LOG="${LOG_DIR}/takeoff.log"
POST_TAKEOFF_WAIT_SEC="${POST_TAKEOFF_WAIT_SEC:-2}"

SIM_PID=""
LAUNCH_PID=""
CLEANUP_DONE=0

timestamp() {
  date '+%Y-%m-%d %H:%M:%S'
}

info() {
  echo "[$(timestamp)] [INFO] $*"
}

warn() {
  echo "[$(timestamp)] [WARN] $*" >&2
}

error() {
  echo "[$(timestamp)] [ERROR] $*" >&2
}

require_file() {
  local path="$1"
  if [[ ! -e "${path}" ]]; then
    error "缺少文件: ${path}"
    exit 1
  fi
}

kill_pid_if_alive() {
  local pid="$1"
  local name="$2"

  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    info "停止 ${name} (pid=${pid})"
    kill -TERM "${pid}" 2>/dev/null || true
    sleep 2
    if kill -0 "${pid}" 2>/dev/null; then
      warn "${name} 未在超时内退出，强制结束 (pid=${pid})"
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  fi
}

cleanup_rmua_processes() {
  local rmua_pids
  rmua_pids="$(pgrep -f '/RMUA\.sh|/RMUA($| )|LinuxNoEditor/RMUA' || true)"

  if [[ -z "${rmua_pids}" ]]; then
    info "未发现残留的 RMUA 相关进程。"
    return
  fi

  info "检测到 RMUA 相关进程，按 PID 清理:"
  while IFS= read -r pid; do
    [[ -z "${pid}" ]] && continue
    info "  kill RMUA pid=${pid}"
    kill -TERM "${pid}" 2>/dev/null || true
  done <<< "${rmua_pids}"

  sleep 2

  while IFS= read -r pid; do
    [[ -z "${pid}" ]] && continue
    if kill -0 "${pid}" 2>/dev/null; then
      warn "RMUA 进程仍存活，强制结束 pid=${pid}"
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done <<< "${rmua_pids}"
}

cleanup() {
  local reason="${1:-EXIT}"
  if [[ "${CLEANUP_DONE}" -eq 1 ]]; then
    return
  fi
  CLEANUP_DONE=1

  info "开始清理资源，触发原因: ${reason}"
  kill_pid_if_alive "${LAUNCH_PID}" "roslaunch"
  kill_pid_if_alive "${SIM_PID}" "simulator launcher"
  cleanup_rmua_processes
}

on_sigint() {
  warn "收到 Ctrl+C，准备退出并清理 RMUA 相关进程。"
  cleanup "SIGINT"
  exit 130
}

on_sigterm() {
  warn "收到终止信号，准备退出并清理进程。"
  cleanup "SIGTERM"
  exit 143
}

wait_for_ros_service() {
  local service_name="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"

  while true; do
    if bash -lc "source '${ROS_SETUP}' && source '${CATKIN_SETUP}' && rosservice list 2>/dev/null | grep -qx '${service_name}'"; then
      return 0
    fi

    if [[ $(( $(date +%s) - start_ts )) -ge "${timeout_sec}" ]]; then
      return 1
    fi

    sleep 1
  done
}

start_simulator() {
  info "启动模拟器: ${SIM_SCRIPT}"
  (
    cd "${SIM_DIR}" || exit 1
    bash "${SIM_SCRIPT}"
  ) >"${SIM_LOG}" 2>&1 &
  SIM_PID=$!
  info "模拟器启动命令已发出，pid=${SIM_PID}，日志: ${SIM_LOG}"
}

start_controller() {
  info "启动控制器 launch: ${LAUNCH_FILE}"
  (
    cd "${WS_DIR}" || exit 1
    source "${ROS_SETUP}"
    source "${CATKIN_SETUP}"
    roslaunch controller pos_control.launch
  ) >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID=$!
  info "控制器 launch 已启动，pid=${LAUNCH_PID}，日志: ${LAUNCH_LOG}"
}

call_takeoff() {
  info "等待起飞服务 /airsim_node/drone_1/takeoff 就绪"
  if ! wait_for_ros_service "/airsim_node/drone_1/takeoff" 120; then
    error "等待起飞服务超时。请检查模拟器是否启动成功。"
    error "排查日志: ${SIM_LOG}"
    return 1
  fi

  info "发送起飞指令"
  if ! bash -lc "source '${ROS_SETUP}' && source '${CATKIN_SETUP}' && rosservice call /airsim_node/drone_1/takeoff '{}'" \
    >"${TAKEOFF_LOG}" 2>&1; then
    error "起飞指令调用失败。"
    error "排查日志: ${TAKEOFF_LOG}"
    return 1
  fi

  info "起飞指令已发送成功，日志: ${TAKEOFF_LOG}"
  return 0
}

check_component_alive() {
  local pid="$1"
  local name="$2"
  local logfile="$3"

  if [[ -z "${pid}" ]]; then
    return 1
  fi

  if ! kill -0 "${pid}" 2>/dev/null; then
    error "${name} 已退出。"
    error "请检查日志: ${logfile}"
    if [[ -f "${logfile}" ]]; then
      error "${name} 最近日志如下:"
      tail -n 20 "${logfile}" >&2 || true
    fi
    return 1
  fi

  return 0
}

main() {
  mkdir -p "${LOG_DIR}"

  require_file "${SIM_SCRIPT}"
  require_file "${ROS_SETUP}"
  require_file "${CATKIN_SETUP}"
  require_file "${LAUNCH_FILE}"

  trap on_sigint INT
  trap on_sigterm TERM
  trap 'cleanup "EXIT"' EXIT

  start_simulator
  sleep 8

  if ! check_component_alive "${SIM_PID}" "模拟器启动脚本" "${SIM_LOG}"; then
    exit 1
  fi

  if ! call_takeoff; then
    exit 1
  fi

  info "等待起飞动作稳定 ${POST_TAKEOFF_WAIT_SEC}s，再启动控制器。"
  sleep "${POST_TAKEOFF_WAIT_SEC}"

  if ! check_component_alive "${SIM_PID}" "模拟器启动脚本" "${SIM_LOG}"; then
    exit 1
  fi

  start_controller
  sleep 5

  if ! check_component_alive "${LAUNCH_PID}" "控制器 launch" "${LAUNCH_LOG}"; then
    exit 1
  fi

  info "全部启动完成。"
  info "模拟器日志: ${SIM_LOG}"
  info "控制器日志: ${LAUNCH_LOG}"
  info "起飞日志: ${TAKEOFF_LOG}"
  info "按 Ctrl+C 可退出，并自动按 PID 清理 RMUA 相关进程。"

  while true; do
    if ! check_component_alive "${SIM_PID}" "模拟器启动脚本" "${SIM_LOG}"; then
      exit 1
    fi

    if ! check_component_alive "${LAUNCH_PID}" "控制器 launch" "${LAUNCH_LOG}"; then
      exit 1
    fi

    sleep 2
  done
}

main "$@"
