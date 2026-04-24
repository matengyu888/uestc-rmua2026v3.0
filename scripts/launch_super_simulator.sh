#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${ROOT_DIR}/simulator12.0.0.3"
SEED="${1:-1}"

if [[ ! -d "${SIM_DIR}" ]]; then
  echo "Simulator directory not found: ${SIM_DIR}" >&2
  exit 1
fi

if [[ ! -f "${SIM_DIR}/run_simulator.sh" ]]; then
  echo "Simulator launcher not found: ${SIM_DIR}/run_simulator.sh" >&2
  exit 1
fi

cd "${SIM_DIR}"
exec bash ./run_simulator.sh "${SEED}"
