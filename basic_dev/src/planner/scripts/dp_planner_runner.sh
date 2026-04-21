#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_HOME="${CONDA_HOME:-/home/uestc/software/miniconda3}"
CONDA_ENV="${DP_PLANNER_CONDA_ENV:-xzc_learn_flight}"

if [[ ! -f "${CONDA_HOME}/etc/profile.d/conda.sh" ]]; then
  echo "[dp_planner_runner] 未找到 conda.sh: ${CONDA_HOME}/etc/profile.d/conda.sh" >&2
  exit 1
fi

source "${CONDA_HOME}/etc/profile.d/conda.sh"
conda run --no-capture-output -n "${CONDA_ENV}" python3 "${SCRIPT_DIR}/dp_planner_node.py" "$@"
