#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ROOT_DIR}/basic_dev"

source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"

exec roslaunch planner dp_rviz.launch
