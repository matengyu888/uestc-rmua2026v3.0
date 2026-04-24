#!/usr/bin/env bash
set -euo pipefail

if [[ "${KEEP_SIM:-0}" == "1" ]]; then
  pkill -f 'roslaunch|fsm_node|pos_controller_node|local_goal_publisher.py|pose_to_odom_bridge.py|lidar_filter_bridge.py|path_loader.py|takeoff_once.py|rostopic|rosservice|rviz' || true
else
  pkill -f 'roscore|rosmaster|roslaunch|RMUA-Linux-Shipping|run_simulator.sh|run_simulator_offscreen.sh|fsm_node|pos_controller_node|local_goal_publisher.py|pose_to_odom_bridge.py|lidar_filter_bridge.py|path_loader.py|takeoff_once.py|rostopic|rosservice|rviz' || true
fi
sleep 2
