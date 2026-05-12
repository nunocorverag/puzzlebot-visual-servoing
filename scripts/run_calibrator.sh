#!/usr/bin/env bash
set -euo pipefail

xhost +local: || true
ssh -X puzzlebot@10.10.0.100 "cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source ~/ros2_ws/env_jetson.sh && source install/setup.bash && ros2 run puzzlebot_perception hsv_calibrator"
