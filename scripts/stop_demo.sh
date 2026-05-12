#!/usr/bin/env bash
set -u

SESSION="puzzlebot_demo"
JETSON_USER="puzzlebot"
JETSON_HOST="10.10.0.100"
STOP_CMD='source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export ROS_LOCALHOST_ONLY=0; unset FASTRTPS_DEFAULT_PROFILES_FILE; ros2 topic pub --once --qos-reliability best_effort /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'

echo "Trying to publish zero /cmd_vel from Jetson..."
ssh -o BatchMode=yes -o ConnectTimeout=3 "${JETSON_USER}@${JETSON_HOST}" "bash -lc '${STOP_CMD}'" >/dev/null 2>&1 || true

echo "Trying to publish zero /cmd_vel from Docker..."
docker exec ros2_humble_dev bash -lc "${STOP_CMD}" >/dev/null 2>&1 || true

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "Killing tmux session ${SESSION}..."
  tmux kill-session -t "${SESSION}" || true
fi

echo "Stopping Docker container ros2_humble_dev if it exists..."
docker stop ros2_humble_dev >/dev/null 2>&1 || true

echo "Killing relevant Jetson processes if reachable..."
ssh -o BatchMode=yes -o ConnectTimeout=3 "${JETSON_USER}@${JETSON_HOST}" \
  "bash -lc 'pkill -f micro_ros_agent || true; pkill -f vision_node || true; pkill -f perception.launch || true'" >/dev/null 2>&1 || true

echo "Demo stop command finished."
