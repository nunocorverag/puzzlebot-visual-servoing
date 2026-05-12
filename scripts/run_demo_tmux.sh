#!/usr/bin/env bash
set -euo pipefail

SESSION="puzzlebot_demo"
JETSON_USER="puzzlebot"
JETSON_HOST="10.10.0.100"
REPO_IN_DOCKER="/root/dev_ws/src/control/puzzlebot-visual-servoing"
DISPLAY_VALUE="${DISPLAY:-:0}"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed. Install it with: sudo apt install -y tmux" >&2
  exit 1
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  tmux kill-session -t "${SESSION}"
fi

tmux new-session -d -s "${SESSION}" -n "demo"
P_NETWORK=$(tmux display-message -p -t "${SESSION}:demo" '#{pane_id}')
P_MICRO=$(tmux split-window -h -t "${P_NETWORK}" -P -F '#{pane_id}')
P_VISION=$(tmux split-window -v -t "${P_NETWORK}" -P -F '#{pane_id}')
P_CONTROL=$(tmux split-window -v -t "${P_MICRO}" -P -F '#{pane_id}')
P_MONITOR=$(tmux split-window -v -t "${P_VISION}" -P -F '#{pane_id}')
tmux select-layout -t "${SESSION}:demo" tiled

tmux send-keys -t "${P_NETWORK}" \
  "nmcli con up \"RoboNet\" || true; echo \"RoboNet command finished.\"; exec bash" C-m

tmux send-keys -t "${P_MICRO}" \
  "ssh ${JETSON_USER}@${JETSON_HOST} 'bash -lc \"source /opt/ros/humble/setup.bash; source ~/ros2_packages_ws/install/setup.bash; export ROS_DOMAIN_ID=0; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export ROS_LOCALHOST_ONLY=0; unset FASTRTPS_DEFAULT_PROFILES_FILE; ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyUSB0 -v 6\"'" C-m

tmux send-keys -t "${P_VISION}" \
  "xhost +local: || true; ssh -X ${JETSON_USER}@${JETSON_HOST} \"bash -lc 'cd ~/ros2_ws; source /opt/ros/humble/setup.bash; source ~/ros2_ws/env_jetson.sh; source install/setup.bash; ros2 launch puzzlebot_perception perception.launch.py'\"" C-m

tmux send-keys -t "${P_CONTROL}" \
  "docker rm -f ros2_humble_dev >/dev/null 2>&1 || true; docker run -it --rm \
    --name ros2_humble_dev \
    --network host \
    -v /home/gnuno/dev_ws:/root/dev_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=${DISPLAY_VALUE} \
    osrf/ros:humble-desktop bash -lc '
      source /opt/ros/humble/setup.bash
      export ROS_DOMAIN_ID=0
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      export ROS_LOCALHOST_ONLY=0
      unset FASTRTPS_DEFAULT_PROFILES_FILE

      cd ${REPO_IN_DOCKER}

      colcon build \
        --base-paths shared laptop \
        --build-base /tmp/puzzlebot_build \
        --install-base /tmp/puzzlebot_install

      source /tmp/puzzlebot_install/setup.bash
      echo \"=== TOPICS ===\"
      ros2 topic list

      echo \"=== LAUNCHING CONTROL ===\"
      ros2 launch puzzlebot_control control.launch.py
    '" C-m

tmux send-keys -t "${P_MONITOR}" \
  "ssh ${JETSON_USER}@${JETSON_HOST} 'bash -lc \"source /opt/ros/humble/setup.bash; source ~/ros2_ws/env_jetson.sh; source ~/ros2_ws/install/setup.bash; source ~/ros2_packages_ws/install/setup.bash; export ROS_DOMAIN_ID=0; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export ROS_LOCALHOST_ONLY=0; unset FASTRTPS_DEFAULT_PROFILES_FILE; watch -n 1 ros2 topic list\"'" C-m

tmux select-layout -t "${SESSION}:demo" tiled
tmux attach-session -t "${SESSION}"
