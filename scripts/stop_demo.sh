#!/usr/bin/env bash
set -u

SESSION="puzzlebot_demo"
JETSON_USER="puzzlebot"
JETSON_HOST="10.10.0.100"
ZERO_TWIST='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# ── Helper functions ──────────────────────────────────────────────────────────

publish_zero_burst() {
  local context="$1"
  local timeout_sec=3
  echo "[${context}] Publishing zero /cmd_vel burst (${timeout_sec}s max)..."
  
  timeout "${timeout_sec}" bash -c '
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_LOCALHOST_ONLY=0
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist "'"${ZERO_TWIST}"'" 2>/dev/null
  ' >/dev/null 2>&1 || true
  
  echo "[${context}] Zero burst complete (or timed out safely)."
}

safe_ssh() {
  local cmd="$1"
  timeout 5 ssh -o BatchMode=yes -o ConnectTimeout=2 -o ServerAliveInterval=1 \
    "${JETSON_USER}@${JETSON_HOST}" "${cmd}" 2>/dev/null || true
}

safe_docker_exec() {
  local cmd="$1"
  timeout 5 docker exec ros2_humble_dev bash -c "${cmd}" 2>/dev/null || true
}

safe_kill() {
  local process_pattern="$1"
  pkill -f "${process_pattern}" 2>/dev/null || true
  sleep 0.2
}

# ── Main stop sequence ────────────────────────────────────────────────────────

echo "=== EMERGENCY STOP SEQUENCE ==="
echo ""

# PHASE 1: Kill control publishers FIRST to stop command generation
echo "[1/6] Killing control publishers..."
safe_kill "mpc_node"
safe_kill "puzzlebot_control"
safe_docker_exec "pkill -f mpc_node || true; pkill -f puzzlebot_control || true"
safe_ssh "pkill -f mpc_node 2>/dev/null || true"
sleep 0.3

# PHASE 2: Publish zero burst from all possible sources
echo "[2/6] Publishing zero commands from local..."
publish_zero_burst "LOCAL"

echo "[3/6] Publishing zero commands from Docker..."
if docker ps --format '{{.Names}}' | grep -q "^ros2_humble_dev$"; then
  safe_docker_exec "
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_LOCALHOST_ONLY=0
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    timeout 3 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '${ZERO_TWIST}' 2>/dev/null || true
  "
else
  echo "[DOCKER] Container not running, skipping."
fi

echo "[4/6] Publishing zero commands from Jetson..."
safe_ssh "
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_LOCALHOST_ONLY=0
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  timeout 3 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '${ZERO_TWIST}' 2>/dev/null || true
"

# PHASE 3: Kill tmux session (which may restart publishers)
echo "[5/6] Killing tmux session..."
if tmux has-session -t "${SESSION}" 2>/dev/null; then
  tmux kill-session -t "${SESSION}" 2>/dev/null || true
  sleep 0.5
fi

# PHASE 4: Final zero burst after killing everything
echo "[6/6] Final zero burst..."
publish_zero_burst "FINAL"

# PHASE 5: Cleanup
echo ""
echo "=== CLEANUP ==="
safe_ssh "pkill -f micro_ros_agent 2>/dev/null || true; pkill -f vision_node 2>/dev/null || true; pkill -f perception.launch 2>/dev/null || true"

# Don't stop Docker container - let it run for inspection
# docker stop ros2_humble_dev >/dev/null 2>&1 || true

echo ""
echo "=== VERIFICATION ==="
echo "Checking for remaining suspicious processes..."
echo ""
echo "Local ROS processes:"
pgrep -af "ros2|mpc_node|puzzlebot" || echo "  (none found)"
echo ""
echo "Docker processes:"
docker exec ros2_humble_dev pgrep -af "ros2|mpc_node|puzzlebot" 2>/dev/null || echo "  (container not running or no processes)"
echo ""
echo "=== STOP SEQUENCE COMPLETE ==="
echo "If robot is still moving, IMMEDIATELY cut motor power physically."
echo "Do NOT attempt to restart until you verify all processes are stopped."
