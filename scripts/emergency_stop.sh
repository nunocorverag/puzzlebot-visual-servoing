#!/usr/bin/env bash
set -u

JETSON_USER="puzzlebot"
JETSON_HOST="10.10.0.100"
ZERO_TWIST='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                    EMERGENCY STOP                             ║"
echo "║  This script will IMMEDIATELY stop the robot by:             ║"
echo "║  1. Publishing /emergency_stop true                          ║"
echo "║  2. Flooding /cmd_vel with zeros from all sources            ║"
echo "║  3. Killing all control publishers                           ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# ── Phase 1: Publish emergency stop signal ───────────────────────────────────

echo "[1/5] Publishing /emergency_stop signal..."
timeout 2 bash -c '
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_LOCALHOST_ONLY=0
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}" 2>/dev/null || true
' 2>/dev/null || true

# ── Phase 2: Flood /cmd_vel with zeros (parallel) ────────────────────────────

echo "[2/5] Flooding /cmd_vel with zeros (3 seconds)..."

# Local
timeout 3 bash -c '
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_LOCALHOST_ONLY=0
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist "'"${ZERO_TWIST}"'" 2>/dev/null
' >/dev/null 2>&1 &
LOCAL_PID=$!

# Docker
if docker ps --format '{{.Names}}' | grep -q "^ros2_humble_dev$" 2>/dev/null; then
  timeout 3 docker exec ros2_humble_dev bash -c "
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_LOCALHOST_ONLY=0
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '${ZERO_TWIST}' 2>/dev/null
  " >/dev/null 2>&1 &
  DOCKER_PID=$!
fi

# Jetson
timeout 5 ssh -o BatchMode=yes -o ConnectTimeout=2 -o ServerAliveInterval=1 \
  "${JETSON_USER}@${JETSON_HOST}" "
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_LOCALHOST_ONLY=0
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    timeout 3 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '${ZERO_TWIST}' 2>/dev/null || true
  " >/dev/null 2>&1 &
JETSON_PID=$!

# Wait for all zero bursts to complete
wait $LOCAL_PID 2>/dev/null || true
wait $DOCKER_PID 2>/dev/null || true
wait $JETSON_PID 2>/dev/null || true

echo "[2/5] Zero flood complete."

# ── Phase 3: Kill control publishers ─────────────────────────────────────────

echo "[3/5] Killing control publishers..."

pkill -9 -f "mpc_node" 2>/dev/null || true
pkill -9 -f "puzzlebot_control" 2>/dev/null || true

timeout 3 docker exec ros2_humble_dev bash -c "
  pkill -9 -f mpc_node 2>/dev/null || true
  pkill -9 -f puzzlebot_control 2>/dev/null || true
" 2>/dev/null || true

timeout 5 ssh -o BatchMode=yes -o ConnectTimeout=2 \
  "${JETSON_USER}@${JETSON_HOST}" "
    pkill -9 -f mpc_node 2>/dev/null || true
  " 2>/dev/null || true

sleep 0.5

# ── Phase 4: Final zero burst ────────────────────────────────────────────────

echo "[4/5] Final zero burst..."
timeout 3 bash -c '
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_LOCALHOST_ONLY=0
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist "'"${ZERO_TWIST}"'" 2>/dev/null
' >/dev/null 2>&1 || true

# ── Phase 5: Verification ─────────────────────────────────────────────────────

echo "[5/5] Verifying stop..."
echo ""
echo "Remaining ROS processes:"
pgrep -af "ros2.*cmd_vel|mpc_node|puzzlebot_control" || echo "  ✓ None found"
echo ""

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                  EMERGENCY STOP COMPLETE                      ║"
echo "║                                                               ║"
echo "║  If robot is STILL MOVING:                                   ║"
echo "║    → Cut motor power IMMEDIATELY (physical disconnect)       ║"
echo "║    → Do NOT attempt software recovery                        ║"
echo "║                                                               ║"
echo "║  Next steps:                                                 ║"
echo "║    1. Verify robot has stopped moving                        ║"
echo "║    2. Check CSV logs: /tmp/puzzlebot_logs/                   ║"
echo "║    3. Run: ros2 topic info /cmd_vel -v                       ║"
echo "║    4. Investigate root cause before restarting               ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
