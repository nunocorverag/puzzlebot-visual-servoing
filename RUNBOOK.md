# Puzzlebot Visual Servoing Runbook

Use `ROS_DOMAIN_ID=0` for this project. The micro-ROS motor topics from the
Hackerboard are in domain 0, so Jetson vision and laptop/Docker control must
also run in domain 0.

Do not build from `/root/dev_ws`. Build only this project with `--base-paths
shared laptop`.

---

## ⚠️ EMERGENCY PROCEDURES ⚠️

### If Robot is Moving Dangerously

**IMMEDIATE ACTION:**

```bash
# Option 1: Emergency stop (fastest, recommended)
bash scripts/emergency_stop.sh

# Option 2: Standard stop
bash scripts/stop_demo.sh
```

**If scripts fail or robot continues moving:**

1. **PHYSICALLY CUT MOTOR POWER** (disconnect battery/power supply)
2. **DO NOT** attempt software recovery while robot is moving
3. **DO NOT** restart demo until root cause is identified

### If Stop Script Hangs

If `stop_demo.sh` or `emergency_stop.sh` appears frozen:

1. Open a new terminal
2. Run: `pkill -9 -f "mpc_node|puzzlebot_control"`
3. Run: `docker exec ros2_humble_dev pkill -9 -f mpc_node`
4. Manually publish zeros:
   ```bash
   timeout 3 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```
5. If still moving: **CUT POWER PHYSICALLY**

### Verify Multiple Publishers

Before restarting, check for rogue `/cmd_vel` publishers:

```bash
ros2 topic info /cmd_vel -v
```

Expected: Only `mpc_node` should publish to `/cmd_vel` during operation.

If multiple publishers exist:
```bash
# Kill all ROS processes
pkill -9 -f ros2
docker exec ros2_humble_dev pkill -9 -f ros2
ssh puzzlebot@10.10.0.100 "pkill -9 -f ros2"
```

### Check Black Box Logs

After any incident, inspect CSV logs:

```bash
# On laptop
ls -lh /tmp/puzzlebot_logs/
tail -50 /tmp/puzzlebot_logs/mpc_fsm_log_*.csv

# From Docker
docker cp ros2_humble_dev:/tmp/puzzlebot_logs ./puzzlebot_logs
```

**Critical CSV fields to check:**
- `steering_sign_ok`: Should be `True` when tracking. If `False`, steering is reversed.
- `expected_turn_direction` vs `actual_turn_direction`: Should match.
- `emergency_stop_active`: Should be `False` during normal operation.
- `command_is_finite`: Should always be `True`. If `False`, NaN/inf detected.
- `stop_reason`: Explains why robot stopped.

### Steering Direction Verification

**Expected behavior (ROS REP-103):**
- Target **left** of center (`ex < 0`) → Robot turns **left** (`omega_cmd > 0`)
- Target **right** of center (`ex > 0`) → Robot turns **right** (`omega_cmd < 0`)

**If robot turns OPPOSITE direction:**

1. Check CSV: `steering_sign_ok` will be `False`
2. Check logs for warning: `"STEERING APPEARS REVERSED"`
3. Fix by editing `laptop/puzzlebot_control/config/mpc_params.yaml`:
   ```yaml
   angular_sign: -1.0  # Flip between 1.0 and -1.0
   ```
4. Rebuild and test in SAFE environment

**Current setting:** `angular_sign: 1.0` (MPC generates correct ROS convention signs)

### Camera Troubleshooting

If vision_node reports:
```
No cameras available
Frame read failed
```

**Diagnosis:**

1. Check if nvargus-daemon is running (Jetson only):
   ```bash
   ssh puzzlebot@10.10.0.100
   pgrep nvargus
   ```

2. Restart nvargus-daemon:
   ```bash
   sudo systemctl restart nvargus-daemon
   ```

3. Test GStreamer pipeline:
   ```bash
   gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink
   ```

4. Check CSI cable connection (physical inspection)

5. If all fails: Reboot Jetson
   ```bash
   sudo reboot
   ```

**Note:** Vision node will continue running and publish `object_detected=false` if camera fails. This prevents control node from crashing but robot will enter SEARCH mode.

---

## One-command tmux demo

Install tmux on the laptop if needed:

```bash
sudo apt install -y tmux
```

The demo script opens a single tmux window named `demo` with visible panels for
RoboNet, micro-ROS, vision, Docker control, and topic monitoring.

Run the full demo:

```bash
cd ~/dev_ws/src/control/puzzlebot-visual-servoing
./scripts/run_demo_tmux.sh
```

Stop the demo:

```bash
cd ~/dev_ws/src/control/puzzlebot-visual-servoing
./scripts/stop_demo.sh
```

Basic tmux shortcuts:

```text
Ctrl+B then arrows   change panel
Ctrl+B then Z        maximize/restore panel
Ctrl+B then D        detach
Ctrl+B then N        next window, if more windows are added
```

Use `./scripts/stop_demo.sh` to stop the tmux session, Docker container, Jetson
processes, and publish a zero `/cmd_vel` when ROS is reachable.

The stop script publishes 20 zero `/cmd_vel` messages before killing sessions or
containers. It never sends a nonzero angular command.

If X11 fails for the camera preview:

```bash
xhost +local:
ssh -X puzzlebot@10.10.0.100
echo $DISPLAY
```

If Docker says the container already exists:

```bash
docker stop ros2_humble_dev
```

Optional aliases for your shell, not applied automatically:

```bash
alias puzzlebot-demo='~/dev_ws/src/control/puzzlebot-visual-servoing/scripts/run_demo_tmux.sh'
alias puzzlebot-stop='~/dev_ws/src/control/puzzlebot-visual-servoing/scripts/stop_demo.sh'
```

## Terminal 1 - Jetson micro-ROS

```bash
ssh puzzlebot@10.10.0.100
source /opt/ros/humble/setup.bash
source ~/ros2_packages_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset FASTRTPS_DEFAULT_PROFILES_FILE
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyUSB0 -v 6
```

## Terminal 2 - Jetson vision

```bash
ssh -X puzzlebot@10.10.0.100
source ~/ros2_ws/env_jetson.sh
source ~/ros2_ws/install/setup.bash
ros2 launch puzzlebot_perception perception.launch.py
```

## HSV calibration

Run the calibrator from the Jetson with X11 forwarding:

```bash
ssh -X puzzlebot@10.10.0.100
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/env_jetson.sh
source install/setup.bash
ros2 run puzzlebot_perception hsv_calibrator
```

Or run it from the laptop:

```bash
cd ~/dev_ws/src/control/puzzlebot-visual-servoing
./scripts/run_calibrator.sh
```

Use:

```text
1. Put the orange/terracotta object in front of the camera.
2. Adjust HSV until only the object appears white in the mask.
3. Adjust min_area to remove small noise.
4. Use sliders for visual fine tuning.
5. Press e to edit exact values in the terminal.
6. Press d to load the demo preset.
7. Press p to print current values as a compact preset and YAML.
8. Press s to save config/vision_hsv.yaml.
9. Press q to quit.
```

Text edit examples after pressing `e`:

```text
S_min=90
min_area 500
H_min=0 H_max=25 S_min=80 V_min=61
0 25 80 255 61 255 500 45 35 70 140 1 15 65 20
```

The compact preset order is:

```text
H_min H_max S_min S_max V_min V_max min_area circularity_min min_circularity_soft aspect_ratio_min aspect_ratio_max hard_shape_filter area_score_weight shape_score_weight aspect_score_weight
```

Shape tuning:

```text
circularity_min rejects amorphous blobs when hard_shape_filter=1.
aspect_ratio_min/max reject stretched objects when hard_shape_filter=1.
min_fill_ratio/max_fill_ratio use the enclosing circle fill to accept circles
and partial semicircles while rejecting irregular orange background blobs.
min_detection_score rejects low-confidence candidates before the robot moves.
confirm_frames/lost_frames reduce one-frame flicker.
```

Then run vision:

```bash
ros2 launch puzzlebot_perception perception.launch.py
```

## Terminal 3 - Docker laptop

```bash
docker run -it --rm \
  --name ros2_humble_dev \
  --network host \
  -v /home/gnuno/dev_ws:/root/dev_ws \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  osrf/ros:humble-desktop bash
```

Inside Docker:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset FASTRTPS_DEFAULT_PROFILES_FILE

cd /root/dev_ws/src/control/puzzlebot-visual-servoing
colcon build \
  --base-paths shared laptop \
  --build-base /tmp/puzzlebot_build \
  --install-base /tmp/puzzlebot_install

source /tmp/puzzlebot_install/setup.bash
ros2 topic list
ros2 launch puzzlebot_control control.launch.py
```

## Control FSM

The control node runs a high-level FSM around the MPC. It always publishes
`/cmd_vel`, publishes the current state on `/fsm_state`, and publishes JSON
diagnostics on `/mpc_debug`.

States:

```text
IDLE          controller disabled or search disabled with no target
SEARCH        no target visible; rotate slowly in place
ACQUIRE_TARGET target just appeared during SEARCH; stop briefly to confirm it
TRACKING      target visible; run MPC visual servoing
GOAL_REACHED  target area is large enough; stop with hysteresis
AVOID         frontal obstacle is too close; turn to clear it
```

Decision priority:

```text
1. enable_controller=false -> IDLE, stop.
2. recent frontal obstacle distance below threshold -> AVOID.
3. while ACQUIRE_TARGET, hold still for acquire_hold_sec.
4. target visible and area >= target_area_stop -> GOAL_REACHED.
5. while GOAL_REACHED, stay stopped until area < target_area_resume or target is lost.
6. target visible -> TRACKING.
7. target briefly lost during TRACKING -> hold still for target_lost_grace_sec.
8. target not visible -> SEARCH.
9. fallback -> IDLE, stop.
```

Obstacle handling assumes `/LaserDistance` publishes `std_msgs/Float32` as a
frontal distance. The subscriber uses sensor QoS (`BEST_EFFORT`, `VOLATILE`) for
micro-ROS compatibility. If the topic is missing or stale for more than
`obstacle_timeout_sec`, obstacle avoidance is inactive and visual servoing keeps
running normally. `/mpc_debug` reports `obstacle_raw`, `obstacle_distance_m`,
`obstacle_age_sec`, `obstacle_available`, and `obstacle_active`.

Set `obstacle_distance_scale` to match the sensor units:

```text
1.0    LaserDistance already reports meters
0.01   LaserDistance reports centimeters
0.001  LaserDistance reports millimeters
```

Example: if `ros2 topic echo /LaserDistance` prints about `20` for an object at
20 cm, set `obstacle_distance_scale: 0.01`.

State tests:

```text
No target visible:
  Expected /fsm_state: SEARCH
  Expected /cmd_vel: v=0, slow nonzero angular.z

Target appears while searching:
  Expected /fsm_state: ACQUIRE_TARGET
  Expected /cmd_vel: v=0, angular.z=0 for acquire_hold_sec
  Then expected /fsm_state: TRACKING if the target remains visible

Target visible and acquired:
  Expected /fsm_state: TRACKING
  Expected behavior: center the target and advance slowly

Target briefly lost while tracking:
  Expected /fsm_state: TRACKING
  Expected /cmd_vel: v=0, angular.z=0 for target_lost_grace_sec
  Then expected /fsm_state: SEARCH if it does not reappear

Target very close:
  Expected /fsm_state: GOAL_REACHED
  Expected /cmd_vel: v=0, angular.z=0

Obstacle close in front:
  Expected /fsm_state: AVOID
  Expected behavior: no forward motion if below obstacle_stop_distance;
  slow turn using avoid_direction

Obstacle blocks the target:
  Expected /fsm_state: AVOID while obstacle is closer than obstacle_clear_distance
  If target is visible after clearing -> ACQUIRE_TARGET, then TRACKING
  If target is not visible -> SEARCH
```

Useful debug commands:

```bash
ros2 topic echo /fsm_state
ros2 topic echo /mpc_debug
ros2 topic echo /cmd_vel
ros2 topic echo /vision_state
ros2 topic echo /LaserDistance
```

CSV black-box logs are written by `mpc_node`:

```bash
ls -lh /tmp/puzzlebot_logs
```

Default path pattern:

```text
/tmp/puzzlebot_logs/mpc_fsm_log_YYYYMMDD_HHMMSS.csv
```

If the log is inside the Docker container, copy it out with:

```bash
docker cp ros2_humble_dev:/tmp/puzzlebot_logs ./puzzlebot_logs
```

Important CSV columns:

```text
state
previous_state
transition_reason
object_detected
ex
area
last_target_age_sec
obstacle_raw
obstacle_distance_m
obstacle_available
obstacle_active
v_controller
omega_controller
angular_sign
v_cmd
omega_cmd
stop_commanded
stop_reason
```

Steering sign:

```text
angular_sign: -1.0
```

All controller angular commands pass through:

```text
omega_cmd = angular_sign * omega_controller
```

If the target is on the left and the robot turns right, flip `angular_sign`
between `1.0` and `-1.0` in `laptop/puzzlebot_control/config/mpc_params.yaml`.
Do the same if the target is on the right and the robot turns left.

Main parameters to tune in `laptop/puzzlebot_control/config/mpc_params.yaml`:

```text
enable_search
search_omega
search_direction
enable_acquire_state
acquire_hold_sec
acquire_timeout_sec
target_lost_grace_sec
use_last_target_search_direction

target_area_stop
target_area_resume

enable_obstacle_avoidance
obstacle_distance_scale
obstacle_stop_distance
obstacle_avoid_distance
obstacle_clear_distance
obstacle_timeout_sec
avoid_omega
avoid_direction
avoid_forward_speed

angular_sign
enable_csv_log
csv_log_dir
csv_log_prefix
csv_flush_every
safety_zero_burst_count
safety_zero_burst_dt
```

If `/LaserDistance` is not available for a demo, disable obstacle handling:

```yaml
enable_obstacle_avoidance: false
```

GOAL_REACHED uses `/vision_state.area` as distance proxy. Calibrate
`target_area_stop` and `target_area_resume` by watching:

```bash
ros2 topic echo /vision_state
```

## Blue obstacle vision debug

The vision node can mark blue obstacles in the camera preview. This is visual
debug only; `AVOID` still uses `/LaserDistance` unless a later fusion step is
implemented.

Default blue HSV range:

```text
H: 90-135
S: 40-255
V: 20-255
blue_min_area: 800
blue_close_area: 2500
```

Test:

```text
1. Put a blue object in front of the camera.
2. The preview should draw a blue bounding box.
3. The label is BLUE_OBS for a candidate obstacle.
4. The label becomes BLUE_OBS_CLOSE when the contour area is large enough.
```

Debug topic:

```bash
ros2 topic echo /vision_obstacle_debug
```

The JSON contains:

```text
blue_obstacle_detected
blue_obstacle_close
blue_obstacle_area
blue_obstacle_ex
blue_obstacle_bbox
blue_obstacle_count
```

## Verification

Run on Jetson or Docker:

```bash
ros2 topic list
```

Expected topics:

```text
/cmd_vel
/VelocityEncL
/VelocityEncR
/robot_vel
/vision_state
/fsm_state
/mpc_debug
/LaserDistance  # optional, when the micro-ROS distance sensor is publishing
```

Check the motor command subscriber:

```bash
ros2 topic info /cmd_vel -v
```

Expected:

```text
Subscription count: 1
Node: puzzlebot_serial_node
```

Test motor command:

```bash
ros2 topic pub --once --qos-reliability best_effort /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Stop:

```bash
ros2 topic pub --once --qos-reliability best_effort /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
