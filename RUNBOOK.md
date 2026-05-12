# Puzzlebot Visual Servoing Runbook

Use `ROS_DOMAIN_ID=0` for this project. The micro-ROS motor topics from the
Hackerboard are in domain 0, so Jetson vision and laptop/Docker control must
also run in domain 0.

Do not build from `/root/dev_ws`. Build only this project with `--base-paths
shared laptop`.

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
