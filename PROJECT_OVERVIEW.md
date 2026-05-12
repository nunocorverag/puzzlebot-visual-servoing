# Puzzlebot Visual Servoing Project Overview

## Purpose

This repository implements an image-based visual servoing system for a
differential-drive Puzzlebot using ROS 2 Humble. A Jetson Nano runs camera
perception and publishes image measurements. A laptop, normally inside a
host-network Docker container, runs the control stack and publishes velocity
commands.

The main control objective is to center a red/orange circular target in the
camera image, drive toward it, stop at a configured apparent target size, and
handle obstacles safely.

## Runtime Architecture

```text
Jetson Nano
  - CSI camera capture through vision_node
  - Red/orange target detection
  - Red rectangular visual obstacle detection
  - /vision_state publisher
  - /vision_obstacle_debug publisher
  - micro-ROS agent for the Puzzlebot microcontroller

Laptop / Docker
  - mpc_node control loop
  - FSM and safety logic
  - /cmd_vel publisher
  - /fsm_state publisher
  - /mpc_debug publisher
  - CSV black-box logging

Puzzlebot microcontroller
  - Receives /cmd_vel
  - Drives motors
  - Publishes encoders and /LaserDistance when available
```

Both machines use `ROS_DOMAIN_ID=0`, `rmw_fastrtps_cpp`, and
`ROS_LOCALHOST_ONLY=0`.

## Main Topics

| Topic | Type | Publisher | Consumer | Purpose |
| --- | --- | --- | --- | --- |
| `/vision_state` | `puzzlebot_msgs/VisionState` | `vision_node` | `mpc_node` | Target error `ex`, contour `area`, and target visible flag |
| `/vision_obstacle_debug` | `std_msgs/String` JSON | `vision_node` | `mpc_node` | Visual obstacle state and debug fields |
| `/LaserDistance` | `std_msgs/Float32` | micro-ROS | `mpc_node` | Physical frontal obstacle distance |
| `/cmd_vel` | `geometry_msgs/Twist` | `mpc_node` | micro-ROS | Robot velocity command |
| `/fsm_state` | `std_msgs/String` | `mpc_node` | tools, preview | Current FSM state |
| `/mpc_debug` | `std_msgs/String` JSON | `mpc_node` | operator | Control, FSM, obstacle, camera, and CSV diagnostics |
| `/emergency_stop` | `std_msgs/Bool` | stop tools/operator | `mpc_node` | Latches emergency stop when true |

## Target Detection

The target detector lives in `jetson/puzzlebot_perception/puzzlebot_perception/vision_node.py`.
It detects the red/orange circular target using an HSV mask, morphology, contour
analysis, and temporal filtering.

The detector publishes `/vision_state` with:

```text
ex = (target_center_x - image_center_x) / image_center_x
area = accepted contour area
object_detected = target visibility after temporal filtering
```

`ex < 0` means the target is left of image center. `ex > 0` means it is right of
image center.

The target can now be accepted as a perspective ellipse, not only as a frontal
circle. The contour can pass either by normal circularity or by fitted ellipse
ratio:

```yaml
target_min_circularity: 0.45
target_min_circularity_soft: 0.35
target_allow_ellipse: true
target_ellipse_min_aspect_ratio: 0.45
target_ellipse_max_aspect_ratio: 1.00
target_min_fill_ratio: 0.45
target_max_fill_ratio: 1.20
```

This keeps `/vision_state` unchanged while making the target robust to a tilted
sheet. To avoid confusing the target with the red box obstacle, the elliptical
target path rejects simple filled quadrilateral-like contours that are strongly
elongated.

## Visual Obstacle Detection

Visual obstacle detection also runs in `vision_node.py` but remains separate
from the target detector. The current visual obstacle is a low, rectangular red
box on the floor.

The detector uses independent `red_box_*` HSV and shape parameters:

```yaml
enable_red_box_obstacle_detection: true
red_box_h1_min: 0
red_box_h1_max: 15
red_box_h2_min: 170
red_box_h2_max: 179
red_box_s_min: 100
red_box_v_min: 60
red_box_min_area: 1200.0
red_box_close_area: 4500.0
red_box_min_aspect_ratio: 1.25
red_box_max_circularity: 0.60
red_box_bottom_roi_y_min_ratio: 0.45
red_box_allow_vertical_edge_partial: false
```

It publishes `/vision_obstacle_debug`. For compatibility with older controller
code, the JSON still includes `blue_obstacle_*` fields, but those fields now act
as legacy aliases for the active visual obstacle.

The topic also includes clearer `visual_obstacle_*` fields such as:

```text
visual_obstacle_detected
visual_obstacle_close
visual_obstacle_area
visual_obstacle_ex
visual_obstacle_bbox
visual_obstacle_detector
visual_obstacle_tracking_active
visual_obstacle_match_reason
```

## Control FSM

The main controller is `laptop/puzzlebot_control/puzzlebot_control/mpc_node.py`.
It implements a finite-state machine around the MPC command generation.

Important states:

```text
WAIT_FOR_CAMERA
IDLE
SEARCH
ACQUIRE_TARGET
TRACKING
GOAL_REACHED
AVOID
OBSTACLE_CONFIRM
AVOID_TURN
AVOID_FORWARD
POST_AVOID_TURN_BACK
POST_AVOID_REACQUIRE
EMERGENCY_STOP
```

Priority order is safety first:

1. Emergency stop
2. Controller disabled
3. Camera readiness gate
4. Laser stop-zone obstacle
5. Target-behind-obstacle maneuver
6. Normal laser/visual AVOID
7. Target acquire/track/goal/search behavior

## Camera Readiness Gate

The robot does not start searching or turning until the camera is publishing
fresh `/vision_state` messages.

Key parameters:

```yaml
require_camera_ready: true
camera_ready_timeout_sec: 1.0
camera_startup_grace_sec: 0.5
camera_lost_stop: true
camera_ready_min_messages: 3
camera_ready_require_fresh_obstacle_debug: false
```

If the camera is missing or stale, the FSM enters `WAIT_FOR_CAMERA`, publishes
zero velocity, and records camera diagnostics in `/mpc_debug` and CSV.

## Obstacle Avoidance

The robot can enter AVOID from:

```text
/LaserDistance
visual obstacle debug
both sources
```

`/LaserDistance` remains the physical safety source. Visual obstacles can also
trigger avoidance when they are close and contextually relevant.

The controller publishes `avoid_source` in `/mpc_debug`:

```text
laser
vision
both
none
```

## Target Behind Obstacle Maneuver

When the target appears blocked by the red rectangular obstacle, the controller
can run a bypass maneuver:

```text
OBSTACLE_CONFIRM
AVOID_TURN
AVOID_FORWARD
POST_AVOID_TURN_BACK
POST_AVOID_REACQUIRE
```

During `POST_AVOID_REACQUIRE`, the target has priority. If the target is visible
again, the maneuver clears and normal tracking can resume.

To prevent loops caused by false positives, visual obstacles seen during
reacquire must now be stable, central, close, and outside the image edge zone
before they can restart the maneuver:

```yaml
reacquire_obstacle_confirm_sec: 0.50
reacquire_obstacle_confirm_frames: 5
reacquire_obstacle_center_ex_limit: 0.55
reacquire_obstacle_ignore_edge_ex: 0.70
reacquire_obstacle_min_area_ratio: 0.75
reacquire_obstacle_requires_close: true
reacquire_obstacle_retry_cooldown_sec: 1.5
ignore_edge_obstacles_during_reacquire: true
```

Diagnostics include:

```text
reacquire_obstacle_confirmed
reacquire_obstacle_confirm_frames
reacquire_obstacle_reason
visual_obstacle_edge_ignored
visual_obstacle_in_path
reacquire_obstacle_retry_allowed
```

## CSV Black-Box Logging

When enabled, `mpc_node` writes a CSV log under:

```text
/tmp/puzzlebot_logs
```

The log records FSM state, transition reason, target measurements, obstacle
state, camera readiness, velocity commands, emergency stop state, visual
avoidance source, target-behind-obstacle phases, and reacquire robustness fields.

## Safe Stop

The repository includes stop scripts for safe local shutdown:

```text
scripts/stop_demo.sh
scripts/emergency_stop.sh
```

The controller also publishes a zero-command burst during node destruction. The
emergency stop topic has the highest FSM priority.

## Operator Workflow

Typical safe checks before moving the robot:

```bash
ros2 topic echo /vision_state
ros2 topic echo /vision_obstacle_debug
ros2 topic echo /fsm_state
ros2 topic echo /mpc_debug
```

For bench testing, lift the wheels and verify:

```text
camera_ready=true
state is not WAIT_FOR_CAMERA
target object_detected changes correctly
visual_obstacle_detector=red_box when the red box is present
avoid_source=vision only when visual obstacle logic should trigger
```

Do not use the physical demo until camera readiness, emergency stop, and stop
scripts have been checked in the current environment.
