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

## Conceptual Foundations

This project combines four core robotics ideas:

1. **Image-based visual servoing**: the robot is controlled from image
   measurements instead of from a full 3D pose estimate.
2. **Classical computer vision**: the Jetson extracts target and obstacle
   measurements using color segmentation, morphology, contour geometry, and
   temporal filtering.
3. **Model Predictive Control**: the laptop evaluates candidate velocity
   commands over a short horizon and chooses the command with the lowest
   predicted visual error.
4. **Finite-state safety logic**: the controller wraps MPC with discrete states
   for search, acquire, tracking, goal stop, avoidance, camera readiness, and
   emergency stop.

The system is intentionally conservative. It favors low speeds, explicit
diagnostics, simple models, and repeated safety checks because physical robot
testing is sensitive to stale data, wrong signs, networking issues, and multiple
velocity publishers.

## Differential-Drive Robot Model

Puzzlebot is a differential-drive robot. At the command interface level, it is
controlled with a forward linear velocity and a yaw angular velocity:

```text
v     = forward linear velocity [m/s]
omega = yaw angular velocity [rad/s]
```

ROS represents these commands through `geometry_msgs/Twist`:

```text
cmd.linear.x  = v
cmd.angular.z = omega
```

Positive `omega` follows the ROS convention for turning left. Negative `omega`
turns right. The project keeps the controller's internal angular command in this
ROS convention and applies `angular_sign` at the final publishing layer so the
same control logic can be corrected if motor wiring, firmware, or the robot's
embedded convention are inverted.

```yaml
angular_sign: -1.0
```

The final command is:

```text
omega_cmd = angular_sign * omega_controller
```

If the target is on the left side of the image, the robot should command a left
turn after this final sign is applied. If it turns the wrong way, `angular_sign`
is the calibration parameter to change.

## Image-Based Visual Servoing

Visual servoing means using vision measurements directly in the control loop.
This project does not reconstruct a full 3D pose of the target. Instead, it uses
two image-space quantities:

```text
ex:
  normalized horizontal target error

area:
  accepted target contour area in pixels
```

The perception node computes:

```text
ex = (cx_target - cx_image) / cx_image
```

where `cx_target` is the target centroid and `cx_image` is the horizontal image
center. The interpretation is:

```text
ex < 0: target appears left of image center
ex > 0: target appears right of image center
ex = 0: target is centered
```

The controller uses `area` as an approximate distance proxy. As the robot gets
closer to the target, the apparent contour area increases. This is not a metric
depth measurement, but it is sufficient for a controlled demo where the target
size is known and the camera setup is fixed.

The controller forms an area error:

```text
e_area = (area - area_desired) / area_desired
```

The control objective is:

```text
ex     -> 0      center the target
e_area -> 0      reach the desired apparent size
```

## Computer Vision Pipeline

The perception node follows a classical low-latency pipeline:

```text
camera frame
  -> BGR image
  -> HSV image
  -> binary color mask
  -> morphological cleanup
  -> contour extraction
  -> shape filtering
  -> candidate scoring
  -> temporal filtering
  -> ROS message publication
```

The project uses HSV rather than raw RGB because hue, saturation, and value
separate color identity from brightness better than RGB. This makes the
segmentation easier to tune under changing illumination:

```text
H: hue, the dominant color family
S: saturation, how pure the color is
V: value, brightness
```

The target HSV range is calibrated for the red/orange target:

```yaml
hsv_lower: [0, 80, 61]
hsv_upper: [25, 255, 255]
```

After thresholding, morphology reduces noise:

```text
open:
  removes small isolated foreground speckles

close:
  fills small holes inside detected regions
```

The result is a cleaner binary mask from which OpenCV contours are extracted.

## Target Geometry Metrics

The target detector does not accept every red/orange blob. It computes geometric
metrics for each contour:

```text
area:
  number of foreground pixels inside the contour

perimeter:
  contour boundary length

circularity:
  4*pi*area / perimeter^2
  near 1.0 for an ideal circle

aspect_ratio:
  bounding-box width / height

target_fill_ratio:
  contour area / bounding-box area

ellipse_ratio:
  minor_axis / major_axis from cv2.fitEllipse when possible

approx_vertices:
  number of vertices after polygon approximation
```

The older detector expected a mostly circular contour. The current detector also
accepts a reasonable ellipse, because a circular mark on a tilted sheet projects
as an ellipse in the camera image. The acceptance rule is:

```text
accept if:
  circularity >= target_min_circularity

or accept if:
  target_allow_ellipse is true
  and target_ellipse_min_aspect_ratio <= ellipse_ratio <= target_ellipse_max_aspect_ratio
  and circularity >= target_min_circularity_soft
```

Current defaults:

```yaml
target_min_circularity: 0.45
target_min_circularity_soft: 0.35
target_allow_ellipse: true
target_ellipse_min_aspect_ratio: 0.45
target_ellipse_max_aspect_ratio: 1.00
target_min_fill_ratio: 0.45
target_max_fill_ratio: 1.20
```

This improves target detection when the paper is not perpendicular to the
camera. To reduce confusion with the red rectangular obstacle, the target
detector rejects simple filled quadrilateral-like contours that are strongly
elongated. The obstacle detector remains separate and uses independent
`red_box_*` parameters.

## Candidate Scoring

After hard filters remove obvious invalid contours, remaining target candidates
are ranked by a weighted score:

```text
score =
  area_score
  + circularity_score
  + aspect_score
  + fill_score
  + center_score
```

The code uses configurable weights:

```yaml
area_score_weight: 0.10
shape_score_weight: 0.65
aspect_score_weight: 0.20
fill_score_weight: 0.20
center_score_weight: 0.05
min_detection_score: 0.55
```

Shape is weighted strongly because the target should be a compact circle or
ellipse, while random colored blobs should not drive the robot. Center score is
kept small so the robot can still detect a valid target near the edge of the
frame.

## Temporal Filtering

Perception can flicker because of motion blur, lighting, partial occlusion, or
camera auto-exposure. The detector therefore uses temporal filtering:

```yaml
confirm_frames: 3
lost_frames: 4
ex_smoothing_alpha: 0.35
area_smoothing_alpha: 0.35
ex_deadband: 0.05
```

`confirm_frames` prevents one-frame noise from becoming a valid detection.
`lost_frames` keeps the last valid target briefly when a detection drops for a
few frames. Exponential smoothing reduces jitter in `ex` and `area`. The
deadband forces tiny centered errors to zero, which reduces oscillation near the
image center.

The temporal filter is important because the control node treats
`/vision_state` as the primary measurement source.

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

## Simplified MPC Model

The MPC controller uses a deliberately simple image-space model. The goal is not
to simulate the complete differential-drive kinematics, camera projection, or
target geometry. The goal is to choose small, safe velocity commands that reduce
the measured visual errors over a short horizon.

The approximate model is:

```text
ex[k+1]     = ex[k]     + dt * omega[k]
e_area[k+1] = e_area[k] + dt * Kv * v[k]
```

Interpretation:

```text
omega changes horizontal target position in the image
v changes apparent target size
```

This is a local control model. It is valid enough for slow visual servoing where
the target is in view, command limits are low, and the control loop receives
fresh camera measurements.

## Rollout MPC

The controller uses a sampling or rollout style MPC. At every control cycle:

1. Read the latest target state from `/vision_state`.
2. Build the current visual error state.
3. Generate candidate velocity commands `(v, omega)`.
4. Simulate each candidate over a short prediction horizon.
5. Compute a cost for each rollout.
6. Select the candidate with the lowest cost.
7. Publish only the first command.
8. Repeat on the next cycle with fresh measurements.

The cost penalizes visual error and control effort:

```text
Qx * ex^2
Qa * e_area^2
Rv * v^2
Ro * omega^2
terminal penalties for final visual error
```

Candidate commands are bounded by conservative velocity limits:

```yaml
v_max: 0.06
omega_max: 0.20
hard_v_limit: 0.08
hard_omega_limit: 0.25
```

The normal MPC limits shape the candidate search. The hard limits are a final
safety clamp applied to the outgoing command. This two-layer approach makes it
harder for a tuning mistake or maneuver command to exceed the configured safe
range.

## FSM State Semantics

The FSM exists because MPC alone is not enough. The robot also needs to know
when to wait, search, acquire, stop, avoid, or recover. Each state has a clear
role:

```text
WAIT_FOR_CAMERA:
  Camera data is missing, stale, or not sufficiently initialized.
  Publishes zero velocity.

IDLE:
  Controller disabled or no active behavior.
  Publishes zero velocity.

SEARCH:
  No target is visible.
  Rotates slowly in place to scan for the target.

ACQUIRE_TARGET:
  Target just appeared while searching.
  Holds still briefly so the robot does not rotate past the target.

TRACKING:
  Target is confirmed.
  Runs MPC to center and approach.

GOAL_REACHED:
  Target area is above the stop threshold.
  Holds zero command until target is lost or area falls below resume threshold.

AVOID:
  Laser or accepted visual obstacle requires avoidance.
  Publishes a safe avoidance command instead of MPC tracking.

OBSTACLE_CONFIRM:
  Target appears blocked by the visual obstacle.
  Holds still while confirming the target-behind-obstacle condition.

AVOID_TURN:
  Turns away from the blocking obstacle.

AVOID_FORWARD:
  Moves forward slowly to bypass the obstacle.

POST_AVOID_TURN_BACK:
  Turns back toward the expected target direction.

POST_AVOID_REACQUIRE:
  Searches for the target after bypassing the obstacle.

EMERGENCY_STOP:
  Highest-priority latched stop state.
  Publishes zero velocity.
```

The FSM publishes `/fsm_state`, which is useful both for terminal debugging and
for the camera preview overlay.

## Goal Reached Logic

The robot does not estimate target depth directly. Instead, it uses contour area
as a distance proxy:

```yaml
target_area_stop: 25000.0
target_area_resume: 18000.0
```

When:

```text
area >= target_area_stop
```

the FSM enters `GOAL_REACHED` and publishes zero velocity. It exits only when
the target is lost or the target area falls below `target_area_resume`. The
separate resume threshold creates hysteresis so the robot does not rapidly
toggle between `TRACKING` and `GOAL_REACHED`.

## Visual Obstacle Theory

The visual obstacle detector currently targets a low red rectangular object on
the floor. It is intentionally separate from the target detector because the
target is also red/orange. The two detectors differ in shape, position, and
topic semantics:

```text
Target:
  compact circle or perspective ellipse
  publishes /vision_state
  drives visual servoing

Red box obstacle:
  low rectangular object
  appears in lower image region
  publishes /vision_obstacle_debug
  can trigger avoidance when context says it matters
```

The red box detector uses:

```text
two red HSV hue ranges
area threshold
minimum width and height
wide aspect ratio
low circularity
minimum fill ratio
lower-image ROI
red color dominance
mean saturation
target-overlap exclusion
temporal memory
partial-frame handling
```

The target-overlap exclusion is important. If the circular target is currently
detected, the red box detector rejects obstacle candidates that overlap the
target or look circular enough to be the target.

## Visual Obstacle Memory

The visual obstacle debug path includes temporal memory because close objects
can become partially clipped, change apparent shape, or fail one frame of HSV
segmentation. The memory logic tracks:

```text
last bbox
last ex
last area
track id
match reason
partial-frame status
close latch
```

This makes `/vision_obstacle_debug` more stable than a raw frame-by-frame
detector. The controller still applies additional context checks before acting
on visual obstacles.

## LaserDistance and Visual AVOID

The robot has two obstacle sources:

```text
/LaserDistance:
  physical frontal distance signal
  primary safety source

/vision_obstacle_debug:
  camera-based obstacle interpretation
  useful for red box target-blocking behavior
```

`/LaserDistance` uses hysteresis:

```yaml
obstacle_stop_distance: 0.12
obstacle_avoid_distance: 0.30
obstacle_clear_distance: 0.40
```

Conceptually:

```text
distance < stop_distance:
  stop forward motion and turn

stop_distance <= distance < avoid_distance:
  move forward slowly while turning

distance >= clear_distance:
  clear obstacle state
```

Visual AVOID is more contextual. The controller avoids visual obstacles when
they are close and aligned with the target or recent target memory. This avoids
unnecessary turns when the red box is visible but not blocking the robot's
current task.

## Target-Behind-Obstacle Behavior

The target-behind-obstacle maneuver is a higher-level behavior for the case
where the red box blocks the target. It is not just normal AVOID. It is a
scripted bypass sequence wrapped in FSM safety checks.

The condition can start when:

```text
target and obstacle are both visible and horizontally aligned

or

the target was seen recently, disappeared, and a close obstacle appears near the
last target position
```

The maneuver phases are:

```text
OBSTACLE_CONFIRM:
  verify the situation before moving

AVOID_TURN:
  turn away from the obstacle

AVOID_FORWARD:
  move forward slowly to bypass

POST_AVOID_TURN_BACK:
  turn back toward expected target location

POST_AVOID_REACQUIRE:
  search for the target again
```

The robot remembers the last avoidance turn direction. During reacquire, if the
target appears, it has priority and the maneuver clears. If a visual obstacle
appears during reacquire, it must pass additional confirmation before the
controller retries the maneuver. This avoids loops caused by false positives on
image edges.

Reacquire confirmation requires:

```text
close obstacle
central/frontal ex range
not in the image edge zone
stable for enough time or frames
not much smaller than the original blocking obstacle
retry cooldown elapsed
```

This logic is intentionally stricter during `POST_AVOID_REACQUIRE` than during
normal tracking because the robot is already in a recovery behavior and should
not restart the bypass sequence from one noisy frame.

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

Important CSV fields include:

```text
state
previous_state
transition_reason
object_detected
ex
area
last_target_ex
last_target_age_sec
camera_ready
camera_status_reason
obstacle_distance_m
obstacle_active
visual_obstacle_active
avoid_source
v_controller
omega_controller
angular_sign
v_cmd
omega_cmd
emergency_stop_active
stop_reason
target_behind_obstacle_phase
target_behind_obstacle_retry_count
reacquire_obstacle_reason
visual_obstacle_edge_ignored
```

The CSV exists because physical robot bugs often happen too quickly to diagnose
from terminal output. A black-box log lets the operator reconstruct what the FSM
believed, what command it sent, and why it changed state.

## ROS 2 Communication Model

The project uses ROS 2 topics as the integration boundary between perception,
control, visualization, and the embedded motor bridge.

The Jetson and laptop must share:

```text
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_LOCALHOST_ONLY=0
```

The laptop Docker container runs with host networking so DDS discovery can see
the Jetson and micro-ROS topics. The custom `VisionState` message is built on
both machines from `shared/puzzlebot_msgs`.

`/LaserDistance` is subscribed with sensor-style QoS in the controller because
it is live sensor data. Stale data is worse than dropped data for obstacle
avoidance. The controller also applies an obstacle timeout so old readings do
not remain active forever.

## Camera Readiness as a Safety Requirement

Without a camera readiness gate, the robot could start in `SEARCH` and rotate
before the camera is publishing valid data. That is unsafe because the robot
would be moving without its primary perception signal.

The gate checks:

```text
whether /vision_state has ever arrived
whether enough startup messages have arrived
whether /vision_state is fresh
optionally whether /vision_obstacle_debug is fresh
```

If the check fails, the state is `WAIT_FOR_CAMERA` and the controller publishes:

```text
v = 0
omega = 0
```

If the camera is lost after the robot was already running,
`camera_lost_stop=true` returns the robot to the same stopped state.

## Safety Layers

The project uses multiple independent safety layers:

```text
Emergency stop topic:
  /emergency_stop true forces EMERGENCY_STOP.

Camera readiness:
  no camera data means no searching or tracking motion.

Hard velocity clamps:
  final v and omega are bounded before publishing.

Laser stop-zone preemption:
  close physical obstacle can preempt visual maneuvers.

Zero command burst:
  controller shutdown publishes several zero Twist messages.

Stop scripts:
  scripts publish zero commands and terminate known publishers.

CSV and /mpc_debug:
  every decision can be inspected after a test.
```

These layers address different failure modes. For example, camera readiness
handles missing perception, hard clamps handle bad tuning, emergency stop
handles operator intervention, and zero bursts handle stale velocity commands.

## Preview and Operator Feedback

The camera preview is not part of the control loop, but it is important during
testing. It overlays target and obstacle information directly on the video:

```text
target bbox and centroid
target ex, area, circularity, ellipse ratio, fill ratio, score
RED_BOX_OBS / RED_BOX_OBS_CLOSE labels
RED_BOX_OBS_TRACK / RED_BOX_OBS_CLOSE_TRACK memory labels
FSM state overlay
```

This helps connect what the robot is doing with what perception is actually
detecting. If the robot enters AVOID unexpectedly, the operator can compare the
preview, `/vision_obstacle_debug`, `/fsm_state`, and `/mpc_debug`.

## Main Parameters by Concept

Target detection:

```yaml
hsv_lower: [0, 80, 61]
hsv_upper: [25, 255, 255]
min_contour_area: 500.0
min_detection_score: 0.55
target_allow_ellipse: true
target_ellipse_min_aspect_ratio: 0.45
confirm_frames: 3
lost_frames: 4
```

MPC and steering:

```yaml
area_desired: 25000.0
v_max: 0.06
omega_max: 0.20
max_v_step: 0.02
max_omega_step: 0.08
angular_sign: -1.0
```

FSM:

```yaml
enable_search: true
search_omega: 0.08
enable_acquire_state: true
acquire_hold_sec: 0.30
target_lost_grace_sec: 0.40
enable_goal_stop: true
target_area_stop: 25000.0
target_area_resume: 18000.0
```

Obstacle avoidance:

```yaml
enable_obstacle_avoidance: true
obstacle_topic: "/LaserDistance"
obstacle_stop_distance: 0.12
obstacle_avoid_distance: 0.30
obstacle_clear_distance: 0.40
enable_visual_obstacle_avoidance: true
visual_obstacle_close_required: true
visual_obstacle_min_area: 2500.0
```

Camera readiness:

```yaml
require_camera_ready: true
camera_ready_timeout_sec: 1.0
camera_ready_min_messages: 3
camera_lost_stop: true
```

Logging:

```yaml
enable_csv_log: true
csv_log_dir: "/tmp/puzzlebot_logs"
csv_flush_every: 1
```

## How to Validate Behaviors

Target not visible:

```text
Expected state: SEARCH
Expected command: v = 0, small omega
```

Camera not publishing:

```text
Expected state: WAIT_FOR_CAMERA
Expected command: v = 0, omega = 0
Expected debug: camera_ready=false
```

Target appears:

```text
Expected sequence: SEARCH -> ACQUIRE_TARGET -> TRACKING
```

Target centered and close:

```text
Expected sequence: TRACKING -> GOAL_REACHED
Expected command: v = 0, omega = 0
```

Laser obstacle:

```text
Expected state: AVOID
Expected debug: avoid_source=laser
```

Visual red box blocking target:

```text
Expected state: OBSTACLE_CONFIRM or AVOID
Expected debug: visual_obstacle_active=true
Expected debug: avoid_source=vision or target_behind_obstacle_phase set
```

False positive during reacquire:

```text
Expected state: POST_AVOID_REACQUIRE continues
Expected debug: reacquire_obstacle_confirmed=false
Expected debug: visual_obstacle_edge_ignored=true when obstacle is at edge
```

Emergency stop:

```text
Expected state: EMERGENCY_STOP
Expected command: v = 0, omega = 0
```

## Why the Design Uses Classical Vision Instead of Deep Learning

The target and obstacle objects are known, high-contrast, and controlled. A
classical HSV and contour pipeline has practical advantages for this robot:

```text
low latency
low compute cost on Jetson Nano
easy calibration with sliders
transparent failure modes
small dependency footprint
interpretable debug metrics
```

Deep learning could improve robustness to uncontrolled scenes, but it would add
model training, GPU load, dataset collection, deployment complexity, and less
transparent decisions. For this project, the main research and engineering focus
is the closed-loop visual servoing and safety behavior, not object recognition.

## Known Limitations

1. Target distance is inferred from image area, not from true depth.
2. HSV segmentation depends on lighting and camera exposure.
3. The MPC model is a local approximation, so it is designed for low speeds.
4. The red box detector is object-specific and tuned for the current test
   obstacle.
5. Visual obstacle interpretation depends on camera framing and can still
   produce false positives in difficult scenes.
6. micro-ROS and DDS discovery require the Jetson, laptop, and Docker container
   to use the correct network and ROS environment.
7. The system assumes only one active `/cmd_vel` publisher during operation.

## Possible Future Improvements

1. Add automated unit tests for FSM transitions and obstacle edge cases.
2. Add a replay tool that reads CSV logs and reconstructs FSM decisions.
3. Estimate target distance using camera calibration and known target size.
4. Fuse `/LaserDistance` and visual obstacle geometry into a richer obstacle
   confidence model.
5. Add explicit watchdog monitoring for `/cmd_vel` publishers.
6. Add launch-time checks for ROS domain, RMW, and camera readiness.
7. Improve target reacquisition with a probabilistic memory of target position.
8. Add an independent black-box logger node that records all major topics.

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
