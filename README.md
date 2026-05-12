# Puzzlebot Visual Servoing — Optimal Control Final Project

Image-Based Visual Servoing (IBVS) for a differential-drive Puzzlebot using a
sampling-based Model Predictive Controller (MPC).

## Architecture

```
Laptop — 10.10.0.1                         RoboNet WiFi (10.10.0.0/24)       Jetson Nano — 10.10.0.100
TP-Link USB hotspot (wlx105a95f6aa8d)  ←──────────────────────────────────→  Ubuntu 20.04, ROS2 Humble
┌──────────────────────────────────┐                                          ┌──────────────────────────────────┐
│  Docker: ros2_humble_dev         │  ←────── /vision_state (30 Hz) ───────  │  vision_node                     │
│  ROS2 Humble                     │  ──────── /cmd_vel ────────────────────→ │   └─ IMX219 CSI (/dev/video0)    │
│   mpc_node       (MPC solver)    │                                          │                                  │
│   visualizer_node (live plots)   │                                          │  micro_ros_agent                 │
└──────────────────────────────────┘                                          │   └─ /dev/ttyUSB0 default baud   │
                                                                              │       └─ Puzzlebot motors/encoders│
                                                                              └──────────────────────────────────┘
```

Both machines build `puzzlebot_msgs` locally from the shared source.

---

## Package layout

```
puzzlebot-visual-servoing/
├── shared/puzzlebot_msgs/         # CMake — custom VisionState.msg
├── jetson/puzzlebot_perception/   # Python — vision node (Humble/NVIDIA port)
├── laptop/puzzlebot_control/      # Python — MPC + visualizer (Humble)
├── env_laptop.sh                  # source before launching on laptop
└── env_jetson.sh                  # source before launching on Jetson
```

---

## 1. Prerequisites

### Both machines

`rmw_fastrtps_cpp` is the default RMW shipped with ROS2 — no extra apt install
required on either machine.

### Laptop (Docker)

The Docker container must share the host network to reach the Jetson:

```bash
docker run --rm -it \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/dev_ws:/workspace \
  osrf/ros:humble-desktop \
  bash
```

### Jetson Nano

Install OpenCV with GStreamer support (already present on JetPack):

```bash
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep GStreamer
# should show: GStreamer: YES
```

---

## 2. Build — shared messages (both machines)

Clone the repo into `~/dev_ws/src/control/` on each machine, then:

```bash
cd ~/dev_ws
colcon build --packages-select puzzlebot_msgs
source install/setup.bash
```

---

## 3. Build — perception (Jetson only)

```bash
cd ~/dev_ws
colcon build --packages-select puzzlebot_perception
source install/setup.bash
```

---

## 4. Build — control (laptop only)

```bash
cd ~/dev_ws   # or /workspace inside Docker
colcon build --packages-select puzzlebot_control
source install/setup.bash
```

---

## 5. Network setup — ROS_DOMAIN_ID=0

Both machines use `rmw_fastrtps_cpp` (the default ROS2 RMW — no extra install
needed) and `ROS_DOMAIN_ID=0`. FastRTPS discovers peers via multicast on the
shared subnet automatically.

The Jetson workspace is `~/ros2_ws/`. The env script there sets the two
required variables:

```bash
# ~/ros2_ws/env_jetson.sh exports:
#   ROS_DOMAIN_ID=0
#   RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

On the laptop the variables are set inline inside the Docker container (see
Section 6).

---

## 6. How to Run

Open four terminals in this order. The robot will not move until all four are
running.

---

### Terminal 1 — Laptop: bring up RoboNet hotspot

```bash
nmcli con up "RoboNet"
```

The TP-Link USB adapter (`wlx105a95f6aa8d`) advertises the private WiFi network.
The laptop becomes the gateway at `10.10.0.1`. Wait until the Jetson reconnects
(ping `10.10.0.100` to confirm) before starting the remaining terminals.

---

### Terminal 2 — Jetson: start micro-ROS agent (motor bridge)

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

This bridges the Puzzlebot's microcontroller, connected via USB serial, into
ROS2 and exposes the wheel velocity topics.

---

### Terminal 3 — Jetson: start vision node

```bash
ssh puzzlebot@10.10.0.100
source ~/ros2_ws/env_jetson.sh
source ~/ros2_ws/install/setup.bash
ros2 launch puzzlebot_perception perception.launch.py
```

The IMX219 CSI camera (`/dev/video0`) opens via the GStreamer
`nvarguscamerasrc` pipeline. The node publishes `/vision_state` at 30 Hz.
If X11 forwarding is active (`ssh -X`), a "Robot View" preview window appears.

---

### Terminal 4 — Laptop Docker: start MPC controller

```bash
docker exec -it ros2_humble_dev bash
source /tmp/puzzlebot_install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch puzzlebot_control control.launch.py
```

The MPC node subscribes to `/vision_state` and optional frontal `/LaserDistance`,
runs the SEARCH/ACQUIRE_TARGET/TRACKING/GOAL_REACHED/AVOID FSM at 20 Hz, and publishes
`/cmd_vel` back to the Jetson. The visualizer node opens live error and velocity
plots if a display is available.

---

### Expected behaviour

Place the calibrated orange/terracotta circle in front of the camera. The robot
centers the object horizontally (drives `e_x → 0`) and approaches until the
contour area reaches `target_area_stop`, then holds `GOAL_REACHED`. Remove the
object and the robot enters `SEARCH`, rotating slowly until the target returns.
When the target appears during SEARCH, the robot holds `ACQUIRE_TARGET` briefly
before TRACKING so it does not spin past the object.
If a recent frontal `/LaserDistance` reading is below the obstacle threshold,
the robot enters `AVOID` and turns in place before resuming SEARCH or TRACKING.
The controller writes a CSV black-box log under `/tmp/puzzlebot_logs`, and the
camera preview can also mark blue visual obstacles for debugging. Blue obstacle
vision does not trigger AVOID yet; `/LaserDistance` remains the safety trigger.

### Monitoring

```bash
# FSM state and JSON diagnostics: e_x, area, obstacle distance, v, omega, solve_ms
ros2 topic echo /fsm_state
ros2 topic echo /mpc_debug

# Verify cross-machine topic discovery
ros2 topic hz /vision_state   # run from either machine
```

---

## 7. Control formulation

### State

| Symbol   | Definition                                              | Range   |
|----------|---------------------------------------------------------|---------|
| `e_x`    | `(cx_pixels − cx_image) / cx_image`                    | [−1, 1] |
| `e_area` | `(area − area_d) / area_d`                              | ℝ       |

`e_x = 0` ↔ target centered. `e_area = 0` ↔ target at desired distance.

### Simplified interaction matrix (linearized IBVS)

```
ė_x    ≈ −ω          angular velocity rotates the centroid
ė_area ≈  Kv · v     forward speed drives area toward desired value
```

### Discrete-time model (Euler, step `dt`)

```
e_x[k+1]    = e_x[k]    − dt · ω[k]
e_area[k+1] = e_area[k] + dt · Kv · v[k]
```

### Cost function (horizon N)

```
J = Σ_{k=0}^{N−1} [ Qx·e_x[k]²  + Qa·e_area[k]²  + Rv·v[k]²  + Rω·ω[k]² ]
    + Px·e_x[N]² + Pa·e_area[N]²
```

### Constraints

```
v   ∈ [0, v_max]          forward-only motion
ω   ∈ [−ω_max, ω_max]
```

### Solver — exhaustive grid search (ZOH)

`Nv × Nω` constant actions (zero-order hold over the horizon) are enumerated.
All `N`-step rollouts are computed simultaneously via numpy vectorization.
The lowest-cost action is applied; the problem is re-solved at each control
step (receding horizon principle).

Default grid: 7 × 11 = 77 candidates, typically < 0.3 ms per solve on CPU.

---

## 8. Vision pipeline

```
BGR frame
  └─→ cvtColor(BGR→HSV)
        └─→ inRange(lower1, upper1)  ⎫
            inRange(lower2, upper2)  ⎬─→ bitwise_or → mask
        └─→ morphologyEx(OPEN)   — removes salt-and-pepper noise
        └─→ morphologyEx(CLOSE)  — fills holes in the blob
        └─→ findContours → select largest contour above min_area
        └─→ moments(contour) → sub-pixel centroid cx
        └─→ e_x = (cx − cx_image) / cx_image
        └─→ publish VisionState{ex, area, object_detected}
```

Red targets use two HSV ranges (hue wraps at 0°/180° in OpenCV).
Tune the bounds with the ROS parameter `hsv_lower1/upper1/lower2/upper2`.

---

## 9. Parameter tuning

| Parameter      | Default  | Effect                                                      |
|----------------|----------|-------------------------------------------------------------|
| `area_desired` | 25000 px²| Desired approach distance — larger = closer to target       |
| `Kv`           | 0.3      | Area interaction gain — increase if approach is too slow    |
| `Qx`           | 10.0     | Penalizes centering error (horizontal)                      |
| `Qa`           | 1.0      | Penalizes distance error                                    |
| `Px / Pa`      | 50 / 5   | Terminal penalty — drives error to zero at horizon end      |
| `v_max`        | 0.25 m/s | Cap forward speed for safety                                |
| `omega_max`    | 0.5 rad/s| Cap rotation speed                                          |
| `N`            | 5        | Horizon length — longer = smoother but higher compute cost  |

Edit `laptop/puzzlebot_control/config/mpc_params.yaml` and rebuild, or pass
overrides directly:

```bash
ros2 run puzzlebot_control mpc_node \
  --ros-args -p area_desired:=30000.0 -p Qx:=15.0
```

---

## 10. Troubleshooting

**RoboNet hotspot drops mid-session**

The USB WiFi adapter occasionally loses the connection profile. On the laptop:

```bash
nmcli con up "RoboNet"
```

---

**Jetson doesn't reconnect to RoboNet after a reboot or hotspot restart**

If the Jetson is unreachable over WiFi, connect it temporarily via Ethernet,
SSH in, then bring the WiFi connection up manually:

```bash
ssh puzzlebot@<ethernet-ip>
sudo nmcli con up RoboNet
```

After this the Jetson will auto-connect on future boots.

---

**Camera not detected (`Camera failed to open` in Terminal 3)**

1. Check that the CSI ribbon cable is fully seated at both ends (camera and
   carrier board connector).
2. Verify the device exists: `ls /dev/video*` should show `/dev/video0`.
3. If the device is missing after a confirmed-good cable, reboot the Jetson —
   the CSI stack does not always recover without a full power cycle.

---

**RMW mismatch — nodes don't see each other's topics**

Both machines must use the same middleware. Confirm with:

```bash
echo $RMW_IMPLEMENTATION   # must print rmw_fastrtps_cpp on both
```

If not set, export it before any `ros2` command:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

**SSH asks for a password every time**

Copy your laptop's public key to the Jetson once:

```bash
ssh-copy-id puzzlebot@10.10.0.100
```

Subsequent `ssh` and `scp` commands will be passwordless.

---

## 11. Rubric compliance

| Requirement                          | Implementation                                          |
|--------------------------------------|---------------------------------------------------------|
| Classical CV pipeline (no NN)        | HSV segmentation + morphological ops + contour moments  |
| Explicit cost function               | Quadratic stage + terminal cost (Section 7)             |
| Explicit constraints                 | `v ≥ 0`, `|ω| ≤ ω_max` enforced in grid construction   |
| Optimal control formulation          | Receding-horizon MPC with interaction matrix model      |
| End-to-end closed-loop demo          | vision_node → /vision_state → mpc_node → /cmd_vel      |
| Cross-machine ROS2 communication     | FastRTPS, ROS_DOMAIN_ID=0, multicast on RoboNet          |
