# Puzzlebot Visual Servoing — Optimal Control Final Project

Image-Based Visual Servoing (IBVS) for a differential-drive Puzzlebot using a
sampling-based Model Predictive Controller (MPC).

## Architecture

```
┌──────────────────────────────────────┐   WiFi (RoboNet)   ┌──────────────────────────────┐
│  Laptop — 10.10.0.1                  │ ←── /vision_state ──│  Jetson Nano — 10.10.0.100   │
│  ROS2 Humble (Docker)                │                      │  ROS2 Foxy                   │
│                                      │ ──── /cmd_vel ──────→│                              │
│  puzzlebot_control                   │                      │  puzzlebot_perception        │
│   ├── mpc_node           (MPC loop)  │                      │   └── vision_node            │
│   └── visualizer_node   (live plots) │                      │       (GStreamer + HSV + CV)  │
└──────────────────────────────────────┘                      └──────────────────────────────┘
```

Both machines share the `puzzlebot_msgs` package (compiled locally on each).

---

## Package layout

```
puzzlebot-visual-servoing/
├── shared/puzzlebot_msgs/         # CMake — custom VisionState.msg
├── jetson/puzzlebot_perception/   # Python — vision node (Foxy)
├── laptop/puzzlebot_control/      # Python — MPC + visualizer (Humble)
├── cyclonedds_laptop.xml          # DDS network config for laptop
├── cyclonedds_jetson.xml          # DDS network config for Jetson
├── env_laptop.sh                  # source before launching on laptop
└── env_jetson.sh                  # source before launching on Jetson
```

---

## 1. Prerequisites

### Both machines

```bash
sudo apt install ros-<distro>-rmw-cyclonedds-cpp
```

Replace `<distro>` with `humble` (laptop) or `foxy` (Jetson).

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

## 5. Network setup — ROS_DOMAIN_ID=42 via CycloneDDS

Source the environment script **before** any `ros2` command.

**On Jetson:**
```bash
source ~/dev_ws/src/control/puzzlebot-visual-servoing/env_jetson.sh
```

**On laptop (inside Docker):**
```bash
source /workspace/src/control/puzzlebot-visual-servoing/env_laptop.sh
```

These scripts export:
- `ROS_DOMAIN_ID=42`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI` pointing to the machine-specific XML config

Verify discovery after launching both nodes:

```bash
ros2 topic list        # /vision_state and /cmd_vel should appear on both machines
ros2 topic hz /vision_state
```

---

## 6. Run

### Jetson — launch vision node

```bash
source ~/dev_ws/src/control/puzzlebot-visual-servoing/env_jetson.sh
source ~/dev_ws/install/setup.bash
ros2 launch puzzlebot_perception perception.launch.py
```

### Laptop — launch MPC + visualizer

```bash
source /workspace/src/control/puzzlebot-visual-servoing/env_laptop.sh
source /workspace/install/setup.bash
ros2 launch puzzlebot_control control.launch.py
```

### Monitoring

```bash
# Watch MPC diagnostics (JSON with e_x, e_area, v, omega, solve_ms)
ros2 topic echo /mpc_debug

# Plot errors with rqt (alternative to the built-in visualizer)
ros2 run rqt_plot rqt_plot /mpc_debug
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

## 10. Rubric compliance

| Requirement                          | Implementation                                          |
|--------------------------------------|---------------------------------------------------------|
| Classical CV pipeline (no NN)        | HSV segmentation + morphological ops + contour moments  |
| Explicit cost function               | Quadratic stage + terminal cost (Section 7)             |
| Explicit constraints                 | `v ≥ 0`, `|ω| ≤ ω_max` enforced in grid construction   |
| Optimal control formulation          | Receding-horizon MPC with interaction matrix model      |
| End-to-end closed-loop demo          | vision_node → /vision_state → mpc_node → /cmd_vel      |
| Cross-machine ROS2 communication     | CycloneDDS, ROS_DOMAIN_ID=42, multicast on RoboNet      |
