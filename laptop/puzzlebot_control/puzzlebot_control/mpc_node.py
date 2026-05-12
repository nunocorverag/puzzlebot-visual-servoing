#!/usr/bin/env python3
"""
MPC control node for Puzzlebot visual servoing.

Runs on the laptop (ROS2 Humble, Docker). Implements a sampling-based
receding-horizon Model Predictive Controller (MPC) for Image-Based Visual
Servoing (IBVS).

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
STATE  s = [e_x, e_area]ᵀ
  e_x    ≜ (cx_pixels - cx_image) / cx_image  ∈ [-1, 1]
             positive when target is right of center
  e_area ≜ (area - area_d) / area_d
             positive when target is closer than desired
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
INTERACTION MATRIX (linearized, decoupled approximation):
  ė_x    ≈  ω          ROS +ω turns left; positive image error needs right turn
  ė_area ≈  Kv · v     forward speed scales area toward desired
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
DISCRETE MODEL (Euler, step dt):
  e_x[k+1]    = e_x[k]    + dt · ω[k]
  e_area[k+1] = e_area[k] + dt · Kv · v[k]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
COST FUNCTION (horizon N):
  J = Σ_{k=0}^{N−1} [ Qx·e_x[k]² + Qa·e_area[k]² + Rv·v[k]² + Rω·ω[k]² ]
      + Px·e_x[N]² + Pa·e_area[N]²
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CONSTRAINTS:
  v  ∈ [0, v_max]          (forward-only — robot should not reverse)
  ω  ∈ [−ω_max, ω_max]

SOLVER: exhaustive grid search over (Nv × Nω) constant actions (ZOH over
horizon). All rollouts are computed in a single vectorized numpy pass.
Re-solved at every control timestep (receding horizon principle).
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Subscribed:
  /vision_state  [puzzlebot_msgs/VisionState]
  /LaserDistance  [std_msgs/Float32]  optional frontal obstacle distance

Published:
  /cmd_vel       [geometry_msgs/Twist]
  /fsm_state     [std_msgs/String]
  /mpc_debug     [std_msgs/String]   JSON diagnostics
"""
import json
import csv
import os
import time
from datetime import datetime
from typing import Any, Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String

from puzzlebot_msgs.msg import VisionState


# ─── Solver (no ROS dependencies) ─────────────────────────────────────────────

class MPCController:
    """
    Vectorized exhaustive-search MPC.

    Candidate set: Cartesian product of Nv linear-velocity candidates and
    Nω angular-velocity candidates.  All N-step rollouts are propagated
    simultaneously via numpy broadcasting — typically < 1 ms on CPU.
    """

    def __init__(
        self,
        N: int, dt: float, Kv: float,
        Qx: float, Qa: float,
        Rv: float, Ro: float,
        Px: float, Pa: float,
        v_max: float, omega_max: float,
        Nv: int, No: int,
    ) -> None:
        self.N  = N
        self.dt = dt
        self.Kv = Kv
        self.Qx, self.Qa = Qx, Qa
        self.Rv, self.Ro = Rv, Ro
        self.Px, self.Pa = Px, Pa

        # Pre-build flat candidate arrays — shape (Nv*Nω,)
        v_grid = np.linspace(0.0,       v_max,     Nv)
        o_grid = np.linspace(-omega_max, omega_max, No)
        V, O   = np.meshgrid(v_grid, o_grid)
        self._v_cands = V.ravel()
        self._o_cands = O.ravel()

    def solve(self, e_x0: float, e_area0: float) -> Tuple[float, float, float]:
        """
        Return (v*, ω*, J*) for the given initial error state.

        The action held constant over the horizon (zero-order hold) that
        minimises J is returned as the optimal first control input.
        """
        v  = self._v_cands          # (N_a,)
        o  = self._o_cands          # (N_a,)
        ex = np.full(len(v), e_x0,    dtype=np.float64)
        ea = np.full(len(v), e_area0, dtype=np.float64)
        cost = np.zeros(len(v),       dtype=np.float64)

        for _ in range(self.N):
            cost += self.Qx * ex**2 + self.Qa * ea**2
            cost += self.Rv * v**2  + self.Ro * o**2
            # Discrete-time interaction matrix (Euler step)
            ex = ex + self.dt * o
            ea = ea + self.dt * self.Kv * v

        # Terminal penalty — larger weights to enforce convergence
        cost += self.Px * ex**2 + self.Pa * ea**2

        idx = int(np.argmin(cost))
        return float(self._v_cands[idx]), float(self._o_cands[idx]), float(cost[idx])


# ─── ROS2 node ────────────────────────────────────────────────────────────────

class MPCNode(Node):

    def __init__(self) -> None:
        super().__init__('mpc_node')

        # ── Parameters (overridable from mpc_params.yaml) ─────────────────────
        self.declare_parameter('N',                 5)
        self.declare_parameter('control_rate',      20.0)   # Hz
        self.declare_parameter('dt',                0.05)   # seconds
        self.declare_parameter('area_desired',      25000.0)# pixels²
        self.declare_parameter('Kv',                0.3)    # area interaction gain
        self.declare_parameter('Qx',                10.0)
        self.declare_parameter('Qa',                1.0)
        self.declare_parameter('Rv',                0.2)
        self.declare_parameter('Ro',                0.2)
        self.declare_parameter('Px',                50.0)   # terminal weight e_x
        self.declare_parameter('Pa',                5.0)    # terminal weight e_area
        self.declare_parameter('v_max',             0.06)   # m/s
        self.declare_parameter('omega_max',         0.20)   # rad/s
        self.declare_parameter('v_candidates',      7)
        self.declare_parameter('omega_candidates',  11)
        self.declare_parameter('max_v_step',        0.015)  # m/s per tick
        self.declare_parameter('max_omega_step',    0.05)   # rad/s per tick
        self.declare_parameter('detection_timeout', 0.5)    # seconds
        self.declare_parameter('angular_sign', -1.0)
        self.declare_parameter('enable_controller', True)
        self.declare_parameter('enable_search', True)
        self.declare_parameter('search_omega', 0.08)
        self.declare_parameter('search_direction', 1.0)
        self.declare_parameter('enable_acquire_state', True)
        self.declare_parameter('acquire_hold_sec', 0.30)
        self.declare_parameter('acquire_timeout_sec', 0.80)
        self.declare_parameter('target_lost_grace_sec', 0.40)
        self.declare_parameter('use_last_target_search_direction', False)
        self.declare_parameter('enable_goal_stop', True)
        self.declare_parameter('target_area_stop', 25000.0)
        self.declare_parameter('target_area_resume', 18000.0)
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.declare_parameter('obstacle_topic', '/LaserDistance')
        self.declare_parameter('vision_obstacle_topic', '/vision_obstacle_debug')
        self.declare_parameter('obstacle_distance_scale', 1.0)
        self.declare_parameter('obstacle_stop_distance', 0.12)
        self.declare_parameter('obstacle_avoid_distance', 0.30)
        self.declare_parameter('obstacle_clear_distance', 0.40)
        self.declare_parameter('obstacle_timeout_sec', 1.0)
        self.declare_parameter('avoid_omega', 0.14)
        self.declare_parameter('avoid_direction', 1.0)
        self.declare_parameter('avoid_forward_speed', 0.02)
        self.declare_parameter('avoid_reverse_speed', 0.0)
        self.declare_parameter('enable_visual_obstacle_avoidance', True)
        self.declare_parameter('visual_obstacle_close_required', True)
        self.declare_parameter('visual_obstacle_timeout_sec', 0.5)
        self.declare_parameter('visual_obstacle_clear_grace_sec', 0.30)
        self.declare_parameter('visual_obstacle_min_area', 2500.0)
        self.declare_parameter('visual_obstacle_center_deadband', 0.10)
        self.declare_parameter('visual_avoid_omega', 0.14)
        self.declare_parameter('visual_avoid_forward_speed', 0.0)
        self.declare_parameter('visual_avoid_default_direction', 1.0)
        self.declare_parameter('use_post_avoid_search_direction', True)
        self.declare_parameter('post_avoid_search_memory_sec', 2.0)
        self.declare_parameter('visual_obstacle_avoid_requires_target_context', True)
        self.declare_parameter('ignore_visual_obstacle_during_search_without_target', True)
        self.declare_parameter('visual_obstacle_blocks_target_ex_threshold', 0.30)
        self.declare_parameter('visual_obstacle_allow_avoid_in_search_if_laser_close', True)
        self.declare_parameter('require_camera_ready', True)
        self.declare_parameter('camera_ready_timeout_sec', 1.0)
        self.declare_parameter('camera_startup_grace_sec', 0.5)
        self.declare_parameter('camera_lost_stop', True)
        self.declare_parameter('camera_ready_min_messages', 3)
        self.declare_parameter('camera_ready_require_fresh_obstacle_debug', False)
        self.declare_parameter('camera_ready_obstacle_timeout_sec', 1.0)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_log_period_sec', 1.0)
        self.declare_parameter('enable_csv_log', True)
        self.declare_parameter('csv_log_dir', '/tmp/puzzlebot_logs')
        self.declare_parameter('csv_log_prefix', 'mpc_fsm_log')
        self.declare_parameter('csv_flush_every', 1)
        self.declare_parameter('safety_zero_burst_count', 10)
        self.declare_parameter('safety_zero_burst_dt', 0.05)
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')
        self.declare_parameter('enable_cmd_safety_clamp', True)
        self.declare_parameter('hard_v_limit', 0.08)
        self.declare_parameter('hard_omega_limit', 0.25)
        self.declare_parameter('enable_steering_sign_check', True)
        self.declare_parameter('enable_target_behind_obstacle_maneuver', True)
        self.declare_parameter('target_obstacle_confirm_sec', 0.4)
        self.declare_parameter('target_obstacle_min_confirm_frames', 5)
        self.declare_parameter('target_memory_sec', 2.0)
        self.declare_parameter('target_obstacle_ex_alignment_threshold', 0.35)
        self.declare_parameter('target_obstacle_requires_close', True)
        self.declare_parameter('target_behind_obstacle_cooldown_sec', 1.0)
        self.declare_parameter('target_behind_obstacle_max_retries', 3)
        self.declare_parameter('target_behind_avoid_turn_omega', 0.16)
        self.declare_parameter('target_behind_avoid_turn_sec', 0.8)
        self.declare_parameter('target_behind_forward_speed', 0.05)
        self.declare_parameter('target_behind_forward_omega', 0.0)
        self.declare_parameter('target_behind_forward_sec', 0.9)
        self.declare_parameter('target_behind_turn_back_omega', 0.14)
        self.declare_parameter('target_behind_turn_back_sec', 0.7)
        self.declare_parameter('target_behind_reacquire_omega', 0.10)
        self.declare_parameter('target_behind_reacquire_timeout_sec', 2.5)
        self.declare_parameter('reacquire_obstacle_confirm_sec', 0.50)
        self.declare_parameter('reacquire_obstacle_confirm_frames', 5)
        self.declare_parameter('reacquire_obstacle_center_ex_limit', 0.55)
        self.declare_parameter('reacquire_obstacle_ignore_edge_ex', 0.70)
        self.declare_parameter('reacquire_obstacle_min_area_ratio', 0.75)
        self.declare_parameter('reacquire_obstacle_requires_close', True)
        self.declare_parameter('reacquire_obstacle_retry_cooldown_sec', 1.5)
        self.declare_parameter('ignore_edge_obstacles_during_reacquire', True)

        self._area_d  = self.get_parameter('area_desired').value
        self._timeout = self.get_parameter('detection_timeout').value
        self._max_v_step = float(self.get_parameter('max_v_step').value)
        self._max_o_step = float(self.get_parameter('max_omega_step').value)
        self._v_max = float(self.get_parameter('v_max').value)
        self._omega_max = float(self.get_parameter('omega_max').value)
        self._debug_period = float(self.get_parameter('debug_log_period_sec').value)

        self._latest_vs: Optional[VisionState] = None
        self._last_seen: float = 0.0
        self._last_target_ex: Optional[float] = None
        self._last_target_seen_time: float = 0.0
        self._last_vision_state_time: float = 0.0
        self._vision_state_count: int = 0
        self._camera_ready: bool = False
        self._camera_was_ready: bool = False
        self._last_camera_ready_warn_time: float = 0.0
        self._last_camera_status: Dict[str, Any] = {
            'ready': False,
            'reason': 'no_vision_state_received',
            'vision_age_sec': None,
            'waiting_for_camera': True,
        }
        self._last_v: float = 0.0
        self._last_omega: float = 0.0
        self._last_v_controller: float = 0.0
        self._last_omega_controller: float = 0.0
        self._last_v_cmd: float = 0.0
        self._last_omega_cmd: float = 0.0
        self._last_smooth_applied: bool = False
        self._state: str = 'IDLE'
        self._previous_state: str = 'IDLE'
        self._transition_reason: str = 'startup_no_target'
        self._acquire_started: float = 0.0
        self._last_obstacle_raw: Optional[float] = None
        self._last_obstacle_distance: Optional[float] = None
        self._last_obstacle_time: float = 0.0
        self._latest_vision_obstacle: Dict[str, Any] = {}
        self._last_vision_obstacle_time: float = 0.0
        self._last_visual_obstacle_active_time: float = 0.0
        self._last_visual_obstacle_warn_time: float = 0.0
        self._last_visual_obstacle_status: Dict[str, Any] = {}
        self._last_avoid_source: str = 'none'
        self._last_avoid_turn_direction: float = 0.0
        self._last_avoid_end_time: float = 0.0
        self._last_visual_avoid_turn_direction: float = 0.0
        self._last_visual_avoid_reason: str = ''
        self._last_search_direction_used: float = 0.0
        self._post_avoid_search_active: bool = False
        self._post_avoid_search_direction: float = 0.0
        self._last_log_time: float = 0.0
        self._last_obstacle_missing_log_time: float = 0.0
        self._start_time: float = time.time()
        self._cycle_index: int = 0
        self._csv_file = None
        self._csv_writer: Optional[csv.DictWriter] = None
        self._csv_log_path: str = ''
        self._csv_rows_since_flush: int = 0
        self._csv_warn_time: float = 0.0
        self._shutdown_burst_done: bool = False
        self._emergency_stop_active: bool = False
        self._last_steering_check_log: float = 0.0
        self._target_behind_obstacle_active: bool = False
        self._target_behind_obstacle_confirmed: bool = False
        self._target_behind_obstacle_phase: str = 'none'
        self._target_behind_obstacle_phase_start_time: float = 0.0
        self._target_behind_obstacle_retry_count: int = 0
        self._target_behind_obstacle_confirm_frames: int = 0
        self._last_target_behind_turn_direction: float = 0.0
        self._last_target_ex_before_obstacle: Optional[float] = None
        self._last_target_area_before_obstacle: Optional[float] = None
        self._last_target_seen_before_obstacle_time: float = 0.0
        self._last_obstacle_ex_before_avoid: Optional[float] = None
        self._last_obstacle_area_before_avoid: Optional[float] = None
        self._last_target_behind_obstacle_end_time: float = 0.0
        self._reacquire_obstacle_first_seen_time: float = 0.0
        self._reacquire_obstacle_last_seen_time: float = 0.0
        self._reacquire_obstacle_confirm_frames: int = 0
        self._reacquire_obstacle_candidate_ex: float = 0.0
        self._reacquire_obstacle_candidate_area: float = 0.0
        self._last_reacquire_obstacle_retry_time: float = 0.0
        self._last_reacquire_obstacle_reason: str = 'no_visual_obstacle'
        self._last_reacquire_obstacle_confirmed: bool = False
        self._last_visual_obstacle_edge_ignored: bool = False
        self._last_visual_obstacle_in_path: bool = False
        self._last_visual_obstacle_ignore_reason: str = ''
        self._last_reacquire_obstacle_retry_allowed: bool = False

        self._mpc = self._build_mpc()
        self._init_csv_log()

        # ── Pub / Sub ─────────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            VisionState, '/vision_state', self._vision_cb, 10
        )
        obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._sub_obstacle = self.create_subscription(
            Float32, obstacle_topic, self._obstacle_cb, sensor_qos
        )
        vision_obstacle_topic = str(self.get_parameter('vision_obstacle_topic').value)
        self._sub_vision_obstacle = self.create_subscription(
            String, vision_obstacle_topic, self._vision_obstacle_cb, 10
        )
        emergency_stop_topic = str(self.get_parameter('emergency_stop_topic').value)
        self._sub_emergency_stop = self.create_subscription(
            Bool, emergency_stop_topic, self._emergency_stop_cb, 10
        )
        self._pub_cmd  = self.create_publisher(Twist,  '/cmd_vel',   10)
        self._pub_diag = self.create_publisher(String, '/mpc_debug', 10)
        self._pub_state = self.create_publisher(String, '/fsm_state', 10)

        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_step)
        self.get_logger().info(
            f'MPC node ready  N={self.get_parameter("N").value}'
            f'  dt={self.get_parameter("dt").value}s'
            f'  candidates={self.get_parameter("v_candidates").value}'
            f'x{self.get_parameter("omega_candidates").value}'
        )
        self.get_logger().info(
            f'Obstacle avoidance assumes frontal std_msgs/Float32 on {obstacle_topic}'
        )
        if self._csv_log_path:
            self.get_logger().info(f'MPC FSM CSV log: {self._csv_log_path}')

    # ── Private ────────────────────────────────────────────────────────────────

    def _build_mpc(self) -> MPCController:
        p = self.get_parameter
        return MPCController(
            N         = p('N').value,
            dt        = p('dt').value,
            Kv        = p('Kv').value,
            Qx        = p('Qx').value,
            Qa        = p('Qa').value,
            Rv        = p('Rv').value,
            Ro        = p('Ro').value,
            Px        = p('Px').value,
            Pa        = p('Pa').value,
            v_max     = p('v_max').value,
            omega_max = p('omega_max').value,
            Nv        = p('v_candidates').value,
            No        = p('omega_candidates').value,
        )

    def _vision_cb(self, msg: VisionState) -> None:
        self._latest_vs = msg
        now = time.time()
        self._last_vision_state_time = now
        self._vision_state_count += 1
        if msg.object_detected:
            self._last_seen = now
            self._last_target_seen_time = now
            self._last_target_ex = float(msg.ex)

    def _obstacle_cb(self, msg: Float32) -> None:
        raw_distance = float(msg.data)
        scale = float(self.get_parameter('obstacle_distance_scale').value)
        distance = raw_distance * scale
        if np.isfinite(distance) and distance >= 0.0:
            self._last_obstacle_raw = raw_distance
            self._last_obstacle_distance = distance
            self._last_obstacle_time = time.time()

    def _vision_obstacle_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self._latest_vision_obstacle = data
                self._last_vision_obstacle_time = time.time()
        except json.JSONDecodeError:
            return

    def _emergency_stop_cb(self, msg: Bool) -> None:
        if msg.data and not self._emergency_stop_active:
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
        self._emergency_stop_active = bool(msg.data)

    def _control_step(self) -> None:
        vs  = self._latest_vs
        now = time.time()
        self._cycle_index += 1
        detected = self._target_detected(vs, now)
        area = float(vs.area) if detected and vs is not None else 0.0
        ex = float(vs.ex) if detected and vs is not None else 0.0
        last_target_age = self._target_age(now)
        camera_ready, camera_reason, vision_age = self._camera_ready_status(now)
        self._last_camera_status = {
            'ready': camera_ready,
            'reason': camera_reason,
            'vision_age_sec': vision_age,
            'waiting_for_camera': not camera_ready,
        }
        obstacle_available, d_obs, obstacle_raw, obstacle_age = self._obstacle_status(now)
        laser_obstacle_active = self._obstacle_active(obstacle_available, d_obs)
        visual_status = self._visual_obstacle_status(now)
        avoid_active, avoid_source, avoid_turn_direction, avoid_v, avoid_omega = (
            self._avoid_status(laser_obstacle_active, d_obs, visual_status, now, self._state, detected, ex, area)
        )
        obstacle_active = avoid_active
        cost = None
        solve_ms = None

        if self._emergency_stop_active:
            state = 'EMERGENCY_STOP'
            v, omega = 0.0, 0.0
            self._publish_stop(state, 'emergency_stop_topic_active')
            self._publish_debug(
                state, ex, area, detected, last_target_age,
                obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                v, omega, stop_commanded=True, stop_reason='emergency_stop_topic_active'
            )
            return

        if not bool(self.get_parameter('enable_controller').value):
            state = 'IDLE'
            v, omega = 0.0, 0.0
            self._publish_stop(state, 'controller_disabled')
            self._publish_debug(
                state, ex, area, detected, last_target_age,
                obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                v, omega, stop_commanded=True, stop_reason='controller_disabled'
            )
            return

        if not camera_ready:
            state = 'WAIT_FOR_CAMERA'
            v, omega = 0.0, 0.0
            self._camera_ready = False
            self._publish_stop(state, camera_reason)
            self._log_camera_not_ready(now, camera_reason)
            self._publish_debug(
                state, ex, area, detected, last_target_age,
                obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                v, omega, stop_commanded=True, stop_reason='camera_not_ready',
                notes=camera_reason
            )
            return

        self._camera_ready = True
        self._camera_was_ready = True

        laser_in_stop_zone = (
            laser_obstacle_active
            and d_obs is not None
            and d_obs < float(self.get_parameter('obstacle_stop_distance').value)
        )
        if laser_in_stop_zone:
            state = 'AVOID'
            v, omega = avoid_v, avoid_omega
            self._last_avoid_source = avoid_source
            self._last_visual_avoid_turn_direction = (
                avoid_turn_direction if avoid_source in ('vision', 'both') else 0.0
            )
            self._publish_cmd(v, omega, state, smooth=False, transition_reason='laser_stop_zone_preempts_maneuver')
            self._publish_debug(
                state, ex, area, detected, last_target_age,
                obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                v, omega
            )
            return

        # ── Target behind obstacle maneuver ───────────────────────────────────────
        target_behind_suspected, target_behind_confirmed_check, target_behind_reason = (
            self._target_behind_obstacle_status(now, detected, ex, area, visual_status)
        )

        # Check if maneuver is already active
        maneuver_state, maneuver_v, maneuver_omega, maneuver_reason = (
            self._target_behind_obstacle_maneuver_command(now, detected, ex, area, visual_status)
        )
        if maneuver_reason in (
            'target_reacquired_after_obstacle',
            'target_behind_obstacle_give_up',
            'max_retries_reached',
        ):
            target_behind_suspected = False

        if maneuver_state is not None:
            # Maneuver is active, execute it
            self._publish_cmd(maneuver_v, maneuver_omega, maneuver_state, smooth=False, transition_reason=maneuver_reason)
            self._publish_debug(
                maneuver_state, ex, area, detected, last_target_age,
                obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                maneuver_v, maneuver_omega
            )
            return

        # Start maneuver if suspected and not in cooldown
        if target_behind_suspected and not self._target_behind_obstacle_active:
            cooldown_sec = float(self.get_parameter('target_behind_obstacle_cooldown_sec').value)
            since_last_maneuver = now - self._last_target_behind_obstacle_end_time
            if self._last_target_behind_obstacle_end_time <= 0.0 or since_last_maneuver >= cooldown_sec:
                # Start confirmation phase
                self._target_behind_obstacle_active = True
                self._target_behind_obstacle_confirmed = False
                self._target_behind_obstacle_phase = 'confirm'
                self._target_behind_obstacle_phase_start_time = now
                self._target_behind_obstacle_confirm_frames = 0
                self._last_target_seen_before_obstacle_time = self._last_target_seen_time
                
                state = 'OBSTACLE_CONFIRM'
                v, omega = 0.0, 0.0
                self._publish_stop(state, 'target_obstacle_confirm_start')
                self._publish_debug(
                    state, ex, area, detected, last_target_age,
                    obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                    v, omega, stop_commanded=True, stop_reason='target_obstacle_confirm_start',
                    notes=target_behind_reason
                )
                return

        if avoid_active:
            state = 'AVOID'
            if avoid_source == 'both':
                reason = 'both_obstacles_enter_avoid'
            elif avoid_source == 'vision':
                reason = 'visual_obstacle_enter_avoid'
            else:
                reason = (
                    'obstacle_enter_stop_zone'
                    if d_obs is not None and d_obs < float(self.get_parameter('obstacle_stop_distance').value)
                    else 'laser_obstacle_enter_avoid'
                )
            v, omega = avoid_v, avoid_omega
            self._last_avoid_source = avoid_source
            self._last_visual_avoid_turn_direction = (
                avoid_turn_direction if avoid_source in ('vision', 'both') else 0.0
            )
            self._publish_cmd(v, omega, state, smooth=False, transition_reason=reason)
            self._publish_debug(
                state, ex, area, detected, last_target_age,
                obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                v, omega
            )
            return

        if self._state == 'ACQUIRE_TARGET':
            if detected:
                hold_sec = float(self.get_parameter('acquire_hold_sec').value)
                if self._acquire_started <= 0.0:
                    self._acquire_started = now
                if (now - self._acquire_started) < hold_sec:
                    state = 'ACQUIRE_TARGET'
                    v, omega = 0.0, 0.0
                    self._publish_stop(state, 'acquire_hold')
                    self._publish_debug(
                        state, ex, area, detected, last_target_age,
                        obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                        v, omega, stop_commanded=True, stop_reason='acquire_hold'
                    )
                    return
            else:
                timeout_sec = float(self.get_parameter('acquire_timeout_sec').value)
                reference_time = max(self._last_seen, self._acquire_started)
                if reference_time > 0.0 and (now - reference_time) < timeout_sec:
                    state = 'ACQUIRE_TARGET'
                    v, omega = 0.0, 0.0
                    self._publish_stop(state, 'acquire_wait_for_target')
                    self._publish_debug(
                        state, ex, area, detected, last_target_age,
                        obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                        v, omega, stop_commanded=True, stop_reason='acquire_wait_for_target'
                    )
                    return
                self._acquire_started = 0.0

        if (
            self._state in ('SEARCH', 'AVOID')
            and detected
            and bool(self.get_parameter('enable_acquire_state').value)
        ):
            state = 'ACQUIRE_TARGET'
            self._acquire_started = now
            v, omega = 0.0, 0.0
            reason = (
                self._avoid_clear_reason(target_visible=True)
                if self._state == 'AVOID'
                else 'search_target_detected_acquire'
            )
            self._publish_stop(state, reason)
            self._publish_debug(
                state, ex, area, detected, last_target_age,
                obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                v, omega, stop_commanded=True, stop_reason=reason
            )
            return

        if (
            detected
            and bool(self.get_parameter('enable_goal_stop').value)
            and area >= float(self.get_parameter('target_area_stop').value)
        ):
            state = 'GOAL_REACHED'
            v, omega = 0.0, 0.0
            self._publish_stop(state, 'goal_area_reached')
            self._publish_debug(
                state, ex, area, detected, last_target_age,
                obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                v, omega, goal_reached=True, stop_commanded=True,
                stop_reason='goal_area_reached'
            )
            return

        if self._state == 'GOAL_REACHED':
            if detected and area >= float(self.get_parameter('target_area_resume').value):
                state = 'GOAL_REACHED'
                v, omega = 0.0, 0.0
                self._publish_stop(state, 'goal_hold_hysteresis')
                self._publish_debug(
                    state, ex, area, detected, last_target_age,
                    obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                    v, omega, goal_reached=True, stop_commanded=True,
                    stop_reason='goal_hold_hysteresis'
                )
                return
            if not detected:
                state, v, omega = self._search_command()
                self._publish_cmd(v, omega, state, smooth=False, transition_reason='goal_target_lost_search')
                self._publish_debug(
                    state, ex, area, detected, last_target_age,
                    obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                    v, omega
                )
                return

        if detected:
            self._acquire_started = 0.0
            state = 'TRACKING'
            e_area = (area - self._area_d) / max(self._area_d, 1.0)
            t0             = time.perf_counter()
            v, omega, cost = self._mpc.solve(ex, e_area)
            solve_ms       = (time.perf_counter() - t0) * 1_000.0
            v, omega       = self._limit_command_step(v, omega)
            reason = (
                'acquire_hold_complete_tracking'
                if self._previous_state == 'ACQUIRE_TARGET' or self._state == 'ACQUIRE_TARGET'
                else 'tracking_target_visible'
            )
            self._publish_cmd(v, omega, state, smooth=False, transition_reason=reason)
            self._publish_debug(
                state, ex, area, detected, last_target_age,
                obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                v, omega, cost=cost, solve_ms=solve_ms
            )
            return

        if self._state == 'TRACKING':
            grace_sec = float(self.get_parameter('target_lost_grace_sec').value)
            if self._last_seen > 0.0 and (now - self._last_seen) < grace_sec:
                state = 'TRACKING'
                v, omega = 0.0, 0.0
                self._publish_stop(state, 'tracking_target_lost_grace')
                self._publish_debug(
                    state, ex, area, detected, last_target_age,
                    obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
                    v, omega, stop_commanded=True, stop_reason='tracking_target_lost_grace'
                )
                return

        state, v, omega = self._search_command()
        reason = 'tracking_target_lost_search' if self._state == 'TRACKING' else 'startup_no_target'
        if self._state == 'ACQUIRE_TARGET':
            reason = 'acquire_target_lost_timeout_search'
        elif self._state == 'AVOID':
            reason = self._avoid_clear_reason(target_visible=False)
        if state == 'SEARCH' and self._post_avoid_search_active:
            reason = 'post_avoid_search_opposite_direction'
        elif self._state == 'IDLE' and state == 'IDLE':
            reason = 'idle_no_search'
        self._publish_cmd(v, omega, state, smooth=False, transition_reason=reason)
        self._publish_debug(
            state, ex, area, detected, last_target_age,
            obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
            v, omega
        )

    def _target_detected(self, vs: Optional[VisionState], now: float) -> bool:
        return (
            vs is not None
            and bool(vs.object_detected)
            and (now - self._last_seen) <= self._timeout
        )

    def _target_age(self, now: float) -> Optional[float]:
        if self._last_target_seen_time <= 0.0:
            return None
        return now - self._last_target_seen_time

    def _camera_ready_status(self, now: float) -> Tuple[bool, str, Optional[float]]:
        if not bool(self.get_parameter('require_camera_ready').value):
            return True, 'camera_not_required', None

        if self._last_vision_state_time <= 0.0:
            return False, 'no_vision_state_received', None

        vision_age = now - self._last_vision_state_time
        min_messages = max(int(self.get_parameter('camera_ready_min_messages').value), 1)
        if self._vision_state_count < min_messages:
            return False, 'not_enough_vision_messages', vision_age

        if vision_age > float(self.get_parameter('camera_ready_timeout_sec').value):
            return False, 'vision_state_stale', vision_age

        if bool(self.get_parameter('camera_ready_require_fresh_obstacle_debug').value):
            if self._last_vision_obstacle_time <= 0.0:
                return False, 'no_vision_obstacle_debug_received', vision_age
            obstacle_age = now - self._last_vision_obstacle_time
            if obstacle_age > float(self.get_parameter('camera_ready_obstacle_timeout_sec').value):
                return False, 'vision_obstacle_debug_stale', vision_age

        return True, 'camera_ready', vision_age

    def _log_camera_not_ready(self, now: float, reason: str) -> None:
        startup_grace = float(self.get_parameter('camera_startup_grace_sec').value)
        if (now - self._start_time) < startup_grace:
            return
        if (now - self._last_camera_ready_warn_time) < self._debug_period:
            return
        if self._camera_was_ready and bool(self.get_parameter('camera_lost_stop').value):
            self.get_logger().warn(
                f'Camera lost/stale; stopping robot until vision_state is fresh again. '
                f'reason={reason}'
            )
        else:
            self.get_logger().warn(f'Waiting for camera before moving. reason={reason}')
        self._last_camera_ready_warn_time = now

    def _obstacle_status(
        self, now: float
    ) -> Tuple[bool, Optional[float], Optional[float], Optional[float]]:
        if not bool(self.get_parameter('enable_obstacle_avoidance').value):
            return False, None, self._last_obstacle_raw, None
        if self._last_obstacle_distance is None:
            self._log_missing_obstacle_data(now)
            return False, None, self._last_obstacle_raw, None
        timeout = float(self.get_parameter('obstacle_timeout_sec').value)
        age = now - self._last_obstacle_time
        if age > timeout:
            self._log_missing_obstacle_data(now)
            return False, None, self._last_obstacle_raw, age
        return True, self._last_obstacle_distance, self._last_obstacle_raw, age

    def _log_missing_obstacle_data(self, now: float) -> None:
        if not bool(self.get_parameter('enable_obstacle_avoidance').value):
            return
        if (now - self._last_obstacle_missing_log_time) < self._debug_period:
            return
        topic = str(self.get_parameter('obstacle_topic').value)
        self.get_logger().warn(f'No recent obstacle data on {topic}')
        self._last_obstacle_missing_log_time = now

    def _obstacle_active(self, available: bool, distance: Optional[float]) -> bool:
        if not available or distance is None:
            return False
        avoid_distance = float(self.get_parameter('obstacle_avoid_distance').value)
        clear_distance = float(self.get_parameter('obstacle_clear_distance').value)
        threshold = clear_distance if self._state == 'AVOID' else avoid_distance
        return distance < threshold

    def _visual_obstacle_status(self, now: float) -> Dict[str, Any]:
        age = None if self._last_vision_obstacle_time <= 0.0 else now - self._last_vision_obstacle_time
        data = self._latest_vision_obstacle
        status = {
            'detected': bool(data.get('blue_obstacle_detected', False)),
            'close': bool(data.get('blue_obstacle_close', False)),
            'area': 0.0,
            'ex': 0.0,
            'age_sec': age,
            'raw_active': False,
            'active': False,
            'turn_direction': 0.0,
        }

        if not bool(self.get_parameter('enable_visual_obstacle_avoidance').value):
            self._last_visual_obstacle_status = status
            return status

        if age is None or age > float(self.get_parameter('visual_obstacle_timeout_sec').value):
            self._last_visual_obstacle_status = status
            return status

        try:
            area = float(data.get('blue_obstacle_area', 0.0))
            ex = float(data.get('blue_obstacle_ex', 0.0))
        except (TypeError, ValueError):
            self._warn_bad_visual_obstacle(now, 'non_numeric_visual_obstacle')
            self._last_visual_obstacle_status = status
            return status

        if not (np.isfinite(area) and np.isfinite(ex)):
            self._warn_bad_visual_obstacle(now, 'non_finite_visual_obstacle')
            self._last_visual_obstacle_status = status
            return status

        status['area'] = area
        status['ex'] = ex
        close_ok = bool(status['close']) or not bool(
            self.get_parameter('visual_obstacle_close_required').value
        )
        area_ok = area >= float(self.get_parameter('visual_obstacle_min_area').value)
        raw_active = bool(status['detected']) and close_ok and area_ok
        if raw_active:
            self._last_visual_obstacle_active_time = now

        clear_grace = float(self.get_parameter('visual_obstacle_clear_grace_sec').value)
        grace_active = (
            self._state == 'AVOID'
            and self._last_visual_obstacle_active_time > 0.0
            and (now - self._last_visual_obstacle_active_time) <= clear_grace
        )
        active = raw_active or grace_active
        status['raw_active'] = raw_active
        status['active'] = active
        if raw_active:
            status['turn_direction'] = self._visual_avoid_turn_direction(ex)
        elif grace_active and self._last_visual_avoid_turn_direction != 0.0:
            status['turn_direction'] = self._last_visual_avoid_turn_direction
        else:
            status['turn_direction'] = self._visual_avoid_turn_direction(ex)
        self._last_visual_obstacle_status = status
        return status

    def _warn_bad_visual_obstacle(self, now: float, reason: str) -> None:
        if (now - self._last_visual_obstacle_warn_time) < self._debug_period:
            return
        self.get_logger().warn(f'Ignoring malformed visual obstacle debug: {reason}')
        self._last_visual_obstacle_warn_time = now

    def _visual_avoid_turn_direction(self, obstacle_ex: float) -> float:
        deadband = float(self.get_parameter('visual_obstacle_center_deadband').value)
        if obstacle_ex > deadband:
            return 1.0   # obstacle right -> turn left
        if obstacle_ex < -deadband:
            return -1.0  # obstacle left -> turn right
        default_direction = float(self.get_parameter('visual_avoid_default_direction').value)
        return 1.0 if default_direction >= 0.0 else -1.0

    def _visual_obstacle_requires_avoidance(
        self,
        now: float,
        state: str,
        detected: bool,
        ex: float,
        area: float,
        visual_status: Dict[str, Any],
        laser_active: bool,
    ) -> Tuple[bool, str]:
        """
        Determine if visual obstacle requires avoidance based on context.
        Returns: (requires_avoidance, reason)
        """
        if not bool(self.get_parameter('visual_obstacle_avoid_requires_target_context').value):
            # Legacy behavior: visual obstacle always triggers avoid if active
            return bool(visual_status.get('active', False)), 'legacy_always_avoid'

        visual_active = bool(visual_status.get('active', False))
        if not visual_active:
            return False, 'no_visual_obstacle'

        obstacle_ex = float(visual_status.get('ex', 0.0))
        ex_threshold = float(self.get_parameter('visual_obstacle_blocks_target_ex_threshold').value)

        if self._target_behind_obstacle_phase == 'reacquire' or state == 'POST_AVOID_REACQUIRE':
            in_path, reason = self._visual_obstacle_in_path(visual_status, 'reacquire')
            self._last_visual_obstacle_in_path = in_path
            self._last_visual_obstacle_edge_ignored = reason == 'obstacle_on_edge_ignored'
            self._last_visual_obstacle_ignore_reason = '' if in_path else reason
            return False, reason if not in_path else 'reacquire_waiting_for_obstacle_confirmation'

        if self._visual_obstacle_is_edge_ignored(obstacle_ex) and state == 'SEARCH':
            self._last_visual_obstacle_edge_ignored = True
            self._last_visual_obstacle_in_path = False
            self._last_visual_obstacle_ignore_reason = 'obstacle_on_edge_ignored'
            return False, 'obstacle_on_edge_ignored'

        # Special case: SEARCH without target
        if state == 'SEARCH':
            ignore_in_search = bool(
                self.get_parameter('ignore_visual_obstacle_during_search_without_target').value
            )
            allow_if_laser = bool(
                self.get_parameter('visual_obstacle_allow_avoid_in_search_if_laser_close').value
            )
            
            # If laser is close, allow avoid even in SEARCH
            if laser_active and allow_if_laser:
                return True, 'search_laser_close_visual_confirm'
            
            # Check if target is stable/recent and aligned
            target_memory_sec = float(self.get_parameter('target_memory_sec').value)
            has_recent_target = (
                self._last_target_seen_time > 0.0
                and (now - self._last_target_seen_time) <= target_memory_sec
            )
            
            if not has_recent_target and ignore_in_search:
                return False, 'search_ignore_visual_obstacle_no_target'
            
            # Has recent target, check alignment
            if has_recent_target and self._last_target_ex is not None:
                ex_diff = abs(self._last_target_ex - obstacle_ex)
                if ex_diff <= ex_threshold:
                    return True, 'search_visual_obstacle_blocks_recent_target'
                else:
                    return False, 'search_visual_obstacle_not_aligned_with_target'
            
            # No target context, ignore if configured
            if ignore_in_search:
                return False, 'search_ignore_visual_obstacle_no_target_context'

        # Check if target-behind-obstacle maneuver is suspected/active
        if self._target_behind_obstacle_active or self._target_behind_obstacle_confirmed:
            return True, 'target_behind_obstacle_maneuver_active'

        # Check if target is currently visible and aligned with obstacle
        if detected:
            ex_diff = abs(ex - obstacle_ex)
            self._last_visual_obstacle_in_path = ex_diff <= ex_threshold
            if ex_diff <= ex_threshold:
                return True, 'visual_obstacle_blocks_visible_target'
            else:
                return False, 'visual_obstacle_not_blocking_visible_target'

        # Check if target was seen recently and obstacle is aligned
        target_memory_sec = float(self.get_parameter('target_memory_sec').value)
        if self._last_target_seen_time > 0.0:
            target_age = now - self._last_target_seen_time
            if target_age <= target_memory_sec and self._last_target_ex is not None:
                ex_diff = abs(self._last_target_ex - obstacle_ex)
                self._last_visual_obstacle_in_path = ex_diff <= ex_threshold
                if ex_diff <= ex_threshold:
                    return True, 'visual_obstacle_blocks_recent_target'
                else:
                    return False, 'visual_obstacle_not_aligned_with_recent_target'

        # No target context, don't avoid
        return False, 'visual_obstacle_not_blocking_target'

    def _visual_obstacle_is_edge_ignored(self, obstacle_ex: float) -> bool:
        if not bool(self.get_parameter('ignore_edge_obstacles_during_reacquire').value):
            return False
        edge_limit = abs(float(self.get_parameter('reacquire_obstacle_ignore_edge_ex').value))
        return abs(float(obstacle_ex)) >= edge_limit

    def _reset_reacquire_obstacle_confirmation(self, reason: str) -> None:
        self._reacquire_obstacle_first_seen_time = 0.0
        self._reacquire_obstacle_last_seen_time = 0.0
        self._reacquire_obstacle_confirm_frames = 0
        self._reacquire_obstacle_candidate_ex = 0.0
        self._reacquire_obstacle_candidate_area = 0.0
        self._last_reacquire_obstacle_reason = reason
        self._last_reacquire_obstacle_confirmed = False
        self._last_visual_obstacle_in_path = False

    def _visual_obstacle_in_path(
        self,
        visual_status: Dict[str, Any],
        context: str,
        target_ex: Optional[float] = None,
    ) -> Tuple[bool, str]:
        if not bool(visual_status.get('active', False)):
            return False, 'no_visual_obstacle'

        obstacle_ex = float(visual_status.get('ex', 0.0))
        obstacle_close = bool(visual_status.get('close', False))

        if self._visual_obstacle_is_edge_ignored(obstacle_ex):
            return False, 'obstacle_on_edge_ignored'

        if context == 'reacquire':
            if bool(self.get_parameter('reacquire_obstacle_requires_close').value) and not obstacle_close:
                return False, 'obstacle_not_close'
            center_limit = abs(float(self.get_parameter('reacquire_obstacle_center_ex_limit').value))
            if abs(obstacle_ex) > center_limit:
                return False, 'obstacle_not_in_path'
            return True, 'obstacle_in_reacquire_path'

        if context == 'tracking':
            if not obstacle_close or target_ex is None:
                return False, 'obstacle_not_close'
            ex_threshold = float(self.get_parameter('visual_obstacle_blocks_target_ex_threshold').value)
            if abs(float(target_ex) - obstacle_ex) <= ex_threshold:
                return True, 'visual_obstacle_blocks_visible_target'
            return False, 'visual_obstacle_not_blocking_visible_target'

        return False, 'search_visual_obstacle_not_in_path'

    def _reacquire_obstacle_confirmed(
        self,
        now: float,
        visual_status: Dict[str, Any],
    ) -> Tuple[bool, str]:
        visual_active = bool(visual_status.get('active', False))
        visual_detected = bool(visual_status.get('detected', False))
        if not (visual_active or visual_detected):
            self._last_visual_obstacle_edge_ignored = False
            self._last_visual_obstacle_ignore_reason = ''
            self._reset_reacquire_obstacle_confirmation('no_visual_obstacle')
            return False, 'no_visual_obstacle'

        obstacle_ex = float(visual_status.get('ex', 0.0))
        obstacle_area = float(visual_status.get('area', 0.0))
        self._reacquire_obstacle_candidate_ex = obstacle_ex
        self._reacquire_obstacle_candidate_area = obstacle_area

        in_path, path_reason = self._visual_obstacle_in_path(visual_status, 'reacquire')
        self._last_visual_obstacle_in_path = in_path
        self._last_visual_obstacle_edge_ignored = path_reason == 'obstacle_on_edge_ignored'
        self._last_visual_obstacle_ignore_reason = '' if in_path else path_reason
        if not in_path:
            self._reset_reacquire_obstacle_confirmation(path_reason)
            return False, path_reason

        previous_area = self._last_obstacle_area_before_avoid
        min_area_ratio = float(self.get_parameter('reacquire_obstacle_min_area_ratio').value)
        if previous_area is not None and previous_area > 0.0:
            area_ratio = obstacle_area / previous_area
            if area_ratio < min_area_ratio:
                reason = 'obstacle_area_too_small'
                self._last_visual_obstacle_ignore_reason = reason
                self._reset_reacquire_obstacle_confirmation(reason)
                return False, reason

        if self._reacquire_obstacle_first_seen_time <= 0.0:
            self._reacquire_obstacle_first_seen_time = now
            self._reacquire_obstacle_confirm_frames = 0
        self._reacquire_obstacle_last_seen_time = now
        self._reacquire_obstacle_confirm_frames += 1

        seen_duration = now - self._reacquire_obstacle_first_seen_time
        confirm_sec = float(self.get_parameter('reacquire_obstacle_confirm_sec').value)
        confirm_frames = int(self.get_parameter('reacquire_obstacle_confirm_frames').value)
        confirmed = (
            seen_duration >= confirm_sec
            or self._reacquire_obstacle_confirm_frames >= confirm_frames
        )
        if confirmed:
            self._last_reacquire_obstacle_reason = 'reacquire_obstacle_confirmed'
            self._last_reacquire_obstacle_confirmed = True
            self._last_visual_obstacle_ignore_reason = ''
            return True, 'reacquire_obstacle_confirmed'

        self._last_reacquire_obstacle_reason = 'reacquire_ignore_unconfirmed_obstacle'
        self._last_reacquire_obstacle_confirmed = False
        return False, 'reacquire_ignore_unconfirmed_obstacle'

    def _avoid_status(
        self,
        laser_active: bool,
        distance: Optional[float],
        visual_status: Dict[str, Any],
        now: float,
        state: str,
        detected: bool,
        ex: float,
        area: float,
    ) -> Tuple[bool, str, float, float, float]:
        # Check if visual obstacle requires avoidance based on context
        visual_requires_avoid, visual_avoid_reason = self._visual_obstacle_requires_avoidance(
            now, state, detected, ex, area, visual_status, laser_active
        )
        
        # Store reason for debugging
        self._last_visual_avoid_reason = visual_avoid_reason
        
        if laser_active and visual_requires_avoid:
            source = 'both'
        elif laser_active:
            source = 'laser'
        elif visual_requires_avoid:
            source = 'vision'
        else:
            return False, 'none', 0.0, 0.0, 0.0

        laser_v, laser_omega = self._avoid_command(distance)
        visual_direction = float(visual_status.get('turn_direction', 0.0))
        visual_v = float(self.get_parameter('visual_avoid_forward_speed').value)
        visual_omega = visual_direction * float(self.get_parameter('visual_avoid_omega').value)

        if source == 'laser':
            turn_direction = 1.0 if laser_omega >= 0.0 else -1.0
            return True, source, turn_direction, laser_v, laser_omega

        if source == 'vision':
            return True, source, visual_direction, visual_v, visual_omega

        turn_direction = visual_direction
        v = min(laser_v, visual_v)
        return True, source, turn_direction, v, visual_omega

    def _avoid_command(self, distance: Optional[float]) -> Tuple[float, float]:
        stop_distance = float(self.get_parameter('obstacle_stop_distance').value)
        v = float(self.get_parameter('avoid_forward_speed').value)
        if distance is None or distance < stop_distance:
            v = 0.0
        omega = (
            float(self.get_parameter('avoid_direction').value)
            * float(self.get_parameter('avoid_omega').value)
        )
        return v, omega

    def _search_command(self) -> Tuple[str, float, float]:
        if not bool(self.get_parameter('enable_search').value):
            self._last_search_direction_used = 0.0
            self._post_avoid_search_active = False
            self._post_avoid_search_direction = 0.0
            return 'IDLE', 0.0, 0.0
        direction = float(self.get_parameter('search_direction').value)
        self._post_avoid_search_active = False
        self._post_avoid_search_direction = 0.0
        now = time.time()
        within_post_avoid_window = (
            self._state == 'AVOID'
            or (
                self._last_avoid_end_time > 0.0
                and (now - self._last_avoid_end_time)
                <= float(self.get_parameter('post_avoid_search_memory_sec').value)
            )
        )
        if (
            bool(self.get_parameter('use_post_avoid_search_direction').value)
            and self._last_avoid_turn_direction != 0.0
            and within_post_avoid_window
        ):
            direction = -float(np.sign(self._last_avoid_turn_direction))
            self._post_avoid_search_active = True
            self._post_avoid_search_direction = direction
        if (
            not self._post_avoid_search_active
            and
            bool(self.get_parameter('use_last_target_search_direction').value)
            and self._last_target_ex is not None
            and abs(self._last_target_ex) > 1.0e-3
        ):
            # ex > 0 means target was to the right; ROS negative omega turns right.
            direction = -float(np.sign(self._last_target_ex))
        omega = (
            direction * float(self.get_parameter('search_omega').value)
        )
        self._last_search_direction_used = direction
        return 'SEARCH', 0.0, omega

    def _avoid_clear_reason(self, target_visible: bool) -> str:
        if self._last_avoid_source == 'vision':
            return (
                'visual_obstacle_clear_target_visible'
                if target_visible else 'visual_obstacle_clear_no_target_search'
            )
        if self._last_avoid_source == 'both':
            return (
                'both_obstacles_clear_target_visible'
                if target_visible else 'both_obstacles_clear_no_target_search'
            )
        return 'obstacle_clear_target_visible' if target_visible else 'obstacle_clear_no_target_search'

    def _target_behind_obstacle_status(
        self,
        now: float,
        detected: bool,
        ex: float,
        area: float,
        visual_status: Dict[str, Any],
    ) -> Tuple[bool, bool, str]:
        """
        Detect if target is behind/blocked by visual obstacle.
        Returns: (suspected, confirmed, reason)
        """
        if not bool(self.get_parameter('enable_target_behind_obstacle_maneuver').value):
            return False, False, 'maneuver_disabled'

        visual_detected = bool(visual_status.get('detected', False))
        visual_close = bool(visual_status.get('close', False))
        visual_ex = float(visual_status.get('ex', 0.0))
        visual_area = float(visual_status.get('area', 0.0))

        target_memory_sec = float(self.get_parameter('target_memory_sec').value)
        ex_threshold = float(self.get_parameter('target_obstacle_ex_alignment_threshold').value)
        requires_close = bool(self.get_parameter('target_obstacle_requires_close').value)

        # Case A: Both visible and aligned
        if detected and visual_detected:
            if requires_close and not visual_close:
                return False, False, 'obstacle_not_close'
            
            ex_diff = abs(ex - visual_ex)
            if ex_diff < ex_threshold:
                self._last_target_ex_before_obstacle = ex
                self._last_target_area_before_obstacle = area
                self._last_obstacle_ex_before_avoid = visual_ex
                self._last_obstacle_area_before_avoid = visual_area
                return True, False, 'both_visible_aligned'
            return False, False, 'not_aligned'

        # Case B: Target was seen recently, obstacle appeared
        if not detected and visual_detected:
            if requires_close and not visual_close:
                return False, False, 'obstacle_not_close'
            
            if self._last_target_seen_time <= 0.0:
                return False, False, 'no_target_memory'
            
            target_age = now - self._last_target_seen_time
            if target_age > target_memory_sec:
                return False, False, 'target_memory_expired'
            
            if self._last_target_ex is None:
                return False, False, 'no_target_ex_memory'
            
            ex_diff = abs(self._last_target_ex - visual_ex)
            if ex_diff < ex_threshold:
                self._last_target_ex_before_obstacle = self._last_target_ex
                self._last_obstacle_ex_before_avoid = visual_ex
                self._last_obstacle_area_before_avoid = visual_area
                return True, False, 'target_lost_obstacle_appeared'
            return False, False, 'not_aligned_memory'

        return False, False, 'no_conditions_met'

    def _target_behind_obstacle_maneuver_command(
        self,
        now: float,
        detected: bool,
        ex: float,
        area: float,
        visual_status: Dict[str, Any],
    ) -> Tuple[Optional[str], float, float, str]:
        """
        Execute target behind obstacle maneuver phases.
        Returns: (state, v, omega, reason) or (None, 0, 0, '') if not active
        """
        if not self._target_behind_obstacle_active:
            return None, 0.0, 0.0, ''

        phase = self._target_behind_obstacle_phase
        phase_elapsed = now - self._target_behind_obstacle_phase_start_time

        # OBSTACLE_CONFIRM phase
        if phase == 'confirm':
            confirm_sec = float(self.get_parameter('target_obstacle_confirm_sec').value)
            if phase_elapsed >= confirm_sec:
                # Confirmation complete, start turn
                self._target_behind_obstacle_confirmed = True
                self._target_behind_obstacle_phase = 'turn'
                self._target_behind_obstacle_phase_start_time = now
                
                # Decide turn direction based on obstacle position
                if self._last_obstacle_ex_before_avoid is not None:
                    if self._last_obstacle_ex_before_avoid > 0.0:
                        self._last_target_behind_turn_direction = -1.0  # obstacle right, turn left
                    else:
                        self._last_target_behind_turn_direction = 1.0   # obstacle left, turn right
                else:
                    default_dir = float(self.get_parameter('visual_avoid_default_direction').value)
                    self._last_target_behind_turn_direction = default_dir
                
                return 'OBSTACLE_CONFIRM', 0.0, 0.0, 'target_obstacle_confirmed'
            
            return 'OBSTACLE_CONFIRM', 0.0, 0.0, 'target_obstacle_confirm_hold'

        # AVOID_TURN phase
        elif phase == 'turn':
            turn_sec = float(self.get_parameter('target_behind_avoid_turn_sec').value)
            if phase_elapsed >= turn_sec:
                self._target_behind_obstacle_phase = 'forward'
                self._target_behind_obstacle_phase_start_time = now
                return 'AVOID_TURN', 0.0, 0.0, 'target_behind_avoid_turn_complete'
            
            turn_omega = float(self.get_parameter('target_behind_avoid_turn_omega').value)
            omega = self._last_target_behind_turn_direction * turn_omega
            return 'AVOID_TURN', 0.0, omega, 'target_behind_avoid_turn'

        # AVOID_FORWARD phase
        elif phase == 'forward':
            forward_sec = float(self.get_parameter('target_behind_forward_sec').value)
            if phase_elapsed >= forward_sec:
                self._target_behind_obstacle_phase = 'turn_back'
                self._target_behind_obstacle_phase_start_time = now
                return 'AVOID_FORWARD', 0.0, 0.0, 'target_behind_forward_complete'
            
            forward_v = float(self.get_parameter('target_behind_forward_speed').value)
            forward_omega = float(self.get_parameter('target_behind_forward_omega').value)
            omega = self._last_target_behind_turn_direction * forward_omega
            return 'AVOID_FORWARD', forward_v, omega, 'target_behind_forward_clear'

        # POST_AVOID_TURN_BACK phase
        elif phase == 'turn_back':
            turn_back_sec = float(self.get_parameter('target_behind_turn_back_sec').value)
            if phase_elapsed >= turn_back_sec:
                self._target_behind_obstacle_phase = 'reacquire'
                self._target_behind_obstacle_phase_start_time = now
                return 'POST_AVOID_TURN_BACK', 0.0, 0.0, 'target_behind_turn_back_complete'
            
            turn_back_omega = float(self.get_parameter('target_behind_turn_back_omega').value)
            omega = -self._last_target_behind_turn_direction * turn_back_omega
            return 'POST_AVOID_TURN_BACK', 0.0, omega, 'target_behind_turn_back'

        # POST_AVOID_REACQUIRE phase
        elif phase == 'reacquire':
            # Check if target reappeared
            if detected:
                self._clear_target_behind_obstacle_maneuver()
                return None, 0.0, 0.0, 'target_reacquired_after_obstacle'

            confirmed, obstacle_reason = self._reacquire_obstacle_confirmed(now, visual_status)
            if confirmed:
                self._last_visual_avoid_reason = 'reacquire_obstacle_confirmed'
                retry_cooldown = float(self.get_parameter('reacquire_obstacle_retry_cooldown_sec').value)
                retry_age = now - self._last_reacquire_obstacle_retry_time
                retry_allowed = (
                    self._last_reacquire_obstacle_retry_time <= 0.0
                    or retry_age >= retry_cooldown
                )
                self._last_reacquire_obstacle_retry_allowed = retry_allowed
                if not retry_allowed:
                    reacquire_omega = float(self.get_parameter('target_behind_reacquire_omega').value)
                    omega = -self._last_target_behind_turn_direction * reacquire_omega
                    return 'POST_AVOID_REACQUIRE', 0.0, omega, 'reacquire_obstacle_retry_cooldown'

                max_retries = int(self.get_parameter('target_behind_obstacle_max_retries').value)
                if self._target_behind_obstacle_retry_count >= max_retries:
                    self._clear_target_behind_obstacle_maneuver()
                    return None, 0.0, 0.0, 'max_retries_reached'

                self._target_behind_obstacle_retry_count += 1
                self._target_behind_obstacle_phase = 'confirm'
                self._target_behind_obstacle_phase_start_time = now
                self._target_behind_obstacle_confirm_frames = 0
                self._last_reacquire_obstacle_retry_time = now
                self._last_reacquire_obstacle_retry_allowed = True
                self._reset_reacquire_obstacle_confirmation('retry_started')
                self._last_reacquire_obstacle_confirmed = True
                self._last_reacquire_obstacle_reason = 'reacquire_confirmed_obstacle_retry'
                return 'OBSTACLE_CONFIRM', 0.0, 0.0, 'reacquire_confirmed_obstacle_retry'

            self._last_reacquire_obstacle_retry_allowed = False
            self._last_visual_avoid_reason = obstacle_reason
            
            reacquire_timeout = float(self.get_parameter('target_behind_reacquire_timeout_sec').value)
            if phase_elapsed >= reacquire_timeout:
                self._clear_target_behind_obstacle_maneuver()
                return None, 0.0, 0.0, 'target_behind_obstacle_give_up'
            
            reacquire_omega = float(self.get_parameter('target_behind_reacquire_omega').value)
            omega = -self._last_target_behind_turn_direction * reacquire_omega
            if obstacle_reason not in ('no_visual_obstacle', ''):
                return 'POST_AVOID_REACQUIRE', 0.0, omega, obstacle_reason
            return 'POST_AVOID_REACQUIRE', 0.0, omega, 'target_behind_reacquire'

        return None, 0.0, 0.0, ''

    def _clear_target_behind_obstacle_maneuver(self) -> None:
        """Clear target behind obstacle maneuver state."""
        self._target_behind_obstacle_active = False
        self._target_behind_obstacle_confirmed = False
        self._target_behind_obstacle_phase = 'none'
        self._target_behind_obstacle_phase_start_time = 0.0
        self._target_behind_obstacle_confirm_frames = 0
        self._last_target_behind_obstacle_end_time = time.time()
        self._reset_reacquire_obstacle_confirmation('maneuver_cleared')

    def _publish_stop(self, state: str, reason: str) -> None:
        self._last_v = 0.0
        self._last_omega = 0.0
        self._last_v_controller = 0.0
        self._last_omega_controller = 0.0
        self._last_v_cmd = 0.0
        self._last_omega_cmd = 0.0
        self._publish_cmd(0.0, 0.0, state, smooth=False, transition_reason=reason)

    def _check_steering_sign(self, ex: float, omega_cmd: float) -> None:
        """
        Verify steering direction matches expected behavior.
        ex > 0: target right -> should turn right (omega_cmd < 0 in ROS)
        ex < 0: target left  -> should turn left  (omega_cmd > 0 in ROS)
        """
        deadband = 0.05
        if abs(ex) < deadband or abs(omega_cmd) < 0.01:
            return

        expected_turn = 'left' if ex < -deadband else 'right' if ex > deadband else 'center'
        actual_turn = 'left' if omega_cmd > 0.01 else 'right' if omega_cmd < -0.01 else 'none'

        steering_ok = (
            (ex < -deadband and omega_cmd > 0.01) or
            (ex > deadband and omega_cmd < -0.01)
        )

        if not steering_ok:
            now = time.time()
            if (now - self._last_steering_check_log) >= 2.0:
                self.get_logger().warn(
                    f'STEERING APPEARS REVERSED: target {expected_turn} (ex={ex:.3f}) '
                    f'but omega_cmd turns {actual_turn} ({omega_cmd:.3f}). '
                    f'Consider flipping angular_sign parameter.',
                    throttle_duration_sec=5.0
                )
                self._last_steering_check_log = now

    def _publish_cmd(
        self,
        v: float,
        omega: float,
        state: str,
        smooth: bool = False,
        transition_reason: str = '',
    ) -> None:
        v, omega_controller = self._clamp_command(v, omega)
        if smooth:
            v, omega_controller = self._limit_command_step(v, omega_controller)

        angular_sign = float(self.get_parameter('angular_sign').value)
        omega_cmd = angular_sign * omega_controller
        omega_cmd = float(np.clip(omega_cmd, -self._omega_max, self._omega_max))

        # Safety: Check for NaN/inf
        if not (np.isfinite(v) and np.isfinite(omega_cmd)):
            self.get_logger().error(
                f'NaN/inf command detected: v={v}, omega_cmd={omega_cmd}. Publishing STOP.'
            )
            v, omega_cmd = 0.0, 0.0
            omega_controller = 0.0
            self._write_csv_row({
                'timestamp_wall': datetime.now().isoformat(timespec='milliseconds'),
                'time_since_start_sec': round(time.time() - self._start_time, 4),
                'cycle_index': self._cycle_index,
                'state': state,
                'stop_commanded': True,
                'stop_reason': 'nan_or_inf_command',
                'notes': f'v_raw={v}, omega_cmd_raw={omega_cmd}',
            })

        # Safety: Hard clamps (final layer of protection)
        if bool(self.get_parameter('enable_cmd_safety_clamp').value):
            hard_v_limit = float(self.get_parameter('hard_v_limit').value)
            hard_omega_limit = float(self.get_parameter('hard_omega_limit').value)
            v_clamped = float(np.clip(v, -hard_v_limit, hard_v_limit))
            omega_clamped = float(np.clip(omega_cmd, -hard_omega_limit, hard_omega_limit))
            if abs(v_clamped - v) > 1e-6 or abs(omega_clamped - omega_cmd) > 1e-6:
                self.get_logger().warn(
                    f'Hard safety clamp applied: v {v:.4f}->{v_clamped:.4f}, '
                    f'omega {omega_cmd:.4f}->{omega_clamped:.4f}',
                    throttle_duration_sec=2.0
                )
            v = v_clamped
            omega_cmd = omega_clamped

        # Steering sign check (only when tracking and object detected)
        if (
            state == 'TRACKING'
            and bool(self.get_parameter('enable_steering_sign_check').value)
            and self._latest_vs is not None
            and self._latest_vs.object_detected
        ):
            self._check_steering_sign(self._latest_vs.ex, omega_cmd)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega_cmd
        self._pub_cmd.publish(cmd)

        previous_state = self._state
        if state == 'AVOID' and abs(omega_controller) > 1.0e-6:
            self._last_avoid_turn_direction = float(np.sign(omega_controller))
        if previous_state == 'AVOID' and state != 'AVOID':
            self._last_avoid_end_time = time.time()
        self._last_v = v
        self._last_omega = omega_controller
        self._last_v_controller = v
        self._last_omega_controller = omega_controller
        self._last_v_cmd = v
        self._last_omega_cmd = omega_cmd
        self._last_smooth_applied = bool(smooth)
        self._previous_state = previous_state
        if state != previous_state or transition_reason:
            self._transition_reason = transition_reason or f'{previous_state.lower()}_to_{state.lower()}'
        self._state = state
        self._pub_state.publish(String(data=state))

    def publish_zero_cmd_burst(self, reason: str) -> None:
        count = int(self.get_parameter('safety_zero_burst_count').value)
        dt = float(self.get_parameter('safety_zero_burst_dt').value)
        count = max(count, 1)
        cmd = Twist()
        for _ in range(count):
            self._pub_cmd.publish(cmd)
            self._last_v = 0.0
            self._last_omega = 0.0
            self._last_v_controller = 0.0
            self._last_omega_controller = 0.0
            self._last_v_cmd = 0.0
            self._last_omega_cmd = 0.0
            time.sleep(max(dt, 0.0))
        self._previous_state = self._state
        self._transition_reason = reason
        self._write_csv_row({
            'timestamp_wall': datetime.now().isoformat(timespec='milliseconds'),
            'time_since_start_sec': round(time.time() - self._start_time, 4),
            'cycle_index': self._cycle_index,
            'state': self._state,
            'previous_state': self._previous_state,
            'transition_reason': reason,
            'v_controller': 0.0,
            'omega_controller': 0.0,
            'angular_sign': float(self.get_parameter('angular_sign').value),
            'v_cmd': 0.0,
            'omega_cmd': 0.0,
            'stop_commanded': True,
            'stop_reason': reason,
            'notes': 'safety_zero_cmd_burst',
        })

    def _publish_debug(
        self,
        state: str,
        ex: float,
        area: float,
        detected: bool,
        last_target_age: Optional[float],
        obstacle_available: bool,
        obstacle_active: bool,
        d_obs: Optional[float],
        obstacle_raw: Optional[float],
        obstacle_age: Optional[float],
        v: float,
        omega: float,
        cost: Optional[float] = None,
        solve_ms: Optional[float] = None,
        goal_reached: bool = False,
        stop_commanded: bool = False,
        stop_reason: str = '',
        notes: str = '',
    ) -> None:
        diag = self._build_debug_row(
            state, ex, area, detected, last_target_age,
            obstacle_available, obstacle_active, d_obs, obstacle_raw, obstacle_age,
            v, omega, cost, solve_ms, goal_reached, stop_commanded,
            stop_reason, notes
        )
        self._write_csv_row(diag)
        if not bool(self.get_parameter('publish_debug').value):
            return
        debug_msg = {
            'state': diag['state'],
            'previous_state': diag['previous_state'],
            'transition_reason': diag['transition_reason'],
            'ex': diag['ex'],
            'area': diag['area'],
            'object_detected': diag['object_detected'],
            'last_target_age_sec': diag['last_target_age_sec'],
            'camera_ready': diag['camera_ready'],
            'camera_status_reason': diag['camera_status_reason'],
            'vision_state_age_sec': diag['vision_state_age_sec'],
            'vision_state_count': diag['vision_state_count'],
            'require_camera_ready': diag['require_camera_ready'],
            'camera_lost_stop': diag['camera_lost_stop'],
            'waiting_for_camera': diag['waiting_for_camera'],
            'obstacle_available': diag['obstacle_available'],
            'obstacle_active': diag['obstacle_active'],
            'd_obs': diag['d_obs'],
            'obstacle_raw': diag['obstacle_raw'],
            'obstacle_distance_m': diag['obstacle_distance_m'],
            'obstacle_age_sec': diag['obstacle_age_sec'],
            'obstacle_thresholds': {
                'stop': diag['obstacle_stop_distance'],
                'avoid': diag['obstacle_avoid_distance'],
                'clear': diag['obstacle_clear_distance'],
            },
            'v_controller': diag['v_controller'],
            'omega_controller': diag['omega_controller'],
            'angular_sign': diag['angular_sign'],
            'v_cmd': diag['v_cmd'],
            'omega_cmd': diag['omega_cmd'],
            'cost': diag['cost'],
            'solve_ms': diag['solve_ms'],
            'csv_log_path': self._csv_log_path,
            'vision_blue_obstacle_detected': diag['vision_blue_obstacle_detected'],
            'vision_blue_obstacle_close': diag['vision_blue_obstacle_close'],
            'vision_blue_obstacle_area': diag['vision_blue_obstacle_area'],
            'vision_blue_obstacle_ex': diag['vision_blue_obstacle_ex'],
            'vision_blue_obstacle_count': diag['vision_blue_obstacle_count'],
            'vision_obstacle_age_sec': diag['vision_obstacle_age_sec'],
            'visual_obstacle_detected': diag['visual_obstacle_detected'],
            'visual_obstacle_close': diag['visual_obstacle_close'],
            'visual_obstacle_area': diag['visual_obstacle_area'],
            'visual_obstacle_ex': diag['visual_obstacle_ex'],
            'visual_obstacle_age_sec': diag['visual_obstacle_age_sec'],
            'visual_obstacle_active': diag['visual_obstacle_active'],
            'visual_obstacle_source_active': diag['visual_obstacle_source_active'],
            'avoid_source': diag['avoid_source'],
            'last_avoid_turn_direction': diag['last_avoid_turn_direction'],
            'visual_avoid_turn_direction_controller': diag['visual_avoid_turn_direction_controller'],
            'post_avoid_search_active': diag['post_avoid_search_active'],
            'post_avoid_search_direction': diag['post_avoid_search_direction'],
            'search_direction_used': diag['search_direction_used'],
            'target_behind_obstacle_suspected': diag['target_behind_obstacle_suspected'],
            'target_behind_obstacle_confirmed': diag['target_behind_obstacle_confirmed'],
            'target_behind_obstacle_active': diag['target_behind_obstacle_active'],
            'target_behind_obstacle_phase': diag['target_behind_obstacle_phase'],
            'target_behind_obstacle_phase_elapsed_sec': diag['target_behind_obstacle_phase_elapsed_sec'],
            'target_behind_obstacle_retry_count': diag['target_behind_obstacle_retry_count'],
            'target_behind_obstacle_reason': diag['target_behind_obstacle_reason'],
            'last_target_behind_turn_direction': diag['last_target_behind_turn_direction'],
            'last_target_ex_before_obstacle': diag['last_target_ex_before_obstacle'],
            'last_obstacle_ex_before_avoid': diag['last_obstacle_ex_before_avoid'],
            'post_avoid_reacquire_direction': diag['post_avoid_reacquire_direction'],
            'maneuver_v_controller': diag['maneuver_v_controller'],
            'maneuver_omega_controller': diag['maneuver_omega_controller'],
            'visual_obstacle_requires_avoidance': diag['visual_obstacle_requires_avoidance'],
            'visual_obstacle_avoidance_reason': diag['visual_obstacle_avoidance_reason'],
            'visual_obstacle_ignored_in_search': diag['visual_obstacle_ignored_in_search'],
            'visual_obstacle_ignore_reason': diag['visual_obstacle_ignore_reason'],
            'visual_obstacle_blocks_target': diag['visual_obstacle_blocks_target'],
            'visual_obstacle_target_ex_diff': diag['visual_obstacle_target_ex_diff'],
            'reacquire_obstacle_confirmed': diag['reacquire_obstacle_confirmed'],
            'reacquire_obstacle_confirm_frames': diag['reacquire_obstacle_confirm_frames'],
            'reacquire_obstacle_seen_duration_sec': diag['reacquire_obstacle_seen_duration_sec'],
            'reacquire_obstacle_reason': diag['reacquire_obstacle_reason'],
            'reacquire_obstacle_ex': diag['reacquire_obstacle_ex'],
            'reacquire_obstacle_area': diag['reacquire_obstacle_area'],
            'visual_obstacle_edge_ignored': diag['visual_obstacle_edge_ignored'],
            'visual_obstacle_in_path': diag['visual_obstacle_in_path'],
            'reacquire_obstacle_retry_allowed': diag['reacquire_obstacle_retry_allowed'],
            'last_reacquire_obstacle_retry_age_sec': diag['last_reacquire_obstacle_retry_age_sec'],
        }
        self._pub_diag.publish(String(data=json.dumps(debug_msg)))

        now = time.time()
        if (now - self._last_log_time) >= self._debug_period:
            self.get_logger().info(json.dumps(debug_msg))
            self._last_log_time = now

    def _build_debug_row(
        self,
        state: str,
        ex: float,
        area: float,
        detected: bool,
        last_target_age: Optional[float],
        obstacle_available: bool,
        obstacle_active: bool,
        d_obs: Optional[float],
        obstacle_raw: Optional[float],
        obstacle_age: Optional[float],
        v_controller: float,
        omega_controller: float,
        cost: Optional[float],
        solve_ms: Optional[float],
        goal_reached: bool,
        stop_commanded: bool,
        stop_reason: str,
        notes: str,
    ) -> Dict[str, Any]:
        now = time.time()
        acquire_elapsed = (
            now - self._acquire_started
            if self._acquire_started > 0.0 and state == 'ACQUIRE_TARGET'
            else None
        )
        visual_status = self._last_visual_obstacle_status
        camera_status = self._last_camera_status
        current_avoid_source = self._last_avoid_source if state == 'AVOID' else 'none'
        return {
            'timestamp_wall': datetime.now().isoformat(timespec='milliseconds'),
            'time_since_start_sec': round(now - self._start_time, 4),
            'cycle_index': self._cycle_index,
            'state': state,
            'previous_state': self._previous_state,
            'transition_reason': self._transition_reason,
            'object_detected': bool(detected),
            'ex': round(float(ex), 5),
            'area': round(float(area), 2),
            'last_target_ex': (
                None if self._last_target_ex is None else round(float(self._last_target_ex), 5)
            ),
            'last_target_age_sec': (
                None if last_target_age is None else round(float(last_target_age), 4)
            ),
            'camera_ready': bool(camera_status.get('ready', False)),
            'camera_status_reason': str(camera_status.get('reason', 'unknown')),
            'vision_state_age_sec': (
                None if camera_status.get('vision_age_sec') is None
                else round(float(camera_status.get('vision_age_sec')), 4)
            ),
            'vision_state_count': int(self._vision_state_count),
            'require_camera_ready': bool(self.get_parameter('require_camera_ready').value),
            'camera_lost_stop': bool(self.get_parameter('camera_lost_stop').value),
            'waiting_for_camera': bool(camera_status.get('waiting_for_camera', False)),
            'acquire_active': state == 'ACQUIRE_TARGET',
            'acquire_elapsed_sec': (
                None if acquire_elapsed is None else round(float(acquire_elapsed), 4)
            ),
            'acquire_hold_sec': float(self.get_parameter('acquire_hold_sec').value),
            'acquire_timeout_sec': float(self.get_parameter('acquire_timeout_sec').value),
            'target_lost_grace_sec': float(self.get_parameter('target_lost_grace_sec').value),
            'obstacle_raw': (
                None if obstacle_raw is None else round(float(obstacle_raw), 5)
            ),
            'obstacle_distance_m': (
                None if d_obs is None else round(float(d_obs), 5)
            ),
            'd_obs': None if d_obs is None else round(float(d_obs), 5),
            'obstacle_available': bool(obstacle_available),
            'obstacle_age_sec': (
                None if obstacle_age is None else round(float(obstacle_age), 4)
            ),
            'obstacle_active': bool(obstacle_active),
            'obstacle_stop_distance': float(self.get_parameter('obstacle_stop_distance').value),
            'obstacle_avoid_distance': float(self.get_parameter('obstacle_avoid_distance').value),
            'obstacle_clear_distance': float(self.get_parameter('obstacle_clear_distance').value),
            'obstacle_distance_scale': float(self.get_parameter('obstacle_distance_scale').value),
            'v_controller': round(float(v_controller), 5),
            'omega_controller': round(float(omega_controller), 5),
            'angular_sign': float(self.get_parameter('angular_sign').value),
            'v_cmd': round(float(self._last_v_cmd), 5),
            'omega_cmd': round(float(self._last_omega_cmd), 5),
            'v_last': round(float(self._last_v), 5),
            'omega_last': round(float(self._last_omega), 5),
            'smooth_applied': bool(self._last_smooth_applied),
            'cost': None if cost is None else round(float(cost), 5),
            'solve_ms': None if solve_ms is None else round(float(solve_ms), 3),
            'best_candidate_v': round(float(v_controller), 5),
            'best_candidate_omega': round(float(omega_controller), 5),
            'goal_reached': bool(goal_reached),
            'search_active': state == 'SEARCH',
            'avoid_active': state == 'AVOID',
            'stop_commanded': bool(stop_commanded),
            'stop_reason': stop_reason,
            'notes': notes,
            'csv_log_path': self._csv_log_path,
            'vision_blue_obstacle_detected': bool(
                self._latest_vision_obstacle.get('blue_obstacle_detected', False)
            ),
            'vision_blue_obstacle_close': bool(
                self._latest_vision_obstacle.get('blue_obstacle_close', False)
            ),
            'vision_blue_obstacle_area': self._latest_vision_obstacle.get(
                'blue_obstacle_area', 0.0
            ),
            'vision_blue_obstacle_ex': self._latest_vision_obstacle.get(
                'blue_obstacle_ex', 0.0
            ),
            'vision_blue_obstacle_count': self._latest_vision_obstacle.get(
                'blue_obstacle_count', 0
            ),
            'vision_obstacle_age_sec': (
                None if self._last_vision_obstacle_time <= 0.0
                else round(now - self._last_vision_obstacle_time, 4)
            ),
            'visual_obstacle_detected': bool(visual_status.get('detected', False)),
            'visual_obstacle_close': bool(visual_status.get('close', False)),
            'visual_obstacle_area': round(float(visual_status.get('area', 0.0)), 2),
            'visual_obstacle_ex': round(float(visual_status.get('ex', 0.0)), 5),
            'visual_obstacle_age_sec': (
                None if visual_status.get('age_sec') is None
                else round(float(visual_status.get('age_sec')), 4)
            ),
            'visual_obstacle_active': bool(visual_status.get('active', False)),
            'visual_obstacle_source_active': bool(visual_status.get('raw_active', False)),
            'avoid_source': current_avoid_source,
            'last_avoid_turn_direction': round(float(self._last_avoid_turn_direction), 5),
            'visual_avoid_turn_direction_controller': round(
                float(self._last_visual_avoid_turn_direction), 5
            ),
            'last_avoid_end_time_sec': (
                None if self._last_avoid_end_time <= 0.0
                else round(float(now - self._last_avoid_end_time), 4)
            ),
            'post_avoid_search_active': bool(self._post_avoid_search_active),
            'post_avoid_search_direction': round(float(self._post_avoid_search_direction), 5),
            'search_direction_used': round(float(self._last_search_direction_used), 5),
            'emergency_stop_active': bool(self._emergency_stop_active),
            'hard_v_limit': float(self.get_parameter('hard_v_limit').value),
            'hard_omega_limit': float(self.get_parameter('hard_omega_limit').value),
            'command_is_finite': bool(
                np.isfinite(self._last_v_cmd) and np.isfinite(self._last_omega_cmd)
            ),
            'expected_turn_direction': self._get_expected_turn_direction(ex),
            'actual_turn_direction': self._get_actual_turn_direction(self._last_omega_cmd),
            'steering_sign_ok': self._check_steering_sign_ok(ex, self._last_omega_cmd),
            'target_behind_obstacle_suspected': False,  # Will be set by caller if needed
            'target_behind_obstacle_confirmed': bool(self._target_behind_obstacle_confirmed),
            'target_behind_obstacle_active': bool(self._target_behind_obstacle_active),
            'target_behind_obstacle_phase': self._target_behind_obstacle_phase,
            'target_behind_obstacle_phase_elapsed_sec': (
                None if self._target_behind_obstacle_phase_start_time <= 0.0
                else round(now - self._target_behind_obstacle_phase_start_time, 4)
            ),
            'target_behind_obstacle_retry_count': self._target_behind_obstacle_retry_count,
            'target_behind_obstacle_reason': '',  # Will be set by caller if needed
            'last_target_behind_turn_direction': round(float(self._last_target_behind_turn_direction), 5),
            'last_target_ex_before_obstacle': (
                None if self._last_target_ex_before_obstacle is None
                else round(float(self._last_target_ex_before_obstacle), 5)
            ),
            'last_obstacle_ex_before_avoid': (
                None if self._last_obstacle_ex_before_avoid is None
                else round(float(self._last_obstacle_ex_before_avoid), 5)
            ),
            'post_avoid_reacquire_direction': (
                -self._last_target_behind_turn_direction
                if self._target_behind_obstacle_phase == 'reacquire'
                else 0.0
            ),
            'maneuver_v_controller': round(float(v_controller), 5),
            'maneuver_omega_controller': round(float(omega_controller), 5),
            'visual_obstacle_requires_avoidance': self._last_visual_avoid_reason not in [
                'no_visual_obstacle', 'search_ignore_visual_obstacle_no_target',
                'search_ignore_visual_obstacle_no_target_context',
                'search_visual_obstacle_not_aligned_with_target',
                'visual_obstacle_not_blocking_visible_target',
                'visual_obstacle_not_aligned_with_recent_target',
                'visual_obstacle_not_blocking_target',
                'obstacle_not_close',
                'obstacle_on_edge_ignored',
                'obstacle_not_in_path',
                'obstacle_area_too_small',
                'reacquire_ignore_unconfirmed_obstacle',
                'reacquire_waiting_for_obstacle_confirmation',
            ],
            'visual_obstacle_avoidance_reason': self._last_visual_avoid_reason,
            'visual_obstacle_ignored_in_search': (
                state == 'SEARCH' and self._last_visual_avoid_reason in [
                    'search_ignore_visual_obstacle_no_target',
                    'search_ignore_visual_obstacle_no_target_context',
                    'search_visual_obstacle_not_aligned_with_target',
                    'obstacle_on_edge_ignored',
                ]
            ),
            'visual_obstacle_ignore_reason': (
                self._last_visual_obstacle_ignore_reason
                if self._last_visual_obstacle_ignore_reason else
                self._last_visual_avoid_reason
                if (state == 'SEARCH' or state == 'POST_AVOID_REACQUIRE')
                and (
                    'ignore' in self._last_visual_avoid_reason
                    or self._last_visual_avoid_reason in [
                        'obstacle_not_close',
                        'obstacle_on_edge_ignored',
                        'obstacle_not_in_path',
                        'obstacle_area_too_small',
                    ]
                )
                else ''
            ),
            'visual_obstacle_blocks_target': self._last_visual_avoid_reason in [
                'visual_obstacle_blocks_visible_target',
                'visual_obstacle_blocks_recent_target',
                'search_visual_obstacle_blocks_recent_target',
                'target_behind_obstacle_maneuver_active',
                'reacquire_obstacle_confirmed',
            ],
            'visual_obstacle_target_ex_diff': (
                None if not visual_status.get('detected', False) or ex is None
                else round(abs(float(ex) - float(visual_status.get('ex', 0.0))), 5)
            ),
            'reacquire_obstacle_confirmed': bool(self._last_reacquire_obstacle_confirmed),
            'reacquire_obstacle_confirm_frames': int(self._reacquire_obstacle_confirm_frames),
            'reacquire_obstacle_seen_duration_sec': (
                None if self._reacquire_obstacle_first_seen_time <= 0.0
                else round(now - self._reacquire_obstacle_first_seen_time, 4)
            ),
            'reacquire_obstacle_reason': self._last_reacquire_obstacle_reason,
            'reacquire_obstacle_ex': round(float(self._reacquire_obstacle_candidate_ex), 5),
            'reacquire_obstacle_area': round(float(self._reacquire_obstacle_candidate_area), 2),
            'visual_obstacle_edge_ignored': bool(self._last_visual_obstacle_edge_ignored),
            'visual_obstacle_in_path': bool(self._last_visual_obstacle_in_path),
            'reacquire_obstacle_retry_allowed': bool(self._last_reacquire_obstacle_retry_allowed),
            'last_reacquire_obstacle_retry_age_sec': (
                None if self._last_reacquire_obstacle_retry_time <= 0.0
                else round(now - self._last_reacquire_obstacle_retry_time, 4)
            ),
        }

    def _get_expected_turn_direction(self, ex: float) -> str:
        deadband = 0.05
        if ex < -deadband:
            return 'left'
        elif ex > deadband:
            return 'right'
        else:
            return 'center'

    def _get_actual_turn_direction(self, omega_cmd: float) -> str:
        deadband = 0.01
        if omega_cmd > deadband:
            return 'left'
        elif omega_cmd < -deadband:
            return 'right'
        else:
            return 'none'

    def _check_steering_sign_ok(self, ex: float, omega_cmd: float) -> bool:
        deadband_ex = 0.05
        deadband_omega = 0.01
        if abs(ex) < deadband_ex or abs(omega_cmd) < deadband_omega:
            return True
        return (
            (ex < -deadband_ex and omega_cmd > deadband_omega) or
            (ex > deadband_ex and omega_cmd < -deadband_omega)
        )

    def _init_csv_log(self) -> None:
        if not bool(self.get_parameter('enable_csv_log').value):
            return
        try:
            log_dir = str(self.get_parameter('csv_log_dir').value)
            prefix = str(self.get_parameter('csv_log_prefix').value)
            os.makedirs(log_dir, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._csv_log_path = os.path.join(log_dir, f'{prefix}_{stamp}.csv')
            self._csv_file = open(self._csv_log_path, 'w', newline='', encoding='utf-8')
            self._csv_writer = csv.DictWriter(
                self._csv_file,
                fieldnames=self._csv_fieldnames(),
                extrasaction='ignore',
            )
            self._csv_writer.writeheader()
            self._csv_file.flush()
        except Exception as exc:
            self._csv_log_path = ''
            self._csv_file = None
            self._csv_writer = None
            self.get_logger().warn(f'CSV logging disabled: {exc}')

    def _csv_fieldnames(self):
        return [
            'timestamp_wall', 'time_since_start_sec', 'cycle_index',
            'state', 'previous_state', 'transition_reason',
            'object_detected', 'ex', 'area', 'last_target_ex',
            'last_target_age_sec', 'acquire_active', 'acquire_elapsed_sec',
            'camera_ready', 'camera_status_reason', 'vision_state_age_sec',
            'vision_state_count', 'require_camera_ready', 'camera_lost_stop',
            'waiting_for_camera',
            'acquire_hold_sec', 'acquire_timeout_sec', 'target_lost_grace_sec',
            'obstacle_raw', 'obstacle_distance_m', 'obstacle_available',
            'obstacle_age_sec', 'obstacle_active', 'obstacle_stop_distance',
            'obstacle_avoid_distance', 'obstacle_clear_distance',
            'obstacle_distance_scale', 'v_controller', 'omega_controller',
            'angular_sign', 'v_cmd', 'omega_cmd', 'v_last', 'omega_last',
            'smooth_applied', 'cost', 'solve_ms', 'best_candidate_v',
            'best_candidate_omega', 'goal_reached', 'search_active',
            'avoid_active', 'stop_commanded', 'stop_reason', 'notes',
            'd_obs', 'csv_log_path',
            'vision_blue_obstacle_detected', 'vision_blue_obstacle_close',
            'vision_blue_obstacle_area', 'vision_blue_obstacle_ex',
            'vision_blue_obstacle_count', 'vision_obstacle_age_sec',
            'visual_obstacle_detected', 'visual_obstacle_close',
            'visual_obstacle_area', 'visual_obstacle_ex',
            'visual_obstacle_age_sec', 'visual_obstacle_active',
            'visual_obstacle_source_active', 'avoid_source',
            'last_avoid_turn_direction', 'visual_avoid_turn_direction_controller',
            'last_avoid_end_time_sec', 'post_avoid_search_active',
            'post_avoid_search_direction', 'search_direction_used',
            'emergency_stop_active', 'hard_v_limit', 'hard_omega_limit',
            'command_is_finite', 'expected_turn_direction', 'actual_turn_direction',
            'steering_sign_ok',
            'target_behind_obstacle_suspected', 'target_behind_obstacle_confirmed',
            'target_behind_obstacle_active', 'target_behind_obstacle_phase',
            'target_behind_obstacle_phase_elapsed_sec', 'target_behind_obstacle_retry_count',
            'target_behind_obstacle_reason', 'last_target_behind_turn_direction',
            'last_target_ex_before_obstacle', 'last_obstacle_ex_before_avoid',
            'post_avoid_reacquire_direction', 'maneuver_v_controller', 'maneuver_omega_controller',
            'visual_obstacle_requires_avoidance', 'visual_obstacle_avoidance_reason',
            'visual_obstacle_ignored_in_search', 'visual_obstacle_ignore_reason',
            'visual_obstacle_blocks_target', 'visual_obstacle_target_ex_diff',
            'reacquire_obstacle_confirmed', 'reacquire_obstacle_confirm_frames',
            'reacquire_obstacle_seen_duration_sec', 'reacquire_obstacle_reason',
            'reacquire_obstacle_ex', 'reacquire_obstacle_area',
            'visual_obstacle_edge_ignored', 'visual_obstacle_in_path',
            'reacquire_obstacle_retry_allowed', 'last_reacquire_obstacle_retry_age_sec',
        ]

    def _write_csv_row(self, row: Dict[str, Any]) -> None:
        if self._csv_writer is None or self._csv_file is None:
            return
        try:
            self._csv_writer.writerow(row)
            self._csv_rows_since_flush += 1
            flush_every = max(int(self.get_parameter('csv_flush_every').value), 1)
            if self._csv_rows_since_flush >= flush_every:
                self._csv_file.flush()
                self._csv_rows_since_flush = 0
        except Exception as exc:
            now = time.time()
            if (now - self._csv_warn_time) >= self._debug_period:
                self.get_logger().warn(f'CSV write failed: {exc}')
                self._csv_warn_time = now

    def _close_csv_log(self) -> None:
        if self._csv_file is None:
            return
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except Exception as exc:
            self.get_logger().warn(f'CSV close failed: {exc}')
        finally:
            self._csv_file = None
            self._csv_writer = None

    def destroy_node(self) -> bool:
        if not self._shutdown_burst_done:
            self._shutdown_burst_done = True
            try:
                self.publish_zero_cmd_burst('safety_stop_shutdown')
            except Exception as exc:
                self.get_logger().warn(f'Failed to publish shutdown zero burst: {exc}')
        self._close_csv_log()
        return super().destroy_node()

    def _clamp_command(self, v: float, omega: float) -> Tuple[float, float]:
        return (
            float(np.clip(v, 0.0, self._v_max)),
            float(np.clip(omega, -self._omega_max, self._omega_max)),
        )

    def _limit_command_step(self, v: float, omega: float) -> Tuple[float, float]:
        v_min = self._last_v - self._max_v_step
        v_max = self._last_v + self._max_v_step
        o_min = self._last_omega - self._max_o_step
        o_max = self._last_omega + self._max_o_step
        return (
            float(np.clip(v, v_min, v_max)),
            float(np.clip(omega, o_min, o_max)),
        )


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
