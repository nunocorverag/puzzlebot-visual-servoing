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
  ė_x    ≈ -ω          angular rate directly rotates the centroid
  ė_area ≈  Kv · v     forward speed scales area toward desired
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
DISCRETE MODEL (Euler, step dt):
  e_x[k+1]    = e_x[k]    − dt · ω[k]
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

Published:
  /cmd_vel       [geometry_msgs/Twist]
  /mpc_debug     [std_msgs/String]   JSON diagnostics
"""
import json
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

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
            ex = ex - self.dt * o
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
        self.declare_parameter('Rv',                0.1)
        self.declare_parameter('Ro',                0.1)
        self.declare_parameter('Px',                50.0)   # terminal weight e_x
        self.declare_parameter('Pa',                5.0)    # terminal weight e_area
        self.declare_parameter('v_max',             0.25)   # m/s
        self.declare_parameter('omega_max',         0.5)    # rad/s
        self.declare_parameter('v_candidates',      7)
        self.declare_parameter('omega_candidates',  11)
        self.declare_parameter('detection_timeout', 0.5)    # seconds

        self._area_d  = self.get_parameter('area_desired').value
        self._timeout = self.get_parameter('detection_timeout').value

        self._latest_vs: Optional[VisionState] = None
        self._last_seen: float = 0.0

        self._mpc = self._build_mpc()

        # ── Pub / Sub ─────────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            VisionState, '/vision_state', self._vision_cb, 10
        )
        self._pub_cmd  = self.create_publisher(Twist,  '/cmd_vel',   10)
        self._pub_diag = self.create_publisher(String, '/mpc_debug', 10)

        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_step)
        self.get_logger().info(
            f'MPC node ready  N={self.get_parameter("N").value}'
            f'  dt={self.get_parameter("dt").value}s'
            f'  candidates={self.get_parameter("v_candidates").value}'
            f'x{self.get_parameter("omega_candidates").value}'
        )

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
        if msg.object_detected:
            self._last_seen = time.time()

    def _control_step(self) -> None:
        vs  = self._latest_vs
        now = time.time()

        if vs is None or not vs.object_detected or (now - self._last_seen) > self._timeout:
            self._pub_cmd.publish(Twist())  # safe stop
            return

        e_x    = float(vs.ex)
        e_area = (float(vs.area) - self._area_d) / max(self._area_d, 1.0)

        t0             = time.perf_counter()
        v, omega, cost = self._mpc.solve(e_x, e_area)
        solve_ms       = (time.perf_counter() - t0) * 1_000.0

        cmd           = Twist()
        cmd.linear.x  = v
        cmd.angular.z = omega
        self._pub_cmd.publish(cmd)

        diag = {
            'e_x':      round(e_x,     4),
            'e_area':   round(e_area,  4),
            'v':        round(v,       4),
            'omega':    round(omega,   4),
            'cost':     round(cost,    4),
            'solve_ms': round(solve_ms, 2),
        }
        self._pub_diag.publish(String(data=json.dumps(diag)))


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
