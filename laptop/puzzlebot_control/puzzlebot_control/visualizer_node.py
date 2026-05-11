#!/usr/bin/env python3
"""
Real-time diagnostic visualizer for Puzzlebot visual servoing.

Runs on the laptop. Subscribes to /vision_state and /cmd_vel and shows
live plots of e_x, e_area, v, ω using matplotlib.

When running inside Docker, enable X11 forwarding:
  docker run ... -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ...

Falls back to terminal logging if matplotlib or a display is unavailable.
"""
import threading
import time
from collections import deque
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from puzzlebot_msgs.msg import VisionState

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    _MPL_OK = True
except Exception:
    _MPL_OK = False


class VisualizerNode(Node):

    _BUF = 300  # samples kept in rolling window

    def __init__(self) -> None:
        super().__init__('visualizer_node')

        self.declare_parameter('area_desired', 25000.0)
        self._area_d = self.get_parameter('area_desired').value

        self._lock = threading.Lock()
        self._t0   = time.time()

        # All buffers share the same time index (synced in _vs_cb)
        self._t_buf  = deque(maxlen=self._BUF)
        self._ex_buf = deque(maxlen=self._BUF)
        self._ea_buf = deque(maxlen=self._BUF)
        self._v_buf  = deque(maxlen=self._BUF)
        self._w_buf  = deque(maxlen=self._BUF)

        # Latest command — carried forward to sync with vision rate
        self._last_v: float = 0.0
        self._last_w: float = 0.0

        self.create_subscription(VisionState, '/vision_state', self._vs_cb,  10)
        self.create_subscription(Twist,       '/cmd_vel',      self._cmd_cb, 10)
        self.create_timer(1.0, self._log_state)  # terminal heartbeat at 1 Hz

        if _MPL_OK:
            threading.Thread(target=self._run_plot, daemon=True).start()
        else:
            self.get_logger().warn(
                'matplotlib unavailable or no display — terminal logging only'
            )

        self.get_logger().info('Visualizer node ready')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _vs_cb(self, msg: VisionState) -> None:
        e_area = (
            (msg.area - self._area_d) / max(self._area_d, 1.0)
            if msg.object_detected else 0.0
        )
        with self._lock:
            self._t_buf.append(time.time() - self._t0)
            self._ex_buf.append(float(msg.ex))
            self._ea_buf.append(e_area)
            self._v_buf.append(self._last_v)
            self._w_buf.append(self._last_w)

    def _cmd_cb(self, msg: Twist) -> None:
        with self._lock:
            self._last_v = float(msg.linear.x)
            self._last_w = float(msg.angular.z)

    def _log_state(self) -> None:
        with self._lock:
            ex = self._ex_buf[-1] if self._ex_buf else 0.0
            ea = self._ea_buf[-1] if self._ea_buf else 0.0
            v  = self._v_buf[-1]  if self._v_buf  else 0.0
            w  = self._w_buf[-1]  if self._w_buf  else 0.0
        self.get_logger().info(
            f'e_x={ex:+.3f}  e_area={ea:+.3f}  v={v:.3f} m/s  ω={w:+.3f} rad/s'
        )

    # ── Matplotlib thread ─────────────────────────────────────────────────────

    def _run_plot(self) -> None:
        plt.ion()
        fig, axes = plt.subplots(2, 2, figsize=(11, 7))
        fig.suptitle('Puzzlebot Visual Servoing — MPC Diagnostics', fontsize=13)

        labels = [
            ('e_x', 'Horizontal error (normalized)'),
            ('e_area', 'Area/distance error (normalized)'),
            ('v  [m/s]', 'Linear velocity'),
            ('ω  [rad/s]', 'Angular velocity'),
        ]
        lines = []
        for ax, (ylabel, title) in zip(axes.ravel(), labels):
            ax.set_title(title, fontsize=10)
            ax.set_xlabel('t [s]', fontsize=9)
            ax.set_ylabel(ylabel, fontsize=9)
            ax.axhline(0.0, color='k', linewidth=0.6, linestyle='--')
            (line,) = ax.plot([], [], linewidth=1.5)
            ax.grid(True, alpha=0.3)
            lines.append(line)

        plt.tight_layout()

        buffers = [self._ex_buf, self._ea_buf, self._v_buf, self._w_buf]

        while rclpy.ok():
            with self._lock:
                t = list(self._t_buf)
                data = [list(b) for b in buffers]

            for line, ax, d in zip(lines, axes.ravel(), data):
                n = min(len(t), len(d))
                if n > 1:
                    line.set_xdata(t[:n])
                    line.set_ydata(d[:n])
                    ax.relim()
                    ax.autoscale_view()

            fig.canvas.draw_idle()
            plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
