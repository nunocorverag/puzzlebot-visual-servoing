#!/usr/bin/env python3
"""
Vision node for Puzzlebot visual servoing.

Runs on Jetson Nano (ROS2 Foxy). Opens the CSI camera via GStreamer
(nvarguscamerasrc), detects a colored target using HSV segmentation, and
publishes a VisionState message at camera rate.

Classical CV pipeline:
  BGR → HSV → dual-range mask → morphological clean → contour selection
  → image-moment centroid → normalized error

Published:
  /vision_state  [puzzlebot_msgs/VisionState]
"""
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from puzzlebot_msgs.msg import VisionState


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # ── Parameters ──────────────────────────────────────────────────────────
        self.declare_parameter('camera_width',    640)
        self.declare_parameter('camera_height',   480)
        self.declare_parameter('camera_fps',      30)
        self.declare_parameter('use_gstreamer',   True)
        self.declare_parameter('min_contour_area', 500.0)
        # Red wraps around 0° in OpenCV HSV (0-180 scale), so two ranges:
        self.declare_parameter('hsv_lower1', [0,   100, 80])   # low-hue red
        self.declare_parameter('hsv_upper1', [10,  255, 255])
        self.declare_parameter('hsv_lower2', [170, 100, 80])   # high-hue red
        self.declare_parameter('hsv_upper2', [180, 255, 255])

        W   = self.get_parameter('camera_width').value
        H   = self.get_parameter('camera_height').value
        fps = self.get_parameter('camera_fps').value

        self._cx_image = W / 2.0   # image horizontal center (pixels)
        self._min_area = self.get_parameter('min_contour_area').value

        # ── Camera ──────────────────────────────────────────────────────────────
        if self.get_parameter('use_gstreamer').value:
            pipeline = (
                f'nvarguscamerasrc ! '
                f'video/x-raw(memory:NVMM), '
                f'width=(int){W}, height=(int){H}, framerate=(fraction){fps}/1 ! '
                f'nvvidconv flip-method=0 ! '
                f'video/x-raw, format=(string)BGRx ! '
                f'videoconvert ! '
                f'video/x-raw, format=(string)BGR ! '
                f'appsink drop=1'
            )
            self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            # Fallback: standard USB/V4L2 camera (useful for desktop testing)
            self._cap = cv2.VideoCapture(0)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  W)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
            self._cap.set(cv2.CAP_PROP_FPS, fps)

        if not self._cap.isOpened():
            self.get_logger().error('Camera failed to open — check GStreamer pipeline')
            raise RuntimeError('Camera unavailable')

        # ── Morphological kernel reused every frame ───────────────────────────
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        # ── Publisher ────────────────────────────────────────────────────────────
        self._pub = self.create_publisher(VisionState, '/vision_state', 10)

        self.create_timer(1.0 / fps, self._process_frame)
        self.get_logger().info(
            f'Vision node started — {W}x{H} @ {fps} fps  '
            f'(gstreamer={self.get_parameter("use_gstreamer").value})'
        )

    # ── Pipeline steps ────────────────────────────────────────────────────────

    def _process_frame(self) -> None:
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('Frame read failed', throttle_duration_sec=2.0)
            self._publish(0.0, 0.0, False)
            return

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._segment(hsv)
        mask = self._clean(mask)
        ex, area, detected = self._measure(mask)
        self._publish(ex, area, detected)

    def _segment(self, hsv: 'np.ndarray') -> 'np.ndarray':
        """Dual-range HSV threshold — handles red's hue wrap at 0/180."""
        l1 = np.array(self.get_parameter('hsv_lower1').value, dtype=np.uint8)
        u1 = np.array(self.get_parameter('hsv_upper1').value, dtype=np.uint8)
        l2 = np.array(self.get_parameter('hsv_lower2').value, dtype=np.uint8)
        u2 = np.array(self.get_parameter('hsv_upper2').value, dtype=np.uint8)
        return cv2.bitwise_or(cv2.inRange(hsv, l1, u1), cv2.inRange(hsv, l2, u2))

    def _clean(self, mask: 'np.ndarray') -> 'np.ndarray':
        """Open → Close: removes noise speckles, fills small holes."""
        k = self._kernel
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        return mask

    def _measure(self, mask: 'np.ndarray'):
        """Select largest valid contour; compute sub-pixel centroid via moments."""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return 0.0, 0.0, False

        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)

        if area < self._min_area:
            return 0.0, 0.0, False

        M = cv2.moments(best)
        if M['m00'] == 0.0:
            return 0.0, 0.0, False

        cx_px = M['m10'] / M['m00']                       # sub-pixel centroid x
        ex    = (cx_px - self._cx_image) / self._cx_image  # normalize to [-1, 1]
        return float(ex), float(area), True

    def _publish(self, ex: float, area: float, detected: bool) -> None:
        msg              = VisionState()
        msg.ex           = ex
        msg.area         = area
        msg.object_detected = detected
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
