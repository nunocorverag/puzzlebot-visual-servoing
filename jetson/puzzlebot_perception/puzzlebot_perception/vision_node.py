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
import math
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
        self.declare_parameter('hsv_lower', [0, 80, 61])
        self.declare_parameter('hsv_upper', [25, 255, 255])
        self.declare_parameter('circularity_min', 0.38)
        self.declare_parameter('min_circularity_soft', 0.35)
        self.declare_parameter('aspect_ratio_min', 0.65)
        self.declare_parameter('aspect_ratio_max', 1.55)
        self.declare_parameter('min_fill_ratio', 0.35)
        self.declare_parameter('max_fill_ratio', 1.15)
        self.declare_parameter('use_shape_filter', True)
        self.declare_parameter('hard_shape_filter', True)
        self.declare_parameter('area_score_weight', 0.10)
        self.declare_parameter('shape_score_weight', 0.65)
        self.declare_parameter('aspect_score_weight', 0.20)
        self.declare_parameter('fill_score_weight', 0.20)
        self.declare_parameter('center_score_weight', 0.05)
        self.declare_parameter('min_detection_score', 0.55)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('lost_frames', 4)
        self.declare_parameter('ex_smoothing_alpha', 0.35)
        self.declare_parameter('area_smoothing_alpha', 0.35)
        self.declare_parameter('ex_deadband', 0.05)
        self.declare_parameter('show_debug_view', True)
        # Legacy red parameters are kept so old launch overrides do not fail.
        self.declare_parameter('hsv_lower1', [0,   100, 80])   # low-hue red
        self.declare_parameter('hsv_upper1', [10,  255, 255])
        self.declare_parameter('hsv_lower2', [170, 100, 80])   # high-hue red
        self.declare_parameter('hsv_upper2', [180, 255, 255])

        W   = self.get_parameter('camera_width').value
        H   = self.get_parameter('camera_height').value
        fps = self.get_parameter('camera_fps').value

        self._cx_image = W / 2.0   # image horizontal center (pixels)
        self._min_area = float(self.get_parameter('min_contour_area').value)
        self._show_debug = bool(self.get_parameter('show_debug_view').value)
        self._debug_failed = False
        self._confirm_count = 0
        self._lost_count = 0
        self._confirmed = False
        self._smooth_ex = 0.0
        self._smooth_area = 0.0
        self._last_metrics = None

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
        ex, area, detected, contour, metrics = self._measure(mask)
        ex, area, detected = self._apply_temporal_filter(ex, area, detected, metrics)
        self._publish(ex, area, detected)
        self._show_preview(frame, ex, detected, contour, metrics)

    def _segment(self, hsv: 'np.ndarray') -> 'np.ndarray':
        """Single configurable HSV threshold for orange/terracotta targets."""
        lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        return cv2.inRange(hsv, lower, upper)

    def _clean(self, mask: 'np.ndarray') -> 'np.ndarray':
        """Open → Close: removes noise speckles, fills small holes."""
        k = self._kernel
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        return mask

    def _measure(self, mask: 'np.ndarray'):
        """Select best valid contour; compute centroid and shape metrics."""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return 0.0, 0.0, False, None, None

        use_shape_filter = bool(self.get_parameter('use_shape_filter').value)
        hard_shape_filter = bool(self.get_parameter('hard_shape_filter').value)
        circularity_min = float(self.get_parameter('circularity_min').value)
        min_circularity_soft = float(
            self.get_parameter('min_circularity_soft').value
        )
        aspect_min = float(self.get_parameter('aspect_ratio_min').value)
        aspect_max = float(self.get_parameter('aspect_ratio_max').value)
        min_fill = float(self.get_parameter('min_fill_ratio').value)
        max_fill = float(self.get_parameter('max_fill_ratio').value)
        area_w = float(self.get_parameter('area_score_weight').value)
        shape_w = float(self.get_parameter('shape_score_weight').value)
        aspect_w = float(self.get_parameter('aspect_score_weight').value)
        fill_w = float(self.get_parameter('fill_score_weight').value)
        center_w = float(self.get_parameter('center_score_weight').value)
        min_detection_score = float(self.get_parameter('min_detection_score').value)
        candidates = []

        for contour in contours:
            metrics = self._contour_metrics(contour)
            if metrics is None:
                continue
            if metrics['area'] < self._min_area:
                continue
            if use_shape_filter and hard_shape_filter:
                if metrics['circularity'] < circularity_min:
                    continue
                if not (aspect_min <= metrics['aspect_ratio'] <= aspect_max):
                    continue
                if not (min_fill <= metrics['fill_ratio'] <= max_fill):
                    continue
            candidates.append((contour, metrics))

        if not candidates:
            return 0.0, 0.0, False, None, None

        scored = self._score_candidates(
            candidates, min_circularity_soft, min_fill, max_fill,
            area_w, shape_w, aspect_w, fill_w, center_w
        )
        _, best, metrics = scored[0]
        if metrics['total_score'] < min_detection_score:
            metrics['accepted'] = False
            metrics['reject_reason'] = 'LOW SCORE'
            return 0.0, 0.0, False, best, metrics
        cx_px, _ = metrics['centroid']
        ex    = (cx_px - self._cx_image) / self._cx_image  # normalize to [-1, 1]
        metrics['ex'] = ex
        metrics['accepted'] = True
        metrics['reject_reason'] = ''
        return float(ex), float(metrics['area']), True, best, metrics

    def _score_candidates(
        self,
        candidates,
        min_circularity_soft: float,
        min_fill: float,
        max_fill: float,
        area_w: float,
        shape_w: float,
        aspect_w: float,
        fill_w: float,
        center_w: float,
    ):
        max_area = max(metrics['area'] for _, metrics in candidates)
        max_area = max(max_area, 1.0)
        min_circularity_soft = min(min_circularity_soft, 0.99)
        weight_sum = max(area_w + shape_w + aspect_w + fill_w + center_w, 1e-6)
        scored = []
        for contour, metrics in candidates:
            cx, _ = metrics['centroid']
            ex = (cx - self._cx_image) / max(self._cx_image, 1.0)
            area_score = self._clamp(metrics['area'] / max_area)
            circularity_score = self._clamp(
                (metrics['circularity'] - min_circularity_soft)
                / (1.0 - min_circularity_soft)
            )
            aspect_score = 1.0 - min(abs(metrics['aspect_ratio'] - 1.0), 1.0)
            fill_score = self._fill_score(metrics['fill_ratio'], min_fill, max_fill)
            center_score = 1.0 - min(abs(ex), 1.0)
            total_score = (
                area_w * area_score
                + shape_w * circularity_score
                + aspect_w * aspect_score
                + fill_w * fill_score
                + center_w * center_score
            ) / weight_sum
            metrics.update({
                'ex': ex,
                'area_score': area_score,
                'circularity_score': circularity_score,
                'aspect_score': aspect_score,
                'fill_score': fill_score,
                'center_score': center_score,
                'total_score': total_score,
            })
            scored.append((total_score, contour, metrics))
        return sorted(scored, key=lambda item: item[0], reverse=True)

    @staticmethod
    def _clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
        return max(low, min(high, value))

    def _fill_score(self, fill_ratio: float, min_fill: float, max_fill: float) -> float:
        if min_fill <= fill_ratio <= max_fill:
            return 1.0
        if fill_ratio < min_fill:
            return self._clamp(fill_ratio / max(min_fill, 1e-6))
        return self._clamp(1.0 - ((fill_ratio - max_fill) / max(max_fill, 1e-6)))

    def _contour_metrics(self, contour):
        area = float(cv2.contourArea(contour))
        perimeter = float(cv2.arcLength(contour, True))
        circularity = 0.0
        if perimeter > 0.0:
            circularity = float((4.0 * math.pi * area) / (perimeter * perimeter))

        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w / h) if h > 0 else 0.0
        rect_area = float(w * h)
        extent = area / rect_area if rect_area > 0.0 else 0.0
        (_, _), radius = cv2.minEnclosingCircle(contour)
        circle_area = math.pi * radius * radius
        fill_ratio = area / circle_area if circle_area > 0.0 else 0.0
        M = cv2.moments(contour)
        if M['m00'] == 0.0:
            return None

        return {
            'area': area,
            'perimeter': perimeter,
            'circularity': circularity,
            'rect': (x, y, w, h),
            'aspect_ratio': aspect_ratio,
            'radius': float(radius),
            'fill_ratio': float(fill_ratio),
            'extent': float(extent),
            'centroid': (M['m10'] / M['m00'], M['m01'] / M['m00']),
        }

    def _apply_temporal_filter(self, ex: float, area: float, detected: bool, metrics):
        confirm_frames = int(self.get_parameter('confirm_frames').value)
        lost_frames = int(self.get_parameter('lost_frames').value)
        ex_alpha = float(self.get_parameter('ex_smoothing_alpha').value)
        area_alpha = float(self.get_parameter('area_smoothing_alpha').value)
        ex_deadband = float(self.get_parameter('ex_deadband').value)

        if detected:
            self._confirm_count = min(self._confirm_count + 1, confirm_frames)
            self._lost_count = 0
            self._confirmed = self._confirm_count >= confirm_frames
            if self._confirmed:
                self._smooth_ex = (
                    ex_alpha * ex + (1.0 - ex_alpha) * self._smooth_ex
                    if self._last_metrics is not None else ex
                )
                self._smooth_area = (
                    area_alpha * area + (1.0 - area_alpha) * self._smooth_area
                    if self._last_metrics is not None else area
                )
                if abs(self._smooth_ex) < ex_deadband:
                    self._smooth_ex = 0.0
                self._last_metrics = metrics
                return self._smooth_ex, self._smooth_area, True
            self._last_metrics = metrics
            return 0.0, 0.0, False

        self._confirm_count = 0
        if self._confirmed and self._lost_count < lost_frames and self._last_metrics is not None:
            self._lost_count += 1
            return self._smooth_ex, self._smooth_area, True

        self._lost_count = min(self._lost_count + 1, lost_frames)
        self._confirmed = False
        self._last_metrics = None
        return 0.0, 0.0, False

    def _publish(self, ex: float, area: float, detected: bool) -> None:
        msg              = VisionState()
        msg.ex           = ex
        msg.area         = area
        msg.object_detected = detected
        self._pub.publish(msg)

    def _show_preview(
        self, frame: 'np.ndarray', ex: float, detected: bool, contour, metrics
    ) -> None:
        if not self._show_debug or self._debug_failed:
            return

        preview = frame.copy()
        if detected and contour is not None and metrics.get('accepted', False):
            # Bounding box
            x, y, w, h = metrics['rect']
            cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cx = int(metrics['centroid'][0])
            cy = int(metrics['centroid'][1])
            cv2.drawMarker(preview, (cx, cy), (0, 0, 255),
                           cv2.MARKER_CROSS, markerSize=24, thickness=2)
            cv2.putText(preview, f'ex={ex:+.2f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(
                preview,
                f"area={metrics['area']:.0f} circ={metrics['circularity']:.2f} "
                f"ar={metrics['aspect_ratio']:.2f} fill={metrics['fill_ratio']:.2f} "
                f"score={metrics.get('total_score', 0.0):.2f}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )
        elif contour is not None and metrics is not None:
            x, y, w, h = metrics['rect']
            cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(
                preview,
                f"REJECTED score={metrics.get('total_score', 0.0):.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
        else:
            cv2.putText(preview, 'SEARCHING...', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)
        try:
            cv2.imshow('Robot View', preview)
            cv2.waitKey(1)
        except cv2.error as exc:
            self._debug_failed = True
            self.get_logger().warn(
                f'OpenCV debug window failed; continuing without preview: {exc}'
            )

    def destroy_node(self) -> None:
        if self._cap.isOpened():
            self._cap.release()
        if self._show_debug and not self._debug_failed:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
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
