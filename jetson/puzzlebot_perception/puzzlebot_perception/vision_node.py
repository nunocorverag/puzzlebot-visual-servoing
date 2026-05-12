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
import json
import time
import rclpy
from rclpy.node import Node
import cv2
import math
import numpy as np
from puzzlebot_msgs.msg import VisionState
from std_msgs.msg import String


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
        self.declare_parameter('target_min_circularity', 0.45)
        self.declare_parameter('target_min_circularity_soft', 0.35)
        self.declare_parameter('target_allow_ellipse', True)
        self.declare_parameter('target_ellipse_min_aspect_ratio', 0.45)
        self.declare_parameter('target_ellipse_max_aspect_ratio', 1.00)
        self.declare_parameter('target_min_fill_ratio', 0.45)
        self.declare_parameter('target_max_fill_ratio', 1.20)
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
        self.declare_parameter('enable_blue_obstacle_detection', False)
        self.declare_parameter('blue_h_min', 102)
        self.declare_parameter('blue_h_max', 110)
        self.declare_parameter('blue_s_min', 130)
        self.declare_parameter('blue_s_max', 255)
        self.declare_parameter('blue_v_min', 70)
        self.declare_parameter('blue_v_max', 170)
        self.declare_parameter('blue_min_area', 800.0)
        self.declare_parameter('blue_close_area', 2500.0)
        self.declare_parameter('enable_cream_obstacle_detection', False)
        self.declare_parameter('cream_h_min', 15)
        self.declare_parameter('cream_h_max', 40)
        self.declare_parameter('cream_s_min', 10)
        self.declare_parameter('cream_s_max', 90)
        self.declare_parameter('cream_v_min', 120)
        self.declare_parameter('cream_v_max', 255)
        self.declare_parameter('cream_min_area', 1500.0)
        self.declare_parameter('cream_close_area', 5000.0)
        self.declare_parameter('cream_min_height_px', 120)
        self.declare_parameter('cream_min_aspect_ratio', 1.6)
        self.declare_parameter('cream_max_aspect_ratio', 6.0)
        self.declare_parameter('enable_red_box_obstacle_detection', True)
        self.declare_parameter('red_box_h1_min', 0)
        self.declare_parameter('red_box_h1_max', 15)
        self.declare_parameter('red_box_h2_min', 170)
        self.declare_parameter('red_box_h2_max', 179)
        self.declare_parameter('red_box_s_min', 100)
        self.declare_parameter('red_box_s_max', 255)
        self.declare_parameter('red_box_v_min', 60)
        self.declare_parameter('red_box_v_max', 255)
        self.declare_parameter('red_box_min_area', 1200.0)
        self.declare_parameter('red_box_close_area', 4500.0)
        self.declare_parameter('red_box_min_width_px', 70)
        self.declare_parameter('red_box_min_height_px', 25)
        self.declare_parameter('red_box_min_aspect_ratio', 1.25)
        self.declare_parameter('red_box_max_aspect_ratio', 6.0)
        self.declare_parameter('red_box_max_circularity', 0.60)
        self.declare_parameter('red_box_min_fill_ratio', 0.35)
        self.declare_parameter('red_box_bottom_roi_y_min_ratio', 0.45)
        self.declare_parameter('red_box_exclude_target_iou_threshold', 0.10)
        self.declare_parameter('red_box_exclude_circular_targets', True)
        self.declare_parameter('show_fsm_state_overlay', True)
        self.declare_parameter('fsm_state_topic', '/fsm_state')
        self.declare_parameter('fsm_state_stale_sec', 1.0)
        self.declare_parameter('red_box_min_red_dominance', 1.25)
        self.declare_parameter('red_box_min_mean_saturation', 90)
        self.declare_parameter('red_box_allow_vertical_edge_partial', False)
        self.declare_parameter('visual_obstacle_memory_sec', 0.75)
        self.declare_parameter('visual_obstacle_iou_match_threshold', 0.10)
        self.declare_parameter('visual_obstacle_ex_match_threshold', 0.35)
        self.declare_parameter('visual_obstacle_area_growth_max_ratio', 6.0)
        self.declare_parameter('visual_obstacle_area_shrink_max_ratio', 0.15)
        self.declare_parameter('visual_obstacle_allow_partial_frame', True)
        self.declare_parameter('visual_obstacle_partial_margin_px', 20)
        self.declare_parameter('visual_obstacle_close_latch_sec', 0.75)
        # Legacy red parameters are kept so old launch overrides do not fail.
        self.declare_parameter('hsv_lower1', [0,   100, 80])   # low-hue red
        self.declare_parameter('hsv_upper1', [10,  255, 255])
        self.declare_parameter('hsv_lower2', [170, 100, 80])   # high-hue red
        self.declare_parameter('hsv_upper2', [180, 255, 255])

        W   = self.get_parameter('camera_width').value
        H   = self.get_parameter('camera_height').value
        fps = self.get_parameter('camera_fps').value

        self._cx_image = W / 2.0   # image horizontal center (pixels)
        self._frame_width = int(W)
        self._frame_height = int(H)
        self._min_area = float(self.get_parameter('min_contour_area').value)
        self._show_debug = bool(self.get_parameter('show_debug_view').value)
        self._debug_failed = False
        self._confirm_count = 0
        self._lost_count = 0
        self._confirmed = False
        self._smooth_ex = 0.0
        self._smooth_area = 0.0
        self._last_metrics = None
        self._camera_available = False
        self._frame_fail_count = 0
        self._last_camera_error_log = 0.0
        self._last_visual_obstacle_seen_time = 0.0
        self._last_visual_obstacle_bbox = None
        self._last_visual_obstacle_ex = 0.0
        self._last_visual_obstacle_area = 0.0
        self._last_visual_obstacle_aspect_ratio = 0.0
        self._last_visual_obstacle_track_id = 0
        self._visual_obstacle_tracking_active = False
        self._last_visual_obstacle_close_time = 0.0
        self._visual_obstacle_track_counter = 0
        self._latest_fsm_state = "UNKNOWN"
        self._last_fsm_state_time = 0.0

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
            self.get_logger().error(
                'Camera failed to open — check GStreamer pipeline or nvargus-daemon. '
                'Vision node will continue publishing object_detected=false.'
            )
            self._camera_available = False
        else:
            self._camera_available = True

        # ── Morphological kernel reused every frame ───────────────────────────
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        # ── Publisher ────────────────────────────────────────────────────────────
        self._pub = self.create_publisher(VisionState, '/vision_state', 10)
        self._pub_obstacle = self.create_publisher(
            String, '/vision_obstacle_debug', 10
        )

        # ── Subscriber for FSM state overlay ────────────────────────────────────
        fsm_topic = str(self.get_parameter('fsm_state_topic').value)
        self._sub_fsm = self.create_subscription(
            String, fsm_topic, self._fsm_state_callback, 10
        )

        self.create_timer(1.0 / fps, self._process_frame)
        self.get_logger().info(
            f'Vision node started — {W}x{H} @ {fps} fps  '
            f'(gstreamer={self.get_parameter("use_gstreamer").value})'
        )

    # ── Pipeline steps ────────────────────────────────────────────────────────

    def _process_frame(self) -> None:
        if not self._camera_available:
            self._publish(0.0, 0.0, False)
            return

        ret, frame = self._cap.read()
        if not ret:
            self._frame_fail_count += 1
            now = time.time()
            if (now - self._last_camera_error_log) >= 5.0:
                self.get_logger().warn(
                    f'Frame read failed (count={self._frame_fail_count}). '
                    f'Check camera connection or restart nvargus-daemon.',
                    throttle_duration_sec=5.0
                )
                self._last_camera_error_log = now
            self._publish(0.0, 0.0, False)
            return

        self._frame_fail_count = 0
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._segment(hsv)
        mask = self._clean(mask)
        ex, area, detected, contour, metrics = self._measure(mask)
        obstacle_debug = self._detect_visual_obstacles(frame, hsv, metrics)
        self._publish_visual_obstacle_debug(obstacle_debug)
        ex, area, detected = self._apply_temporal_filter(ex, area, detected, metrics)
        self._publish(ex, area, detected)
        self._show_preview(frame, ex, detected, contour, metrics, obstacle_debug)

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
        target_min_circularity = float(self.get_parameter('target_min_circularity').value)
        target_min_circularity_soft = float(
            self.get_parameter('target_min_circularity_soft').value
        )
        target_allow_ellipse = bool(self.get_parameter('target_allow_ellipse').value)
        ellipse_min = float(self.get_parameter('target_ellipse_min_aspect_ratio').value)
        ellipse_max = float(self.get_parameter('target_ellipse_max_aspect_ratio').value)
        aspect_min = float(self.get_parameter('aspect_ratio_min').value)
        aspect_max = float(self.get_parameter('aspect_ratio_max').value)
        min_fill = float(self.get_parameter('target_min_fill_ratio').value)
        max_fill = float(self.get_parameter('target_max_fill_ratio').value)
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
                if not (min_fill <= metrics['target_fill_ratio'] <= max_fill):
                    metrics['reject_reason'] = 'TARGET_FILL'
                    continue
                legacy_aspect_ok = aspect_min <= metrics['aspect_ratio'] <= aspect_max
                circle_ok = (
                    metrics['circularity'] >= target_min_circularity
                    and legacy_aspect_ok
                )
                ellipse_ok = (
                    target_allow_ellipse
                    and ellipse_min <= metrics['ellipse_ratio'] <= ellipse_max
                    and metrics['circularity'] >= target_min_circularity_soft
                    and not self._looks_like_red_box_target_candidate(metrics)
                )
                if not (circle_ok or ellipse_ok):
                    metrics['reject_reason'] = 'TARGET_SHAPE'
                    continue
            candidates.append((contour, metrics))

        if not candidates:
            return 0.0, 0.0, False, None, None

        scored = self._score_candidates(
            candidates, target_min_circularity_soft, min_fill, max_fill,
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
            fill_score = self._fill_score(metrics['target_fill_ratio'], min_fill, max_fill)
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

    def _looks_like_red_box_target_candidate(self, metrics: dict) -> bool:
        # A real tilted circle has a smooth contour. A red floor box usually
        # approximates to a filled quadrilateral and should stay out of /vision_state.
        return (
            int(metrics.get('approx_vertices', 0)) <= 5
            and float(metrics.get('target_fill_ratio', 0.0)) >= 0.80
            and float(metrics.get('bbox_aspect_ratio', 1.0)) < 0.75
        )

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
        bbox_aspect_ratio = (
            float(min(w, h) / max(w, h)) if max(w, h) > 0 else 0.0
        )
        ellipse_ratio = bbox_aspect_ratio
        ellipse_axes = None
        approx = cv2.approxPolyDP(contour, 0.03 * perimeter, True) if perimeter > 0.0 else contour
        if len(contour) >= 5:
            try:
                (_, _), axes, _ = cv2.fitEllipse(contour)
                axis_a = float(axes[0])
                axis_b = float(axes[1])
                max_axis = max(axis_a, axis_b)
                min_axis = min(axis_a, axis_b)
                if max_axis > 0.0:
                    ellipse_ratio = min_axis / max_axis
                    ellipse_axes = (axis_a, axis_b)
            except cv2.error:
                ellipse_axes = None
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
            'bbox_aspect_ratio': bbox_aspect_ratio,
            'ellipse_ratio': float(ellipse_ratio),
            'ellipse_axes': ellipse_axes,
            'approx_vertices': int(len(approx)),
            'radius': float(radius),
            'fill_ratio': float(fill_ratio),
            'target_fill_ratio': float(extent),
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

    def _fsm_state_callback(self, msg: String) -> None:
        """Callback to receive FSM state from control node."""
        self._latest_fsm_state = msg.data
        self._last_fsm_state_time = time.time()

    def _detect_visual_obstacles(
        self, frame: 'np.ndarray', hsv: 'np.ndarray', target_metrics=None
    ) -> dict:
        if not bool(self.get_parameter('enable_red_box_obstacle_detection').value):
            return self._empty_visual_obstacle_debug('disabled')

        mask = self._red_box_mask(hsv)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        candidates = []
        relaxed_candidates = []
        rejected = []
        for contour in contours:
            candidate = self._visual_obstacle_candidate(contour, frame, hsv, mask)
            if candidate is None:
                continue
            if self._is_target_overlap(candidate, target_metrics):
                candidate['reject_reason'] = 'target_overlap'
                rejected.append(candidate)
                continue
            if self._passes_red_box_obstacle_filters(candidate):
                candidates.append(candidate)
            else:
                candidate['reject_reason'] = self._red_box_reject_reason(candidate)
                rejected.append(candidate)
            if self._passes_relaxed_visual_obstacle_filters(candidate):
                relaxed_candidates.append(candidate)

        debug_stats = self._visual_obstacle_rejection_debug(
            candidates, relaxed_candidates, rejected
        )
        return self._resolve_visual_obstacle_candidates(
            candidates, relaxed_candidates, debug_stats
        )

    def _red_box_mask(self, hsv: 'np.ndarray') -> 'np.ndarray':
        lower = np.array([
            int(self.get_parameter('red_box_h1_min').value),
            int(self.get_parameter('red_box_s_min').value),
            int(self.get_parameter('red_box_v_min').value),
        ], dtype=np.uint8)
        upper = np.array([
            int(self.get_parameter('red_box_h1_max').value),
            int(self.get_parameter('red_box_s_max').value),
            int(self.get_parameter('red_box_v_max').value),
        ], dtype=np.uint8)
        lower2 = np.array([
            int(self.get_parameter('red_box_h2_min').value),
            int(self.get_parameter('red_box_s_min').value),
            int(self.get_parameter('red_box_v_min').value),
        ], dtype=np.uint8)
        upper2 = np.array([
            int(self.get_parameter('red_box_h2_max').value),
            int(self.get_parameter('red_box_s_max').value),
            int(self.get_parameter('red_box_v_max').value),
        ], dtype=np.uint8)
        mask1 = cv2.inRange(hsv, lower, upper)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        return self._clean(cv2.bitwise_or(mask1, mask2))

    def _resolve_visual_obstacle_candidates(
        self, candidates, relaxed_candidates, debug_stats: dict
    ) -> dict:
        now = time.time()
        best = self._select_best_visual_obstacle(candidates)
        if best is not None:
            match_reason, iou, area_ratio = self._match_previous_visual_obstacle(best, now)
            if match_reason == 'new_detection':
                self._visual_obstacle_track_counter += 1
                best['track_id'] = self._visual_obstacle_track_counter
            else:
                best['track_id'] = self._last_visual_obstacle_track_id
            best['match_reason'] = match_reason
            best['iou_with_last'] = iou
            best['area_ratio'] = area_ratio
            return self._update_visual_obstacle_memory(
                best, now, match_reason, debug_stats=debug_stats
            )

        matched = self._select_memory_matched_visual_obstacle(relaxed_candidates, now)
        if matched is not None:
            candidate, match_reason, iou, area_ratio = matched
            candidate['track_id'] = self._last_visual_obstacle_track_id
            candidate['match_reason'] = match_reason
            candidate['iou_with_last'] = iou
            candidate['area_ratio'] = area_ratio
            return self._update_visual_obstacle_memory(
                candidate, now, match_reason, debug_stats=debug_stats
            )

        if self._visual_obstacle_memory_recent(now):
            memory_candidate = self._memory_visual_obstacle_candidate(now)
            return self._update_visual_obstacle_memory(
                memory_candidate, now, 'partial_frame_match',
                update_seen_time=False, debug_stats=debug_stats
            )

        if self._visual_obstacle_tracking_active:
            self._clear_visual_obstacle_memory()
            return self._empty_visual_obstacle_debug('memory_expired', debug_stats)
        return self._empty_visual_obstacle_debug('no_detection', debug_stats)

    def _visual_obstacle_candidate(self, contour, frame, hsv, mask):
        area = float(cv2.contourArea(contour))
        min_area = float(self.get_parameter('red_box_min_area').value)
        if area < max(1.0, min_area * 0.25):
            return None
        x, y, w, h = cv2.boundingRect(contour)
        M = cv2.moments(contour)
        if M['m00'] == 0.0:
            return None
        cx = float(M['m10'] / M['m00'])
        cy = float(M['m01'] / M['m00'])
        ex = (cx - self._cx_image) / max(self._cx_image, 1.0)
        aspect_ratio = float(w / max(h, 1))
        perimeter = float(cv2.arcLength(contour, True))
        circularity = 0.0
        if perimeter > 0.0:
            circularity = float((4.0 * math.pi * area) / (perimeter * perimeter))
        rect_area = float(max(w * h, 1))
        fill_ratio = area / rect_area
        bottom_y_ratio = float((y + h) / max(self._frame_height, 1))
        center_y_ratio = float(cy / max(self._frame_height, 1))
        bbox = [int(x), int(y), int(w), int(h)]
        color_stats = self._red_box_color_stats(frame, hsv, mask, bbox)
        return {
            'area': area,
            'bbox': bbox,
            'centroid': [cx, cy],
            'ex': float(ex),
            'aspect_ratio': aspect_ratio,
            'circularity': circularity,
            'fill_ratio': fill_ratio,
            'bottom_y_ratio': bottom_y_ratio,
            'center_y_ratio': center_y_ratio,
            'partial': self._is_partial_frame_bbox(
                bbox, self._frame_width, self._frame_height
            ),
            **color_stats,
        }

    def _passes_red_box_obstacle_filters(self, candidate: dict) -> bool:
        roi_min = float(self.get_parameter('red_box_bottom_roi_y_min_ratio').value)
        return (
            candidate['area'] >= float(self.get_parameter('red_box_min_area').value)
            and candidate['bbox'][2] >= int(self.get_parameter('red_box_min_width_px').value)
            and candidate['bbox'][3] >= int(self.get_parameter('red_box_min_height_px').value)
            and float(self.get_parameter('red_box_min_aspect_ratio').value)
            <= candidate['aspect_ratio']
            <= float(self.get_parameter('red_box_max_aspect_ratio').value)
            and candidate['circularity']
            <= float(self.get_parameter('red_box_max_circularity').value)
            and candidate['fill_ratio']
            >= float(self.get_parameter('red_box_min_fill_ratio').value)
            and self._passes_red_box_color_filters(candidate)
            and (
                candidate['center_y_ratio'] >= roi_min
                or candidate['bottom_y_ratio'] >= roi_min
            )
        )

    def _passes_relaxed_visual_obstacle_filters(self, candidate: dict) -> bool:
        min_area = float(self.get_parameter('red_box_min_area').value)
        min_width = int(self.get_parameter('red_box_min_width_px').value)
        min_height = int(self.get_parameter('red_box_min_height_px').value)
        roi_min = float(self.get_parameter('red_box_bottom_roi_y_min_ratio').value)
        if candidate['area'] < max(1.0, min_area * 0.40):
            return False
        if candidate['bbox'][2] < max(1, int(min_width * 0.50)):
            return False
        if candidate['bbox'][3] < max(1, int(min_height * 0.50)):
            return False
        if (
            candidate['center_y_ratio'] < (roi_min * 0.85)
            and candidate['bottom_y_ratio'] < roi_min
        ):
            return False
        if (
            bool(self.get_parameter('visual_obstacle_allow_partial_frame').value)
            and candidate['partial']
        ):
            return self._passes_red_box_partial_filters(candidate)
        min_ar = float(self.get_parameter('red_box_min_aspect_ratio').value) * 0.70
        max_ar = float(self.get_parameter('red_box_max_aspect_ratio').value) * 1.35
        return (
            min_ar <= candidate['aspect_ratio'] <= max_ar
            and candidate['circularity']
            <= float(self.get_parameter('red_box_max_circularity').value) * 1.25
            and candidate['fill_ratio']
            >= float(self.get_parameter('red_box_min_fill_ratio').value) * 0.70
            and self._passes_red_box_color_filters(candidate)
        )

    def _passes_red_box_partial_filters(self, candidate: dict) -> bool:
        if self._is_vertical_side_partial(candidate):
            return False
        return (
            candidate['aspect_ratio']
            >= float(self.get_parameter('red_box_min_aspect_ratio').value) * 0.80
            and candidate['circularity']
            <= float(self.get_parameter('red_box_max_circularity').value) * 1.25
            and candidate['fill_ratio']
            >= float(self.get_parameter('red_box_min_fill_ratio').value) * 0.70
            and self._passes_red_box_color_filters(candidate)
        )

    def _passes_red_box_color_filters(self, candidate: dict) -> bool:
        return (
            candidate.get('red_dominance', 0.0)
            >= float(self.get_parameter('red_box_min_red_dominance').value)
            and candidate.get('mean_saturation', 0.0)
            >= float(self.get_parameter('red_box_min_mean_saturation').value)
        )

    def _is_vertical_side_partial(self, candidate: dict) -> bool:
        if bool(self.get_parameter('red_box_allow_vertical_edge_partial').value):
            return False
        x, _, w, h = candidate['bbox']
        margin = int(self.get_parameter('visual_obstacle_partial_margin_px').value)
        touches_side = x <= margin or (x + w) >= (self._frame_width - margin)
        return bool(touches_side and candidate['aspect_ratio'] < 1.0 and h > w)

    def _select_best_visual_obstacle(self, candidates):
        if not candidates:
            return None
        for candidate in candidates:
            area_score = candidate['area']
            ar_score = min(candidate['aspect_ratio'], 3.0) / 3.0
            rect_score = 1.0 - min(candidate['circularity'], 1.0)
            center_score = 1.0 - min(abs(candidate['ex']), 1.0)
            low_score = min(max(candidate['center_y_ratio'], candidate['bottom_y_ratio']), 1.0)
            candidate['score'] = area_score * (
                1.0 + 0.20 * ar_score + 0.25 * rect_score
                + 0.10 * center_score + 0.10 * low_score
            )
        return sorted(candidates, key=lambda item: item['score'], reverse=True)[0]

    def _is_target_overlap(self, candidate: dict, target_metrics) -> bool:
        if target_metrics is None:
            return False
        if bool(self.get_parameter('red_box_exclude_circular_targets').value):
            if candidate['circularity'] > float(self.get_parameter('red_box_max_circularity').value):
                return True
        target_bbox = target_metrics.get('rect')
        if target_bbox is not None:
            target_bbox = [
                int(target_bbox[0]), int(target_bbox[1]),
                int(target_bbox[2]), int(target_bbox[3]),
            ]
            iou = self._bbox_iou(candidate['bbox'], target_bbox)
            if iou > float(self.get_parameter('red_box_exclude_target_iou_threshold').value):
                return True
        target_centroid = target_metrics.get('centroid')
        if target_centroid is not None:
            dx = abs(candidate['centroid'][0] - float(target_centroid[0]))
            dy = abs(candidate['centroid'][1] - float(target_centroid[1]))
            area_ratio = candidate['area'] / max(float(target_metrics.get('area', 1.0)), 1.0)
            if dx < 45.0 and dy < 45.0 and 0.35 <= area_ratio <= 2.8:
                return True
        return False

    def _red_box_color_stats(self, frame, hsv, mask, bbox) -> dict:
        x, y, w, h = bbox
        roi_frame = frame[y:y + h, x:x + w]
        roi_hsv = hsv[y:y + h, x:x + w]
        roi_mask = mask[y:y + h, x:x + w]
        selected = roi_mask > 0
        if roi_frame.size == 0:
            return {
                'mean_b': 0.0, 'mean_g': 0.0, 'mean_r': 0.0,
                'red_dominance': 0.0, 'mean_saturation': 0.0,
            }
        if np.count_nonzero(selected) < 10:
            pixels = roi_frame.reshape(-1, 3)
            sat_values = roi_hsv[:, :, 1].reshape(-1)
        else:
            pixels = roi_frame[selected]
            sat_values = roi_hsv[:, :, 1][selected]
        mean_b, mean_g, mean_r = np.mean(pixels, axis=0)
        red_dominance = float(mean_r / max(mean_g, mean_b, 1.0))
        mean_saturation = float(np.mean(sat_values))
        return {
            'mean_b': float(mean_b),
            'mean_g': float(mean_g),
            'mean_r': float(mean_r),
            'red_dominance': red_dominance,
            'mean_saturation': mean_saturation,
        }

    def _red_box_reject_reason(self, candidate: dict) -> str:
        roi_min = float(self.get_parameter('red_box_bottom_roi_y_min_ratio').value)
        checks = [
            (candidate['area'] < float(self.get_parameter('red_box_min_area').value), 'area'),
            (candidate['bbox'][2] < int(self.get_parameter('red_box_min_width_px').value), 'width'),
            (candidate['bbox'][3] < int(self.get_parameter('red_box_min_height_px').value), 'height'),
            (
                candidate['aspect_ratio'] < float(self.get_parameter('red_box_min_aspect_ratio').value),
                'aspect_low',
            ),
            (
                candidate['aspect_ratio'] > float(self.get_parameter('red_box_max_aspect_ratio').value),
                'aspect_high',
            ),
            (
                candidate['circularity'] > float(self.get_parameter('red_box_max_circularity').value),
                'circularity',
            ),
            (
                candidate['fill_ratio'] < float(self.get_parameter('red_box_min_fill_ratio').value),
                'fill',
            ),
            (not self._passes_red_box_color_filters(candidate), 'red_color'),
            (
                candidate['center_y_ratio'] < roi_min and candidate['bottom_y_ratio'] < roi_min,
                'roi',
            ),
        ]
        for failed, reason in checks:
            if failed:
                return reason
        return 'unknown'

    def _visual_obstacle_rejection_debug(self, candidates, relaxed_candidates, rejected) -> dict:
        all_seen = list(candidates) + list(relaxed_candidates) + list(rejected)
        largest = None
        if all_seen:
            largest = sorted(all_seen, key=lambda item: item.get('area', 0.0), reverse=True)[0]
        return {
            'visual_obstacle_candidate_count': len(candidates) + len(relaxed_candidates),
            'visual_obstacle_rejected_count': len(rejected),
            'visual_obstacle_largest_raw_area': 0.0 if largest is None else float(largest.get('area', 0.0)),
            'visual_obstacle_largest_raw_bbox': [] if largest is None else largest.get('bbox', []),
            'visual_obstacle_largest_raw_aspect_ratio': 0.0 if largest is None else float(largest.get('aspect_ratio', 0.0)),
            'visual_obstacle_largest_raw_circularity': 0.0 if largest is None else float(largest.get('circularity', 0.0)),
            'visual_obstacle_largest_raw_fill_ratio': 0.0 if largest is None else float(largest.get('fill_ratio', 0.0)),
            'visual_obstacle_largest_raw_red_dominance': 0.0 if largest is None else float(largest.get('red_dominance', 0.0)),
            'visual_obstacle_reject_reason_top': '' if largest is None else largest.get('reject_reason', ''),
        }

    def _select_memory_matched_visual_obstacle(self, candidates, now: float):
        if not candidates or not self._visual_obstacle_memory_recent(now):
            return None
        matches = []
        for candidate in candidates:
            if not self._passes_relaxed_visual_obstacle_filters(candidate):
                continue
            match_reason, iou, area_ratio = self._match_previous_visual_obstacle(
                candidate, now, allow_new=False
            )
            if match_reason == 'new_detection':
                continue
            matches.append((candidate, match_reason, iou, area_ratio))
        if not matches:
            return None
        return sorted(matches, key=lambda item: item[0]['area'], reverse=True)[0]

    def _match_previous_visual_obstacle(
        self, candidate: dict, now: float, allow_new: bool = True
    ):
        if not self._visual_obstacle_memory_recent(now):
            return ('new_detection', None, None) if allow_new else ('new_detection', None, None)

        iou = self._bbox_iou(candidate['bbox'], self._last_visual_obstacle_bbox)
        area_ratio = candidate['area'] / max(self._last_visual_obstacle_area, 1.0)
        min_ratio = float(self.get_parameter('visual_obstacle_area_shrink_max_ratio').value)
        max_ratio = float(self.get_parameter('visual_obstacle_area_growth_max_ratio').value)
        area_ok = min_ratio <= area_ratio <= max_ratio
        if not area_ok:
            return ('new_detection', iou, area_ratio)

        iou_threshold = float(self.get_parameter('visual_obstacle_iou_match_threshold').value)
        ex_threshold = float(self.get_parameter('visual_obstacle_ex_match_threshold').value)
        if iou is not None and iou >= iou_threshold:
            return 'iou_match', iou, area_ratio
        if abs(candidate['ex'] - self._last_visual_obstacle_ex) <= ex_threshold:
            if candidate['partial'] and bool(
                self.get_parameter('visual_obstacle_allow_partial_frame').value
            ):
                if not self._passes_red_box_partial_filters(candidate):
                    return ('new_detection', iou, area_ratio)
                return 'partial_frame_match', iou, area_ratio
            return 'ex_match', iou, area_ratio
        if candidate['partial'] and bool(
            self.get_parameter('visual_obstacle_allow_partial_frame').value
        ):
            if not self._passes_red_box_partial_filters(candidate):
                return ('new_detection', iou, area_ratio)
            return 'partial_frame_match', iou, area_ratio
        return ('new_detection', iou, area_ratio) if allow_new else ('new_detection', iou, area_ratio)

    def _update_visual_obstacle_memory(
        self,
        candidate: dict,
        now: float,
        match_reason: str,
        update_seen_time: bool = True,
        debug_stats=None,
    ) -> dict:
        if update_seen_time:
            self._last_visual_obstacle_seen_time = now
            self._last_visual_obstacle_bbox = candidate['bbox']
            self._last_visual_obstacle_ex = float(candidate['ex'])
            self._last_visual_obstacle_area = float(candidate['area'])
            self._last_visual_obstacle_aspect_ratio = float(candidate['aspect_ratio'])
            self._last_visual_obstacle_track_id = int(candidate.get('track_id', 0))
        self._visual_obstacle_tracking_active = True

        raw_close = candidate['area'] >= float(self.get_parameter('red_box_close_area').value)
        if raw_close:
            self._last_visual_obstacle_close_time = now
        close_latched = (
            self._last_visual_obstacle_close_time > 0.0
            and (now - self._last_visual_obstacle_close_time)
            <= float(self.get_parameter('visual_obstacle_close_latch_sec').value)
        )
        close = bool(raw_close or close_latched)
        candidate['close'] = close
        candidate['close_latched'] = bool(close_latched and not raw_close)
        candidate['tracking_active'] = True
        candidate['match_reason'] = match_reason
        return self._visual_obstacle_debug(candidate, [candidate], debug_stats)

    def _memory_visual_obstacle_candidate(self, now: float) -> dict:
        age = now - self._last_visual_obstacle_seen_time
        return {
            'area': self._last_visual_obstacle_area,
            'bbox': self._last_visual_obstacle_bbox or [],
            'centroid': [],
            'ex': self._last_visual_obstacle_ex,
            'aspect_ratio': self._last_visual_obstacle_aspect_ratio,
            'circularity': 0.0,
            'fill_ratio': 1.0,
            'mean_b': 0.0,
            'mean_g': 0.0,
            'mean_r': 0.0,
            'red_dominance': 0.0,
            'mean_saturation': 0.0,
            'partial': self._is_partial_frame_bbox(
                self._last_visual_obstacle_bbox, self._frame_width, self._frame_height
            ),
            'track_id': self._last_visual_obstacle_track_id,
            'iou_with_last': None,
            'area_ratio': 1.0,
            'memory_age_sec': age,
        }

    def _visual_obstacle_memory_recent(self, now: float) -> bool:
        return (
            self._visual_obstacle_tracking_active
            and self._last_visual_obstacle_seen_time > 0.0
            and (now - self._last_visual_obstacle_seen_time)
            <= float(self.get_parameter('visual_obstacle_memory_sec').value)
        )

    def _clear_visual_obstacle_memory(self) -> None:
        self._last_visual_obstacle_seen_time = 0.0
        self._last_visual_obstacle_bbox = None
        self._last_visual_obstacle_ex = 0.0
        self._last_visual_obstacle_area = 0.0
        self._last_visual_obstacle_aspect_ratio = 0.0
        self._visual_obstacle_tracking_active = False

    def _visual_obstacle_debug(self, best: dict, candidates, debug_stats=None) -> dict:
        debug_stats = debug_stats or self._visual_obstacle_rejection_debug([], [], [])
        memory_age = best.get('memory_age_sec')
        if memory_age is None and self._last_visual_obstacle_seen_time > 0.0:
            memory_age = time.time() - self._last_visual_obstacle_seen_time
        obstacles = [
            {
                'area': float(item['area']),
                'bbox': item['bbox'],
                'centroid': item.get('centroid', []),
                'ex': float(item['ex']),
                'aspect_ratio': float(item.get('aspect_ratio', 0.0)),
                'circularity': float(item.get('circularity', 0.0)),
                'fill_ratio': float(item.get('fill_ratio', 0.0)),
                'mean_b': float(item.get('mean_b', 0.0)),
                'mean_g': float(item.get('mean_g', 0.0)),
                'mean_r': float(item.get('mean_r', 0.0)),
                'red_dominance': float(item.get('red_dominance', 0.0)),
                'mean_saturation': float(item.get('mean_saturation', 0.0)),
                'partial': bool(item.get('partial', False)),
                'track_id': int(best.get('track_id', self._last_visual_obstacle_track_id)),
            }
            for item in candidates[:5]
        ]
        debug = {
            'visual_obstacle_detected': True,
            'visual_obstacle_close': bool(best.get('close', False)),
            'visual_obstacle_area': float(best['area']),
            'visual_obstacle_ex': float(best['ex']),
            'visual_obstacle_bbox': best['bbox'],
            'visual_obstacle_count': len(candidates),
            'visual_obstacles': obstacles,
            'visual_obstacle_color': 'red',
            'visual_obstacle_shape': 'rectangular_box',
            'visual_obstacle_aspect_ratio': float(best.get('aspect_ratio', 0.0)),
            'visual_obstacle_circularity': float(best.get('circularity', 0.0)),
            'visual_obstacle_fill_ratio': float(best.get('fill_ratio', 0.0)),
            'visual_obstacle_mean_r': float(best.get('mean_r', 0.0)),
            'visual_obstacle_mean_g': float(best.get('mean_g', 0.0)),
            'visual_obstacle_mean_b': float(best.get('mean_b', 0.0)),
            'visual_obstacle_red_dominance': float(best.get('red_dominance', 0.0)),
            'visual_obstacle_mean_saturation': float(best.get('mean_saturation', 0.0)),
            'visual_obstacle_tracking_active': bool(best.get('tracking_active', False)),
            'visual_obstacle_track_id': int(best.get('track_id', self._last_visual_obstacle_track_id)),
            'visual_obstacle_memory_age_sec': (
                None if memory_age is None else round(float(memory_age), 4)
            ),
            'visual_obstacle_partial': bool(best.get('partial', False)),
            'visual_obstacle_iou_with_last': best.get('iou_with_last'),
            'visual_obstacle_area_ratio': best.get('area_ratio'),
            'visual_obstacle_match_reason': best.get('match_reason', 'new_detection'),
            'visual_obstacle_close_latched': bool(best.get('close_latched', False)),
            'visual_obstacle_detector': 'red_box',
            **debug_stats,
        }
        debug.update(self._legacy_blue_obstacle_alias(debug, obstacles))
        return debug

    def _legacy_blue_obstacle_alias(self, debug: dict, obstacles) -> dict:
        return {
            'blue_obstacle_detected': bool(debug['visual_obstacle_detected']),
            'blue_obstacle_close': bool(debug['visual_obstacle_close']),
            'blue_obstacle_area': float(debug['visual_obstacle_area']),
            'blue_obstacle_ex': float(debug['visual_obstacle_ex']),
            'blue_obstacle_bbox': debug['visual_obstacle_bbox'],
            'blue_obstacle_count': int(debug['visual_obstacle_count']),
            'blue_obstacles': obstacles,
        }

    def _empty_visual_obstacle_debug(
        self, match_reason: str = 'no_detection', debug_stats=None
    ) -> dict:
        debug_stats = debug_stats or self._visual_obstacle_rejection_debug([], [], [])
        debug = {
            'visual_obstacle_detected': False,
            'visual_obstacle_close': False,
            'visual_obstacle_area': 0.0,
            'visual_obstacle_ex': 0.0,
            'visual_obstacle_bbox': [],
            'visual_obstacle_count': 0,
            'visual_obstacles': [],
            'visual_obstacle_color': 'red',
            'visual_obstacle_shape': 'rectangular_box',
            'visual_obstacle_aspect_ratio': 0.0,
            'visual_obstacle_circularity': 0.0,
            'visual_obstacle_fill_ratio': 0.0,
            'visual_obstacle_mean_r': 0.0,
            'visual_obstacle_mean_g': 0.0,
            'visual_obstacle_mean_b': 0.0,
            'visual_obstacle_red_dominance': 0.0,
            'visual_obstacle_mean_saturation': 0.0,
            'visual_obstacle_tracking_active': False,
            'visual_obstacle_track_id': int(self._last_visual_obstacle_track_id),
            'visual_obstacle_memory_age_sec': None,
            'visual_obstacle_partial': False,
            'visual_obstacle_iou_with_last': None,
            'visual_obstacle_area_ratio': None,
            'visual_obstacle_match_reason': match_reason,
            'visual_obstacle_close_latched': False,
            'visual_obstacle_detector': 'red_box',
            **debug_stats,
        }
        debug.update(self._legacy_blue_obstacle_alias(debug, []))
        return debug

    def _bbox_iou(self, bbox_a, bbox_b) -> float:
        if not bbox_a or not bbox_b:
            return 0.0
        ax, ay, aw, ah = bbox_a
        bx, by, bw, bh = bbox_b
        ax2, ay2 = ax + aw, ay + ah
        bx2, by2 = bx + bw, by + bh
        ix1, iy1 = max(ax, bx), max(ay, by)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
        inter = float(iw * ih)
        union = float(aw * ah + bw * bh - inter)
        return 0.0 if union <= 0.0 else inter / union

    def _is_partial_frame_bbox(self, bbox, frame_width: int, frame_height: int) -> bool:
        if not bbox:
            return False
        margin = int(self.get_parameter('visual_obstacle_partial_margin_px').value)
        x, y, w, h = bbox
        return (
            x <= margin
            or y <= margin
            or (x + w) >= (frame_width - margin)
            or (y + h) >= (frame_height - margin)
        )

    def _publish_visual_obstacle_debug(self, obstacle_debug: dict) -> None:
        self._pub_obstacle.publish(String(data=json.dumps(obstacle_debug)))

    def _show_preview(
        self, frame: 'np.ndarray', ex: float, detected: bool, contour, metrics,
        obstacle_debug: dict
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
                f"ell={metrics.get('ellipse_ratio', 0.0):.2f} "
                f"ar={metrics['aspect_ratio']:.2f} fill={metrics['target_fill_ratio']:.2f} "
                f"vtx={metrics.get('approx_vertices', 0)} score={metrics.get('total_score', 0.0):.2f}",
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
                f"REJECTED {metrics.get('reject_reason', '')} "
                f"circ={metrics.get('circularity', 0.0):.2f} "
                f"ell={metrics.get('ellipse_ratio', 0.0):.2f} "
                f"fill={metrics.get('target_fill_ratio', 0.0):.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
        else:
            cv2.putText(preview, 'SEARCHING...', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)

        for obstacle in obstacle_debug.get('visual_obstacles', []):
            x, y, w, h = obstacle['bbox']
            close = bool(obstacle_debug.get('visual_obstacle_close', False))
            tracking = bool(obstacle_debug.get('visual_obstacle_tracking_active', False))
            match_reason = str(obstacle_debug.get('visual_obstacle_match_reason', ''))[:18]
            track_id = int(obstacle_debug.get('visual_obstacle_track_id', 0))
            thickness = 3 if close else 2
            if close and tracking:
                label = 'RED_BOX_OBS_CLOSE_TRACK'
            elif close:
                label = 'RED_BOX_OBS_CLOSE'
            elif tracking:
                label = 'RED_BOX_OBS_TRACK'
            else:
                label = 'RED_BOX_OBS'
            color = (255, 255, 0) if close else (0, 255, 255)
            cv2.rectangle(preview, (x, y), (x + w, y + h), color, thickness)
            if obstacle.get('centroid'):
                cx = int(obstacle['centroid'][0])
                cy = int(obstacle['centroid'][1])
                cv2.drawMarker(preview, (cx, cy), color,
                               cv2.MARKER_CROSS, markerSize=18, thickness=2)
            cv2.putText(
                preview,
                f"{label} area={obstacle['area']:.0f} "
                f"ar={obstacle.get('aspect_ratio', 0.0):.1f} "
                f"circ={obstacle.get('circularity', 0.0):.2f}",
                (x, max(y - 8, 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )
            cv2.putText(
                preview,
                f"id={track_id} {match_reason}",
                (x, min(y + h + 18, preview.shape[0] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                color,
                2,
            )
        
        # Draw FSM state overlay
        if bool(self.get_parameter('show_fsm_state_overlay').value):
            now = time.time()
            stale_sec = float(self.get_parameter('fsm_state_stale_sec').value)
            age = now - self._last_fsm_state_time if self._last_fsm_state_time > 0.0 else 999.0
            
            if age > stale_sec:
                fsm_text = "FSM: STALE/UNKNOWN"
                fsm_color = (128, 128, 128)  # Gray
            else:
                fsm_text = f"FSM: {self._latest_fsm_state}"
                # Color based on state
                state = self._latest_fsm_state
                if state == "EMERGENCY_STOP":
                    fsm_color = (0, 0, 255)  # Red
                elif state == "WAIT_FOR_CAMERA":
                    fsm_color = (0, 255, 255)  # Yellow
                elif state in ["TRACKING", "ACQUIRE_TARGET", "GOAL_REACHED"]:
                    fsm_color = (0, 255, 0)  # Green
                elif state in ["AVOID", "OBSTACLE_CONFIRM", "AVOID_TURN", "AVOID_FORWARD", 
                               "POST_AVOID_TURN_BACK", "POST_AVOID_REACQUIRE"]:
                    fsm_color = (255, 165, 0)  # Orange
                elif state == "SEARCH":
                    fsm_color = (255, 255, 255)  # White
                else:
                    fsm_color = (200, 200, 200)  # Light gray
            
            # Draw background rectangle
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0
            thickness = 2
            (text_w, text_h), baseline = cv2.getTextSize(fsm_text, font, font_scale, thickness)
            padding = 10
            rect_x1 = 10
            rect_y1 = preview.shape[0] - text_h - baseline - 2 * padding
            rect_x2 = rect_x1 + text_w + 2 * padding
            rect_y2 = preview.shape[0] - padding
            
            # Semi-transparent black background
            overlay = preview.copy()
            cv2.rectangle(overlay, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, preview, 0.4, 0, preview)
            
            # Draw text
            text_x = rect_x1 + padding
            text_y = rect_y2 - padding - baseline
            cv2.putText(preview, fsm_text, (text_x, text_y), font, font_scale, fsm_color, thickness)
        
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
