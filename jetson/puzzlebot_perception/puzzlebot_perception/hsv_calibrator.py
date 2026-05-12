#!/usr/bin/env python3
"""
Interactive HSV + circular shape calibrator for the Puzzlebot target.

Keys:
  s  save current parameters to config/vision_hsv.yaml
  q  quit
"""
from pathlib import Path
import math

import cv2
import numpy as np


WINDOW_CONTROLS = 'Controls'
WINDOW_VIEW = 'Calibration View'


DEFAULTS = {
    'H_min': 0,
    'H_max': 25,
    'S_min': 80,
    'S_max': 255,
    'V_min': 61,
    'V_max': 255,
    'min_area': 500,
    'circularity_min': 38,
    'min_circularity_soft': 35,
    'aspect_ratio_min': 65,
    'aspect_ratio_max': 155,
    'min_fill_ratio': 35,
    'max_fill_ratio': 115,
    'hard_shape_filter': 1,
    'area_score_weight': 10,
    'shape_score_weight': 65,
    'aspect_score_weight': 20,
    'fill_score_weight': 20,
    'center_score_weight': 5,
    'min_detection_score': 55,
    'confirm_frames': 3,
    'lost_frames': 4,
    'ex_smoothing_alpha': 35,
    'area_smoothing_alpha': 35,
    'ex_deadband': 5,
}

DEMO_PRESET = DEFAULTS.copy()
PRESET_ORDER = [
    'H_min',
    'H_max',
    'S_min',
    'S_max',
    'V_min',
    'V_max',
    'min_area',
    'circularity_min',
    'min_circularity_soft',
    'aspect_ratio_min',
    'aspect_ratio_max',
    'min_fill_ratio',
    'max_fill_ratio',
    'hard_shape_filter',
    'area_score_weight',
    'shape_score_weight',
    'aspect_score_weight',
    'fill_score_weight',
    'center_score_weight',
    'min_detection_score',
    'confirm_frames',
    'lost_frames',
    'ex_smoothing_alpha',
    'area_smoothing_alpha',
    'ex_deadband',
]

TRACKBAR_LIMITS = {
    'H_min': (0, 180),
    'H_max': (0, 180),
    'S_min': (0, 255),
    'S_max': (0, 255),
    'V_min': (0, 255),
    'V_max': (0, 255),
    'min_area': (0, 20000),
    'circularity_min': (0, 100),
    'min_circularity_soft': (0, 100),
    'aspect_ratio_min': (0, 300),
    'aspect_ratio_max': (0, 300),
    'min_fill_ratio': (0, 200),
    'max_fill_ratio': (0, 200),
    'hard_shape_filter': (0, 1),
    'area_score_weight': (0, 100),
    'shape_score_weight': (0, 100),
    'aspect_score_weight': (0, 100),
    'fill_score_weight': (0, 100),
    'center_score_weight': (0, 100),
    'min_detection_score': (0, 100),
    'confirm_frames': (1, 20),
    'lost_frames': (0, 20),
    'ex_smoothing_alpha': (0, 100),
    'area_smoothing_alpha': (0, 100),
    'ex_deadband': (0, 100),
}


def build_gstreamer_pipeline(width: int = 640, height: int = 480, fps: int = 30) -> str:
    return (
        'nvarguscamerasrc ! '
        'video/x-raw(memory:NVMM), '
        f'width=(int){width}, height=(int){height}, framerate=(fraction){fps}/1 ! '
        'nvvidconv flip-method=0 ! '
        'video/x-raw, format=(string)BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=(string)BGR ! '
        'appsink drop=1'
    )


def noop(_: int) -> None:
    pass


def create_trackbars() -> None:
    cv2.namedWindow(WINDOW_CONTROLS, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_VIEW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_CONTROLS, 360, 620)
    cv2.resizeWindow(WINDOW_VIEW, 1280, 560)
    cv2.createTrackbar('H_min', WINDOW_CONTROLS, DEFAULTS['H_min'], 180, noop)
    cv2.createTrackbar('H_max', WINDOW_CONTROLS, DEFAULTS['H_max'], 180, noop)
    cv2.createTrackbar('S_min', WINDOW_CONTROLS, DEFAULTS['S_min'], 255, noop)
    cv2.createTrackbar('S_max', WINDOW_CONTROLS, DEFAULTS['S_max'], 255, noop)
    cv2.createTrackbar('V_min', WINDOW_CONTROLS, DEFAULTS['V_min'], 255, noop)
    cv2.createTrackbar('V_max', WINDOW_CONTROLS, DEFAULTS['V_max'], 255, noop)
    cv2.createTrackbar('min_area', WINDOW_CONTROLS, DEFAULTS['min_area'], 20000, noop)
    cv2.createTrackbar(
        'circularity_min', WINDOW_CONTROLS, DEFAULTS['circularity_min'], 100, noop
    )
    cv2.createTrackbar(
        'min_circularity_soft', WINDOW_CONTROLS,
        DEFAULTS['min_circularity_soft'], 100, noop
    )
    cv2.createTrackbar(
        'aspect_ratio_min', WINDOW_CONTROLS, DEFAULTS['aspect_ratio_min'], 300, noop
    )
    cv2.createTrackbar(
        'aspect_ratio_max', WINDOW_CONTROLS, DEFAULTS['aspect_ratio_max'], 300, noop
    )
    cv2.createTrackbar(
        'min_fill_ratio', WINDOW_CONTROLS, DEFAULTS['min_fill_ratio'], 200, noop
    )
    cv2.createTrackbar(
        'max_fill_ratio', WINDOW_CONTROLS, DEFAULTS['max_fill_ratio'], 200, noop
    )
    cv2.createTrackbar(
        'hard_shape_filter', WINDOW_CONTROLS,
        DEFAULTS['hard_shape_filter'], 1, noop
    )
    cv2.createTrackbar(
        'area_score_weight', WINDOW_CONTROLS,
        DEFAULTS['area_score_weight'], 100, noop
    )
    cv2.createTrackbar(
        'shape_score_weight', WINDOW_CONTROLS,
        DEFAULTS['shape_score_weight'], 100, noop
    )
    cv2.createTrackbar(
        'aspect_score_weight', WINDOW_CONTROLS,
        DEFAULTS['aspect_score_weight'], 100, noop
    )
    cv2.createTrackbar(
        'fill_score_weight', WINDOW_CONTROLS,
        DEFAULTS['fill_score_weight'], 100, noop
    )
    cv2.createTrackbar(
        'center_score_weight', WINDOW_CONTROLS,
        DEFAULTS['center_score_weight'], 100, noop
    )
    cv2.createTrackbar(
        'min_detection_score', WINDOW_CONTROLS,
        DEFAULTS['min_detection_score'], 100, noop
    )
    cv2.createTrackbar(
        'confirm_frames', WINDOW_CONTROLS, DEFAULTS['confirm_frames'], 20, noop
    )
    cv2.createTrackbar(
        'lost_frames', WINDOW_CONTROLS, DEFAULTS['lost_frames'], 20, noop
    )
    cv2.createTrackbar(
        'ex_smoothing_alpha', WINDOW_CONTROLS,
        DEFAULTS['ex_smoothing_alpha'], 100, noop
    )
    cv2.createTrackbar(
        'area_smoothing_alpha', WINDOW_CONTROLS,
        DEFAULTS['area_smoothing_alpha'], 100, noop
    )
    cv2.createTrackbar(
        'ex_deadband', WINDOW_CONTROLS, DEFAULTS['ex_deadband'], 100, noop
    )


def read_params() -> dict:
    params = {name: cv2.getTrackbarPos(name, WINDOW_CONTROLS) for name in DEFAULTS}
    params['H_min'] = min(params['H_min'], params['H_max'])
    params['S_min'] = min(params['S_min'], params['S_max'])
    params['V_min'] = min(params['V_min'], params['V_max'])
    params['min_fill_ratio'] = min(params['min_fill_ratio'], params['max_fill_ratio'])
    return {
        'hsv_lower': [params['H_min'], params['S_min'], params['V_min']],
        'hsv_upper': [params['H_max'], params['S_max'], params['V_max']],
        'min_contour_area': float(params['min_area']),
        'circularity_min': params['circularity_min'] / 100.0,
        'min_circularity_soft': params['min_circularity_soft'] / 100.0,
        'aspect_ratio_min': params['aspect_ratio_min'] / 100.0,
        'aspect_ratio_max': params['aspect_ratio_max'] / 100.0,
        'min_fill_ratio': params['min_fill_ratio'] / 100.0,
        'max_fill_ratio': params['max_fill_ratio'] / 100.0,
        'use_shape_filter': True,
        'hard_shape_filter': bool(params['hard_shape_filter']),
        'area_score_weight': params['area_score_weight'] / 100.0,
        'shape_score_weight': params['shape_score_weight'] / 100.0,
        'aspect_score_weight': params['aspect_score_weight'] / 100.0,
        'fill_score_weight': params['fill_score_weight'] / 100.0,
        'center_score_weight': params['center_score_weight'] / 100.0,
        'min_detection_score': params['min_detection_score'] / 100.0,
        'confirm_frames': max(1, params['confirm_frames']),
        'lost_frames': params['lost_frames'],
        'ex_smoothing_alpha': params['ex_smoothing_alpha'] / 100.0,
        'area_smoothing_alpha': params['area_smoothing_alpha'] / 100.0,
        'ex_deadband': params['ex_deadband'] / 100.0,
        'show_debug_view': True,
    }


def clamp_int(value: int, low: int, high: int) -> int:
    return max(low, min(high, int(value)))


def set_trackbar_value(name: str, value: int) -> bool:
    if name not in TRACKBAR_LIMITS:
        print_valid_parameters()
        return False
    low, high = TRACKBAR_LIMITS[name]
    value = clamp_int(value, low, high)
    cv2.setTrackbarPos(name, WINDOW_CONTROLS, value)
    print(f'set {name} = {value}')
    return True


def apply_preset(values: dict) -> None:
    for name in PRESET_ORDER:
        set_trackbar_value(name, values[name])


def reset_defaults() -> None:
    print('Resetting to DEFAULTS')
    apply_preset(DEFAULTS)


def load_demo_preset() -> None:
    print('Loading demo preset')
    apply_preset(DEMO_PRESET)


def print_valid_parameters() -> None:
    print('Valid parameters:')
    print('  ' + ' '.join(PRESET_ORDER))


def current_trackbar_values() -> dict:
    return {name: cv2.getTrackbarPos(name, WINDOW_CONTROLS) for name in PRESET_ORDER}


def print_current_values() -> None:
    raw = current_trackbar_values()
    params = read_params()
    compact = ' '.join(str(raw[name]) for name in PRESET_ORDER)
    print('Compact preset line:')
    print(compact)
    print('YAML values:')
    print(f"hsv_lower: {params['hsv_lower']}")
    print(f"hsv_upper: {params['hsv_upper']}")
    print(f"min_contour_area: {params['min_contour_area']:.1f}")
    print('use_shape_filter: true')
    print(f"hard_shape_filter: {str(params['hard_shape_filter']).lower()}")
    print(f"circularity_min: {params['circularity_min']:.2f}")
    print(f"min_circularity_soft: {params['min_circularity_soft']:.2f}")
    print(f"aspect_ratio_min: {params['aspect_ratio_min']:.2f}")
    print(f"aspect_ratio_max: {params['aspect_ratio_max']:.2f}")
    print(f"min_fill_ratio: {params['min_fill_ratio']:.2f}")
    print(f"max_fill_ratio: {params['max_fill_ratio']:.2f}")
    print(f"area_score_weight: {params['area_score_weight']:.2f}")
    print(f"shape_score_weight: {params['shape_score_weight']:.2f}")
    print(f"aspect_score_weight: {params['aspect_score_weight']:.2f}")
    print(f"fill_score_weight: {params['fill_score_weight']:.2f}")
    print(f"center_score_weight: {params['center_score_weight']:.2f}")
    print(f"min_detection_score: {params['min_detection_score']:.2f}")
    print(f"confirm_frames: {params['confirm_frames']}")
    print(f"lost_frames: {params['lost_frames']}")
    print(f"ex_smoothing_alpha: {params['ex_smoothing_alpha']:.2f}")
    print(f"area_smoothing_alpha: {params['area_smoothing_alpha']:.2f}")
    print(f"ex_deadband: {params['ex_deadband']:.2f}")
    print('show_debug_view: true')


def parse_assignment_token(token: str):
    if '=' in token:
        key, value = token.split('=', 1)
    else:
        return None
    key = key.strip()
    value = value.strip()
    if not key or not value:
        return None
    return key, value


def apply_edit_line(line: str) -> bool:
    tokens = line.split()
    if not tokens:
        return False

    if len(tokens) == len(PRESET_ORDER) and all('=' not in token for token in tokens):
        try:
            values = [int(float(token)) for token in tokens]
        except ValueError:
            print('Preset line must contain numeric values only.')
            return False
        for name, value in zip(PRESET_ORDER, values):
            set_trackbar_value(name, value)
        return True

    if len(tokens) == 2 and '=' not in tokens[0] and '=' not in tokens[1]:
        key, value = tokens
        try:
            return set_trackbar_value(key, int(float(value)))
        except ValueError:
            print(f'Invalid numeric value for {key}: {value}')
            return False

    applied = False
    for token in tokens:
        parsed = parse_assignment_token(token)
        if parsed is None:
            print(f'Ignoring invalid token: {token}')
            continue
        key, value = parsed
        try:
            applied = set_trackbar_value(key, int(float(value))) or applied
        except ValueError:
            print(f'Invalid numeric value for {key}: {value}')
    return applied


def edit_mode() -> None:
    print('')
    print('Edit mode. Enter blank line or "done" to return to camera view.')
    print('Examples:')
    print('  S_min=90')
    print('  min_area 500')
    print('  H_min=0 H_max=25 S_min=80 V_min=61')
    print('  0 25 80 255 61 255 500 38 35 65 155 35 115 1 10 65 20 20 5 55 3 4 35 35 5')
    print_valid_parameters()
    while True:
        try:
            line = input('calib> ').strip()
        except EOFError:
            print('EOF in edit mode; returning to camera view.')
            return
        except KeyboardInterrupt:
            print('')
            print('Leaving edit mode.')
            return
        if line == '' or line.lower() == 'done':
            return
        apply_edit_line(line)


def clean_mask(mask: 'np.ndarray') -> 'np.ndarray':
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def contour_metrics(contour) -> dict:
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
    moments = cv2.moments(contour)
    centroid = None
    if moments['m00'] != 0.0:
        centroid = (
            int(moments['m10'] / moments['m00']),
            int(moments['m01'] / moments['m00']),
        )
    return {
        'area': area,
        'perimeter': perimeter,
        'circularity': circularity,
        'rect': (x, y, w, h),
        'aspect_ratio': aspect_ratio,
        'radius': float(radius),
        'fill_ratio': float(fill_ratio),
        'extent': float(extent),
        'centroid': centroid,
    }


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def fill_score(fill_ratio: float, min_fill: float, max_fill: float) -> float:
    if min_fill <= fill_ratio <= max_fill:
        return 1.0
    if fill_ratio < min_fill:
        return clamp(fill_ratio / max(min_fill, 1e-6))
    return clamp(1.0 - ((fill_ratio - max_fill) / max(max_fill, 1e-6)))


def score_candidates(candidates: list, params: dict, image_width: int) -> list:
    if not candidates:
        return []

    max_area = max(metrics['area'] for _, metrics in candidates)
    max_area = max(max_area, 1.0)
    min_circularity_soft = min(params['min_circularity_soft'], 0.99)
    area_w = params['area_score_weight']
    shape_w = params['shape_score_weight']
    aspect_w = params['aspect_score_weight']
    fill_w = params['fill_score_weight']
    center_w = params['center_score_weight']
    weight_sum = max(area_w + shape_w + aspect_w + fill_w + center_w, 1e-6)
    half_width = max(image_width / 2.0, 1.0)

    scored = []
    for contour, metrics in candidates:
        cx, _ = metrics['centroid']
        ex = (cx - half_width) / half_width
        area_score = clamp(metrics['area'] / max_area)
        circularity_score = clamp(
            (metrics['circularity'] - min_circularity_soft)
            / (1.0 - min_circularity_soft)
        )
        aspect_score = 1.0 - min(abs(metrics['aspect_ratio'] - 1.0), 1.0)
        fill_ratio_score = fill_score(
            metrics['fill_ratio'], params['min_fill_ratio'], params['max_fill_ratio']
        )
        center_score = 1.0 - min(abs(ex), 1.0)
        total_score = (
            area_w * area_score
            + shape_w * circularity_score
            + aspect_w * aspect_score
            + fill_w * fill_ratio_score
            + center_w * center_score
        ) / weight_sum
        hard_reject_reason = metrics.get('reject_reason', '')
        accepted = (not hard_reject_reason) and total_score >= params['min_detection_score']
        metrics.update({
            'ex': ex,
            'area_score': area_score,
            'circularity_score': circularity_score,
            'aspect_score': aspect_score,
            'fill_score': fill_ratio_score,
            'center_score': center_score,
            'total_score': total_score,
            'accepted': accepted,
            'reject_reason': '' if accepted else (hard_reject_reason or 'LOW SCORE'),
        })
        scored.append((total_score, contour, metrics))
    return sorted(scored, key=lambda item: item[0], reverse=True)


def valid_candidates(mask: 'np.ndarray', params: dict, image_width: int) -> list:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        metrics = contour_metrics(contour)
        if metrics['centroid'] is None:
            continue
        if metrics['area'] < params['min_contour_area']:
            continue
        if params['use_shape_filter'] and params['hard_shape_filter']:
            if metrics['circularity'] < params['circularity_min']:
                metrics['accepted'] = False
                metrics['reject_reason'] = 'LOW CIRC'
                candidates.append((contour, metrics))
                continue
            if not (
                params['aspect_ratio_min']
                <= metrics['aspect_ratio']
                <= params['aspect_ratio_max']
            ):
                metrics['accepted'] = False
                metrics['reject_reason'] = 'BAD ASPECT'
                candidates.append((contour, metrics))
                continue
            if not (
                params['min_fill_ratio']
                <= metrics['fill_ratio']
                <= params['max_fill_ratio']
            ):
                metrics['accepted'] = False
                metrics['reject_reason'] = 'BAD FILL'
                candidates.append((contour, metrics))
                continue
        candidates.append((contour, metrics))
    return score_candidates(candidates, params, image_width)


def draw_detection(frame: 'np.ndarray', candidates: list) -> 'np.ndarray':
    debug = frame.copy()
    for _, contour, metrics in candidates:
        x, y, w, h = metrics['rect']
        color = (0, 180, 255) if metrics.get('accepted', False) else (0, 0, 180)
        cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
        if metrics['centroid'] is not None:
            cv2.drawMarker(
                debug, metrics['centroid'], color,
                cv2.MARKER_CROSS, markerSize=18, thickness=2
            )

    accepted = [item for item in candidates if item[2].get('accepted', False)]
    if accepted:
        _, best, best_metrics = accepted[0]
        x, y, w, h = best_metrics['rect']
        cv2.drawContours(debug, [best], -1, (0, 255, 0), 3)
        cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 3)
        if best_metrics['centroid'] is not None:
            cv2.drawMarker(
                debug, best_metrics['centroid'], (0, 0, 255),
                cv2.MARKER_CROSS, markerSize=24, thickness=2
            )
    return debug


def put_lines(
    image: 'np.ndarray',
    lines: list,
    x: int = 18,
    y: int = 36,
    line_height: int = 28,
    font_scale: float = 0.62,
) -> None:
    for line in lines:
        if line == '':
            y += line_height // 2
            continue
        color = (0, 220, 255) if line.endswith(':') else (235, 235, 235)
        cv2.putText(
            image,
            line,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            color,
            1,
            cv2.LINE_AA,
        )
        y += line_height


def build_dashboard(
    frame: 'np.ndarray',
    mask: 'np.ndarray',
    candidates: list,
    params: dict,
) -> 'np.ndarray':
    left_width = 320
    image_width = 640
    image_height = 480
    mask_width = 320
    mask_height = 240
    dashboard_height = 520

    debug = draw_detection(frame, candidates)
    debug = cv2.resize(debug, (image_width, image_height))
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    mask_bgr = cv2.resize(mask_bgr, (mask_width, mask_height))

    camera_panel = np.zeros((dashboard_height, image_width, 3), dtype=np.uint8)
    camera_panel[:image_height, :] = debug
    cv2.putText(
        camera_panel,
        'Camera detection - valid candidates in amber, best in green',
        (10, 505),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )

    mask_panel = np.zeros((dashboard_height, mask_width, 3), dtype=np.uint8)
    mask_panel[:mask_height, :] = mask_bgr
    cv2.rectangle(mask_panel, (0, 0), (mask_width - 1, mask_height - 1), (0, 180, 255), 2)
    mask_lines = [
        'HSV mask',
        '',
        'White pixels should be',
        'mostly the target.',
        '',
        'Soft scoring keeps',
        'partial circles alive.',
    ]
    put_lines(mask_panel, mask_lines, x=14, y=282, line_height=26)

    left_panel = np.zeros((dashboard_height, left_width, 3), dtype=np.uint8)
    accepted = [item for item in candidates if item[2].get('accepted', False)]
    best_metrics = accepted[0][2] if accepted else (candidates[0][2] if candidates else None)
    if best_metrics:
        status = 'ACCEPTED' if best_metrics.get('accepted', False) else (
            'REJECTED ' + best_metrics.get('reject_reason', '')
        )
        best_lines = [
            'Best candidate:',
            status,
            f"area: {best_metrics['area']:.0f}",
            f"circularity: {best_metrics['circularity']:.2f}",
            f"aspect ratio: {best_metrics['aspect_ratio']:.2f}",
            f"fill_ratio: {best_metrics['fill_ratio']:.2f}",
            f"extent: {best_metrics['extent']:.2f}",
            f"ex: {best_metrics.get('ex', 0.0):+.3f}",
            f"score: {best_metrics.get('total_score', 0.0):.2f}",
            f"area_score: {best_metrics.get('area_score', 0.0):.2f}",
            f"circ_score: {best_metrics.get('circularity_score', 0.0):.2f}",
            f"aspect_score: {best_metrics.get('aspect_score', 0.0):.2f}",
            f"fill_score: {best_metrics.get('fill_score', 0.0):.2f}",
        ]
    else:
        best_lines = ['Best candidate:', 'none']

    lines = [
        'Puzzlebot HSV Calibration',
        'sliders or e text edit',
        '',
        'HSV:',
        f"lower: {params['hsv_lower']}",
        f"upper: {params['hsv_upper']}",
        '',
        'Shape filters:',
        f"min_area: {params['min_contour_area']:.0f}",
        f"hard_shape: {params['hard_shape_filter']}",
        f"circularity_min: {params['circularity_min']:.2f}",
        f"min_circ_soft: {params['min_circularity_soft']:.2f}",
        f"aspect_ratio_min: {params['aspect_ratio_min']:.2f}",
        f"aspect_ratio_max: {params['aspect_ratio_max']:.2f}",
        f"fill: {params['min_fill_ratio']:.2f}-{params['max_fill_ratio']:.2f}",
        f"min_score: {params['min_detection_score']:.2f}",
        '',
        *best_lines,
        '',
        'Instructions:',
        's = save',
        'q = quit',
        'd = demo preset',
        'r = reset defaults',
        'p = print values',
        'e = edit values',
        '',
    ]
    put_lines(left_panel, lines, y=24, line_height=18, font_scale=0.46)
    return np.hstack([left_panel, camera_panel, mask_panel])


def find_config_path() -> Path:
    cwd = Path.cwd()
    candidates = [
        cwd / 'jetson' / 'puzzlebot_perception' / 'config' / 'vision_hsv.yaml',
        cwd / 'src' / 'puzzlebot_perception' / 'config' / 'vision_hsv.yaml',
        cwd / 'config' / 'vision_hsv.yaml',
    ]
    for candidate in candidates:
        if candidate.parent.exists():
            return candidate
    return candidates[0]


def save_yaml(path: Path, params: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = (
        f"hsv_lower: {params['hsv_lower']}\n"
        f"hsv_upper: {params['hsv_upper']}\n"
        f"min_contour_area: {params['min_contour_area']:.1f}\n"
        "use_shape_filter: true\n"
        f"hard_shape_filter: {str(params['hard_shape_filter']).lower()}\n"
        f"circularity_min: {params['circularity_min']:.2f}\n"
        f"min_circularity_soft: {params['min_circularity_soft']:.2f}\n"
        f"aspect_ratio_min: {params['aspect_ratio_min']:.2f}\n"
        f"aspect_ratio_max: {params['aspect_ratio_max']:.2f}\n"
        f"min_fill_ratio: {params['min_fill_ratio']:.2f}\n"
        f"max_fill_ratio: {params['max_fill_ratio']:.2f}\n"
        f"area_score_weight: {params['area_score_weight']:.2f}\n"
        f"shape_score_weight: {params['shape_score_weight']:.2f}\n"
        f"aspect_score_weight: {params['aspect_score_weight']:.2f}\n"
        f"fill_score_weight: {params['fill_score_weight']:.2f}\n"
        f"center_score_weight: {params['center_score_weight']:.2f}\n"
        f"min_detection_score: {params['min_detection_score']:.2f}\n"
        f"confirm_frames: {params['confirm_frames']}\n"
        f"lost_frames: {params['lost_frames']}\n"
        f"ex_smoothing_alpha: {params['ex_smoothing_alpha']:.2f}\n"
        f"area_smoothing_alpha: {params['area_smoothing_alpha']:.2f}\n"
        f"ex_deadband: {params['ex_deadband']:.2f}\n"
        "show_debug_view: true\n"
    )
    path.write_text(text, encoding='utf-8')
    print(f'Saved calibration to {path}')


def main() -> None:
    cap = cv2.VideoCapture(build_gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError('Camera failed to open with GStreamer pipeline')

    create_trackbars()
    config_path = find_config_path()
    print(f'Press s to save to {config_path}')
    print('Press q to quit')

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print('Frame read failed')
                continue

            params = read_params()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array(params['hsv_lower'], dtype=np.uint8)
            upper = np.array(params['hsv_upper'], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            mask = clean_mask(mask)
            candidates = valid_candidates(mask, params, frame.shape[1])
            dashboard = build_dashboard(frame, mask, candidates, params)

            cv2.imshow(WINDOW_VIEW, dashboard)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                save_yaml(config_path, params)
            elif key == ord('q'):
                break
            elif key == ord('d'):
                load_demo_preset()
            elif key == ord('r'):
                reset_defaults()
            elif key == ord('p'):
                print_current_values()
            elif key == ord('e'):
                edit_mode()
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
