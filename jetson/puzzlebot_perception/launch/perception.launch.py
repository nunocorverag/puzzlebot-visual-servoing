from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Network — both machines must use the same domain + RMW ───────────
        SetEnvironmentVariable('ROS_DOMAIN_ID',       '0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION',  'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY',  '0'),
        # Source env_jetson.sh before calling ros2 launch to set ROS_DOMAIN_ID.

        Node(
            package='puzzlebot_perception',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[{
                'camera_width':      640,
                'camera_height':     480,
                'camera_fps':        30,
                'use_gstreamer':     True,
                'min_contour_area':  500.0,
                'hsv_lower': [0, 80, 61],
                'hsv_upper': [25, 255, 255],
                'circularity_min': 0.38,
                'min_circularity_soft': 0.35,
                'aspect_ratio_min': 0.65,
                'aspect_ratio_max': 1.55,
                'min_fill_ratio': 0.35,
                'max_fill_ratio': 1.15,
                'use_shape_filter': True,
                'hard_shape_filter': True,
                'area_score_weight': 0.10,
                'shape_score_weight': 0.65,
                'aspect_score_weight': 0.20,
                'fill_score_weight': 0.20,
                'center_score_weight': 0.05,
                'min_detection_score': 0.55,
                'confirm_frames': 3,
                'lost_frames': 4,
                'ex_smoothing_alpha': 0.35,
                'area_smoothing_alpha': 0.35,
                'ex_deadband': 0.05,
                'show_debug_view': True,
                'enable_blue_obstacle_detection': True,
                'blue_h_min': 90,
                'blue_h_max': 135,
                'blue_s_min': 40,
                'blue_s_max': 255,
                'blue_v_min': 20,
                'blue_v_max': 255,
                'blue_min_area': 800.0,
                'blue_close_area': 2500.0,
                # Legacy red parameters remain supported by vision_node.
                'hsv_lower1': [0,   100, 80],
                'hsv_upper1': [10,  255, 255],
                'hsv_lower2': [170, 100, 80],
                'hsv_upper2': [180, 255, 255],
            }],
        ),
    ])
