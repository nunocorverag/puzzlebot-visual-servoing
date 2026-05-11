from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Network — both machines must use the same domain + RMW ───────────
        SetEnvironmentVariable('ROS_DOMAIN_ID',       '42'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION',  'rmw_cyclonedds_cpp'),
        # Tip: set CYCLONEDDS_URI before calling ros2 launch, e.g.:
        #   source /path/to/env_jetson.sh && ros2 launch puzzlebot_perception perception.launch.py

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
                # HSV bounds for red — tune with a calibration script if needed
                'hsv_lower1': [0,   100, 80],
                'hsv_upper1': [10,  255, 255],
                'hsv_lower2': [170, 100, 80],
                'hsv_upper2': [180, 255, 255],
            }],
        ),
    ])
