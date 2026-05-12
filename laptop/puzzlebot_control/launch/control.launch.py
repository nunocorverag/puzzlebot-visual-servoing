from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('puzzlebot_control'), 'config', 'mpc_params.yaml'
    ])

    return LaunchDescription([
        # ── Network — must match the Jetson ─────────────────────────────────
        SetEnvironmentVariable('ROS_DOMAIN_ID',      '0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        # Source env_laptop.sh before calling ros2 launch to set ROS_DOMAIN_ID.

        Node(
            package='puzzlebot_control',
            executable='mpc_node',
            name='mpc_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='puzzlebot_control',
            executable='visualizer_node',
            name='visualizer_node',
            output='screen',
            parameters=[params],
        ),
    ])
