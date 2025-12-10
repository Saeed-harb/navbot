# localization_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('navbot_bringup')
    config_file = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization_node',
            output='screen',
            parameters=[config_file]
        ),
    ])
