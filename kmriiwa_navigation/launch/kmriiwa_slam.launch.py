# kmriiwa_slam.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_config = os.path.join(
        get_package_share_directory('kmriiwa_navigation'),
        'config',
        'kmriiwa_slam_config.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config]
        )
    ])