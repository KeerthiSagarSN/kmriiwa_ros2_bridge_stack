
# Todo check compile properly
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the RViz configuration file
    pkg_kmriiwa_vis = get_package_share_directory('kmriiwa_vis')
    rviz_config_file = os.path.join(pkg_kmriiwa_vis, 'rviz', 'moveit.rviz')

    # Create RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Return launch description
    return LaunchDescription([
        rviz_node
    ])