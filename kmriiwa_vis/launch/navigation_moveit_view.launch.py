from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_kmriiwa_vis = get_package_share_directory('kmriiwa_vis')
    
    # Define paths to RViz configuration files
    map_rviz_config = os.path.join(pkg_kmriiwa_vis, 'rviz', 'map_navigation_moveit.rviz')
    mapless_rviz_config = os.path.join(pkg_kmriiwa_vis, 'rviz', 'mapless_navigation_moveit.rviz')

    # Declare the launch argument
    no_static_map_arg = DeclareLaunchArgument(
        'no_static_map',
        default_value='false',
        description='If true, uses mapless navigation configuration'
    )

    # Create RViz node with map (used when no_static_map is false)
    rviz_with_map = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', map_rviz_config],
        condition=UnlessCondition(LaunchConfiguration('no_static_map')),
        output='screen'
    )

    # Create RViz node without map (used when no_static_map is true)
    rviz_without_map = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', mapless_rviz_config],
        condition=IfCondition(LaunchConfiguration('no_static_map')),
        output='screen'
    )

    # Return launch description with all nodes and arguments
    return LaunchDescription([
        no_static_map_arg,
        rviz_with_map,
        rviz_without_map
    ])
