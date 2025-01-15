from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('kmriiwa_navigation')
    
    # Declare arguments
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='scan_multi',
        description='Topic name for the merged laser scan'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='kmriiwa',
        description='Robot name prefix for frames'
    )
    
    # Include laser scan merger launch file
    laserscan_merge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'laserscan_merge.launch.py')
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name')
        }.items()
    )
    
    # Configure SLAM node
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': [LaunchConfiguration('robot_name'), '_odom'],
            'base_frame': [LaunchConfiguration('robot_name'), '_base_link'],
            'map_frame': 'map',
            'mode': 'mapping',
            'map_update_interval': 5.0,
            'resolution': 0.05,
            'max_laser_range': 20.0,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'use_scan_matching': True,
            'minimum_travel_distance': 0.5,
            'minimum_travel_heading': 0.5,
            'scan_buffer_size': 10,
            'loop_search_maximum_distance': 3.0,
            'do_loop_closing': True,
        }],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic'))
        ]
    )
    
    return LaunchDescription([
        scan_topic_arg,
        robot_name_arg,
        laserscan_merge,
        slam_node
    ])
