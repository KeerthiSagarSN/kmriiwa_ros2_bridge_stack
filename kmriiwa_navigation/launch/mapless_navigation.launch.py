#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    nav_pkg_dir = get_package_share_directory('kmriiwa_navigation')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')

    # Declare launch arguments
    declare_use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to use namespace'
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='kmriiwa',
        description='Robot name'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz'
    )

    # Include the Nav2 bringup launch
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_pkg_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'use_composition': 'True',
            'params_file': os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    # Include RViz launch if requested
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('kmriiwa_vis'),
            'launch', 'navigation_view.launch.py')
        ]),
        condition=IfCondition(LaunchConfiguration('rviz')),
        launch_arguments={
            'use_namespace': LaunchConfiguration('use_namespace'),
            'namespace': LaunchConfiguration('robot_name')
        }.items()
    )

    # Create the navigation group with namespace
    navigation_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('robot_name')),
            nav2_bringup,
            rviz_launch
        ],
        condition=IfCondition(LaunchConfiguration('use_namespace'))
    )

    return LaunchDescription([
        # Launch arguments
        declare_use_namespace,
        declare_robot_name,
        declare_rviz,
        # Navigation stack
        navigation_group
    ])