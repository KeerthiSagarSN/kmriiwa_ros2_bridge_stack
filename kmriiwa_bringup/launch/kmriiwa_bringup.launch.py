#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, Command, PathJoinSubstitution)
from launch_ros.actions import Node, PushRosNamespace
import xacro

def generate_launch_description():
    # Package directories
    kmriiwa_description_dir = get_package_share_directory('kmriiwa_description')

    # Launch Arguments
    declare_use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to use a namespace for the robot'
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='kmriiwa',
        description='Name of the robot'
    )
    declare_robot_extras = DeclareLaunchArgument(
        'robot_extras',
        default_value=os.path.join(kmriiwa_description_dir, 'urdf', 'robot', 'empty.xacro'),
        description='Additional robot XACRO file'
    )
    declare_publish_frequency = DeclareLaunchArgument(
        'publish_frequency',
        default_value='10.00',
        description='Frequency of robot state publishing'
    )

    # XACRO file path
    xacro_file = os.path.join(kmriiwa_description_dir, 'urdf', 'robot', 'kmriiwa.urdf.xacro')

    # Load and process XACRO file
    robot_description_content = Command(
        [
            'xacro ',
            xacro_file,
            ' robot_extras:=',
            LaunchConfiguration('robot_extras')
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[robot_description, {
            'publish_frequency': LaunchConfiguration('publish_frequency')
        }],
        remappings=[
            ('joint_states', ['/', LaunchConfiguration('robot_name'), '/arm/joint_states'])
        ]
    )

    # Description publisher node
    robot_description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_description_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Group action for namespace handling
    group_action = GroupAction([
        PushRosNamespace(
            condition=IfCondition(LaunchConfiguration('use_namespace')),
            namespace=LaunchConfiguration('robot_name')
        ),
        robot_state_publisher,
        robot_description_publisher
    ])

    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_namespace)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_robot_extras)
    ld.add_action(declare_publish_frequency)
    
    # Add group action to launch description
    ld.add_action(group_action)
    
    return ld