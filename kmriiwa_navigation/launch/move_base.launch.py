#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                          IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get package directory
    kmriiwa_navigation_dir = get_package_share_directory('kmriiwa_navigation')
    
    # Launch arguments
    declare_use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Enable namespace for robot'
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='kmriiwa',
        description='Name of the robot'
    )

    declare_no_static_map = DeclareLaunchArgument(
        'no_static_map',
        default_value='false',
        description='Use laser-based global costmap instead of static map'
    )

    declare_base_global_planner = DeclareLaunchArgument(
        'base_global_planner',
        default_value='nav2_navfn_planner/NavfnPlanner',
        description='Global path planner to use'
    )

    declare_base_local_planner = DeclareLaunchArgument(
        'base_local_planner',
        default_value='nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController',
        description='Local path planner to use'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Create a dictionary for parameter substitutions
    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'base_global_planner': LaunchConfiguration('base_global_planner'),
        'base_local_planner': LaunchConfiguration('base_local_planner')
    }

    # Configure params with substitutions
    configured_params = RewrittenYaml(
        source_file=os.path.join(kmriiwa_navigation_dir, 'config', 'nav2_params.yaml'),
        root_key=LaunchConfiguration('robot_name'),
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Laserscan merge launch
    laserscan_merge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(kmriiwa_navigation_dir, 'launch', 'laserscan_merge.launch.py')
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'global_costmap',
                'local_costmap'
            ]}
        ]
    )

    # Navigation stack nodes
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('cmd_vel', PathJoinSubstitution([LaunchConfiguration('robot_name'), '/base/command/cmd_vel']))            
        ]
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params]
    )

    # Costmap nodes
    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[
            configured_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'global_frame': 'map'},
            {'robot_base_frame': 'base_link'},
            {'width': 100.0 if LaunchConfiguration('no_static_map') else None},
            {'height': 100.0 if LaunchConfiguration('no_static_map') else None}
        ]
    )

    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[
            configured_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'global_frame': 'odom'},
            {'robot_base_frame': 'base_link'}
        ]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_namespace)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_no_static_map)
    ld.add_action(declare_base_global_planner)
    ld.add_action(declare_base_local_planner)
    ld.add_action(declare_use_sim_time)

    # Add nodes wrapped in namespace if enabled
    navigation_nodes = GroupAction([
        PushRosNamespace(condition=IfCondition(LaunchConfiguration('use_namespace')),
                        namespace=LaunchConfiguration('robot_name')),
        laserscan_merge_launch,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        global_costmap_node,
        local_costmap_node,
        lifecycle_manager_node  # Add this line
    ])

    ld.add_action(navigation_nodes)

    return ld