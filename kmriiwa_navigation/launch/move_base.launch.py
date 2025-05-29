#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the package directory
    nav_pkg_dir = get_package_share_directory('kmriiwa_navigation')
    
    # Declare all launch arguments
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
    
    declare_no_static_map = DeclareLaunchArgument(
        'no_static_map',
        default_value='false',
        description='Use laser-based costmap instead of static map'
    )
    
    declare_base_global_planner = DeclareLaunchArgument(
        'base_global_planner',
        default_value='nav2_navfn_planner/NavfnPlanner',
        description='Global planner plugin'
    )
    
    declare_base_local_planner = DeclareLaunchArgument(
        'base_local_planner',
        default_value='nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController',
        description='Local planner plugin'
    )

    # Parameter substitutions
    param_substitutions = {
        'use_sim_time': 'false',
        'base_global_planner': LaunchConfiguration('base_global_planner'),
        'base_local_planner': LaunchConfiguration('base_local_planner'),
        'robot_name': LaunchConfiguration('robot_name')
    }

    # Configure params with substitutions
    local_planner_params = RewrittenYaml(
        source_file=os.path.join(nav_pkg_dir, 'config', 'local_planner.yaml'),
        root_key=LaunchConfiguration('robot_name'),
        param_rewrites=param_substitutions,
        convert_types=True
    )

    common_costmap_params = RewrittenYaml(
        source_file=os.path.join(nav_pkg_dir, 'config', 'common_costmap.yaml'),
        root_key=LaunchConfiguration('robot_name'),
        param_rewrites=param_substitutions,
        convert_types=True
    )

    global_static_params = RewrittenYaml(
        source_file=os.path.join(nav_pkg_dir, 'config', 'global_costmap_static_map.yaml'),
        root_key=LaunchConfiguration('robot_name'),
        param_rewrites=param_substitutions,
        convert_types=True
    )

    global_laser_params = RewrittenYaml(
        source_file=os.path.join(nav_pkg_dir, 'config', 'global_costmap_laser.yaml'),
        root_key=LaunchConfiguration('robot_name'),
        param_rewrites=param_substitutions,
        convert_types=True
    )

    local_costmap_params = RewrittenYaml(
        source_file=os.path.join(nav_pkg_dir, 'config', 'local_costmap.yaml'),
        root_key=LaunchConfiguration('robot_name'),
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Include laserscan merge launch
    laserscan_merge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg_dir, 'launch', 'laserscan_merge.launch.py')
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name')
        }.items()
    )

    # Controller Server Node
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'controller_frequency': 20.0,
                'min_x_velocity_threshold': 0.001,
                'min_y_velocity_threshold': 0.5,
                'min_theta_velocity_threshold': 0.001,
                'failure_tolerance': 0.3,
                'odom_frame_id': 'kmriiwa_odom',
                'base_frame_id': 'kmriiwa_base_footprint',
                'global_frame_id': 'map',
                'transform_tolerance': 0.1,
                'state_transition_timeout': 5.0,
                'progress_checker_plugin': 'nav2_controller::SimpleProgressChecker',
                'goal_checker_plugins': ['nav2_controller::SimpleGoalChecker'],
                'controller_plugins': ['nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController'],
                
                'progress_checker': {
                    'plugin': 'nav2_controller::SimpleProgressChecker',
                    'required_movement_radius': 0.5,
                    'movement_time_allowance': 10.0
                },
                'goal_checker': {
                    'plugin': 'nav2_controller::SimpleGoalChecker',
                    'xy_goal_tolerance': 0.25,
                    'yaw_goal_tolerance': 0.25,
                    'stateful': True
                },
                'RegulatedPurePursuitController': {
                    'plugin': 'nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController',
                    'desired_linear_vel': 0.5,
                    'lookahead_dist': 0.6,
                    'min_lookahead_dist': 0.3,
                    'max_lookahead_dist': 0.9,
                    'lookahead_time': 1.5,
                    'rotate_to_heading_angular_vel': 1.8,
                    'transform_tolerance': 0.1,
                    'use_velocity_scaled_lookahead_dist': False,
                    'min_approach_linear_velocity': 0.05,
                    'max_allowed_time_to_collision': 1.0,
                    'use_regulated_linear_velocity_scaling': True,
                    'use_cost_regulated_linear_velocity_scaling': False,
                    'regulated_linear_scaling_min_radius': 0.9,
                    'regulated_linear_scaling_min_speed': 0.25,
                    'use_rotate_to_heading': True,
                    'rotate_to_heading_min_angle': 0.785,
                    'max_angular_accel': 3.2,
                    'max_robot_pose_search_dist': 10.0,
                    'use_interpolation': True
                }
            },
            local_planner_params,
            common_costmap_params
        ],
        remappings=[
            ('/cmd_vel', PathJoinSubstitution([LaunchConfiguration('robot_name'), '/base/command/cmd_vel'])),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odom', 'kmriiwa_odom')
        ]
    )

    # Planner Server Node
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'expected_planner_frequency': 20.0,
                'planner_plugins': ['GridBased'],
                'GridBased': {
                    'plugin': 'nav2_navfn_planner/NavfnPlanner',
                    'tolerance': 0.5,
                    'use_astar': False,
                    'allow_unknown': True
                }
            },
            {'base_global_planner': LaunchConfiguration('base_global_planner')},
            common_costmap_params
        ]
    )

    # BT Navigator Node
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'global_frame': 'map',
                'robot_base_frame': 'base_link',
                'transform_tolerance': 0.1,
                'default_bt_xml_filename': 'navigate_w_replanning_and_recovery.xml',
                'plugin_lib_names': [
                    'nav2_compute_path_to_pose_action_bt_node',
                    'nav2_follow_path_action_bt_node',
                    'nav2_back_up_action_bt_node',
                    'nav2_spin_action_bt_node',
                    'nav2_wait_action_bt_node',
                    'nav2_clear_costmap_service_bt_node',
                    'nav2_is_stuck_condition_bt_node',
                    'nav2_goal_reached_condition_bt_node',
                    'nav2_initial_pose_received_condition_bt_node',
                    'nav2_reinitialize_global_localization_service_bt_node',
                    'nav2_rate_controller_bt_node',
                    'nav2_distance_controller_bt_node',
                    'nav2_speed_controller_bt_node',
                    'nav2_recovery_node_bt_node'
                ]
            }
        ]
    )

    # Costmap Nodes
    global_costmap_static = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[common_costmap_params, global_static_params],
        condition=UnlessCondition(LaunchConfiguration('no_static_map'))
    )

    global_costmap_laser = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[
            common_costmap_params,
            global_laser_params,
            {
                'global_costmap': {
                    'width': 100.0,
                    'height': 100.0
                }
            }
        ],
        condition=IfCondition(LaunchConfiguration('no_static_map'))
    )

    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[common_costmap_params, local_costmap_params]
    )

    # Lifecycle Manager Node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'global_costmap',
                    'local_costmap'
                ],
                'bond_nodes': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'global_costmap',
                    'local_costmap'
                ]
            }
        ]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_namespace)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_no_static_map)
    ld.add_action(declare_base_global_planner)
    ld.add_action(declare_base_local_planner)

    # Add nodes wrapped in namespace if enabled
    navigation_nodes = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('robot_name')),
            laserscan_merge_launch,
            controller_server_node,
            planner_server_node,
            bt_navigator_node,
            global_costmap_static,
            global_costmap_laser,
            local_costmap_node,
            lifecycle_manager_node
        ],
        condition=IfCondition(LaunchConfiguration('use_namespace'))
    )

    ld.add_action(navigation_nodes)

    return ld