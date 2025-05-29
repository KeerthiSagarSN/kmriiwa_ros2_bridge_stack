from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    # Get package share directories
    pkg_kmriiwa_vis = get_package_share_directory('kmriiwa_vis')
    pkg_kmriiwa_bringup = get_package_share_directory('kmriiwa_bringup')

    # Include robot bringup launch file
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kmriiwa_bringup, 'launch', 'kmriiwa_bringup.launch.py')
        ])
    )


    # Declare the map file argument
    # declare_map_yaml_cmd = DeclareLaunchArgument(
    #     'map',
    #     default_value=os.path.join(pkg_kmriiwa_vis, 'maps', 'map.yaml'),
    #     description='Full path to map yaml file to load'
    # )

    # # Map Server Node
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'yaml_filename': LaunchConfiguration('map')}]
    # )

    # # Lifecycle Manager for Map Server
    # map_server_lifecycle_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_map',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': False},
    #         {'autostart': True},
    #         {'node_names': ['map_server']}
    #     ]
    # )
    # Create RViz node
    rviz_config = os.path.join(pkg_kmriiwa_vis, 'rviz', 'robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )


    # Real-time transform publisher for current robot pose
    # robot_state_publisher = Node(
    #     package='tf2_ros',
    #     executable='transform_broadcaster',
    #     name='robot_tf_publisher',
    #     parameters=[{
    #         'use_sim_time': False,
    #         'frame_id': 'kmriiwa_odom',
    #         'child_frame_id': 'kmriiwa_base_link',
    #         'publish_frequency': 50.0
    #     }]
    # )

    # Static transform from map to odom
    # static_transform_pub = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     output='screen',
    #     arguments=[
    #         '--frame-id', 'map',
    #         '--child-frame-id', 'kmriiwa_odom',
    #         '--x', '0', '--y', '0', '--z', '0',
    #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
    #         '--publish-frequency', '50'
    #     ]
    # )

    


    # Add AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'kmriiwa_base_footprint',
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.9,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'global_frame_id': 'map',
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 100.0,
            'laser_min_range': 0.15,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'max_particles': 2000,
            'min_particles': 500,
            'odom_frame_id': '/kmriiwa/base/state/odom',
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,
            'resample_interval': 1,
            'robot_model_type': 'nav2_amcl::OmniMotionModel',
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'tf_broadcast': True,
            'transform_tolerance': 100.0,
            'update_min_a': 0.2,
            'update_min_d': 0.25,
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
            'scan_topic': '/scan_multi'
            #'scan_topic': '/kmriiwa/base/state/LaserB1Scan'
        }],
        remappings=[
            # ('/tf', 'tf'),
            # ('/tf_static', 'tf_static'),
            ('/cmd_vel', '/kmriiwa/base/command/cmd_vel'),
            ('/odom', '/kmriiwa/base/state/odom'),
            ('/clock', '/kmriiwa_clock')
            
        ]
    )

    # Add BT Navigator Node
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'global_frame': 'map',
            'robot_base_frame': 'kmriiwa_base_footprint',
            'odom_frame': 'kmriiwa_odom',
            'bt_loop_duration': 10,
            'default_server_timeout': 20,
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
                'nav2_goal_updated_condition_bt_node',
                'nav2_initial_pose_received_condition_bt_node',
                'nav2_reinitialize_global_localization_service_bt_node',
                'nav2_rate_controller_bt_node',
                'nav2_distance_controller_bt_node',
                'nav2_speed_controller_bt_node',
                'nav2_truncate_path_action_bt_node',
                'nav2_goal_updater_node_bt_node',
                'nav2_recovery_node_bt_node',
                'nav2_pipeline_sequence_bt_node',
                'nav2_round_robin_node_bt_node',
                'nav2_transform_available_condition_bt_node',
                'nav2_time_expired_condition_bt_node',
                'nav2_distance_traveled_condition_bt_node'
            ]
        }],
        remappings=[
            # ('/tf', 'tf'),
            # ('/tf_static', 'tf_static'),
            ('/cmd_vel', '/kmriiwa/base/command/cmd_vel'),
            ('/odom', '/kmriiwa/base/state/odom'),
            ('/clock', '/kmriiwa_clock')
            
        ]
    )

    # Update lifecycle manager to include AMCL
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            #'node_names': ['map_server', 'amcl']
            'node_names': ['amcl']
        }],
        remappings=[
            # ('/tf', 'tf'),
            # ('/tf_static', 'tf_static'),
            ('/cmd_vel', '/kmriiwa/base/command/cmd_vel'),
            ('/odom', '/kmriiwa/base/state/odom'),
            ('/clock', '/kmriiwa_clock')
            
        ]
    )

    # Add teleop keyboard node
    # teleop_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_keyboard',
    #     prefix='xterm -e',
    #     output='screen',
    #     remappings=[
    #         ('/cmd_vel', '/kmriiwa/base/command/cmd_vel'),
    #     ]
    # )

    # Add waypoint navigation node
    nav_through_poses_node = Node(
        package='nav2_simple_commander',
        executable='example_nav_through_poses',  
        name='move_through_waypoints',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_base_frame': 'kmriiwa_base_footprint',
            'odom_frame_id': 'kmriiwa_odom',
            'global_frame_id': 'map'
        }],
        remappings=[
            # ('/tf', 'tf'),
            # ('/tf_static', 'tf_static'),
            ('/cmd_vel', '/kmriiwa/base/command/cmd_vel'),
            ('/odom', '/kmriiwa/base/state/odom'),
            ('/clock', '/kmriiwa_clock')
            
        ]
    )

    return LaunchDescription([
        #declare_map_yaml_cmd,
        #map_server_node,
        amcl_node,
        lifecycle_manager_node,
        robot_bringup,
        rviz,
        #static_transform_pub,
        #robot_state_publisher,
        #teleop_node,
        bt_navigator_node,
        nav_through_poses_node
    ])