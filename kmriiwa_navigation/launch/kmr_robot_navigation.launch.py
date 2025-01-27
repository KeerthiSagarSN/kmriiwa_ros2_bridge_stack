from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directories
    pkg_kmriiwa_nav = get_package_share_directory('kmriiwa_navigation')
    pkg_kmriiwa_vis = get_package_share_directory('kmriiwa_vis')
    pkg_kmriiwa_bringup = get_package_share_directory('kmriiwa_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch arguments
    map_yaml_file = LaunchConfiguration('map')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_kmriiwa_nav, 'maps', 'bay3_map.yaml'),
        description='Full path to map yaml file to load'
    )

    # Map Server Node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': False,
        }]
    )

    # AMCL Node
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg_kmriiwa_nav, 'config', 'nav2_params.yaml')],
        remappings=[
            ('/scan', '/scan_multi'),
            ('/map', '/map'),
            ('/initialpose', '/initialpose')
        ]
    )

    # Navigation Controller Node
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join(pkg_kmriiwa_nav, 'config', 'nav2_params.yaml')],
        remappings=[('/cmd_vel', '/kmriiwa/base/command/cmd_vel')]
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(pkg_kmriiwa_nav, 'config', 'nav2_params.yaml')],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[os.path.join(pkg_kmriiwa_nav, 'config', 'nav2_params.yaml')]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[os.path.join(pkg_kmriiwa_nav, 'config', 'nav2_params.yaml')]
    )

    # Single Lifecycle Manager for all navigation nodes
    nav_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ],
            'bond_timeout': 10.0,
            'configure_timeout': 20.0,
            'activate_timeout': 20.0,
            'transform_timeout': 5.0
        }]
    )

    # Include robot bringup launch file
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kmriiwa_bringup, 'launch', 'kmriiwa_bringup.launch.py')
        ])
    )

    # Create RViz node with navigation configuration
    rviz_config = os.path.join(pkg_kmriiwa_vis, 'rviz', 'robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'qos_overrides': {
                '/tf': {
                    'subscription': {
                        'depth': 100,
                        'durability': 'volatile',
                        'history': 'keep_last',
                        'reliability': 'reliable'
                    }
                },
                '/tf_static': {
                    'subscription': {
                        'depth': 100,
                        'durability': 'transient_local',
                        'history': 'keep_last',
                        'reliability': 'reliable'
                    }
                }
            }
        }],
        remappings=[('/cmd_vel', '/kmriiwa/base/command/cmd_vel')]
    )

    # Return launch description with navigation components
    return LaunchDescription([
        map_arg,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        nav_lifecycle_manager,
        robot_bringup,
        rviz
    ])