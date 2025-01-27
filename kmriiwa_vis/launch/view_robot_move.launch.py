from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Create RViz node
    rviz_config = os.path.join(pkg_kmriiwa_vis, 'rviz', 'robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Add teleop keyboard node

    ### Added to check if teleop works with this code
    ## Wrks quiote smoothly
    # teleop_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_keyboard',
    #     prefix='xterm -e',  # This opens in a new terminal window (interesting, finally no cmd prompt shit !)
    #     output='screen',
    #     remappings=[
    #         ('/cmd_vel', '/kmriiwa/base/command/cmd_vel'),  # Remap to stoic's topic based on ros1-bridge (Do not change this- If robot needs to work !! )
    #     ]
    # )

    # Add these nodes to your launch file:

    # Map Server Node
    map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    parameters=[{'yaml_filename': os.path.join(pkg_kmriiwa_vis, 'maps', 'bay3_map.yaml')}]
    )

    # AMCL for localization
    amcl_node = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    parameters=[{
        'use_sim_time': True,
        'base_frame_id': 'kmriiwa_base_footprint',
        'global_frame_id': 'map',
        'odom_frame_id': '/kmriiwa/base/state/odom',
        'scan_topic': '/kmriiwa/base/state/LaserB1Scan'
    }],
    remappings=[('/clock', '/kmriiwa_clock')]
    )

    # Nav2 Controller
    controller_node = Node(
    package='nav2_controller',
    executable='controller_server',
    parameters=[{
        'use_sim_time': True,
        'controller_frequency': 20.0,
    }],
    remappings=[
        ('/cmd_vel', '/kmriiwa/base/command/cmd_vel'),
        ('/clock', '/kmriiwa_clock')
    ]
    )

    # Nav2 Planner
    planner_node = Node(
    package='nav2_planner',
    executable='planner_server',
    parameters=[{
        'use_sim_time': True,
    }],
    remappings=[('/clock', '/kmriiwa_clock')]
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    parameters=[{
        'use_sim_time': True,
        'autostart': True,
        'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server']
    }],
    remappings=[('/clock', '/kmriiwa_clock')]
    )

    # Add to LaunchDescription
    return LaunchDescription([
    robot_bringup,
    rviz,
    #teleop_node,
    map_server_node,
    amcl_node,
    controller_node, 
    planner_node,
    lifecycle_manager
    ])

    # # Return launch description with added teleop
    # return LaunchDescription([
    #     robot_bringup,
    #     rviz,
    #     teleop_node
    # ])