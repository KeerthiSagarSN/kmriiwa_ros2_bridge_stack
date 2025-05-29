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
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        prefix='xterm -e',
        output='screen',
        remappings=[
            ('/cmd_vel', '/kmriiwa/base/command/cmd_vel'),
        ]
    )

    # Add waypoint navigation node
    nav_through_poses_node = Node(
        package='nav2_simple_commander',
        executable='demo_navigation',  # This is an example, you might want to create your own script
        name='waypoint_nav',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_base_frame': 'kmriiwa_base_link',
            'odom_frame_id': 'kmriiwa_odom',
            'global_frame_id': 'map'
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('cmd_vel', '/kmriiwa/base/command/cmd_vel'),
            ('odom', '/kmriiwa/odom')
        ]
    )

    return LaunchDescription([
        robot_bringup,
        rviz,
        teleop_node,
        nav_through_poses_node
    ])