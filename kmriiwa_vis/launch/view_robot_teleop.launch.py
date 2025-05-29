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
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        prefix='xterm -e',  # This opens in a new terminal window (interesting, finally no cmd prompt shit !)
        output='screen',
        remappings=[
            ('/cmd_vel', '/kmriiwa/base/command/cmd_vel'),  # Remap to stoic's topic based on ros1-bridge (Do not change this- If robot needs to work !! )
        ]
    )

    # Return launch description with added teleop
    return LaunchDescription([
        robot_bringup,
        rviz,
        teleop_node
    ])