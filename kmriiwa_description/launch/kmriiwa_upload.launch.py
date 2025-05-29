import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    kmriiwa_description_dir = get_package_share_directory('kmriiwa_description')
    xacro_share_dir = get_package_share_directory('xacro')

    # Launch Arguments
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='kmriiwa',
        description='Name of the robot'
    )

    declare_hardware_interface = DeclareLaunchArgument(
        'hardware_interface',
        default_value='PositionJointInterface',
        description='Hardware interface type'
    )

    declare_robot_extras = DeclareLaunchArgument(
        'robot_extras',
        default_value=os.path.join(kmriiwa_description_dir, 'urdf', 'robot', 'kmriiwa.urdf.xacro'),
        description='Additional robot XACRO file'
    )

    # Robot description command
    robot_description_command = Command([
        'xacro ',
        os.path.join(kmriiwa_description_dir, 'urdf', 'robot', 'kmriiwa.urdf.xacro'),
        ' robot_name:=', LaunchConfiguration('robot_name'),
        ' hardware_interface:=', LaunchConfiguration('hardware_interface'),
        ' robot_extras:=', LaunchConfiguration('robot_extras')
    ])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_command,
            'use_sim_time': False
        }]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_robot_name)
    ld.add_action(declare_hardware_interface)
    ld.add_action(declare_robot_extras)

    # Add robot state publisher node
    ld.add_action(robot_state_publisher_node)

    return ld
