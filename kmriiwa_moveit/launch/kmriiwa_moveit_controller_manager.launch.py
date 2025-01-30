from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml
import os

def generate_launch_description():
    # Declare arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='kmriiwa'
    )
    
    moveit_controller_manager_arg = DeclareLaunchArgument(
        'moveit_controller_manager',
        default_value='moveit_simple_controller_manager/MoveItSimpleControllerManager'
    )

    # Get the path to the controllers config file
    controllers_file = PathJoinSubstitution([
        FindPackageShare('kmriiwa_moveit'),
        'config',
        'ros_controllers.yaml'
    ])

    # Load the controllers yaml file
    with open(controllers_file.perform(None), 'r') as f:
        controllers_config = yaml.safe_load(f)

    return LaunchDescription([
        robot_name_arg,
        moveit_controller_manager_arg,
        
        # Load controller manager parameter
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            parameters=[
                {
                    'moveit_controller_manager': LaunchConfiguration('moveit_controller_manager'),
                    # Load controllers config with robot name substitution
                    **{LaunchConfiguration('robot_name'): controllers_config}
                }
            ]
        )
    ])
