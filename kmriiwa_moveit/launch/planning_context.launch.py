from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'load_robot_description',
            default_value='false',
            description='Load robot description to parameter server'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='kmriiwa',
            description='Robot name'
        ),
        DeclareLaunchArgument(
            'no_virtual_joint',
            default_value='true',
            description='Disable virtual joint'
        ),
        DeclareLaunchArgument(
            'robot_description',
            default_value='robot_description',
            description='Robot description parameter name'
        ),
    ]

    # Get package paths
    kmriiwa_description_path = FindPackageShare('kmriiwa_description')
    kmriiwa_moveit_path = FindPackageShare('kmriiwa_moveit')

    # URDF/XACRO file path
    urdf_xacro = PathJoinSubstitution([
        kmriiwa_description_path,
        'urdf',
        'robot',
        'kmriiwa.urdf.xacro'
    ])

    # SRDF/XACRO file path
    srdf_xacro = PathJoinSubstitution([
        kmriiwa_moveit_path,
        'config',
        'kmriiwa.srdf.xacro'
    ])

    # Load and process files
    robot_description_content = Command([
        'xacro ',
        urdf_xacro
    ])

    robot_description_semantic_content = Command([
        'xacro ',
        srdf_xacro,
        ' robot_name:=',
        LaunchConfiguration('robot_name'),
        ' no_virtual_joint:=',
        LaunchConfiguration('no_virtual_joint')
    ])

    # Load joint limits
    joint_limits_yaml = PathJoinSubstitution([
        kmriiwa_moveit_path, 
        'config',
        'joint_limits.yaml'
    ])

    with open(joint_limits_yaml.perform(None), 'r') as file:
        joint_limits_params = yaml.safe_load(file)

    # Load kinematics
    kinematics_yaml = PathJoinSubstitution([
        kmriiwa_moveit_path,
        'config',
        'kinematics.yaml'
    ])

    with open(kinematics_yaml.perform(None), 'r') as file:
        kinematics_params = yaml.safe_load(file)

    # Robot description node
    robot_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description_content,
                value_type=str
            ),
            f'{LaunchConfiguration("robot_description")}_semantic': ParameterValue(
                robot_description_semantic_content,
                value_type=str
            ),
            f'{LaunchConfiguration("robot_description")}_planning': {
                LaunchConfiguration('robot_name'): joint_limits_params
            },
            f'{LaunchConfiguration("robot_description")}_kinematics': {
                LaunchConfiguration('robot_name'): kinematics_params
            }
        }],
        condition=IfCondition(LaunchConfiguration('load_robot_description'))
    )

    return LaunchDescription(
        declared_arguments + 
        [
            robot_description_node
        ]
    )
