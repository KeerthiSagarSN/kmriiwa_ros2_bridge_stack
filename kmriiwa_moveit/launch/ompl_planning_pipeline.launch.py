from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'planning_plugin',
            default_value='ompl_interface/OMPLPlanner'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='kmriiwa'
        ),
        DeclareLaunchArgument(
            'planning_adapters',
            default_value=[
                'default_planner_request_adapters/AddTimeParameterization',
                'default_planner_request_adapters/ResolveConstraintFrames',
                'default_planner_request_adapters/FixWorkspaceBounds',
                'default_planner_request_adapters/FixStartStateBounds',
                'default_planner_request_adapters/FixStartStateCollision',
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ]
        ),
        DeclareLaunchArgument(
            'start_state_max_bounds_error',
            default_value='0.1'
        )
    ]

    # Get the OMPL config file
    ompl_config = PathJoinSubstitution([
        FindPackageShare('kmriiwa_moveit'),
        'config',
        'ompl_planning.yaml'
    ])

    # Load and process OMPL configuration
    def load_yaml(yaml_file):
        with open(yaml_file, 'r') as f:
            return yaml.safe_load(f)

    # Load the YAML with robot name substitution
    ompl_params = load_yaml(ompl_config.perform(None))

    # Create move_group node with OMPL parameters
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            {
                'planning_plugin': LaunchConfiguration('planning_plugin'),
                'request_adapters': LaunchConfiguration('planning_adapters'),
                'start_state_max_bounds_error': LaunchConfiguration('start_state_max_bounds_error'),
                LaunchConfiguration('robot_name'): ompl_params  # Apply robot name as namespace
            }
        ],
        output='screen'
    )

    return LaunchDescription(
        declared_arguments + 
        [
            move_group_node
        ]
    )
