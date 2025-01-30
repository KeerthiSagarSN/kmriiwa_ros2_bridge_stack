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
            default_value='chomp_interface/CHOMPPlanner',
        ),
        DeclareLaunchArgument(
            'start_state_max_bounds_error',
            default_value='0.1'
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
        )
    ]

    # Get the CHOMP config file
    chomp_config = PathJoinSubstitution([
        FindPackageShare('kmriiwa_moveit'),
        'config',
        'chomp_planning.yaml'
    ])

    # Load CHOMP configuration
    with open(chomp_config.perform(None), 'r') as f:
        chomp_params = yaml.safe_load(f)

    # Create move_group node with CHOMP parameters
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            {
                'planning_plugin': LaunchConfiguration('planning_plugin'),
                'request_adapters': LaunchConfiguration('planning_adapters'),
                'start_state_max_bounds_error': LaunchConfiguration('start_state_max_bounds_error'),
                **chomp_params
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
