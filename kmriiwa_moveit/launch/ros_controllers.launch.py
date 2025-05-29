from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_name',
            default_value='kmriiwa',
            description='Robot name'
        ),
    ]

    # Get the path to controllers config
    controllers_file = PathJoinSubstitution([
        FindPackageShare('kmriiwa_moveit'),
        'config',
        'ros_controllers.yaml'
    ])

    # Load controller parameters from YAML
    with open(controllers_file.perform(None), 'r') as f:
        controllers_config = yaml.safe_load(f)

    # Controller manager node for loading and starting controllers
    controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[],  # Add your controller names here
        parameters=[{
            LaunchConfiguration('robot_name'): controllers_config
        }]
    )

    return LaunchDescription(
        declared_arguments + 
        [
            controller_spawner_node
        ]
    )
