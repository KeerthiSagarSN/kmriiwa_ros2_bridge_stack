from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_name',
            default_value='kmriiwa',
            description='Robot name'
        ),
        DeclareLaunchArgument(
            'moveit_manage_controllers',
            default_value='true',
            description='Flag indicating whether MoveIt is allowed to load/unload or switch controllers'
        ),
        DeclareLaunchArgument(
            'moveit_controller_manager',
            default_value='kmriiwa',
            description='Robot-specific controller manager'
        ),
    ]

    # Create node for trajectory execution parameters
    trajectory_execution_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[{
            'moveit_manage_controllers': LaunchConfiguration('moveit_manage_controllers'),
            'trajectory_execution/execution_duration_monitoring': False,
            'trajectory_execution/allowed_execution_duration_scaling': 1.2,
            'trajectory_execution/allowed_goal_duration_margin': 0.5,
            'trajectory_execution/allowed_start_tolerance': 0.01
        }]
    )

    # Include robot-specific controller manager
    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kmriiwa_moveit'),
                'launch',
                LaunchConfiguration('moveit_controller_manager') + '_moveit_controller_manager.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name')
        }.items()
    )

    return LaunchDescription(
        declared_arguments + 
        [
            trajectory_execution_node,
            controller_manager_launch
        ]
    )
