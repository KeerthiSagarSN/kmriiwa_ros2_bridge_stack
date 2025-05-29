from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'reset',
            default_value='false',
            description='Reset the warehouse database'
        ),
        DeclareLaunchArgument(
            'moveit_warehouse_database_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('kmriiwa_moveit'),
                'default_warehouse_mongo_db'
            ]),
            description='Path to the warehouse database'
        ),
    ]

    # Include warehouse launch file
    warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kmriiwa_moveit'),
                'launch',
                'warehouse.launch.py'
            ])
        ]),
        launch_arguments={
            'moveit_warehouse_database_path': LaunchConfiguration('moveit_warehouse_database_path'),
        }.items()
    )

    # Node to reset the database if requested
    reset_db_node = Node(
        package='moveit_ros_warehouse',
        executable='moveit_init_demo_warehouse',
        name='moveit_default_db_reset',
        output='screen',
        condition=IfCondition(LaunchConfiguration('reset'))
    )

    return LaunchDescription(
        declared_arguments + 
        [
            warehouse_launch,
            reset_db_node
        ]
    )
