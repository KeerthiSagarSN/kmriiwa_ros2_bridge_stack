from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'cfg',
            description='List of .cfg files to process for benchmarking'
        ),
    ]

    # Package paths
    pkg_moveit = FindPackageShare('kmriiwa_moveit')

    # Include planning context launch
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_moveit, 'launch', 'planning_context.launch.py'])
        ]),
        launch_arguments={
            'load_robot_description': 'true'
        }.items()
    )

    # Include warehouse launch
    warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_moveit, 'launch', 'warehouse.launch.py'])
        ]),
        launch_arguments={
            'moveit_warehouse_database_path': 'moveit_ompl_benchmark_warehouse'
        }.items()
    )

    # Load OMPL config
    ompl_config = PathJoinSubstitution([
        pkg_moveit,
        'config',
        'ompl_planning.yaml'
    ])

    with open(ompl_config.perform(None), 'r') as f:
        ompl_params = yaml.safe_load(f)

    # Benchmark node
    benchmark_node = Node(
        package='moveit_ros_benchmarks',
        executable='moveit_run_benchmark',
        name='moveit_benchmark',
        output='screen',
        arguments=[
            LaunchConfiguration('cfg'),
            '--benchmark-planners'
        ],
        parameters=[ompl_params]
    )

    return LaunchDescription(
        declared_arguments + 
        [
            planning_context,
            warehouse_launch,
            benchmark_node
        ]
    )
