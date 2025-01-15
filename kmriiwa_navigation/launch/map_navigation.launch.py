import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, 
                             IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PythonExpression, 
                                   TextSubstitution)
from nav2_common.launch import LaunchReplacement

def generate_launch_description():
    # Get package directories
    kmriiwa_navigation_dir = get_package_share_directory('kmriiwa_navigation')
    kmriiwa_vis_dir = get_package_share_directory('kmriiwa_vis')

    # Launch Arguments
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(kmriiwa_navigation_dir, 'maps', 'dryLab.yaml'),
        description='Full path to map file'
    )

    declare_base_global_planner = DeclareLaunchArgument(
        'base_global_planner',
        default_value='navfn/NavfnROS',
        description='Global path planner to use'
    )

    declare_base_local_planner = DeclareLaunchArgument(
        'base_local_planner',
        default_value='teb_local_planner/TebLocalPlannerROS',
        description='Local path planner to use'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Run RViz if true'
    )

    declare_use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Enable namespace for robot'
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='kmriiwa',
        description='Name of the robot'
    )

    # Namespace configuration
    namespace = PythonExpression([
        "'', '", 
        LaunchConfiguration('robot_name'), 
        "' '[if LaunchConfiguration('use_namespace')=='true' else '']"
    ])

    # Environment setup for namespace
    namespace_launch_arg = SetEnvironmentVariable(
        'RCUTILS_LOGGING_SEVERITY_THRESHOLD', 
        'WARN'  # Suppress debug messages in namespace
    )

    # Group action for namespaced nodes
    group_action = GroupAction([
        # Map server
        LaunchReplacement(
            'map_server', 
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map_file'),
                'use_sim_time': False
            }]
        ),

        # AMCL (Adaptive Monte Carlo Localization)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(kmriiwa_navigation_dir, 'launch', 'amcl.launch.py')
            ),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name')
            }.items()
        ),

        # Optional RViz visualization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(kmriiwa_vis_dir, 'launch', 'navigation_view.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('rviz')),
            launch_arguments={
                'no_static_map': 'false'
            }.items()
        )
    ])

    # Move base configuration
    move_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kmriiwa_navigation_dir, 'launch', 'move_base.launch.py')
        ),
        launch_arguments={
            'use_namespace': LaunchConfiguration('use_namespace'),
            'robot_name': LaunchConfiguration('robot_name'),
            'no_static_map': 'false',
            'base_global_planner': LaunchConfiguration('base_global_planner'),
            'base_local_planner': LaunchConfiguration('base_local_planner')
        }.items()
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_map_file)
    ld.add_action(declare_base_global_planner)
    ld.add_action(declare_base_local_planner)
    ld.add_action(declare_rviz)
    ld.add_action(declare_use_namespace)
    ld.add_action(declare_robot_name)

    # Add namespace and group actions
    ld.add_action(namespace_launch_arg)
    ld.add_action(group_action)

    # Add move base launch
    ld.add_action(move_base_launch)

    return ld
