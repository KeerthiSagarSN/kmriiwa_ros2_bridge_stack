import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package directories
    kmriiwa_navigation_dir = get_package_share_directory('kmriiwa_navigation')
    kmriiwa_moveit_dir = get_package_share_directory('kmriiwa_moveit')
    kmriiwa_vis_dir = get_package_share_directory('kmriiwa_vis')

    # Launch Arguments
    declare_use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to use a namespace for the robot'
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='kmriiwa',
        description='Name of the robot'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    declare_no_static_map = DeclareLaunchArgument(
        'no_static_map',
        default_value='false',
        description='Use mapless navigation'
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(kmriiwa_navigation_dir, 'maps', 'dryLab.yaml'),
        description='Path to map file'
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

    # Navigation Launch (Conditional based on static map)
    map_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kmriiwa_navigation_dir, 'launch', 'map_navigation.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('no_static_map')),
        launch_arguments={
            'base_global_planner': LaunchConfiguration('base_global_planner'),
            'base_local_planner': LaunchConfiguration('base_local_planner'),
            'map_file': LaunchConfiguration('map_file'),
            'use_namespace': LaunchConfiguration('use_namespace'),
            'robot_name': LaunchConfiguration('robot_name')
        }.items()
    )

    mapless_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kmriiwa_navigation_dir, 'launch', 'mapless_navigation.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('no_static_map')),
        launch_arguments={
            'base_global_planner': LaunchConfiguration('base_global_planner'),
            'base_local_planner': LaunchConfiguration('base_local_planner'),
            'use_namespace': LaunchConfiguration('use_namespace'),
            'robot_name': LaunchConfiguration('robot_name')
        }.items()
    )

    # MoveIt Launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kmriiwa_moveit_dir, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'pipeline': 'ompl',
            'use_namespace': LaunchConfiguration('use_namespace'),
            'robot_name': LaunchConfiguration('robot_name'),
            'no_virtual_joint': LaunchConfiguration('no_static_map')
        }.items()
    )

    # RViz Visualization Launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kmriiwa_vis_dir, 'launch', 'navigation_moveit_view.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('rviz')),
        launch_arguments={
            'no_static_map': LaunchConfiguration('no_static_map')
        }.items()
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_namespace)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_rviz)
    ld.add_action(declare_no_static_map)
    ld.add_action(declare_map_file)
    ld.add_action(declare_base_global_planner)
    ld.add_action(declare_base_local_planner)

    # Add launches
    ld.add_action(map_navigation_launch)
    ld.add_action(mapless_navigation_launch)
    ld.add_action(moveit_launch)
    ld.add_action(rviz_launch)

    return ld
