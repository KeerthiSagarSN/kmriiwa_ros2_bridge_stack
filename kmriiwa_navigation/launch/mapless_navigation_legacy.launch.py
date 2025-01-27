from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_nav = get_package_share_directory('kmriiwa_navigation')
    pkg_vis = get_package_share_directory('kmriiwa_vis')

    # Declare all arguments
    planner_arg = DeclareLaunchArgument(
        'planner',
        default_value='nav2_navfn_planner/NavfnPlanner',
        description='Global planner to use'
    )

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='teb_local_planner/TebLocalPlannerROS',
        description='Local planner to use'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )

    use_namespace_arg = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to use namespace'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='kmriiwa',
        description='Robot name'
    )

    # Include Nav2 launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_nav, 'launch', 'nav2_bringup.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'map': '',  # Empty string for no static map
            'planner': LaunchConfiguration('planner'),
            'controller': LaunchConfiguration('controller'),
            'use_namespace': LaunchConfiguration('use_namespace'),
            'robot_name': LaunchConfiguration('robot_name')
        }.items()
    )

    # Create namespace group for visualization
    viz_group = GroupAction([
        # Push namespace if use_namespace is true
        PushRosNamespace(
            namespace=PythonExpression([
                "'", LaunchConfiguration('robot_name'), "' if '",
                LaunchConfiguration('use_namespace'), "' == 'true' else ''"
            ])
        ),
        # Include visualization launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_vis, 'launch', 'navigation_view.launch.py')
            ]),
            condition=IfCondition(LaunchConfiguration('rviz')),
            launch_arguments={
                'use_sim_time': 'false',
                'no_static_map': 'true'
            }.items()
        )
    ])

    return LaunchDescription([
        planner_arg,
        controller_arg,
        rviz_arg,
        use_namespace_arg,
        robot_name_arg,
        nav2_launch,
        viz_group
    ])
