import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, 
                             IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PythonExpression, 
                                   TextSubstitution)
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Get package directory
    kmriiwa_navigation_dir = get_package_share_directory('kmriiwa_navigation')

    # Declare launch arguments
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

    declare_no_static_map = DeclareLaunchArgument(
        'no_static_map',
        default_value='false',
        description='Use laser-based global costmap instead of static map'
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

    # Namespace configuration
    namespace = PythonExpression([
        "'', '", 
        LaunchConfiguration('robot_name'), 
        "' '[if LaunchConfiguration('use_namespace')=='true' else '']"
    ])

    # Laser scan merge launch
    laserscan_merge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kmriiwa_navigation_dir, 'launch', 'laserscan_merge.launch.py')
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name')
        }.items()
    )

    # Move base node configuration
    move_base_node = Node(
        package='nav2_controller',  # Note: ROS2 uses nav2_controller instead of move_base
        executable='controller_server',
        name='move_base',
        output='screen',
        parameters=[
            # Load local planner configuration
            os.path.join(kmriiwa_navigation_dir, 'config', 'local_planner.yaml'),
            
            # Common costmap configurations
            {
                'global_costmap.yaml_filename': os.path.join(kmriiwa_navigation_dir, 'config', 'common_costmap.yaml'),
                'local_costmap.yaml_filename': os.path.join(kmriiwa_navigation_dir, 'config', 'common_costmap.yaml'),
            },
            
            # Conditional global costmap configuration
            {
                # Static map configuration
                'global_costmap.yaml_filename': os.path.join(
                    kmriiwa_navigation_dir, 
                    'config', 
                    'global_costmap_static_map.yaml' if not LaunchConfiguration('no_static_map').perform() else 'global_costmap_laser.yaml'
                ),
                
                # If no static map, adjust global costmap dimensions
                'global_costmap.width': 100.0 if LaunchConfiguration('no_static_map').perform() else None,
                'global_costmap.height': 100.0 if LaunchConfiguration('no_static_map').perform() else None,
            },
            
            # Local costmap configuration
            {
                'local_costmap.yaml_filename': os.path.join(kmriiwa_navigation_dir, 'config', 'local_costmap.yaml')
            },
            
            # Planner specifications
            {
                'base_global_planner': LaunchConfiguration('base_global_planner'),
                'base_local_planner': LaunchConfiguration('base_local_planner')
            }
        ],
        remappings=[
            ('cmd_vel', f'/{LaunchConfiguration("robot_name").perform()}/base/command/cmd_vel')
        ]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_namespace)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_no_static_map)
    ld.add_action(declare_base_global_planner)
    ld.add_action(declare_base_local_planner)

    # Conditional namespace wrapping
    group_action = GroupAction([
        PushRosNamespace(
            condition=IfCondition(LaunchConfiguration('use_namespace')), 
            namespace=LaunchConfiguration('robot_name')
        ),
        
        # Add laserscan merge launch
        laserscan_merge_launch,
        
        # Add move base node
        move_base_node
    ])

    # Add group action to launch description
    ld.add_action(group_action)

    return ld
