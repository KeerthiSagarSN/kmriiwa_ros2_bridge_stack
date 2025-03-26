from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    # Get package share directories
    pkg_kmriiwa_nav = get_package_share_directory('kmriiwa_navigation')
    pkg_kmriiwa_vis = get_package_share_directory('kmriiwa_vis')
    pkg_kmriiwa_bringup = get_package_share_directory('kmriiwa_bringup')
    pkg_kmriiwa_gazebo = get_package_share_directory('kmriiwa_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch arguments
    declare_map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_kmriiwa_nav, 'maps', 'bay3_map.yaml'),
        description='Path to map yaml file'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/kmriiwa/cmd_vel',
        description='Topic for velocity commands'
    )
    
    declare_joint_damping = DeclareLaunchArgument(
        'joint_damping',
        default_value='20.0',
        description='Damping for robot arm joints to prevent falling'
    )
    
    # Get configurations
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    joint_damping = LaunchConfiguration('joint_damping')

    # Map Server Node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': use_sim_time,
        }]
    )

    # AMCL Node
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg_kmriiwa_nav, 'config', 'nav2_params.yaml')],
        remappings=[
            ('/scan', '/scan_multi'),
            ('/map', '/map'),
            ('/initialpose', '/initialpose')
        ]
    )

    # Navigation Controller Node - IMPORTANT: Updated for Gazebo
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join(pkg_kmriiwa_nav, 'config', 'nav2_params.yaml')],
        remappings=[
            ('/cmd_vel', cmd_vel_topic),  # Use the cmd_vel_topic parameter
            ('/odom', '/kmriiwa/base/state/odom'),  # Make sure odom is correctly mapped
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(pkg_kmriiwa_nav, 'config', 'nav2_params.yaml')],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[os.path.join(pkg_kmriiwa_nav, 'config', 'nav2_params.yaml')]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[os.path.join(pkg_kmriiwa_nav, 'config', 'nav2_params.yaml')]
    )

    # Single Lifecycle Manager for all navigation nodes
    nav_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ],
            'bond_timeout': 15.0,
            'configure_timeout': 60.0,
            'activate_timeout': 60.0,
            'transform_timeout': 15.0
        }]
    )

    # Include robot bringup launch file (real or simulated)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kmriiwa_bringup, 'launch', 'kmriiwa_bringup.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=UnlessCondition(use_sim_time)  # Only for real robot
    )
    
    # Include Gazebo simulation if in simulation mode
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kmriiwa_gazebo, 'launch', 'kmriiwa_coresense_zone.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'paused': 'false',  # Start unpaused
            'hardware_interface': 'EffortJointInterface',  # Use effort interface for gravity compensation
        }.items(),
        condition=IfCondition(use_sim_time)  # Only for simulation
    )
    
    # Joint state publisher with damping to prevent arm from falling
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Add damping parameters to help prevent gravity collapse
            'damping': {
                'kmriiwa_joint_1': joint_damping,
                'kmriiwa_joint_2': joint_damping,
                'kmriiwa_joint_3': joint_damping,
                'kmriiwa_joint_4': joint_damping,
                'kmriiwa_joint_5': joint_damping,
                'kmriiwa_joint_6': joint_damping,
                'kmriiwa_joint_7': joint_damping
            }
        }],
        remappings=[('joint_states', '/kmriiwa/joint_states')],
        output='screen',
        condition=IfCondition(use_sim_time)  # Only needed in simulation
    )
    
    # Command velocity publisher for direct testing
    # This can be used to verify cmd_vel is working by publishing directly
    cmd_vel_publisher = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        parameters=[{
            'input_topic': '/nav2/cmd_vel',
            'output_topic': cmd_vel_topic,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Create RViz node with navigation configuration
    rviz_config = os.path.join(pkg_kmriiwa_vis, 'rviz', 'robot_nav.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'qos_overrides': {
                '/tf': {
                    'subscription': {
                        'depth': 100,
                        'durability': 'volatile',
                        'history': 'keep_last',
                        'reliability': 'reliable'
                    }
                },
                '/tf_static': {
                    'subscription': {
                        'depth': 100,
                        'durability': 'transient_local',
                        'history': 'keep_last',
                        'reliability': 'reliable'
                    }
                }
            }
        }],
        remappings=[('/cmd_vel', cmd_vel_topic)]
    )

    # Return launch description with navigation components
    return LaunchDescription([
        # Launch arguments
        declare_map_yaml_file,
        declare_use_sim_time,
        declare_cmd_vel_topic,
        declare_joint_damping,
        
        # Navigation components
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        nav_lifecycle_manager,
        
        # Robot-specific components
        robot_bringup,
        gazebo_launch,
        joint_state_publisher_node,
        cmd_vel_publisher,
        rviz
    ])
