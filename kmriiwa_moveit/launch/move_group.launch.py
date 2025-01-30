from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare all arguments
    declared_arguments = [
        DeclareLaunchArgument('use_namespace', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='kmriiwa'),
        DeclareLaunchArgument('no_virtual_joint', default_value='true'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('info', default_value='false'),
        DeclareLaunchArgument('pipeline', default_value='ompl'),
        DeclareLaunchArgument('allow_trajectory_execution', default_value='true'),
        DeclareLaunchArgument('fake_execution', default_value='false'),
        DeclareLaunchArgument('max_safe_path_cost', default_value='1'),
        DeclareLaunchArgument('jiggle_fraction', default_value='0.05'),
        DeclareLaunchArgument('publish_monitored_planning_scene', default_value='true'),
        DeclareLaunchArgument('capabilities', default_value=''),
        DeclareLaunchArgument('disable_capabilities', default_value=''),
        DeclareLaunchArgument('load_robot_description', default_value='false'),
    ]

    # Get the package share directory
    pkg_share = FindPackageShare('kmriiwa_moveit')

    # Create launch configuration variables
    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    namespace = PythonExpression(['"', robot_name, '" if ', use_namespace, ' else ""'])

    # Include the planning context launch file
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'planning_context.launch.py'])
        ]),
        launch_arguments={
            'load_robot_description': LaunchConfiguration('load_robot_description'),
            'robot_name': robot_name,
            'no_virtual_joint': LaunchConfiguration('no_virtual_joint'),
        }.items()
    )

    # Include the planning pipeline launch file
    planning_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'planning_pipeline.launch.py'])
        ]),
        launch_arguments={
            'pipeline': LaunchConfiguration('pipeline'),
            'robot_name': robot_name,
        }.items()
    )

    # Include the trajectory execution launch file
    trajectory_execution = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'trajectory_execution.launch.py'])
        ]),
        condition=IfCondition(LaunchConfiguration('allow_trajectory_execution')),
        launch_arguments={
            'robot_name': robot_name,
            'moveit_manage_controllers': 'true',
            'moveit_controller_manager': PythonExpression([
                "'fake' if ", LaunchConfiguration('fake_execution'), " else 'kmriiwa'"
            ]),
        }.items()
    )

    # Include the sensor manager launch file
    sensor_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'sensor_manager.launch.py'])
        ]),
        condition=IfCondition(LaunchConfiguration('allow_trajectory_execution')),
        launch_arguments={
            'moveit_sensor_manager': 'kmriiwa',
        }.items()
    )

    # Configure the move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        parameters=[
            {
                'allow_trajectory_execution': LaunchConfiguration('allow_trajectory_execution'),
                'max_safe_path_cost': LaunchConfiguration('max_safe_path_cost'),
                'jiggle_fraction': LaunchConfiguration('jiggle_fraction'),
                'capabilities': LaunchConfiguration('capabilities'),
                'disable_capabilities': LaunchConfiguration('disable_capabilities'),
                'planning_scene_monitor/publish_planning_scene': LaunchConfiguration('publish_monitored_planning_scene'),
                'planning_scene_monitor/publish_geometry_updates': LaunchConfiguration('publish_monitored_planning_scene'),
                'planning_scene_monitor/publish_state_updates': LaunchConfiguration('publish_monitored_planning_scene'),
                'planning_scene_monitor/publish_transforms_updates': LaunchConfiguration('publish_monitored_planning_scene'),
            }
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        condition=IfCondition(PythonExpression([
            'not ', LaunchConfiguration('debug')
        ])),
        remappings=[
            ('joint_states', f'/{robot_name}/arm/joint_states'),
        ],
        prefix=PythonExpression([
            "'gdb -x " + str(PathJoinSubstitution([pkg_share, 'launch', 'gdb_settings.gdb'])) + " --ex run --args' if ",
            LaunchConfiguration('debug'),
            " else ''"
        ]),
        extra_env={'DISPLAY': EnvironmentVariable('DISPLAY', default_value=':0')},
    )

    return LaunchDescription(
        declared_arguments + 
        [
            GroupAction(
                actions=[
                    planning_context,
                    planning_pipeline,
                    trajectory_execution,
                    sensor_manager,
                    move_group_node,
                ],
                namespace=namespace
            )
        ]
    )
