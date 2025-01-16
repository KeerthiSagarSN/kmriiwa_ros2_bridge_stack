from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math

def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='kmriiwa',
        description='Robot name prefix for frames and topics'
    )

    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_destination_topic',
        default_value='merged_cloud',
        description='Topic name for merged point cloud'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_destination_topic',
        default_value='scan_multi',
        description='Topic name for merged laser scan'
    )

    # Create laser scan merger node
    merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        output='screen',
        respawn=True,
        respawn_delay=10,
        parameters=[{
            'destination_frame': [LaunchConfiguration('robot_name'), '_base_link'],
            'cloud_destination_topic': LaunchConfiguration('cloud_destination_topic'),
            'scan_destination_topic': LaunchConfiguration('scan_destination_topic'),
            'laserscan_topics': [
                '/', LaunchConfiguration('robot_name'), '/base/state/LaserB1Scan ',
                '/', LaunchConfiguration('robot_name'), '/base/state/LaserB4Scan'
            ],
            'angle_min': (-135.0 * 3.1415) / 180.0,
            'angle_max': (135.0 * 3.1415) / 180.0,
            'angle_increment': (0.5 * 3.1415) / 180.0,
            'scan_time': 0.5,
            'range_min': 0.12,
            'range_max': 30.0
            
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        cloud_topic_arg,
        scan_topic_arg,
        merger_node
    ])
