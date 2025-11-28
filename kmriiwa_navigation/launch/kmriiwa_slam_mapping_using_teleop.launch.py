# kmriiwa_slam_mapping_using_teleop.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. Include the robot bringup and visualization
    view_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('kmriiwa_vis'),
                        'launch', 'view_robot_teleop.launch.py')
        ])
    )
    
    # 2. Laser scan merger (start after robot topics are available)
    laser_merger = TimerAction(
        period=3.0,  # Wait 3 seconds for robot topics
        actions=[
            Node(
                package='ira_laser_tools',
                executable='laserscan_multi_merger',
                name='laserscan_multi_merger',
                output='screen',
                respawn=True,
                parameters=[{
                    'destination_frame': 'kmriiwa_base_link',
                    'cloud_destination_topic': '/merged_cloud',
                    'scan_destination_topic': '/scan_multi',
                    'laserscan_topics': '/kmriiwa/base/state/LaserB1Scan /kmriiwa/base/state/LaserB4Scan',
                    'angle_min': -3.05432,  # -175 degrees
                    'angle_max': 3.05432,   # 175 degrees  
                    'angle_increment': 0.008726,  # 0.5 degrees
                    'scan_time': 0.2,
                    'range_min': 0.05,
                    'range_max': 30.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0,
                    # QoS settings to match laser scanners
                    'qos_overrides': {
                        '/scan_multi': {
                            'reliability': 'best_effort',
                            'durability': 'volatile',
                            'history': 'keep_last',
                            'depth': 10
                        }
                    }
                }]
            )
        ]
    )
    
    # 3. SLAM Toolbox (start after laser merger)
    slam_config = os.path.join(
        get_package_share_directory('kmriiwa_navigation'),
        'config',
        'kmriiwa_slam_config.yaml'
    )
    
    slam_node = TimerAction(
        period=6.0,  # Wait 6 seconds for merged scan topic
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_config,
                    {
                        # QoS override to match laser merger
                        'qos_overrides': {
                            '/scan_multi': {
                                'reliability': 'best_effort',
                                'durability': 'volatile',
                                'history': 'keep_last',
                                'depth': 10
                            }
                        }
                    }
                ],
                remappings=[
                    ('/scan', '/scan_multi')  # Use merged scan topic
                ]
            )
        ]
    )
    
    return LaunchDescription([
        view_teleop,
        laser_merger,
        slam_node
    ])