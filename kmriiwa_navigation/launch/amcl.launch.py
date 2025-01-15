from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='kmriiwa',
        description='Robot name prefix for frames'
    )

    # Create AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        remappings=[
            ('scan', 'scan_multi'),
        ],
        parameters=[{
            'use_sim_time': False,
            'use_map_topic': True,
            'odom_frame_id': [LaunchConfiguration('robot_name'), '_odom'],
            'base_frame_id': [LaunchConfiguration('robot_name'), '_base_link'],
            'global_frame_id': 'map',
            
            # Publish scans from best pose at a max of 10 Hz
            'transform_tolerance': 0.2,
            'gui_publish_rate': 10.0,
            
            # Laser params
            'laser_max_beams': 30,
            'laser_model_type': 'likelihood_field',
            'laser_likelihood_max_dist': 2.0,
            'laser_z_hit': 0.5,
            'laser_z_short': 0.05,
            'laser_z_max': 0.05,
            'laser_z_rand': 0.5,
            'laser_sigma_hit': 0.2,
            'laser_lambda_short': 0.1,
            
            # Particle filter params
            'min_particles': 500,
            'max_particles': 5000,
            'kld_err': 0.05,
            'kld_z': 0.99,
            'update_min_d': 0.05,
            'update_min_a': 0.1,
            'resample_interval': 1,
            
            # Odometry model params
            'odom_model_type': 'omni',
            'odom_alpha1': 0.2,  # rotation noise from rotation
            'odom_alpha2': 0.2,  # rotation noise from translation
            'odom_alpha3': 0.8,  # translation noise from translation
            'odom_alpha4': 0.2,  # translation noise from rotation
            'odom_alpha5': 0.1,  # additional rotation noise
            
            # Recovery params
            'recovery_alpha_slow': 0.0,
            'recovery_alpha_fast': 0.0
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        amcl_node
    ])
