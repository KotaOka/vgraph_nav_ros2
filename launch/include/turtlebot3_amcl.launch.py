import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch argument definitions
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic name for laser scan data'
    )
    initial_x_pos_arg = DeclareLaunchArgument(
        'initial_x_pos',
        default_value='0.0',
        description='Initial x position for AMCL'
    )
    initial_y_pos_arg = DeclareLaunchArgument(
        'initial_y_pos',
        default_value='0.0',
        description='Initial y position for AMCL'
    )
    initial_yaw_pos_arg = DeclareLaunchArgument(
        'initial_yaw_pos',
        default_value='0.0',
        description='Initial yaw angle for AMCL'
    )

    # nav2_amcl node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            {'scan_topic': LaunchConfiguration('scan_topic')},
            {'initial_pose_x': LaunchConfiguration('initial_x_pos')},
            {'initial_pose_y': LaunchConfiguration('initial_y_pos')},
            {'initial_pose_a': LaunchConfiguration('initial_yaw_pos')},
            {'min_particles': 500},
            {'max_particles': 3000},
            {'kld_err': 0.02},
            {'update_min_d': 0.20},
            {'update_min_a': 0.20},
            {'resample_interval': 1},
            {'transform_tolerance': 0.5},
            {'recovery_alpha_slow': 0.00},
            {'recovery_alpha_fast': 0.00},
            {'gui_publish_rate': 50.0},
            {'laser_max_range': 3.5},
            {'laser_max_beams': 180},
            {'laser_z_hit': 0.5},
            {'laser_z_short': 0.05},
            {'laser_z_max': 0.05},
            {'laser_z_rand': 0.5},
            {'laser_sigma_hit': 0.2},
            {'laser_lambda_short': 0.1},
            {'laser_likelihood_max_dist': 2.0},
            {'laser_model_type': 'likelihood_field'},
            {'odom_model_type': 'diff'},
            {'odom_alpha1': 0.1},
            {'odom_alpha2': 0.1},
            {'odom_alpha3': 0.1},
            {'odom_alpha4': 0.1},
            {'odom_frame_id': 'odom'},
            {'base_frame_id': 'base_footprint'}
        ],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic'))
        ]
    )

    return LaunchDescription([
        scan_topic_arg,
        initial_x_pos_arg,
        initial_y_pos_arg,
        initial_yaw_pos_arg,
        amcl_node
    ])

