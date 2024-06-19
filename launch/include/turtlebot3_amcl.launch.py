from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from std_msgs.msg import Header

def generate_launch_description():
    return LaunchDescription([
        # 引数の宣言
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='レーザースキャンのトピック名'
        ),
        DeclareLaunchArgument(
            'initial_x_pos',
            default_value='0.0',
            description='初期のX座標'
        ),
        DeclareLaunchArgument(
            'initial_y_pos',
            default_value='0.0',
            description='初期のY座標'
        ),
        DeclareLaunchArgument(
            'initial_yaw_pos',
            default_value='0.0',
            description='初期のヨー角度'
        ),
        
        # AMCLノードの設定
        Node(
            package='nav2_amcl',  # nav2_amcl パッケージを使用
            executable='amcl',
            name='amcl',
            output='screen',
            remappings=[
                ('scan', LaunchConfiguration('scan_topic'))
            ],
            parameters=[{
                'initial_pose_x': LaunchConfiguration('initial_x_pos'),
                'initial_pose_y': LaunchConfiguration('initial_y_pos'),
                'initial_pose_a': LaunchConfiguration('initial_yaw_pos'),
                'min_particles': 500,
                'max_particles': 3000,
                'kld_err': 0.02,
                'update_min_d': 0.20,
                'update_min_a': 0.20,
                'resample_interval': 1,
                'transform_tolerance': 0.5,
                'recovery_alpha_slow': 0.00,
                'recovery_alpha_fast': 0.00,
                'gui_publish_rate': 50.0,
                'laser_max_range': 3.5,
                'laser_max_beams': 180,
                'laser_z_hit': 0.5,
                'laser_z_short': 0.05,
                'laser_z_max': 0.05,
                'laser_z_rand': 0.5,
                'laser_sigma_hit': 0.2,
                'laser_lambda_short': 0.1,
                'laser_likelihood_max_dist': 2.0,
                'laser_model_type': 'likelihood_field',
                'odom_model_type': 'diff',
                'odom_alpha1': 0.1,
                'odom_alpha2': 0.1,
                'odom_alpha3': 0.1,
                'odom_alpha4': 0.1,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_footprint',
                'tf_prefix': 'tf'
            }]
        ),
        
        # Static transform publisher from /map to /odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
            output='screen'
        ),
        
        # TFリスナーノードの設定
        Node(
            package='tf2_ros',
            executable='tf2_echo',
            name='tf2_echo_base_link_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        )
    ])


