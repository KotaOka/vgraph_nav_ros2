import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    declare_model_arg = DeclareLaunchArgument(
        'model', default_value='burger', description='model type [burger, waffle, waffle_pi]'
    )
    declare_x_pos_arg = DeclareLaunchArgument(
        'x_pos', default_value='-2.0'
    )
    declare_y_pos_arg = DeclareLaunchArgument(
        'y_pos', default_value='0.0'
    )
    declare_yaw_pos_arg = DeclareLaunchArgument(
        'yaw_pos', default_value='0.0'
    )
    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vgraph_nav_ros2'), 'maps', 'map.yaml'
        ])
    )
    declare_test_folder_arg = DeclareLaunchArgument(
        'test_folder',
        default_value=PathJoinSubstitution([
            FindPackageShare('vgraph_nav_ros2'), 'test'
        ])
    )
    declare_epsilon_factor_arg = DeclareLaunchArgument(
        'epsilon_factor', default_value='0.05'
    )
    declare_robot_radius_arg = DeclareLaunchArgument(
        'robot_radius', default_value='0.15'
    )
    declare_odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom'
    )
    declare_scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan'
    )
    declare_vel_linear_arg = DeclareLaunchArgument(
        'vel_linear', default_value='0.1'
    )
    declare_vel_theta_arg = DeclareLaunchArgument(
        'vel_theta', default_value='0.3'
    )
    declare_angle_tolerance_arg = DeclareLaunchArgument(
        'angle_tolerance', default_value='0.1'
    )
    declare_goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance', default_value='0.05'
    )
    declare_verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='false'
    )
    declare_open_rviz_arg = DeclareLaunchArgument(
        'open_rviz', default_value='true'
    )

    # Nodes and LaunchDescriptions
    # turtlebot3_world_launch
    turtlebot3_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vgraph_nav_ros2'), 'launch', 'turtlebot3_world.launch.py'
            ])
        ),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'x_pos': LaunchConfiguration('x_pos'),
            'y_pos': LaunchConfiguration('y_pos'),
            'yaw_pos': LaunchConfiguration('yaw_pos'),
        }.items()
    )

    # turtlebot3_remote_launch
    turtlebot3_remote_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'), 'launch', 'turtlebot3_state_publisher.launch.py'
            ])
        ),
        launch_arguments={'model': LaunchConfiguration('model')}.items()
    )

    # turtlebot3_amcl_launch
    turtlebot3_amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vgraph_nav_ros2'), 'launch', 'turtlebot3_amcl.launch.py'
            ])
        ),
        launch_arguments={
            'scan_topic': LaunchConfiguration('scan_topic'),
            'initial_x_pos': LaunchConfiguration('x_pos'),
            'initial_y_pos': LaunchConfiguration('y_pos'),
            'initial_yaw_pos': LaunchConfiguration('yaw_pos'),
        }.items()
    )

    # vgraph_planner_launch
    vgraph_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vgraph_nav_ros2'), 'launch', 'vgraph_planner.launch.py'
            ])
        ),
        launch_arguments={
            'map_file': LaunchConfiguration('map_file'),
            'test_folder': LaunchConfiguration('test_folder'),
            'epsilon_factor': LaunchConfiguration('epsilon_factor'),
            'robot_radius': LaunchConfiguration('robot_radius'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'vel_linear': LaunchConfiguration('vel_linear'),
            'vel_theta': LaunchConfiguration('vel_theta'),
            'angle_tolerance': LaunchConfiguration('angle_tolerance'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'verbose': LaunchConfiguration('verbose'),
        }.items()
    )



    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file')}]
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('open_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('vgraph_nav_ros2'), 'rviz', 'turtlebot3_navigation.rviz'
        ])]
    )

    return LaunchDescription([
        declare_model_arg,
        declare_x_pos_arg,
        declare_y_pos_arg,
        declare_yaw_pos_arg,
        declare_map_file_arg,
        declare_test_folder_arg,
        declare_epsilon_factor_arg,
        declare_robot_radius_arg,
        declare_odom_topic_arg,
        declare_scan_topic_arg,
        declare_vel_linear_arg,
        declare_vel_theta_arg,
        declare_angle_tolerance_arg,
        declare_goal_tolerance_arg,
        declare_verbose_arg,
        declare_open_rviz_arg,
        # turtlebot3_world_launch,
        turtlebot3_remote_launch,
        turtlebot3_amcl_launch,
        vgraph_planner_launch,
        map_server_node,
        rviz_node
    ])
