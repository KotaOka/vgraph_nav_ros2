import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(get_package_share_directory('vgraph_nav_ros2'), 'maps', 'map.yaml'),
        description='Path to the map file'
    )
    test_folder_arg = DeclareLaunchArgument(
        'test_folder',
        default_value=os.path.join(get_package_share_directory('vgraph_nav_ros2'), 'test'),
        description='Path to the test folder'
    )
    epsilon_factor_arg = DeclareLaunchArgument(
        'epsilon_factor',
        default_value='0.05',
        description='Epsilon factor for planner'
    )
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.1',
        description='Robot radius for planner'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic'
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Scan topic'
    )
    vel_linear_arg = DeclareLaunchArgument(
        'vel_linear',
        default_value='0.1',
        description='Linear velocity'
    )
    vel_theta_arg = DeclareLaunchArgument(
        'vel_theta',
        default_value='0.1',
        description='Angular velocity'
    )
    angle_tolerance_arg = DeclareLaunchArgument(
        'angle_tolerance',
        default_value='0.1',
        description='Angle tolerance for goal'
    )
    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.1',
        description='Goal tolerance distance'
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Verbose output'
    )

    # Define the vgraph_planner_node
    vgraph_planner_node = Node(
        package='vgraph_nav_ros2',
        executable='vgraph_nav_ros2_node',
        parameters=[{
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
            'verbose': LaunchConfiguration('verbose')
        }],
        output='screen'
    )

    # Print a message indicating launch success
    print_message = LogInfo(
        msg='Launch file successfully started!'
    )

    return LaunchDescription([
        map_file_arg,
        test_folder_arg,
        epsilon_factor_arg,
        robot_radius_arg,
        odom_topic_arg,
        scan_topic_arg,
        vel_linear_arg,
        vel_theta_arg,
        angle_tolerance_arg,
        goal_tolerance_arg,
        verbose_arg,
        vgraph_planner_node,
        print_message
    ])

