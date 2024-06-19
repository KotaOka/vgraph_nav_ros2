import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Define launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='Model type [burger, waffle, waffle_pi]'
    )
    x_pos_arg = DeclareLaunchArgument(
        'x_pos',
        default_value='0.0',
        description='Initial x position'
    )
    y_pos_arg = DeclareLaunchArgument(
        'y_pos',
        default_value='0.0',
        description='Initial y position'
    )
    yaw_pos_arg = DeclareLaunchArgument(
        'yaw_pos',
        default_value='0.0',
        description='Initial yaw position'
    )

    # Define the paths to the URDF files for different models
    urdf_files = {
        'burger': os.path.join(
            get_package_share_directory('turtlebot3_description'),
            'urdf',
            'turtlebot3_burger.urdf'
        ),
        'waffle': os.path.join(
            get_package_share_directory('turtlebot3_description'),
            'urdf',
            'turtlebot3_waffle.urdf'
        ),
        'waffle_pi': os.path.join(
            get_package_share_directory('turtlebot3_description'),
            'urdf',
            'turtlebot3_waffle_pi.urdf'
        )
    }

    # Select the appropriate URDF file based on the model argument
    urdf_file = PythonExpression(["'", urdf_files['burger'], "'", " if 'burger' == '", LaunchConfiguration('model'), "' else ",
                                  "'", urdf_files['waffle'], "'", " if 'waffle' == '", LaunchConfiguration('model'), "' else ",
                                  "'", urdf_files['waffle_pi'], "'"])

    # Include the empty world launch from gazebo_ros
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')),
        launch_arguments={
            'world_name': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_world.world'),
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items(),
    )

    # Set the robot_description parameter using the selected URDF file
    robot_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_file
        }],
        output='screen'
    )

    # Spawn the URDF model in Gazebo
    spawn_urdf_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-x', LaunchConfiguration('x_pos'),
            '-y', LaunchConfiguration('y_pos'),
            '-Y', LaunchConfiguration('yaw_pos'),
            '-file', urdf_file
        ],
        output='screen'
    )

    # Logging for debugging purposes
    log_info_model = LogInfo(msg=LaunchConfiguration('model'))
    log_info_urdf = LogInfo(msg=urdf_file)

    return LaunchDescription([
        model_arg,
        x_pos_arg,
        y_pos_arg,
        yaw_pos_arg,
        world_launch,
        robot_description_node,
        spawn_urdf_node,
        log_info_model,
        log_info_urdf
    ])

