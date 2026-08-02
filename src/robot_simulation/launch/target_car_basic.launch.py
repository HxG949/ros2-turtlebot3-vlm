import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'burger_cam'

    package_share = FindPackageShare('robot_simulation')
    model_directory = PathJoinSubstitution([package_share, 'models'])
    turtlebot_model_directory = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'models',
    ])
    world_file = PathJoinSubstitution([
        package_share,
        'worlds',
        'target_car_basic.world',
    ])
    gazebo_launch_directory = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
    ])
    turtlebot_launch_directory = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'launch',
    ])

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            gazebo_launch_directory,
            'gzserver.launch.py',
        ])),
        launch_arguments={'world': world_file}.items(),
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            gazebo_launch_directory,
            'gzclient.launch.py',
        ])),
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            turtlebot_launch_directory,
            'robot_state_publisher.launch.py',
        ])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            turtlebot_launch_directory,
            'spawn_turtlebot3.launch.py',
        ])),
        launch_arguments={
            'x_pose': LaunchConfiguration('robot_x'),
            'y_pose': LaunchConfiguration('robot_y'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_x',
            default_value='-0.9015',
            description='Robot start X coordinate',
        ),
        DeclareLaunchArgument(
            'robot_y',
            default_value='-0.845',
            description='Robot start Y coordinate',
        ),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger_cam'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            [
                model_directory,
                ':',
                turtlebot_model_directory,
                ':',
                EnvironmentVariable(
                    'GAZEBO_MODEL_PATH',
                    default_value='',
                ),
            ],
        ),
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_turtlebot,
    ])
