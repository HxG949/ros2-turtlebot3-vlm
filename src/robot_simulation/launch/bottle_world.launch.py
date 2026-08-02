import os
import random

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawn_bottles(context):
    seed = int(LaunchConfiguration('layout_seed').perform(context))
    bottle_x = float(LaunchConfiguration('bottle_x').perform(context))
    center_y = float(LaunchConfiguration('center_y').perform(context))
    minimum_gap = float(LaunchConfiguration('minimum_gap').perform(context))
    maximum_gap = float(LaunchConfiguration('maximum_gap').perform(context))

    if minimum_gap <= 0.06 or maximum_gap < minimum_gap:
        raise ValueError(
            'Bottle gaps must satisfy 0.06 < minimum_gap <= maximum_gap'
        )

    generator = random.Random(seed)
    left_gap = generator.uniform(minimum_gap, maximum_gap)
    right_gap = generator.uniform(minimum_gap, maximum_gap)
    bottle_positions = [
        ('bottle_left', center_y + left_gap),
        ('bottle_center', center_y),
        ('bottle_right', center_y - right_gap),
    ]

    model_file = PathJoinSubstitution([
        FindPackageShare('robot_simulation'),
        'models',
        'bottle_model',
        'model.sdf',
    ]).perform(context)

    actions = [LogInfo(msg=(
        f'Bottle layout seed={seed}: '
        f'left_y={bottle_positions[0][1]:.3f}, '
        f'center_y={center_y:.3f}, '
        f'right_y={bottle_positions[2][1]:.3f}'
    ))]
    for entity_name, bottle_y in bottle_positions:
        actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity',
                entity_name,
                '-file',
                model_file,
                '-x',
                f'{bottle_x:.3f}',
                '-y',
                f'{bottle_y:.3f}',
                '-z',
                '0.0',
            ],
        ))

    return actions


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
        'semantic_bottle.world',
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
            'x_pose': '-2.0',
            'y_pose': '-0.5',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bottle_x',
            default_value='0.0',
            description='Shared world X coordinate for the bottle row',
        ),
        DeclareLaunchArgument(
            'center_y',
            default_value='-0.5',
            description='World Y coordinate of the center bottle',
        ),
        DeclareLaunchArgument(
            'layout_seed',
            default_value='42',
            description='Seed used to generate reproducible bottle gaps',
        ),
        DeclareLaunchArgument(
            'minimum_gap',
            default_value='0.35',
            description='Minimum center-to-center bottle gap',
        ),
        DeclareLaunchArgument(
            'maximum_gap',
            default_value='0.65',
            description='Maximum center-to-center bottle gap',
        ),
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL',
            'burger_cam',
        ),
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
        OpaqueFunction(function=spawn_bottles),
    ])
