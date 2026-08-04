import os
import random

from ament_index_python.packages import get_package_share_directory
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


BOTTLE_COUNT = 3
BOTTLE_RADIUS = 0.035
BOTTLE_CLEARANCE = 0.01
OBSTACLE_X_MIN = -0.20
OBSTACLE_X_MAX = 0.0
OBSTACLE_Y_MIN = -1.05
OBSTACLE_Y_MAX = 1.05
MAX_LAYOUT_ATTEMPTS = 1000


def spawn_bottles(context):
    seed = int(LaunchConfiguration('layout_seed').perform(context))
    generator = random.Random(seed)
    minimum_center_distance = 2.0 * BOTTLE_RADIUS + BOTTLE_CLEARANCE
    bottle_positions = []

    for _ in range(MAX_LAYOUT_ATTEMPTS):
        bottle_x = generator.uniform(
            OBSTACLE_X_MIN + BOTTLE_RADIUS,
            OBSTACLE_X_MAX - BOTTLE_RADIUS,
        )
        bottle_y = generator.uniform(
            OBSTACLE_Y_MIN + BOTTLE_RADIUS,
            OBSTACLE_Y_MAX - BOTTLE_RADIUS,
        )
        if all(
            (bottle_x - existing_x) ** 2
            + (bottle_y - existing_y) ** 2
            >= minimum_center_distance ** 2
            for existing_x, existing_y in bottle_positions
        ):
            bottle_positions.append((bottle_x, bottle_y))
            if len(bottle_positions) == BOTTLE_COUNT:
                break

    if len(bottle_positions) != BOTTLE_COUNT:
        raise RuntimeError(
            f'Unable to generate {BOTTLE_COUNT} non-overlapping bottles '
            f'after {MAX_LAYOUT_ATTEMPTS} attempts'
        )

    model_file = PathJoinSubstitution([
        FindPackageShare('robot_simulation'),
        'models',
        'bottle_model',
        'model.sdf',
    ]).perform(context)

    layout_description = ', '.join(
        f'bottle_{index}=({bottle_x:.3f}, {bottle_y:.3f})'
        for index, (bottle_x, bottle_y) in enumerate(
            bottle_positions,
            start=1,
        )
    )
    actions = [LogInfo(msg=(
        f'Bottle layout seed={seed}: {layout_description}'
    ))]
    for index, (bottle_x, bottle_y) in enumerate(
        bottle_positions,
        start=1,
    ):
        actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity',
                f'competition_bottle_{index}',
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
    package_share_path = get_package_share_directory('robot_simulation')
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
    robot_model_file = PathJoinSubstitution([
        package_share,
        'models',
        'turtlebot3_burger_low_lidar',
        'model.sdf',
    ])
    robot_description_file = os.path.join(
        package_share_path,
        'urdf',
        'turtlebot3_burger_low_lidar.urdf',
    )
    with open(robot_description_file, encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()
    gazebo_launch_directory = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
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

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
    )

    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity',
            'turtlebot3_burger_low_lidar',
            '-file',
            robot_model_file,
            '-x',
            '-0.9015',
            '-y',
            '-0.845',
            '-z',
            '0.01',
            '-Y',
            '0.0',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'layout_seed',
            default_value='42',
            description='Seed used to generate a reproducible 2D layout',
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
