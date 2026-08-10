from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('robot_perception')
    config_directory = PathJoinSubstitution([package_share, 'config'])
    rviz_config = PathJoinSubstitution([
        package_share,
        'rviz',
        'obstacle_planning.rviz',
    ])
    enable_motion = LaunchConfiguration('enable_motion')
    stop_at_cp1 = LaunchConfiguration('stop_at_cp1')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_motion',
            default_value='false',
            description='Allow the follower to publish non-zero velocity',
        ),
        DeclareLaunchArgument(
            'stop_at_cp1',
            default_value='false',
            description='Stop the first commissioning run at CP1',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with the obstacle planning display',
        ),
        Node(
            package='robot_perception',
            executable='lidar_safety_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                config_directory,
                'lidar_safety.yaml',
            ])],
        ),
        Node(
            package='robot_perception',
            executable='axis_aligned_planner_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                config_directory,
                'axis_aligned_planner.yaml',
            ])],
        ),
        Node(
            package='robot_perception',
            executable='axis_aligned_follower_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    config_directory,
                    'axis_aligned_follower.yaml',
                ]),
                {
                    'enabled': ParameterValue(
                        enable_motion,
                        value_type=bool,
                    ),
                    'stop_at_cp1': ParameterValue(
                        stop_at_cp1,
                        value_type=bool,
                    ),
                },
            ],
        ),
        Node(
            package='robot_perception',
            executable='safety_arbiter_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    config_directory,
                    'safety_arbiter.yaml',
                ]),
                {
                    'enabled': ParameterValue(
                        enable_motion,
                        value_type=bool,
                    ),
                },
            ],
        ),
        Node(
            package='robot_perception',
            executable='cmd_vel_watchdog_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                config_directory,
                'cmd_vel_watchdog.yaml',
            ])],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
