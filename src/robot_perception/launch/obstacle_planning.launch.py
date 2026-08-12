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
    use_rviz = LaunchConfiguration('use_rviz')
    parking_space_id = LaunchConfiguration('parking_space_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'parking_space_id',
            default_value='space_2',
            description='Parking space ID selected for planning',
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
            executable='parking_target_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    config_directory,
                    'parking_targets.yaml',
                ]),
                {
                    'selected_space_id': ParameterValue(
                        parking_space_id,
                        value_type=str,
                    ),
                },
            ],
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
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
