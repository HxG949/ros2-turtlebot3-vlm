from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('robot_perception')
    config_directory = PathJoinSubstitution([package_share, 'config'])
    enable_motion = LaunchConfiguration('enable_motion')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_motion',
            default_value='false',
            description='Allow non-zero velocity during the parking mission',
        ),
        Node(
            package='robot_perception',
            executable='parking_decision_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                config_directory,
                'parking_decision.yaml',
            ])],
        ),
        Node(
            package='robot_perception',
            executable='motion_controller_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    config_directory,
                    'motion_controller.yaml',
                ]),
                {
                    'enabled': ParameterValue(
                        enable_motion,
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
