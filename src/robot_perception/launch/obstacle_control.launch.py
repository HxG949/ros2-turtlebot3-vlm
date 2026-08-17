from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('robot_perception')
    config_directory = PathJoinSubstitution([package_share, 'config'])
    enable_motion = LaunchConfiguration('enable_motion')
    stop_at_cp1 = LaunchConfiguration('stop_at_cp1')
    required_desired_subscribers = LaunchConfiguration(
        'required_desired_subscribers'
    )
    required_status_subscribers = LaunchConfiguration(
        'required_status_subscribers'
    )
    subscriber_confirmation_duration = LaunchConfiguration(
        'subscriber_confirmation_duration'
    )

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
            'required_desired_subscribers',
            default_value='1',
            description='Subscribers required before desired motion starts',
        ),
        DeclareLaunchArgument(
            'required_status_subscribers',
            default_value='0',
            description='Control-status subscribers required before motion',
        ),
        DeclareLaunchArgument(
            'subscriber_confirmation_duration',
            default_value='0.0',
            description='Seconds subscriber counts must remain sufficient',
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
                    'required_desired_subscriber_count': ParameterValue(
                        required_desired_subscribers,
                        value_type=int,
                    ),
                    'required_status_subscriber_count': ParameterValue(
                        required_status_subscribers,
                        value_type=int,
                    ),
                    'subscriber_confirmation_duration': ParameterValue(
                        subscriber_confirmation_duration,
                        value_type=float,
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
    ])
