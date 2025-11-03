from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('navigation'),
            'config',
            'params.yaml'
        ]),
        description='Path to parameter config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    global_frame_arg = DeclareLaunchArgument(
        'global_frame',
        default_value='map',
        description='Global frame name'
    )

    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame',
        default_value='base_link',
        description='Robot base frame name'
    )

    # Path tracking node
    path_tracking_node = Node(
        package='navigation',
        executable='path_tracking_node',
        name='path_tracking_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'global_frame': LaunchConfiguration('global_frame'),
                'robot_frame': LaunchConfiguration('robot_frame'),
            }
        ],
        remappings=[
            ('path', '/plan'),
            ('cmd_vel', '/cmd_vel'),
            ('costmap', '/global_costmap/costmap'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        global_frame_arg,
        robot_frame_arg,
        path_tracking_node,
    ])

