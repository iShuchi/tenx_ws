#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Just test the path tracking node without Gazebo
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Use real time for testing
        description='Use simulation time'
    )

    # Path tracking node configuration
    config_file = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'params.yaml'
    )

    path_tracking_node = Node(
        package='navigation',
        executable='path_tracking_node',
        name='path_tracking_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'global_frame': 'map',
                'robot_frame': 'base_footprint',
            }
        ],
        remappings=[
            ('path', '/plan'),
            ('cmd_vel', '/cmd_vel'),
            ('costmap', '/global_costmap/costmap'),
        ]
    )

    # Static transforms for testing
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        static_tf_map_to_odom,
        static_tf_odom_to_base,
        path_tracking_node,
    ])
