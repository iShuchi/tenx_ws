#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Get turtlebot model
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    world = os.path.join(
        pkg_turtlebot3_gazebo,
        'worlds',
        'turtlebot3_world.world'
    )

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'factory': 'true',  # Explicitly enable GazeboRosFactory plugin
            'verbose': 'true'
        }.items()
    )

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn turtlebot with delay to ensure Gazebo service is ready
    urdf_path = os.path.join(
        pkg_turtlebot3_gazebo,
        'models',
        f'turtlebot3_{turtlebot3_model}',
        'model.sdf'
    )

    spawn_turtlebot_cmd = TimerAction(
        period=10.0,  # Wait 10 seconds for Gazebo to fully start and plugins to load
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', turtlebot3_model,
                    '-file', urdf_path,
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', '0.01'
                ],
                output='screen',
            )
        ]
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
