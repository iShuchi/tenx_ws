from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Environment variables
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    # Package directories
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='worlds/empty.world',
        description='Gazebo world file name (use full path or worlds/ prefix)'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI client'
    )
    
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position of the robot'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y position of the robot'
    )
    
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.01',
        description='Initial z position of the robot'
    )
    
    # Robot description for robot_state_publisher
    # Read URDF file directly
    urdf_file = os.path.join(
        pkg_turtlebot3_description,
        'urdf',
        f'turtlebot3_{turtlebot3_model}.urdf'
    )
    
    # Read the URDF file content
    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()
    
    # Substitute namespace variable if present
    namespace = os.environ.get('TURTLEBOT3_NAMESPACE', '')
    robot_desc = robot_desc.replace('${namespace}', namespace)
    
    robot_description = {'robot_description': robot_desc}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Gazebo server (classic Gazebo)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_gazebo_ros,
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )
    
    # Gazebo client (GUI) - conditional launch
    # (IfCondition already imported at top)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_gazebo_ros,
                'launch',
                'gzclient.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # Spawn entity in Gazebo using gazebo_ros spawn_entity
    # Use a timer to ensure robot_state_publisher has started publishing robot_description
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_' + turtlebot3_model,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
            '-robot_namespace', ''
        ],
        output='screen'
    )
    
    # Path tracking node
    config_file = PathJoinSubstitution([
        FindPackageShare('navigation'),
        'config',
        'params.yaml'
    ])
    
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
    
    # Static transform from map to odom (for testing without Nav2)
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # Static transform from odom to base_footprint
    # Note: In production, this would come from robot odometry via gazebo_ros plugins
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        gui_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        static_tf_map_to_odom,
        static_tf_odom_to_base,
        path_tracking_node,
    ])
