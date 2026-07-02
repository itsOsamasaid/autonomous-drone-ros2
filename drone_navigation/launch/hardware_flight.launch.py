import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_navigation')
    mavros_dir = get_package_share_directory('mavros')
    sllidar_dir = get_package_share_directory('sllidar_ros2')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url', default_value='serial:///dev/ttyAMA0:921600'),
        DeclareLaunchArgument(
            'map_file', default_value=os.path.expanduser('~/maps/lab')),
        DeclareLaunchArgument('evo_port', default_value='/dev/ttyACM0'),

        # 1. MAVROS <-> ArduPilot (TELEM1)
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            parameters=[
                os.path.join(mavros_dir, 'launch', 'apm_config.yaml'),
                os.path.join(pkg_dir, 'config', 'mavros.yaml'),
                {'fcu_url': LaunchConfiguration('fcu_url'),
                 'tgt_system': 1, 'tgt_component': 1},
            ],
            output='screen',
        ),

        # 2. RPLIDAR C1 -> /scan
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar_dir, 'launch', 'sllidar_c1_launch.py')
            )
        ),

        # 3. base_link -> laser (measured mount)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['--x', '0', '--y', '0', '--z', '0.08',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),

        # 4. odom -> base_link from EKF3 (replaces the bench identity TF)
        Node(
            package='drone_navigation',
            executable='odom_tf_node.py',
            remappings=[('/odom', '/mavros/local_position/odom')],
            output='screen',
        ),

        # 5. slam_toolbox localization vs saved map -> map->odom
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.join(pkg_dir, 'config', 'slam_localization.yaml'),
                {'map_file_name': LaunchConfiguration('map_file'),
                 'use_sim_time': False},
            ],
            output='screen',
        ),

        # 6. map -> base_link pose -> EKF3 (VISION_POSITION_ESTIMATE)
        Node(
            package='drone_navigation',
            executable='vision_pose_bridge.py',
            output='screen',
        ),

        # 7. Evo Mini -> DISTANCE_SENSOR (EKF3 height)
        Node(
            package='drone_navigation',
            executable='evo_mini_node.py',
            parameters=[{'port': LaunchConfiguration('evo_port')}],
            output='screen',
        ),
    ])
