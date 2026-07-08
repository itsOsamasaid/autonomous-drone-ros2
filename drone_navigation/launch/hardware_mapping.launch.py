"""
hardware_mapping.launch.py — Real-drone SLAM mapping (Phase 9).

Brings up, in one shot:
  1. RPLIDAR C1 driver        -> /scan
  2. base_link -> laser  TF   (real measured mount: centered, 8 cm above FC, 0deg = nose)
  3. odom -> base_link   TF   (IDENTITY — BENCH ONLY; replace with MAVROS odom for flight)
  4. slam_toolbox (async)     -> /map  + map->odom

Run on the Pi:
    ros2 launch drone_navigation hardware_mapping.launch.py

Visualize on the LAPTOP with rviz2 (Fixed Frame = map).

NOTE: the odom->base_link identity transform is only valid while the drone is
hand-carried on the bench. For flight, this must be replaced by real odometry
(odom->base_link from MAVROS / EKF3), or slam will fight the flight controller.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_navigation')
    slam_config = os.path.join(pkg_dir, 'config', 'slam.yaml')

    sllidar_dir = get_package_share_directory('sllidar_ros2')
    slam_dir = get_package_share_directory('slam_toolbox')

    return LaunchDescription([
        # 1. RPLIDAR C1 -> /scan
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar_dir, 'launch', 'sllidar_c1_launch.py')
            )
        ),

        # 2. base_link -> laser  (x0 y0 z0.08; LiDAR yaw offset ~pi/2 vs body nose)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['--x', '0', '--y', '0', '--z', '0.08',
                       '--roll', '0', '--pitch', '0', '--yaw', '-1.5708',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),

        # 3. odom -> base_link  (IDENTITY — BENCH ONLY, remove for flight)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_bench',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'odom', '--child-frame-id', 'base_link'],
        ),

        # 4. slam_toolbox (async) -> /map + map->odom
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_config,
                'use_sim_time': 'false',
            }.items()
        ),
    ])
