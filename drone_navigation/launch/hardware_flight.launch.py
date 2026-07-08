import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            EmitEvent, RegisterEventHandler, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_navigation')
    mavros_dir = get_package_share_directory('mavros')
    sllidar_dir = get_package_share_directory('sllidar_ros2')

    fcu_url = LaunchConfiguration('fcu_url')
    map_file = LaunchConfiguration('map_file')
    evo_port = LaunchConfiguration('evo_port')

    # slam_toolbox localization is a lifecycle node — it must be configured+activated
    # or it never subscribes to /scan. Declared here so the events can reference it.
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'slam_localization.yaml'),
            {'map_file_name': map_file, 'use_sim_time': False},
        ],
    )
    slam_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))
    slam_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[
            LogInfo(msg='slam_toolbox localization activating'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam),
                transition_id=Transition.TRANSITION_ACTIVATE)),
        ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url', default_value='serial:///dev/ttyAMA0:921600'),
        DeclareLaunchArgument(
            'map_file', default_value=os.path.expanduser('~/maps/lab')),
        DeclareLaunchArgument(
            'evo_port',
            default_value='/dev/serial/by-id/usb-Terabee_TeraRanger_Evo_Mini_357E37443437-if00'),

        # 1. MAVROS <-> ArduPilot
        Node(
            package='mavros', executable='mavros_node', namespace='mavros',
            parameters=[
                os.path.join(mavros_dir, 'launch', 'apm_config.yaml'),
                os.path.join(pkg_dir, 'config', 'mavros.yaml'),
                {'fcu_url': fcu_url, 'tgt_system': 1, 'tgt_component': 1},
            ],
            output='screen',
        ),

        # 2. RPLIDAR C1 -> /scan
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(sllidar_dir, 'launch', 'sllidar_c1_launch.py'))),

        # 3. base_link -> laser (measured mount). LiDAR yaw offset ~pi/2 vs body nose.
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
            arguments=['--x', '0', '--y', '0', '--z', '0.08',
                       '--roll', '0', '--pitch', '0', '--yaw', '-1.5708',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),

        # 4. odom -> base_link IDENTITY. External-nav: slam owns the pose (map->odom) and
        # the EKF is fed it as vision. A static identity lets slam start before the EKF has
        # a solution, breaking the odom<->vision bootstrap deadlock. All motion lands in
        # map->odom (not ideal for Nav2 local planning; fine for EKF/pose bring-up).
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='odom_to_base_link',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'odom', '--child-frame-id', 'base_link'],
        ),

        # 5. slam_toolbox localization vs saved map -> map->odom (auto configure+activate)
        slam,
        slam_configure,
        slam_activate,

        # 6. map -> base_link pose -> EKF (VISION_POSITION_ESTIMATE)
        Node(
            package='drone_navigation', executable='vision_pose_bridge.py', output='screen'),

        # 7. Evo Mini -> DISTANCE_SENSOR (EKF height)
        Node(
            package='drone_navigation', executable='evo_mini_node.py',
            parameters=[{'port': evo_port}], output='screen'),
    ])
