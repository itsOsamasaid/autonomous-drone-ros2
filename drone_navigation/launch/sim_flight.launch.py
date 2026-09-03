# SITL counterpart of hardware_flight.launch.py: same nodes, only /scan, /mavros/rangefinder_sub, fcu_url and use_sim_time differ

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _lifecycle_slam(node):
    """slam_toolbox is a lifecycle node — configure+activate or it never subscribes."""
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, start_state='configuring',
        goal_state='inactive',
        entities=[
            LogInfo(msg='slam_toolbox activating'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE)),
        ]))
    return [node, configure, activate]


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_navigation')
    mavros_dir = get_package_share_directory('mavros')

    fcu_url = LaunchConfiguration('fcu_url')
    slam_mode = LaunchConfiguration('slam_mode')
    map_file = LaunchConfiguration('map_file')
    vision_pose = LaunchConfiguration('vision_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')

    is_mapping = PythonExpression(["'", slam_mode, "' == 'mapping'"])
    sim_time = {'use_sim_time': use_sim_time}

    slam_map = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'slam.yaml'), sim_time],
    )
    slam_loc = LifecycleNode(
        package='slam_toolbox', executable='localization_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'slam_localization.yaml'),
                    {'map_file_name': map_file}, sim_time],
    )

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='udp://:14551@'),
        DeclareLaunchArgument('slam_mode', default_value='mapping',
                              description='mapping | localization'),
        DeclareLaunchArgument('map_file',
                              default_value=os.path.expanduser('~/maps/lab'),
                              description='saved .posegraph, no extension'),
        DeclareLaunchArgument('vision_pose', default_value='false',
                              description='feed SLAM pose to EKF3 (S4)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Gazebo drives the clock; true in sim'),

        # 1. MAVROS <-> ArduPilot SITL over UDP
        Node(
            package='mavros', executable='mavros_node', namespace='mavros',
            parameters=[
                os.path.join(mavros_dir, 'launch', 'apm_config.yaml'),
                os.path.join(pkg_dir, 'config', 'mavros.yaml'),
                {'fcu_url': fcu_url, 'tgt_system': 1, 'tgt_component': 1},
                sim_time,
            ],
            output='screen',
        ),

        # 2. gz -> ROS. /clock first, or nodes run on wall time and drop sim-stamped scans.
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='gz_sensor_bridge', output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/rangefinder@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
            remappings=[('/rangefinder', '/rangefinder_scan')],
        ),

        # 3. base_link -> laser, same measured mount as the real drone
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_laser', parameters=[sim_time],
            arguments=['--x', '0', '--y', '0', '--z', '0.08',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),

        # 4. odom -> base_link identity, same bootstrap as hardware (slam owns pose)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='odom_to_base_link', parameters=[sim_time],
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'odom', '--child-frame-id', 'base_link'],
        ),

        # 5. slam_toolbox -> /map + map->odom
        GroupAction(condition=IfCondition(is_mapping),
                    actions=_lifecycle_slam(slam_map)),
        GroupAction(condition=UnlessCondition(is_mapping),
                    actions=_lifecycle_slam(slam_loc)),

        # 6. Evo Mini equivalent -> DISTANCE_SENSOR
        Node(
            package='drone_navigation', executable='sim_rangefinder_bridge.py',
            output='screen', parameters=[sim_time],
        ),

        # 7. map->base_link pose -> EKF3 (S4; off by default)
        Node(
            package='drone_navigation', executable='vision_pose_bridge.py',
            output='screen', parameters=[sim_time],
            condition=IfCondition(vision_pose),
        ),
    ])
