import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_navigation')

    world_file = os.path.join(pkg_dir, 'worlds', 'simple_world.sdf')
    models_path = os.path.join(pkg_dir, 'models')
    map_file = os.path.join(pkg_dir, 'maps', 'map.yaml')
    nav2_yaml = os.path.join(pkg_dir, 'config', 'nav2.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'drone_nav.rviz')

    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if gz_resource_path:
        full_resource_path = models_path + ':' + gz_resource_path
    else:
        full_resource_path = models_path

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=full_resource_path
        ),

        #  GAZEBO + BRIDGE

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                '/mini_drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/mini_drone/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
                '/model/mini_drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            remappings=[
                ('/mini_drone/cmd_vel', '/cmd_vel'),
                ('/mini_drone/enable', '/drone/enable'),
                ('/camera', '/camera/image_raw'),
                ('/model/mini_drone/odometry', '/odom'),
            ],
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        #  TF TREE

        Node(
            package='drone_navigation',
            executable='odom_tf_node.py',
            name='odom_tf_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar',
            arguments=['--x', '0', '--y', '0', '--z', '0.06',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'lidar_link'],
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['--x', '0.15', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'camera_link'],
            parameters=[{'use_sim_time': True}],
        ),

        #  DRONE CONTROL NODES

        # Altitude hold — target starts at 0 (ground), mission_node
        # changes it to 1.2 on goal, back to 0.08 on landing
        Node(
            package='drone_navigation',
            executable='altitude_hold_node.py',
            name='altitude_hold_node',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'target_altitude': 0.0},
                        {'kp': 2.0},
                        {'ki': 0.1},
                        {'kd': 0.8}],
        ),

        # Position hold
        Node(
            package='drone_navigation',
            executable='position_hold_node.py',
            name='position_hold_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Cmd vel mux
        Node(
            package='drone_navigation',
            executable='cmd_vel_mux_node.py',
            name='cmd_vel_mux',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Mission manager — handles takeoff → navigate → land
        Node(
            package='drone_navigation',
            executable='mission_node.py',
            name='mission_node',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'cruise_altitude': 1.2},
                        {'land_altitude': 0.08}],
        ),

        #  NAV2 STACK
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_yaml,
                        {'use_sim_time': True},
                        {'yaml_filename': map_file}],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}],
            remappings=[('cmd_vel', '/nav2/cmd_vel')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'behavior_server',
                                        'bt_navigator']}],
        ),

        #  STARTUP + RVIZ
        # Enable drone motors early (drone stays on ground until goal)
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '/drone/enable',
                         'std_msgs/msg/Bool', '{data: true}', '--once'],
                    output='screen'
                ),
            ]
        ),

        # Initial pose for AMCL
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '--once',
                        '/initialpose',
                        'geometry_msgs/msg/PoseWithCovarianceStamped',
                        '{header: {frame_id: map}, pose: {pose: '
                        '{position: {x: 0.0, y: 0.0, z: 0.0}, '
                        'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, '
                        'covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, '
                        '0.0, 0.25, 0.0, 0.0, 0.0, 0.0, '
                        '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                        '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                        '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                        '0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]}}'
                    ],
                    output='screen'
                ),
            ]
        ),

        # RViz with preconfigured displays
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen',
                ),
            ]
        ),
    ])