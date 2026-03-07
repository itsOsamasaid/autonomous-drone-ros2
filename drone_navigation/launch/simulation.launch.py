import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_navigation')

    world_file = os.path.join(pkg_dir, 'worlds', 'simple_world.sdf')
    models_path = os.path.join(pkg_dir, 'models')
    slam_config = os.path.join(pkg_dir, 'config', 'slam.yaml')

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

        # gazebo simulation
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # ROS-Gazebo bridge
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
                '/model/mini_drone/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',

            ],
            remappings=[
                ('/mini_drone/cmd_vel', '/cmd_vel'),
                ('/mini_drone/enable', '/drone/enable'),
                ('/camera', '/camera/image_raw'),
                ('/model/mini_drone/odometry', '/odom'),
                ('/model/mini_drone/pose', '/drone/pose'),

            ],
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # TF: odom → base_link 
        Node(
            package='drone_navigation',
            executable='odom_tf_node.py',
            name='odom_tf_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Static Transforms 
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

        # Drone Control (takeoff + altitude hold + mux)
        TimerAction(
            period=14.0,
            actions=[
                Node(
                    package='drone_navigation',
                    executable='altitude_hold_node.py',
                    name='altitude_hold_node',
                    output='screen',
                    parameters=[{'use_sim_time': True},
                                {'target_altitude': 1.2}],
                ),
            ]
        ),
        TimerAction(
            period=14.0,
            actions=[
                Node(
                    package='drone_navigation',
                    executable='position_hold_node.py',
                    name='position_hold_node',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
            ]
        ),

        Node(
            package='drone_navigation',
            executable='cmd_vel_mux_node.py',
            name='cmd_vel_mux',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config, {'use_sim_time': True}],
            output='screen',
        ),
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                    output='screen'
                ),
            ]
        ),
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                ),
            ]
        ),

        # Enable + takeoff after Gazebo is ready
        TimerAction(
            period=13.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '/drone/enable',
                         'std_msgs/msg/Bool', '{data: true}', '--once'],
                    output='screen'
                ),
            ]
        ),

    ])
