"""hardware_navigation.launch.py — Nav2 autonomous navigation on real hardware.

Adds the Nav2 stack ON TOP of hardware_flight. Run hardware_flight FIRST
(MAVROS + LiDAR + SLAM localization + vision bridge + tilt filter + Evo) so
map->odom, /scan_filtered and the EKF pose already exist; this launch only
brings up planning/control:

    Nav2 (planner/controller/costmaps) -> /cmd_vel
      -> cmd_vel_to_mavros -> /mavros/setpoint_velocity/cmd_vel
        -> ArduPilot GUIDED velocity control

PREREQUISITES (do NOT expect this to fly otherwise):
  1. hardware_flight running and localized (map->odom valid).
  2. Nav2 installed:  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
  3. Position hold actually works in GUIDED (velocity setpoints hold a spot).
  4. Pilot armed + in GUIDED; thumb on the kill switch.

Send a goal with the RViz "2D Goal Pose" tool (topic /goal_pose). The mission
node arms-gated: publish the goal, then ARM in GUIDED and it takes off, runs
Nav2, and lands. Costmaps use /scan_filtered.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_navigation')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_hardware.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    cruise_alt = LaunchConfiguration('cruise_alt')

    nav2_nodes = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ]
    lifecycle_names = [n for _, n in nav2_nodes]

    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'cruise_alt', default_value='0.5',
            description='Takeoff/hover height (m, above takeoff spot) for 2D nav'),

        # Nav2 core nodes (params from nav2_hardware.yaml: sim_time off, /scan_filtered).
        # bt_navigator's NavigateToPoseNavigator subscribes to /goal_pose itself and
        # auto-forwards it to navigate_to_pose — that would bypass the mission node's
        # arm/takeoff gate. Remap its built-in sub to a dead topic so the mission node
        # is the ONLY /goal_pose consumer; Nav2 is driven via the action after takeoff.
        *[
            Node(package=pkg, executable=exe, name=exe, output='screen',
                 parameters=[nav2_params, {'use_sim_time': use_sim_time}],
                 remappings=([('/goal_pose', '/_nav2_goal_pose_disabled')]
                             if exe == 'bt_navigator' else []))
            for pkg, exe in nav2_nodes
        ],

        # Mission node: /goal_pose -> guided takeoff -> Nav2 XY (rotated to ENU)
        # + Z-hold -> /mavros/setpoint_velocity/cmd_vel -> land. Replaces
        # cmd_vel_to_mavros (it owns the velocity setpoint stream, adds takeoff,
        # altitude hold and the body->ENU rotation Nav2's /cmd_vel needs).
        Node(package='drone_navigation', executable='hardware_mission_node.py',
             output='screen',
             parameters=[{'cruise_alt': ParameterValue(cruise_alt, value_type=float)}]),

        # lifecycle manager: configure+activate the Nav2 nodes
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_names,
            }],
        ),
    ]
    return LaunchDescription(actions)
