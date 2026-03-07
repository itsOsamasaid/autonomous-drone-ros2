#!/usr/bin/env python3
"""
mission_node.py — Manages the full flight mission for navigation.

Sequence when a goal is received:
  1. IDLE (drone on ground)
  2. Goal received → TAKEOFF (climb to cruise altitude)
  3. At altitude → NAVIGATE (Nav2 takes over, flies to goal)
  4. Goal reached → LAND (descend to ground)
  5. On ground → IDLE (wait for next goal)

Subscribes to:
  /goal_pose (from RViz "2D Nav Goal")

Publishes to:
  /altitude_hold/target (Float64 — sets target altitude)
  /navigate_to_pose action client
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from nav2_msgs.action import NavigateToPose
import subprocess
import threading
import re
import time


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('cruise_altitude', 1.2)
        self.declare_parameter('land_altitude', 0.08)
        self.declare_parameter('takeoff_speed', 0.4)
        self.declare_parameter('land_speed', 0.2)

        self.cruise_alt = self.get_parameter('cruise_altitude').value
        self.land_alt = self.get_parameter('land_altitude').value
        self.takeoff_speed = self.get_parameter('takeoff_speed').value
        self.land_speed = self.get_parameter('land_speed').value

        self.state = 'IDLE'
        self.current_alt = 0.0
        self.got_pose = False
        self.pending_goal = None

        # Subscribe to RViz goal
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Action client for Nav2
        self.nav2_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Publisher for direct altitude control during takeoff/landing
        self.alt_cmd_pub = self.create_publisher(
            Float64, '/mission/target_altitude', 10)

        # Background thread for Gazebo pose (same as altitude_hold)
        self._running = True
        self._gz_thread = threading.Thread(
            target=self._gz_pose_reader, daemon=True)
        self._gz_thread.start()

        # State machine at 10 Hz
        self.timer = self.create_timer(0.1, self.state_machine)

        self.get_logger().info('MissionNode started — waiting for goals')

    def _gz_pose_reader(self):
        """Read real altitude from Gazebo."""
        try:
            proc = subprocess.Popen(
                ['gz', 'topic', '-e', '-t',
                 '/world/drone_world/dynamic_pose/info'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True
            )
            in_mini_drone = False
            in_position = False

            for line in proc.stdout:
                if not self._running:
                    proc.terminate()
                    break
                line = line.strip()
                if 'name: "mini_drone"' in line:
                    in_mini_drone = True
                    in_position = False
                    continue
                if in_mini_drone:
                    if line == 'position {':
                        in_position = True
                        continue
                    if in_position:
                        if line == '}':
                            in_position = False
                            in_mini_drone = False
                            continue
                        match = re.match(r'z:\s*([-\d.e+]+)', line)
                        if match:
                            self.current_alt = float(match.group(1))
                            if not self.got_pose:
                                self.got_pose = True
        except Exception as e:
            self.get_logger().error(f'Gazebo pose reader error: {e}')

    def goal_callback(self, msg: PoseStamped):
        if self.state != 'IDLE' and self.state != 'LANDED':
            self.get_logger().warn(
                f'Goal received but drone is busy (state: {self.state}). Ignoring.')
            return

        self.pending_goal = msg
        self.state = 'TAKEOFF'
        self.get_logger().info(
            f'Goal received at ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}). Taking off...')

        # Tell altitude hold to go to cruise altitude
        alt_msg = Float64()
        alt_msg.data = self.cruise_alt
        self.alt_cmd_pub.publish(alt_msg)

    def state_machine(self):
        if self.state == 'IDLE' or self.state == 'LANDED':
            return

        elif self.state == 'TAKEOFF':
            # Wait until drone reaches cruise altitude
            if self.current_alt >= self.cruise_alt * 0.85:
                self.get_logger().info(
                    f'At cruise altitude ({self.current_alt:.2f}m). '
                    f'Starting navigation...')
                self.state = 'NAVIGATE'
                self.send_nav2_goal()

        elif self.state == 'NAVIGATE':
            # Waiting for Nav2 to finish (callback handles transition)
            pass

        elif self.state == 'DESCEND':
            # Wait until drone is near ground
            if self.current_alt <= self.land_alt + 0.05:
                self.get_logger().info('Landed. Ready for next goal.')
                self.state = 'IDLE'
                # Set altitude target to 0 to stay on ground
                alt_msg = Float64()
                alt_msg.data = 0.0
                self.alt_cmd_pub.publish(alt_msg)

    def send_nav2_goal(self):
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            self.state = 'IDLE'
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.pending_goal

        self.get_logger().info('Sending goal to Nav2...')
        self._send_goal_future = self.nav2_client.send_goal_async(
            goal_msg, feedback_callback=self.nav2_feedback)
        self._send_goal_future.add_done_callback(self.nav2_goal_response)

    def nav2_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal!')
            self.state = 'DESCEND'
            alt_msg = Float64()
            alt_msg.data = self.land_alt
            self.alt_cmd_pub.publish(alt_msg)
            return

        self.get_logger().info('Nav2 accepted goal. Flying to target...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav2_result)

    def nav2_feedback(self, feedback_msg):
        # Could log progress here if desired
        pass

    def nav2_result(self, future):
        result = future.result()
        self.get_logger().info('Nav2 goal reached! Landing...')
        self.state = 'DESCEND'

        # Tell altitude hold to descend
        alt_msg = Float64()
        alt_msg.data = self.land_alt
        self.alt_cmd_pub.publish(alt_msg)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()