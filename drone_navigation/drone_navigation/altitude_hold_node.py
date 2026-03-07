#!/usr/bin/env python3
"""
altitude_hold_node.py — Keeps the drone at a target altitude.

Reads real 3D position from Gazebo transport, uses PID to hold altitude.
Accepts dynamic target altitude from /mission/target_altitude topic
so the mission_node can command takeoff (1.2m) and landing (0.08m).

Publishes correction on /safety/cmd_vel for the cmd_vel_mux.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import subprocess
import threading
import re


class AltitudeHoldNode(Node):

    def __init__(self):
        super().__init__('altitude_hold_node')

        # Parameters
        self.declare_parameter('target_altitude', 0.0)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.8)
        self.declare_parameter('max_vz', 1.5)

        self.target_alt = self.get_parameter('target_altitude').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_vz = self.get_parameter('max_vz').value

        self.current_alt = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.got_pose = False
        self.log_count = 0

        # Subscribe to dynamic altitude target from mission node
        self.target_sub = self.create_subscription(
            Float64, '/mission/target_altitude', self.target_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/safety/cmd_vel', 10)

        # Background thread for Gazebo pose
        self._running = True
        self._gz_thread = threading.Thread(
            target=self._gz_pose_reader, daemon=True)
        self._gz_thread.start()

        # Run PID at 20 Hz
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'AltitudeHoldNode started — target: {self.target_alt}m')

    def target_callback(self, msg: Float64):
        old = self.target_alt
        self.target_alt = msg.data
        # Reset integral when target changes to avoid windup
        self.integral = 0.0
        self.get_logger().info(
            f'Target altitude changed: {old:.2f}m -> {self.target_alt:.2f}m')

    def _gz_pose_reader(self):
        """Background thread: reads drone pose from Gazebo."""
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
                            z = float(match.group(1))
                            self.current_alt = z
                            if not self.got_pose:
                                self.got_pose = True
                                self.get_logger().info(
                                    f'First Gazebo pose — altitude: {z:.2f}m')
        except Exception as e:
            self.get_logger().error(f'Gazebo pose reader error: {e}')

    def control_loop(self):
        # If target is 0 (landed/idle), don't command anything
        if self.target_alt <= 0.01:
            msg = Twist()
            msg.linear.z = 0.0
            self.cmd_pub.publish(msg)
            return

        error = self.target_alt - self.current_alt
        self.integral += error * self.dt
        self.integral = max(-2.0, min(2.0, self.integral))
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        vz = (self.kp * error +
              self.ki * self.integral +
              self.kd * derivative)
        vz = max(-self.max_vz, min(self.max_vz, vz))

        msg = Twist()
        msg.linear.z = vz
        self.cmd_pub.publish(msg)

        # Log every 2 seconds
        self.log_count += 1
        if self.log_count % 40 == 0:
            self.get_logger().info(
                f'Alt: {self.current_alt:.2f}m | Target: {self.target_alt}m | '
                f'Error: {error:.2f} | Vz: {vz:.2f} | Pose: {self.got_pose}')

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AltitudeHoldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()