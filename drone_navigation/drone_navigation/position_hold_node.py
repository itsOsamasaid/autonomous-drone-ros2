#!/usr/bin/env python3
"""
position_hold_node.py — Holds the drone's X/Y position when idle.

Reads real 3D position from Gazebo dynamic_pose/info (same as
altitude_hold_node), applies PID to maintain X/Y when no Nav2
commands are active.

Publishes to /position_hold/cmd_vel. The cmd_vel_mux merges this
with Nav2 and altitude hold.

When Nav2 sends commands, the mux prioritizes Nav2 over position hold.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import threading
import re


class PositionHoldNode(Node):

    def __init__(self):
        super().__init__('position_hold_node')

        self.declare_parameter('kp', 1.5)
        self.declare_parameter('kd', 0.6)
        self.declare_parameter('max_vel', 0.3)

        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.max_vel = self.get_parameter('max_vel').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.target_x = None
        self.target_y = None
        self.got_pose = False
        self.log_count = 0

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        # Listen to Nav2 commands to know when to update hold target
        self.nav2_cmd = Twist()
        self.nav2_active = False
        self.nav2_last = 0.0
        self.nav2_sub = self.create_subscription(
            Twist, '/nav2/cmd_vel', self.nav2_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/position_hold/cmd_vel', 10)

        # Background thread for Gazebo pose
        self._running = True
        self._gz_thread = threading.Thread(target=self._gz_pose_reader, daemon=True)
        self._gz_thread.start()

        # Run PD at 20 Hz
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('PositionHoldNode started')

    def nav2_callback(self, msg: Twist):
        self.nav2_cmd = msg
        import time
        self.nav2_last = time.monotonic()

    def _gz_pose_reader(self):
        """Background thread: continuously reads drone pose from Gazebo."""
        try:
            proc = subprocess.Popen(
                ['gz', 'topic', '-e', '-t', '/world/drone_world/dynamic_pose/info'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True
            )

            in_mini_drone = False
            in_position = False
            pos_x = None
            pos_y = None

            for line in proc.stdout:
                if not self._running:
                    proc.terminate()
                    break

                line = line.strip()

                if 'name: "mini_drone"' in line:
                    in_mini_drone = True
                    in_position = False
                    pos_x = None
                    pos_y = None
                    continue

                if in_mini_drone:
                    if line == 'position {':
                        in_position = True
                        continue
                    if in_position:
                        if line == '}':
                            in_position = False
                            in_mini_drone = False
                            if pos_x is not None:
                                self.current_x = pos_x
                            if pos_y is not None:
                                self.current_y = pos_y
                            if not self.got_pose and pos_x is not None:
                                self.got_pose = True
                                self.target_x = pos_x
                                self.target_y = pos_y
                                self.get_logger().info(
                                    f'First pose — hold target: ({pos_x:.2f}, {pos_y:.2f})')
                            continue

                        match_x = re.match(r'x:\s*([-\d.e+]+)', line)
                        if match_x:
                            pos_x = float(match_x.group(1))
                        match_y = re.match(r'y:\s*([-\d.e+]+)', line)
                        if match_y:
                            pos_y = float(match_y.group(1))

        except Exception as e:
            self.get_logger().error(f'Gazebo pose reader error: {e}')

    def control_loop(self):
        if not self.got_pose or self.target_x is None:
            return

        import time
        now = time.monotonic()

        # If Nav2 is actively commanding, update hold target to current pos
        # so when Nav2 stops, we hold wherever we end up
        nav2_active = (now - self.nav2_last) < 0.5
        if nav2_active:
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.prev_error_x = 0.0
            self.prev_error_y = 0.0
            # Don't publish — let Nav2 handle it
            return

        # PD controller for X/Y
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y

        deriv_x = (error_x - self.prev_error_x) / self.dt
        deriv_y = (error_y - self.prev_error_y) / self.dt
        self.prev_error_x = error_x
        self.prev_error_y = error_y

        vx = self.kp * error_x + self.kd * deriv_x
        vy = self.kp * error_y + self.kd * deriv_y

        # Clamp
        vx = max(-self.max_vel, min(self.max_vel, vx))
        vy = max(-self.max_vel, min(self.max_vel, vy))

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        self.cmd_pub.publish(msg)

        # Log every 4 seconds
        self.log_count += 1
        if self.log_count % 80 == 0:
            dist = (error_x**2 + error_y**2)**0.5
            self.get_logger().info(
                f'Hold ({self.target_x:.2f},{self.target_y:.2f}) | '
                f'At ({self.current_x:.2f},{self.current_y:.2f}) | '
                f'Drift: {dist:.3f}m')

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PositionHoldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()