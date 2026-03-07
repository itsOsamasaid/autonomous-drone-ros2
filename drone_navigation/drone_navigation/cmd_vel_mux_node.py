#!/usr/bin/env python3
"""
cmd_vel_mux_node.py — Velocity Command Multiplexer

Merges velocity commands from multiple sources:
    1. Takeoff          (/takeoff/cmd_vel) — full 3D control during takeoff
    2. Nav2 + Altitude  (/nav2/cmd_vel + /safety/cmd_vel) — normal navigation

The key insight: Nav2 provides x, y, yaw (2D). AltitudeHold provides z.
This node MERGES them so the drone navigates while holding altitude.

Publishes:
    /cmd_vel — final merged command to Gazebo (via bridge)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class CmdVelMuxNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_mux')

        # Latest commands from each source
        self.nav2_cmd = Twist()
        self.takeoff_cmd = Twist()
        self.altitude_cmd = Twist()

        # Timestamps (wall clock for timeout detection)
        self.nav2_last = 0.0
        self.takeoff_last = 0.0
        self.altitude_last = 0.0
        self.pos_hold_cmd = Twist()
        self.pos_hold_last = 0.0

        self.cmd_timeout = 0.5  # seconds
        self.log_count = 0

        # Subscribers
        self.nav2_sub = self.create_subscription(
            Twist, '/nav2/cmd_vel', self.nav2_callback, 10)
        self.takeoff_sub = self.create_subscription(
            Twist, '/takeoff/cmd_vel', self.takeoff_callback, 10)
        self.altitude_sub = self.create_subscription(
            Twist, '/safety/cmd_vel', self.altitude_callback, 10)
        self.pos_hold_sub = self.create_subscription(
            Twist, '/position_hold/cmd_vel', self.pos_hold_callback, 10)

        # Publisher — goes to Gazebo bridge
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Merge loop at 20 Hz
        self.timer = self.create_timer(0.05, self.merge_and_publish)

        self.get_logger().info('CmdVelMux started — merging Nav2 + AltitudeHold')

    def nav2_callback(self, msg: Twist):
        self.nav2_cmd = msg
        self.nav2_last = time.monotonic()

    def takeoff_callback(self, msg: Twist):
        self.takeoff_cmd = msg
        self.takeoff_last = time.monotonic()

    def altitude_callback(self, msg: Twist):
        self.altitude_cmd = msg
        self.altitude_last = time.monotonic()


    def pos_hold_callback(self, msg: Twist):
        self.pos_hold_cmd = msg
        self.pos_hold_last = time.monotonic()

    def merge_and_publish(self):
        now = time.monotonic()
        output = Twist()

        takeoff_active = (now - self.takeoff_last) < self.cmd_timeout
        nav2_active = (now - self.nav2_last) < self.cmd_timeout
        altitude_active = (now - self.altitude_last) < self.cmd_timeout

        source = 'IDLE'

        # Priority 1: Takeoff has full 3D control
        if takeoff_active:
            output = self.takeoff_cmd
            source = 'TAKEOFF'
        else:
            # Priority 2: Nav2 horizontal commands
            if nav2_active:
                output.linear.x = self.nav2_cmd.linear.x
                output.linear.y = self.nav2_cmd.linear.y
                output.angular.z = self.nav2_cmd.angular.z
                source = 'NAV2+ALT'
            # Priority 3: Position hold (when Nav2 is not commanding)
            elif (now - self.pos_hold_last) < self.cmd_timeout:
                output.linear.x = self.pos_hold_cmd.linear.x
                output.linear.y = self.pos_hold_cmd.linear.y
                source = 'POS_HOLD'

            # Always apply altitude hold when available
            # (even without Nav2, drone should hover in place)
            if altitude_active:
                output.linear.z = self.altitude_cmd.linear.z
                if source == 'IDLE':
                    source = 'ALT_ONLY'

        if source != 'IDLE':
            self.cmd_pub.publish(output)

        # Log every 2 seconds
        self.log_count += 1
        if self.log_count % 40 == 0:
            self.get_logger().info(
                f'Source: {source} | '
                f'Vx: {output.linear.x:.2f} Vy: {output.linear.y:.2f} '
                f'Vz: {output.linear.z:.2f} Wz: {output.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
