#!/usr/bin/env python3
"""
takeoff_node.py — Automated Takeoff Sequence

Handles: ENABLE → TAKEOFF → HOVER → YIELD
Publishes to /takeoff/cmd_vel.
The cmd_vel_mux routes it to /cmd_vel with proper priority.

After yielding, Nav2 + safety_monitor take over via the mux.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class TakeoffNode(Node):

    def __init__(self):
        super().__init__('takeoff_node')

        self.set_parameters([Parameter(
            'use_sim_time', Parameter.Type.BOOL, True
        )])

        # Publish to mux input, NOT directly to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/takeoff/cmd_vel', 10
        )
        self.enable_pub = self.create_publisher(
            Bool, '/drone/enable', 10
        )

        # State machine
        self.state = 'ENABLE'
        self.takeoff_speed = 0.3        # m/s upward
        self.takeoff_duration = 4.0     # seconds
        self.hover_duration = 2.0       # seconds to stabilise

        self.state_timer = 0.0
        self.dt = 0.05  # 20 Hz

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('TakeoffNode started — state: ENABLE')

    def control_loop(self):
        self.state_timer += self.dt

        if self.state == 'ENABLE':
            self.do_enable()
        elif self.state == 'TAKEOFF':
            self.do_takeoff()
        elif self.state == 'HOVER':
            self.do_hover()
        elif self.state == 'YIELD':
            self.do_yield()

    def change_state(self, new_state):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state
        self.state_timer = 0.0

    def publish_velocity(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.cmd_vel_pub.publish(msg)

    def do_enable(self):
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)
        if self.state_timer >= 1.0:
            self.change_state('TAKEOFF')

    def do_takeoff(self):
        self.publish_velocity(vz=self.takeoff_speed)
        if self.state_timer >= self.takeoff_duration:
            self.change_state('HOVER')

    def do_hover(self):
        self.publish_velocity()
        if self.state_timer >= self.hover_duration:
            self.change_state('YIELD')

    def do_yield(self):
        """Stop publishing — mux will see takeoff as stale and use Nav2."""
        self.get_logger().info(
            'Takeoff complete. Yielding control to Nav2 + SafetyMonitor.'
        )
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()