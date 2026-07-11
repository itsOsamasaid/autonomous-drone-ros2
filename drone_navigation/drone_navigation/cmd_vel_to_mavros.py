#!/usr/bin/env python3
"""cmd_vel_to_mavros.py — Nav2 /cmd_vel -> ArduPilot GUIDED velocity setpoints.

Nav2's controller publishes geometry_msgs/Twist on /cmd_vel (body frame:
+x forward, +z yaw). ArduPilot in GUIDED accepts velocity setpoints via MAVROS
on /mavros/setpoint_velocity/cmd_vel (TwistStamped). This node restamps and
forwards, and keeps a steady stream (GUIDED velocity control needs continuous
setpoints or it times out) by republishing the last command at a fixed rate,
zeroing it if Nav2 goes quiet.

SAFETY: this drives the drone from Nav2 goals. Only meaningful once position
hold works. Pilot override: leave GUIDED (mode switch) or kill (SA).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelToMavros(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_mavros')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('timeout', 0.5)   # zero cmd if no Nav2 for this long
        self.declare_parameter('frame_id', 'base_link')

        self.timeout = float(self.get_parameter('timeout').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.last_twist = Twist()
        self.last_rx = None

        self.pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd, 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value),
                          self._tick)
        self.get_logger().info(
            'cmd_vel_to_mavros: /cmd_vel -> /mavros/setpoint_velocity/cmd_vel')

    def _cmd(self, msg):
        self.last_twist = msg
        self.last_rx = self.get_clock().now()

    def _tick(self):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.frame_id
        stale = (self.last_rx is None or
                 (self.get_clock().now() - self.last_rx).nanoseconds
                 > self.timeout * 1e9)
        ts.twist = Twist() if stale else self.last_twist
        self.pub.publish(ts)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMavros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
