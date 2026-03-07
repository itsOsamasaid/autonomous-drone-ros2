#!/usr/bin/env python3
"""
odom_tf_node.py — Publishes odom → base_link transform.

CRITICAL FOR DRONE NAV2: We publish the TF with z=0 so that
Nav2's 2D costmaps and planners work correctly. The actual 3D
position is still available on /odom for altitude_hold_node.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time


class OdomTfNode(Node):
    def __init__(self):
        super().__init__('odom_tf_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_stamp = None

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(
            'OdomTfNode started — publishing odom→base_link TF (z flattened for Nav2)')

    def odom_callback(self, msg: Odometry):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return

        current_stamp = Time.from_msg(msg.header.stamp)
        if self.last_stamp is not None and current_stamp <= self.last_stamp:
            return
        self.last_stamp = current_stamp

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # XY position from odometry (for Nav2 2D navigation)
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        # Flatten Z to 0 so Nav2 costmaps work on 2D plane
        t.transform.translation.z = 0.0

        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
