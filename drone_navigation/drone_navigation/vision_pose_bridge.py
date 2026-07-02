#!/usr/bin/env python3
"""
vision_pose_bridge.py — SLAM pose -> ArduPilot EKF3 (Phase 11).

Looks up the SLAM pose (the map -> base_link transform produced by
slam_toolbox + odometry) and republishes it to /mavros/vision_pose/pose.
MAVROS then emits a VISION_POSITION_ESTIMATE MAVLink message, so ArduPilot's
EKF3 (configured with an external-nav source) fuses LiDAR-SLAM position + yaw
with the IMU and handles position/altitude hold itself.

This REPLACES the simulation's gz-topic pose hack and the custom
altitude_hold / position_hold PID nodes — on real hardware the flight
controller does the low-level control; our job is just perception -> EKF.

Subscribes to (via TF):
    map -> base_link   (from slam_toolbox + odom)

Publishes:
    /mavros/vision_pose/pose   (geometry_msgs/PoseStamped, ENU; MAVROS -> NED)

SAFETY: a bad / laggy / jumpy pose here = EKF divergence = flyaway.
Validate on the bench (Gate A) before ever arming in GUIDED.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)


class VisionPoseBridge(Node):

    def __init__(self):
        super().__init__('vision_pose_bridge')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate_hz', 30.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate = self.get_parameter('rate_hz').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', 10)

        self._warn_count = 0
        self._ok_logged = False
        self._last_stamp = None
        self.timer = self.create_timer(1.0 / rate, self.publish_pose)

        self.get_logger().info(
            f'VisionPoseBridge started — {self.map_frame} -> {self.base_frame} '
            f'-> /mavros/vision_pose/pose @ {rate:.0f} Hz')

    def publish_pose(self):
        try:
            # latest available transform (Time() = 0 means "latest")
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self._warn_count += 1
            if self._warn_count % 60 == 1:
                self.get_logger().warn(
                    f'No {self.map_frame}->{self.base_frame} transform yet '
                    f'(is slam_toolbox + TF running?)')
            return

        # don't feed EKF3 duplicate-stamped samples
        stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
        if stamp == self._last_stamp:
            return
        self._last_stamp = stamp

        msg = PoseStamped()
        # Keep the transform's own timestamp — EKF3 fuses by time, so the
        # stamp must reflect when the pose was actually valid (latency matters).
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z
        msg.pose.orientation = tf.transform.rotation
        self.pose_pub.publish(msg)

        if not self._ok_logged:
            self._ok_logged = True
            self.get_logger().info('Publishing vision pose to MAVROS.')


def main(args=None):
    rclpy.init(args=args)
    node = VisionPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
