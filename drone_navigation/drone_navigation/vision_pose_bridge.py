#!/usr/bin/env python3
# map -> base_link TF from slam_toolbox -> /mavros/vision_pose/pose -> EKF3 external nav

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from geographic_msgs.msg import GeoPointStamped
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

        # no GPS -> EKF origin must be set manually or arming fails 'waiting for home'
        self.declare_parameter('origin_lat', 47.3977)
        self.declare_parameter('origin_lon', 8.5456)
        self.declare_parameter('origin_alt', 488.0)
        self.origin_pub = self.create_publisher(
            GeoPointStamped, '/mavros/global_position/set_gp_origin', 10)
        self._origin_sent = 0

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
        # keep the transform's own timestamp, EKF3 fuses by time
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z
        msg.pose.orientation = tf.transform.rotation
        self.pose_pub.publish(msg)

        # set the EKF origin ~20 times once vision is up (clears 'waiting for home')
        if self._origin_sent < 20:
            o = GeoPointStamped()
            o.header.stamp = self.get_clock().now().to_msg()
            o.header.frame_id = self.map_frame
            o.position.latitude = self.get_parameter('origin_lat').value
            o.position.longitude = self.get_parameter('origin_lon').value
            o.position.altitude = self.get_parameter('origin_alt').value
            self.origin_pub.publish(o)
            self._origin_sent += 1
            if self._origin_sent == 20:
                self.get_logger().info('EKF origin set (home should be ready).')

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
