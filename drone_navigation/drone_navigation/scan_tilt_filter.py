#!/usr/bin/env python3
"""scan_tilt_filter.py — drop LaserScans while the drone is tilted.

A 2D LiDAR on a multirotor gives a corrupted scan when the airframe tilts
(the horizontal scan plane tilts with it -> walls read at wrong ranges).
Feeding those to SLAM damages the map and jumps the pose. This node passes
/scan through to /scan_filtered ONLY when the FC attitude is near level;
past max_tilt_deg it drops the scan so SLAM holds its last good pose and the
EKF coasts on IMU for that brief moment.

If IMU is stale/absent it passes scans through (losing all localization is
worse than an occasional tilted scan).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu


class ScanTiltFilter(Node):

    def __init__(self):
        super().__init__('scan_tilt_filter')

        self.declare_parameter('input_scan', '/scan')
        self.declare_parameter('output_scan', '/scan_filtered')
        self.declare_parameter('imu_topic', '/mavros/imu/data')
        self.declare_parameter('max_tilt_deg', 8.0)
        self.declare_parameter('imu_timeout', 0.5)
        self.declare_parameter('pass_on_no_imu', True)

        self.max_tilt = math.radians(self.get_parameter('max_tilt_deg').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)
        self.pass_on_no_imu = bool(self.get_parameter('pass_on_no_imu').value)

        self.tilt = 0.0
        self.imu_stamp = None
        self._passed = 0
        self._dropped = 0

        self.pub = self.create_publisher(
            LaserScan, self.get_parameter('output_scan').value, 10)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.imu_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.get_parameter('input_scan').value, self.scan_cb,
            qos_profile_sensor_data)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'ScanTiltFilter: passing /scan when tilt < '
            f'{self.get_parameter("max_tilt_deg").value:.0f} deg')

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        # tilt = angle between body-Z and world-Z (combined roll+pitch)
        cos_t = max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
        self.tilt = math.acos(cos_t)
        self.imu_stamp = self.get_clock().now()

    def scan_cb(self, msg: LaserScan):
        fresh = (self.imu_stamp is not None and
                 (self.get_clock().now() - self.imu_stamp).nanoseconds
                 < self.imu_timeout * 1e9)
        if not fresh:
            if self.pass_on_no_imu:
                self.pub.publish(msg)
                self._passed += 1
            return
        if self.tilt <= self.max_tilt:
            self.pub.publish(msg)
            self._passed += 1
        else:
            self._dropped += 1

    def report(self):
        tot = self._passed + self._dropped
        if tot:
            self.get_logger().info(
                f'scans: {self._passed} passed, {self._dropped} dropped '
                f'({100*self._dropped/tot:.0f}% tilted), tilt now '
                f'{math.degrees(self.tilt):.0f} deg')
        self._passed = self._dropped = 0


def main(args=None):
    rclpy.init(args=args)
    node = ScanTiltFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
