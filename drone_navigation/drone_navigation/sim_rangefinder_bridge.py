#!/usr/bin/env python3
# gz single-ray LaserScan -> Range on /mavros/rangefinder_sub (MAVROS forwards as DISTANCE_SENSOR)

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


class SimRangefinderBridge(Node):

    def __init__(self):
        super().__init__('sim_rangefinder_bridge')

        self.declare_parameter('input_topic', '/rangefinder_scan')
        self.declare_parameter('output_topic', '/mavros/rangefinder_sub')
        self.declare_parameter('frame_id', 'evo_mini')
        self.declare_parameter('min_range', 0.03)
        self.declare_parameter('max_range', 3.3)

        self.frame_id = self.get_parameter('frame_id').value
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.pub = self.create_publisher(
            Range, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('input_topic').value,
            self.scan_cb, qos_profile_sensor_data)

        self.get_logger().info('sim rangefinder bridge -> /mavros/rangefinder_sub')

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            return
        r = msg.ranges[0]

        # gz reports inf/nan past range; report out-of-range instead of dropping (keeps FC's sensor-alive check happy)
        if not math.isfinite(r) or r > self.max_range:
            r = self.max_range + 0.01
        elif r < self.min_range:
            r = self.min_range - 0.01

        out = Range()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.radiation_type = Range.INFRARED
        out.field_of_view = 0.035
        out.min_range = self.min_range
        out.max_range = self.max_range
        out.range = float(r)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimRangefinderBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
