#!/usr/bin/env python3
"""TeraRanger Evo Mini (USB) -> /mavros/distance_sensor/rangefinder_sub.

MAVROS forwards this as DISTANCE_SENSOR, so ArduPilot sees a normal
MAVLink rangefinder (RNGFND1_TYPE=10) and EKF3 uses it for height.
"""

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

FRAME_HEADER = 0x54  # 'T'


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def command(*payload):
    return bytes(payload) + bytes([crc8(payload)])


CMD_BINARY_MODE = command(0x00, 0x11, 0x02)
CMD_SINGLE_PIXEL = command(0x00, 0x21, 0x01)


class EvoMiniNode(Node):

    def __init__(self):
        super().__init__('evo_mini_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('frame_id', 'evo_mini')
        self.declare_parameter('publish_rate_hz', 25.0)
        self.declare_parameter('min_range', 0.03)
        self.declare_parameter('max_range', 3.3)

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pub_period_ns = int(1e9 / self.get_parameter('publish_rate_hz').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)

        # MAVROS distance_sensor plugin subscribes at /mavros/<config-key>, no sub-namespace
        self.pub = self.create_publisher(
            Range, '/mavros/rangefinder_sub', 10)

        self.ser = None
        self.buf = bytearray()
        self.last_pub = self.get_clock().now()
        self.create_timer(0.01, self.read_serial)
        self.get_logger().info(f'EvoMiniNode started on {self.port}')

    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0)
            self.ser.write(CMD_BINARY_MODE)
            self.ser.write(CMD_SINGLE_PIXEL)
            self.ser.reset_input_buffer()
            self.buf.clear()
            self.get_logger().info('Evo Mini connected (binary, single-pixel).')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(f'Evo Mini open failed: {e}', throttle_duration_sec=5.0)

    def read_serial(self):
        if self.ser is None:
            self.open_serial()
            return
        try:
            self.buf += self.ser.read(256)
        except serial.SerialException:
            self.get_logger().error('Evo Mini serial error, reconnecting.')
            self.ser = None
            return

        while len(self.buf) >= 4:
            if self.buf[0] != FRAME_HEADER:
                del self.buf[0]
                continue
            frame = self.buf[:4]
            if crc8(frame[:3]) != frame[3]:
                del self.buf[0]
                continue
            del self.buf[:4]
            self.handle_range((frame[1] << 8) | frame[2])

    def handle_range(self, mm):
        # 0=too close, 1=invalid, 0xFFFF=too far — skip, EKF handles the gap
        if mm in (0x0000, 0x0001, 0xFFFF):
            return
        now = self.get_clock().now()
        if (now - self.last_pub).nanoseconds < self.pub_period_ns:
            return
        self.last_pub = now

        msg = Range()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.035
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = mm / 1000.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EvoMiniNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
