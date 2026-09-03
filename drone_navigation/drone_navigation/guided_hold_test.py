#!/usr/bin/env python3
# arm -> GUIDED -> takeoff -> hold current XY/alt, logging drift. Props on, this flies.

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import SetMode, CommandTOL, CommandLong


class PID:
    """Simple PID with integral clamp and output clamp."""

    def __init__(self, kp, ki, kd, imax, omax):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.imax, self.omax = imax, omax
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev_e = None
        self.prev_t = None

    def step(self, e, t):
        d = 0.0
        if self.prev_t is not None:
            dt = t - self.prev_t
            if dt > 1e-3:
                d = (e - self.prev_e) / dt
                self.i += e * dt
                self.i = max(-self.imax, min(self.imax, self.i))
        self.prev_e, self.prev_t = e, t
        o = self.kp * e + self.ki * self.i + self.kd * d
        return max(-self.omax, min(self.omax, o))


class GuidedHoldTest(Node):

    def __init__(self):
        super().__init__('guided_hold_test')
        self.declare_parameter('hold_alt', 0.8)
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('max_time', 10.0)   # emergency-disarm after this many s
        self.declare_parameter('reach_tol', 0.20)  # |z-alt| to call takeoff done
        # hold controller: 'pid' = our velocity PID, 'guided' = ArduPilot pos setpoint
        self.declare_parameter('hold_mode', 'pid')
        self.declare_parameter('max_drift_abort', 1.5)  # kill if drift exceeds this (m)
        self.declare_parameter('kp_xy', 0.7)
        self.declare_parameter('ki_xy', 0.08)
        self.declare_parameter('kd_xy', 0.15)
        self.declare_parameter('vmax_xy', 0.5)     # cap horizontal vel cmd (m/s)
        self.declare_parameter('kp_z', 1.2)
        self.declare_parameter('ki_z', 0.20)
        self.declare_parameter('kd_z', 0.20)
        self.declare_parameter('vmax_z', 0.4)      # cap vertical vel cmd (m/s)

        self.hold_alt = float(self.get_parameter('hold_alt').value)
        self.max_time = float(self.get_parameter('max_time').value)
        self.reach_tol = float(self.get_parameter('reach_tol').value)
        self.hold_mode = self.get_parameter('hold_mode').value
        self.max_drift_abort = float(self.get_parameter('max_drift_abort').value)
        gp = lambda n: float(self.get_parameter(n).value)
        self.pid_x = PID(gp('kp_xy'), gp('ki_xy'), gp('kd_xy'), 0.3, gp('vmax_xy'))
        self.pid_y = PID(gp('kp_xy'), gp('ki_xy'), gp('kd_xy'), 0.3, gp('vmax_xy'))
        self.pid_z = PID(gp('kp_z'), gp('ki_z'), gp('kd_z'), 0.3, gp('vmax_z'))

        self.state = None
        self.pose = None
        self.phase = 'wait_arm'      # wait_arm -> guided -> takeoff -> hold -> done
        self.hold = None             # captured (x, y, orientation)
        self.z0 = 0.0                # local-frame z at takeoff (ground reference)
        self.t0 = None
        self.max_drift = 0.0
        self._takeoff_sent = False
        self._guided_t = 0.0
        self._retry_t = 0.0

        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.create_subscription(State, '/mavros/state', self._state, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose,
            qos_profile_sensor_data)
        # STATUSTEXT is BEST_EFFORT on the FC side -> must match QoS or we miss it
        self.create_subscription(
            StatusText, '/mavros/statustext/recv', self._statustext,
            qos_profile_sensor_data)
        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.cli_cmd = self.create_client(CommandLong, '/mavros/cmd/command')

        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value),
                          self.loop)
        self.get_logger().info(
            f'GuidedHoldTest: ARM the drone -> I set GUIDED, take off to '
            f'{self.hold_alt:.1f} m and hold. Override = leave GUIDED or kill.')

    def _state(self, m):
        self.state = m

    def _pose(self, m):
        self.pose = m

    def _statustext(self, m):
        self.get_logger().warn(f'FC statustext: {m.text}')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _set_mode(self, mode):
        if self.cli_mode.service_is_ready():
            req = SetMode.Request(); req.custom_mode = mode
            fut = self.cli_mode.call_async(req)
            fut.add_done_callback(lambda f: self.get_logger().info(
                f'set_mode({mode}) -> sent={f.result().mode_sent}'))

    def _emergency_disarm(self):
        # force-disarm (motors off now, drone drops) - used over LAND since LAND drifts on bad vision pose
        if self.cli_cmd.service_is_ready():
            req = CommandLong.Request()
            req.command = 400
            req.param1 = 0.0
            req.param2 = 21196.0
            self.cli_cmd.call_async(req)

    def _do_takeoff(self):
        if not self.cli_takeoff.service_is_ready():
            self.get_logger().warn('takeoff service not ready')
            return
        req = CommandTOL.Request(); req.altitude = self.hold_alt
        fut = self.cli_takeoff.call_async(req)
        def _cb(f):
            r = f.result()
            if r.success:
                self.get_logger().info('takeoff ACCEPTED')
            else:
                self.get_logger().error(
                    f'takeoff REJECTED result={r.result} '
                    f'(mode={self.state.mode if self.state else "?"} '
                    f'armed={self.state.armed if self.state else "?"})')
        fut.add_done_callback(_cb)

    def _send_sp(self):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = self.hold[0]
        sp.pose.position.y = self.hold[1]
        sp.pose.position.z = self.z0 + self.hold_alt
        sp.pose.orientation = self.hold[2]
        self.sp_pub.publish(sp)

    def _pid_hold(self, p):
        # PID on local-frame position error -> velocity setpoint, ENU (MAVROS converts to NED)
        t = self._now()
        vx = self.pid_x.step(self.hold[0] - p.x, t)
        vy = self.pid_y.step(self.hold[1] - p.y, t)
        vz = self.pid_z.step((self.z0 + self.hold_alt) - p.z, t)
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'map'
        ts.twist.linear.x = vx
        ts.twist.linear.y = vy
        ts.twist.linear.z = vz
        ts.twist.angular.z = 0.0   # hold heading
        self.vel_pub.publish(ts)

    def loop(self):
        if self.state is None or self.pose is None:
            return
        p = self.pose.pose.position

        # pilot override — any exit from GUIDED after takeoff stops us
        if self.phase in ('takeoff', 'hold') and self.state.mode != 'GUIDED':
            self.get_logger().warn(
                f'mode -> {self.state.mode}: pilot override, stopping.')
            self.phase = 'done'
            return

        if self.phase == 'wait_arm':
            if self.state.armed:
                self.get_logger().info('armed -> GUIDED')
                self._set_mode('GUIDED')
                self._guided_t = self._now()
                self.phase = 'guided'

        elif self.phase == 'guided':
            # settle ~1.5 s in GUIDED before takeoff (avoids early NAV_TAKEOFF reject)
            if self.state.mode != 'GUIDED':
                self._set_mode('GUIDED')
            elif self._now() - self._guided_t > 1.5:
                self.hold = (p.x, p.y, self.pose.pose.orientation)
                # local z is EKF-origin-relative, not ground-relative - measure climb from here
                self.z0 = p.z
                self._do_takeoff()
                self._takeoff_sent = True
                self.t0 = self._now()
                self._retry_t = self._now()
                self.get_logger().info(
                    f'GUIDED, takeoff to {self.hold_alt:.1f} m; hold '
                    f'({self.hold[0]:+.2f}, {self.hold[1]:+.2f}), '
                    f'ground z0={self.z0:+.2f}')
                self.phase = 'takeoff'

        elif self.phase in ('takeoff', 'hold'):
            drift = math.hypot(p.x - self.hold[0], p.y - self.hold[1])
            self.max_drift = max(self.max_drift, drift)
            rel_z = p.z - self.z0   # height above takeoff spot, not EKF origin
            # flyaway cap: if it wanders past the limit, kill it now (don't chase)
            if drift > self.max_drift_abort:
                self.get_logger().error(
                    f'drift {drift:.2f} m > {self.max_drift_abort:.1f} m '
                    f'-> EMERGENCY DISARM (flyaway abort)')
                self._emergency_disarm()
                self.phase = 'done'
                return
            # retry takeoff if still on the ground after 3 s
            if (self.phase == 'takeoff' and rel_z < 0.15 and
                    self._now() - self._retry_t > 3.0):
                self.get_logger().warn('still on ground -> retrying takeoff')
                self._do_takeoff()
                self._retry_t = self._now()
            if self.phase == 'takeoff' and abs(rel_z - self.hold_alt) < self.reach_tol:
                self.pid_x.reset(); self.pid_y.reset(); self.pid_z.reset()
                self.get_logger().info(
                    f'reached {self.hold_alt:.1f} m -> HOLD ({self.hold_mode})')
                self.phase = 'hold'
            # command only during hold: a setpoint sent mid-takeoff cancels it
            if self.phase == 'hold':
                if self.hold_mode == 'pid':
                    self._pid_hold(p)
                else:
                    self._send_sp()
            self.get_logger().info(
                f'[{self.phase}/{self.hold_mode}] z={rel_z:+.2f} drift={drift:.2f} m '
                f'(max {self.max_drift:.2f})', throttle_duration_sec=0.5)
            if self._now() - self.t0 > self.max_time:
                self.get_logger().warn(
                    f'max_time {self.max_time:.0f}s -> EMERGENCY DISARM '
                    f'(motors OFF, drops ~{self.hold_alt:.1f} m)')
                self._emergency_disarm()
                self.phase = 'done'


def main(args=None):
    rclpy.init(args=args)
    node = GuidedHoldTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
