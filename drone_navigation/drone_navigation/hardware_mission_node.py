#!/usr/bin/env python3
"""hardware_mission_node.py — goal -> guided takeoff -> Nav2 XY -> land, on real HW.

The hardware equivalent of the sim-only mission_node.py. On a real ArduPilot
drone Nav2 alone cannot fly: it only plans in 2D and emits a horizontal
/cmd_vel, and ArduPilot ignores velocity setpoints while landed. This node
fills the two gaps:

  1. TAKEOFF — arms stays with the pilot; once armed + a goal arrives, it sets
     GUIDED and runs ArduPilot's own NAV_TAKEOFF to cruise_alt (nothing is
     published during the climb, or the setpoint stream would cancel takeoff).
  2. NAVIGATE — it becomes the SINGLE publisher to setpoint_velocity/cmd_vel:
       - holds cruise altitude with a Z-velocity PID (Nav2 never touches Z),
       - rotates Nav2's body-frame (x fwd, y left) velocity into the map/ENU
         frame MAVROS expects (fixes the heading mismatch cmd_vel_to_mavros
         had), and passes Nav2's yaw-rate straight through.
  3. LAND — on Nav2 success it descends on Z velocity, then force-disarms near
     the ground (LAND mode drifts on the LiDAR-SLAM pose; see the notes in
     guided_hold_test.py) and returns to IDLE for the next goal.

Because this owns setpoint_velocity/cmd_vel, run it INSTEAD of
cmd_vel_to_mavros.py. It subscribes to /goal_pose (RViz "2D Goal Pose") and
forwards the goal to Nav2's navigate_to_pose action itself.

Pilot override at any time: flip out of GUIDED, or kill (SA). Leaving GUIDED or
disarming stops all publishing so the pilot has the aircraft.

SAFETY: props on, this FLIES the drone from Nav2 goals. A bad/jumpy SLAM pose =
drift. Validate position hold (guided_hold_test.py) before running this.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandTOL, CommandLong
from nav2_msgs.action import NavigateToPose


class PID:
    """Simple PID with integral clamp and output clamp (from guided_hold_test)."""

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


def yaw_from_quat(q):
    """Yaw (rad) of an ENU quaternion — heading of base_link x-axis from East."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class HardwareMissionNode(Node):

    def __init__(self):
        super().__init__('hardware_mission_node')
        self.declare_parameter('cruise_alt', 1.0)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('reach_tol', 0.20)     # |rel_z-cruise| = takeoff done
        self.declare_parameter('land_rel_z', 0.15)    # disarm below this height
        self.declare_parameter('max_flight_time', 120.0)  # emergency-disarm cap (s)
        self.declare_parameter('vmax_xy', 0.5)         # cap horizontal vel cmd (m/s)
        self.declare_parameter('land_speed', 0.25)     # descent speed (m/s)
        self.declare_parameter('kp_z', 1.2)
        self.declare_parameter('ki_z', 0.20)
        self.declare_parameter('kd_z', 0.20)
        self.declare_parameter('vmax_z', 0.4)          # cap vertical vel cmd (m/s)

        self.cruise_alt = float(self.get_parameter('cruise_alt').value)
        self.reach_tol = float(self.get_parameter('reach_tol').value)
        self.land_rel_z = float(self.get_parameter('land_rel_z').value)
        self.max_flight_time = float(self.get_parameter('max_flight_time').value)
        self.vmax_xy = float(self.get_parameter('vmax_xy').value)
        self.land_speed = float(self.get_parameter('land_speed').value)
        gp = lambda n: float(self.get_parameter(n).value)
        self.pid_z = PID(gp('kp_z'), gp('ki_z'), gp('kd_z'), 0.3, gp('vmax_z'))

        self.state = None
        self.pose = None
        self.cmd = Twist()            # latest Nav2 /cmd_vel (body frame)
        self.cmd_t = None
        self.phase = 'idle'           # idle -> guided -> takeoff -> navigate -> land -> idle
        self.pending_goal = None
        self.z0 = 0.0                 # local-frame z at takeoff (ground reference)
        self.t0 = None
        self._guided_t = 0.0
        self._retry_t = 0.0
        self._nav_sent = False
        self._nav_done = False

        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.create_subscription(State, '/mavros/state', self._state, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose,
            qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel, 10)

        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.cli_cmd = self.create_client(CommandLong, '/mavros/cmd/command')
        self.nav2 = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value),
                          self.loop)
        self.get_logger().info(
            f'HardwareMissionNode: ARM + send a /goal_pose -> I set GUIDED, '
            f'take off to {self.cruise_alt:.1f} m, run Nav2, then land.')

    # ---- subscriptions -------------------------------------------------------
    def _state(self, m):
        self.state = m

    def _pose(self, m):
        self.pose = m

    def _cmd_vel(self, m):
        self.cmd = m
        self.cmd_t = self._now()

    def _goal(self, m):
        if self.phase != 'idle':
            self.get_logger().warn(
                f'Goal ignored — busy (phase {self.phase}).')
            return
        self.pending_goal = m
        self.get_logger().info(
            f'Goal ({m.pose.position.x:+.2f}, {m.pose.position.y:+.2f}) '
            f'received — arm to launch.')

    # ---- helpers -------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _set_mode(self, mode):
        if self.cli_mode.service_is_ready():
            req = SetMode.Request(); req.custom_mode = mode
            self.cli_mode.call_async(req)

    def _do_takeoff(self):
        if not self.cli_takeoff.service_is_ready():
            self.get_logger().warn('takeoff service not ready')
            return
        req = CommandTOL.Request(); req.altitude = self.cruise_alt
        self.cli_takeoff.call_async(req)

    def _emergency_disarm(self):
        # force-disarm (motors OFF). MAV_CMD_COMPONENT_ARM_DISARM(400),
        # param1=0 disarm, param2=21196 force magic (allows in flight).
        if self.cli_cmd.service_is_ready():
            req = CommandLong.Request()
            req.command = 400
            req.param1 = 0.0
            req.param2 = 21196.0
            self.cli_cmd.call_async(req)

    def _publish_vel(self, ve, vn, vz, yaw_rate):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'map'            # ENU; MAVROS converts to NED
        ts.twist.linear.x = ve
        ts.twist.linear.y = vn
        ts.twist.linear.z = vz
        ts.twist.angular.z = yaw_rate
        self.vel_pub.publish(ts)

    def _nav_hold(self, p):
        """Hold cruise altitude (Z PID) + Nav2 body-frame XY rotated into ENU."""
        t = self._now()
        vz = self.pid_z.step((self.z0 + self.cruise_alt) - p.z, t)
        # drop Nav2 velocity if it goes quiet (GUIDED needs a live stream; hover)
        if self.cmd_t is None or t - self.cmd_t > 0.5:
            vx = vy = wz = 0.0
        else:
            vx, vy, wz = self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z
        yaw = yaw_from_quat(self.pose.pose.orientation)
        ve = vx * math.cos(yaw) - vy * math.sin(yaw)
        vn = vx * math.sin(yaw) + vy * math.cos(yaw)
        mag = math.hypot(ve, vn)
        if mag > self.vmax_xy:
            ve *= self.vmax_xy / mag
            vn *= self.vmax_xy / mag
        self._publish_vel(ve, vn, vz, wz)

    def _send_nav2_goal(self):
        if not self.nav2.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 action server not up — landing.')
            self.phase = 'land'
            return
        goal = NavigateToPose.Goal(); goal.pose = self.pending_goal
        self.get_logger().info('Sending goal to Nav2...')
        fut = self.nav2.send_goal_async(goal)
        fut.add_done_callback(self._nav2_response)

    def _nav2_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected goal — landing.')
            self._nav_done = True
            return
        self.get_logger().info('Nav2 accepted goal — navigating.')
        handle.get_result_async().add_done_callback(
            lambda f: setattr(self, '_nav_done', True))

    # ---- main loop -----------------------------------------------------------
    def loop(self):
        if self.state is None or self.pose is None:
            return
        p = self.pose.pose.position

        # pilot override — any exit from GUIDED (or disarm) after launch stops us
        if self.phase in ('takeoff', 'navigate', 'land'):
            if self.state.mode != 'GUIDED' or not self.state.armed:
                self.get_logger().warn(
                    f'mode={self.state.mode} armed={self.state.armed}: '
                    f'pilot override — stopping.')
                self.phase = 'idle'
                self.pending_goal = None
                return
            if self._now() - self.t0 > self.max_flight_time:
                self.get_logger().error(
                    'max_flight_time -> EMERGENCY DISARM.')
                self._emergency_disarm()
                self.phase = 'idle'
                self.pending_goal = None
                return

        if self.phase == 'idle':
            if self.pending_goal is not None and self.state.armed:
                self.get_logger().info('armed + goal -> GUIDED')
                self._set_mode('GUIDED')
                self._guided_t = self._now()
                self.phase = 'guided'

        elif self.phase == 'guided':
            # settle ~1.5 s in GUIDED before takeoff (avoids early NAV_TAKEOFF reject)
            if self.state.mode != 'GUIDED':
                self._set_mode('GUIDED')
            elif self._now() - self._guided_t > 1.5:
                self.z0 = p.z          # ground ref: local z is EKF-origin-relative
                self._do_takeoff()
                self.t0 = self._now()
                self._retry_t = self._now()
                self._nav_sent = False
                self._nav_done = False
                self.get_logger().info(
                    f'GUIDED, takeoff to {self.cruise_alt:.1f} m '
                    f'(ground z0={self.z0:+.2f})')
                self.phase = 'takeoff'

        elif self.phase == 'takeoff':
            rel_z = p.z - self.z0
            # publish NOTHING during climb — a setpoint would cancel NAV_TAKEOFF
            if rel_z < 0.15 and self._now() - self._retry_t > 3.0:
                self.get_logger().warn('still on ground -> retrying takeoff')
                self._do_takeoff()
                self._retry_t = self._now()
            if abs(rel_z - self.cruise_alt) < self.reach_tol:
                self.pid_z.reset()
                self._send_nav2_goal()
                self._nav_sent = True
                self.get_logger().info(
                    f'reached {self.cruise_alt:.1f} m -> NAVIGATE')
                self.phase = 'navigate'

        elif self.phase == 'navigate':
            self._nav_hold(p)
            rel_z = p.z - self.z0
            self.get_logger().info(
                f'[navigate] z={rel_z:+.2f} goal_active={not self._nav_done}',
                throttle_duration_sec=1.0)
            if self._nav_done:
                self.get_logger().info('Nav2 finished -> LAND')
                self.phase = 'land'

        elif self.phase == 'land':
            rel_z = p.z - self.z0
            if rel_z <= self.land_rel_z:
                self.get_logger().info('near ground -> DISARM, ready for next goal')
                self._emergency_disarm()
                self.phase = 'idle'
                self.pending_goal = None
                return
            # descend straight down, hold XY (zero horizontal, zero yaw rate)
            self._publish_vel(0.0, 0.0, -self.land_speed, 0.0)
            self.get_logger().info(
                f'[land] z={rel_z:+.2f}', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
