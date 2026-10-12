# scan_safety_guard_test.py -- reactive /scan safety guard gate.
#
# Exercises the ScanSafetyGuard end-to-end: the /scan -> base_link transform, the stop-zone test
# (the footprint, grown by side_pad, swept over the reaction time plus braking distance of the
# commanded velocity), the stop/resume debounce, and the stale-scan fail-safe.
#
# It composes (NO SLAM, NO map frame at all):
#   * a tf2_ros static_transform_publisher for the STATIC base_link->lidar_link transform the guard
#     needs (x=0.20 m, z=0.20 m, no rotation). The test publishes no other transform, so there is no
#     `map` or `odom` frame in the TF tree -- the cleanest possible proof that the guard is decoupled
#     from SLAM / localization (case 2 asserts this explicitly).
#   * the guard node (ros2 run planner scan_safety_guard) with cmd_vel_in <- /cmd_vel_raw,
#     cmd_vel -> /cmd_vel_guard_out (a test-local sink; the test observes the gating decision, it
#     does not close the physical drive loop), scan <- /scan.
#
# Test-side nodes stand in for the real pipeline (keeping this isolated from planner/SLAM
# correctness): a fake /scan publisher and a fake /cmd_vel_raw publisher (standing in for
# ControllerNode), plus a subscriber on /cmd_vel_guard_out.
#
# ctest label `integration_safety`; does NOT gate the core GoogleTest/RapidCheck sweep.
import math
import os
import sys
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


@pytest.mark.launch_test
def generate_test_description():
    planner_share = get_package_share_directory('planner')
    robot_params = os.path.join(planner_share, 'config', 'robot', 'robot_params.yaml')
    guard_params = os.path.join(planner_share, 'config', 'safety', 'scan_safety_guard.yaml')

    # Static base_link->lidar_link TF (the lidar mount: x=0.20 m, z=0.20 m, no rotation). No SLAM,
    # no map frame.
    lidar_tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='lidar_static_tf', output='screen',
        arguments=['--x', '0.20', '--y', '0', '--z', '0.20',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
    )

    guard = launch_ros.actions.Node(
        package='planner', executable='scan_safety_guard',
        name='scan_safety_guard', output='screen',
        parameters=[{
            'robot_frame': 'base_link',
            'robot_params_path': robot_params,
            'guard_params_path': guard_params,
        }],
        remappings=[
            ('scan', '/scan'),
            ('cmd_vel_in', '/cmd_vel_raw'),
            ('cmd_vel', '/cmd_vel_guard_out'),
        ],
    )

    return (
        launch.LaunchDescription([
            lidar_tf,
            guard,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


# --- Geometry and timing facts ----------------------------------------------------------------
#   Footprint: 0.6 m square centred on base_link (half-width 0.30), r_out = 0.424264 m.
#   Lidar mount: lidar_link at (0.20, 0) in base_link: a return at range r, bearing b is the
#   base_link point (0.20 + r cos b, r sin b).
#   Guard (scan_safety_guard.yaml): a = 1.5 m/s^2, alpha = 2.5 rad/s^2, forward_pad = 0.02 m,
#   side_pad = 0.01 m, stop_count 2, clear_count 10, stale_scan_timeout 0.5 s.
#   Reaction time t_r = age of the newest scan + stop_count * cmd_period. This test publishes scans
#   0.1 s apart and commands at 20 Hz (0.05 s), so the scan age runs from about 0 to about 0.1 s
#   (plus delivery) and t_r is in about [0.10, 0.25] s. Every probe below is chosen to hold for any
#   t_r in [0.10, 0.30] s.
#
#   Scan age = max(node clock - header stamp, node clock - arrival time), never negative, so a scan
#   stamped in the future still goes stale 0.5 s after it arrives.
#
#   Straight motion, front limit = 0.30 + |v| t_r + v^2/(2a) + 0.02 + 0.01:
#     v = 0.5 : 0.30 + 0.5 t_r + 0.0833 + 0.03 = 0.4133 + 0.5 t_r  in [0.463, 0.563]
#     v = 0.25: 0.30 + 0.25 t_r + 0.0208 + 0.03 = 0.3508 + 0.25 t_r in [0.376, 0.426]
#   Side limit = 0.30 + side_pad = 0.31.
#   Pivot at w = 0.5: sweep = w t_r + w^2/(2 alpha) + forward_pad/r_out = 0.5 t_r + 0.05 + 0.02/0.424264 in [0.147, 0.247] rad.
#
#   Probes (base_link body point -> expected):
#     (0.45, 0.0)  : lidar range 0.25, bearing 0. v = 0.5: 0.45 <= 0.463 -> STOP for any t_r.
#                    v = 0.25: 0.45 > 0.426 -> PASS for any t_r. v = -0.5: ahead of a reversing
#                    robot -> PASS.
#     (0.0, 0.41)  : lidar range 0.456, bearing 116 deg. 0.41 > 0.31 side limit -> PASS at v = 0.5.
#     (0.2, 0.21)  : side return, inside the footprint at the start pose -> STOP a pivot.
#     (0.36, 0.0)  : lidar range 0.16, bearing 0. Needs a rotation well above 0.247 rad to be
#                    touched by the corners/front edge -> PASS at w = 0.5.
FORWARD_V = 0.5    # m/s forward command for the stop / pass-through cases.
SLOW_V = 0.25      # m/s: front limit < 0.45 for any t_r.
REVERSE_V = -0.5   # m/s reverse command.
PIVOT_W = 0.5      # rad/s pivot command.
SCAN_MIN = 0.15
SCAN_MAX = 20.0
N_BEAMS = 360
PERIOD = 0.05      # s, command period (20 Hz); a scan goes out every second iteration (0.1 s).
# A sustained-pass window is retried when the system clock stepped during it. 0.02 s is far above the
# normal drift between the wall clock and the monotonic clock over 2 s, and far below the 0.4 s a
# forward step needs to make a 0.1 s-old scan look stale to the guard.
MAX_WINDOW_ATTEMPTS = 3
CLOCK_STEP_TOLERANCE = 0.02  # s

AHEAD = [(-2.0, 2.0, 0.25)]       # body (0.45, 0)
BESIDE = [(115.0, 117.0, 0.456)]  # body (0.0, 0.41)
SIDE_PIVOT = [(70.0, 110.0, 0.22)]  # body (0.13..0.28, 0.207): inside the footprint
FAR_PIVOT = [(-1.0, 1.0, 0.16)]   # body (0.36, 0)


class TestScanSafetyGuard(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('scan_safety_guard_test_node')
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        # Best-effort sensor QoS matches the guard's SensorDataQoS scan subscription.
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        cls.scan_pub = cls.node.create_publisher(LaserScan, '/scan', scan_qos)
        cls.cmd_pub = cls.node.create_publisher(Twist, '/cmd_vel_raw', 10)
        cls.latest_out = {'msg': None}
        cls.out_sub = cls.node.create_subscription(
            Twist, '/cmd_vel_guard_out',
            lambda m: cls.latest_out.__setitem__('msg', m), 10)
        cls.iteration = 0
        cls.next_tick = time.time()
        cls.feed = None  # dict while _assert_sustained_pass measures the scan feed

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ---- scan construction -------------------------------------------------------------------
    def _make_scan(self, wedges, stamp_offset=0.0):
        """Build a LaserScan (frame lidar_link, header stamp = now + stamp_offset s): all beams clear (range_max)
        except those whose bearing (deg) falls in a wedge, set to that wedge's range.
        `wedges` = [(lo_deg, hi_deg, r)]."""
        scan = LaserScan()
        scan.header.frame_id = 'lidar_link'
        scan.header.stamp = (self.node.get_clock().now()
                             + Duration(seconds=stamp_offset)).to_msg()
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / N_BEAMS
        scan.range_min = SCAN_MIN
        scan.range_max = SCAN_MAX
        ranges = [SCAN_MAX] * N_BEAMS
        for i in range(N_BEAMS):
            deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            for (lo, hi, rng) in wedges:
                if lo <= deg <= hi:
                    ranges[i] = rng
        scan.ranges = ranges
        return scan

    # ---- pump / poll -------------------------------------------------------------------------
    def _pump_once(self, wedges, cmd, stamp_offset=0.0):
        """One 20 Hz iteration: a command every iteration, a fresh scan every second iteration
        (header stamps 0.1 s apart), then spin until the next 0.05 s boundary."""
        cls = type(self)
        t_in = time.time()
        cls.iteration += 1
        feed = cls.feed
        if feed is not None:
            feed['max_overrun'] = max(feed['max_overrun'], t_in - cls.next_tick)
        if wedges is not None and cls.iteration % 2 == 0:
            self.scan_pub.publish(self._make_scan(wedges, stamp_offset))
            if feed is not None:
                t_pub = time.time()
                if feed['last_pub'] is not None:
                    feed['max_gap'] = max(feed['max_gap'], t_pub - feed['last_pub'])
                feed['last_pub'] = t_pub
                feed['n_scans'] += 1
        if cmd is not None:
            self.cmd_pub.publish(cmd)
        cls.next_tick = max(cls.next_tick + PERIOD, time.time())
        while True:
            remaining = cls.next_tick - time.time()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self.node, timeout_sec=remaining)

    def _latest(self):
        return self.latest_out['msg']

    def _poll(self, predicate, timeout, wedges, cmd, stamp_offset=0.0):
        """Pump scan+cmd until predicate(latest_out) holds or timeout; return whether it held."""
        end = time.time() + timeout
        while time.time() < end:
            self._pump_once(wedges, cmd, stamp_offset)
            if predicate(self._latest()):
                return True
        return False

    @staticmethod
    def _is_zero(t):
        return t is not None and abs(t.linear.x) < 1e-6 and abs(t.angular.z) < 1e-6

    @staticmethod
    def _is_pass(cmd):
        return lambda t: (t is not None and abs(t.linear.x - cmd.linear.x) < 1e-6
                          and abs(t.angular.z - cmd.angular.z) < 1e-6)

    def _wait_static_tf(self):
        """Wait for the static lidar_link->base_link (guard prerequisite)."""
        end = time.time() + 60.0
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.tf_buffer.can_transform('base_link', 'lidar_link', rclpy.time.Time()):
                return True
        return False

    def _establish_passthrough(self, cmd, wedges=None, stamp_offset=0.0):
        """Warm up (1.5 s of scans and commands, so the guard has two samples of each), then confirm
        the guard passes `cmd` through (waits out the clear_count = 10 debounce). Leaves the guard in
        PASS with a measured reaction time."""
        wedges = [] if wedges is None else wedges
        warm_end = time.time() + 1.5
        while time.time() < warm_end:
            self._pump_once(wedges, cmd, stamp_offset)
        ok = self._poll(self._is_pass(cmd), timeout=15.0, wedges=wedges, cmd=cmd,
                        stamp_offset=stamp_offset)
        self.assertTrue(ok, 'guard never passed the command through (baseline failed)')

    def _assert_sustained_pass(self, cmd, wedges, duration=2.0):
        """Keep publishing; the guard output must never leave `cmd` (checked on every iteration).
        The caller has already established pass-through. If the output leaves `cmd` in a window during
        which the system clock stepped (the guard rightly treats that as a stale scan), the window is
        void and is retried after re-establishing pass-through, up to MAX_WINDOW_ATTEMPTS windows. A
        departure with a steady clock fails at once."""
        cls = type(self)
        is_pass = self._is_pass(cmd)
        steps = []
        for attempt in range(MAX_WINDOW_ATTEMPTS):
            if attempt > 0:
                cls.next_tick = time.time()
                self._establish_passthrough(cmd, wedges)
            cls.feed = {'max_gap': 0.0, 'max_overrun': 0.0, 'last_pub': None, 'n_scans': 0,
                        'clock0': time.time() - time.monotonic()}
            end = time.time() + duration
            try:
                while time.time() < end:
                    self._pump_once(wedges, cmd)
                    if not is_pass(self._latest()):
                        step = (time.time() - time.monotonic()) - cls.feed['clock0']
                        if abs(step) <= CLOCK_STEP_TOLERANCE:
                            self.fail(self._feed_report())
                        print(f'{self._feed_report()}; window void: system clock stepped',
                              file=sys.stderr)
                        steps.append(step)
                        break
                else:
                    return
            finally:
                cls.feed = None
        self.fail('system clock stepped in every sustained-pass attempt: wall_clock_step='
                  + ', '.join(f'{x:.3f} s' for x in steps))

    def _feed_report(self):
        # Scans go out every 0.1 s; the guard stops when the newest scan is older than 0.5 s. A gap
        # or wall_clock_step near 0.5 s points at the test feed; all small points at delivery or the guard.
        f = type(self).feed
        since = 'none' if f['last_pub'] is None else f'{time.time() - f["last_pub"]:.3f}'
        step = (time.time() - time.monotonic()) - f['clock0']
        return (f'guard altered a command it should pass: {self._latest()}; '
                f'max_scan_publish_gap={f["max_gap"]:.3f} s, since_last_scan_publish={since} s, '
                f'scans_published={f["n_scans"]}, max_loop_overrun={f["max_overrun"]:.3f} s, '
                f'wall_clock_step={step:.3f} s')

    def _twist(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        return t

    # ---- cases -------------------------------------------------------------------------------
    def test_case1_forward_return_ahead_stops(self):
        self.assertTrue(self._wait_static_tf(),
                        'static lidar_link->base_link TF never became available')
        fwd = self._twist(v=FORWARD_V)
        self._establish_passthrough(fwd)
        stopped = self._poll(self._is_zero, timeout=3.0, wedges=AHEAD, cmd=fwd)
        self.assertTrue(stopped, 'guard did not safe-stop on a return 0.45 m ahead at v = 0.5')

    def test_case2_stops_with_no_map_frame(self):
        # Decoupling made explicit: there is no `map` frame in this test's TF tree at all.
        self.assertFalse(self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()),
                         'a map frame exists -- this test must run with NO map/localization TF')
        self.assertTrue(self._wait_static_tf(),
                        'static lidar_link->base_link TF never became available')
        fwd = self._twist(v=FORWARD_V)
        self._establish_passthrough(fwd)
        stopped = self._poll(self._is_zero, timeout=3.0, wedges=AHEAD, cmd=fwd)
        self.assertTrue(stopped, 'guard did not safe-stop while decoupled from any map frame')
        self.assertFalse(self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()))

    def test_case3_pivot_toward_side_obstacle_stops(self):
        self.assertTrue(self._wait_static_tf(),
                        'static lidar_link->base_link TF never became available')
        pivot = self._twist(w=PIVOT_W)
        self._establish_passthrough(pivot)
        stopped = self._poll(self._is_zero, timeout=3.0, wedges=SIDE_PIVOT, cmd=pivot)
        self.assertTrue(stopped, 'guard did not safe-stop on a return inside the footprint during a pivot')

    def test_case4_return_ahead_does_not_stop_reverse(self):
        self.assertTrue(self._wait_static_tf(),
                        'static lidar_link->base_link TF never became available')
        rev = self._twist(v=REVERSE_V)
        self._establish_passthrough(rev, AHEAD)
        self._assert_sustained_pass(rev, AHEAD)

    def test_case5_stale_scan_failsafe_stops(self):
        self.assertTrue(self._wait_static_tf(),
                        'static lidar_link->base_link TF never became available')
        fwd = self._twist(v=FORWARD_V)
        self._establish_passthrough(fwd)
        # Stop publishing /scan (wedges=None); keep commanding. Once the cached scan ages past
        # stale_scan_timeout (0.5 s), the guard must fail-safe stop.
        stopped = self._poll(self._is_zero, timeout=3.0, wedges=None, cmd=fwd)
        self.assertTrue(stopped, 'guard did not fail-safe stop on a stale scan')

    def test_case6_pivot_ignores_return_beyond_sweep(self):
        # (0.36, 0) needs far more rotation than the 0.147-0.247 rad swept at w = 0.5.
        self.assertTrue(self._wait_static_tf(),
                        'static lidar_link->base_link TF never became available')
        pivot = self._twist(w=PIVOT_W)
        self._establish_passthrough(pivot, FAR_PIVOT)
        self._assert_sustained_pass(pivot, FAR_PIVOT)

    def test_case7_return_beside_robot_does_not_stop_forward(self):
        # (0.0, 0.41) is 0.10 m outside the 0.31 m side limit.
        self.assertTrue(self._wait_static_tf(),
                        'static lidar_link->base_link TF never became available')
        fwd = self._twist(v=FORWARD_V)
        self._establish_passthrough(fwd, BESIDE)
        self._assert_sustained_pass(fwd, BESIDE)

    def test_case8_speed_scales_the_zone(self):
        # (0.45, 0): inside the v = 0.5 zone (front limit >= 0.463), outside the v = 0.25 zone
        # (front limit <= 0.426).
        self.assertTrue(self._wait_static_tf(),
                        'static lidar_link->base_link TF never became available')
        slow = self._twist(v=SLOW_V)
        self._establish_passthrough(slow, AHEAD)
        self._assert_sustained_pass(slow, AHEAD)
        fast = self._twist(v=FORWARD_V)
        stopped = self._poll(self._is_zero, timeout=3.0, wedges=AHEAD, cmd=fast)
        self.assertTrue(stopped, 'guard did not stop v = 0.5 on a return 0.45 m ahead')

    def test_case9_resume_is_debounced(self):
        self.assertTrue(self._wait_static_tf(),
                        'static lidar_link->base_link TF never became available')
        fwd = self._twist(v=FORWARD_V)
        self._establish_passthrough(fwd)
        self.assertTrue(self._poll(self._is_zero, timeout=3.0, wedges=AHEAD, cmd=fwd))
        # Clear the scene: output must stay zero for most of clear_count = 10 commands, then resume.
        n = 0
        resumed = False
        while n < 200:
            self._pump_once([], fwd)
            n += 1
            if not self._is_zero(self._latest()):
                resumed = True
                break
        self.assertTrue(resumed, 'guard never resumed after the scene cleared')
        self.assertGreaterEqual(n, 8, f'guard resumed after only {n} clear commands (clear_count = 10)')

    def test_case_10_future_stamped_scan_still_goes_stale(self):
        self.assertTrue(self._wait_static_tf(),
                        'static lidar_link->base_link TF never became available')
        fwd = self._twist(v=FORWARD_V)
        self._establish_passthrough(fwd, stamp_offset=5.0)
        # Scans stamped 5 s ahead stop; the arrival-time age must still trip the stale fail-safe.
        stopped = self._poll(self._is_zero, timeout=3.0, wedges=None, cmd=fwd)
        self.assertTrue(stopped, 'guard did not fail-safe stop on a stale future-stamped scan')

    def test_case_11_non_finite_command_is_zeroed(self):
        # Pins: no non-finite value ever appears on the output, from the first bad command on (scene
        # clear). Collection starts before the first bad command and is never cleared, so a single
        # leaked NaN / inf fails the case. Finite in-flight pass-through outputs from the baseline are
        # allowed; the tail (after 3 iterations) must be zero. A zero output is right whether or not
        # the clock stepped, so no window retry is needed.
        self.assertTrue(self._wait_static_tf(),
                        'static lidar_link->base_link TF never became available')
        seen = []
        sub = self.node.create_subscription(Twist, '/cmd_vel_guard_out', seen.append, 10)
        try:
            fwd = self._twist(v=FORWARD_V)
            for label, bad in (('linear.x = NaN', self._twist(v=float('nan'))),
                               ('angular.z = inf', self._twist(w=float('inf')))):
                self._establish_passthrough(fwd)
                seen.clear()  # before the first bad command; never cleared afterwards
                for _ in range(3):
                    self._pump_once([], bad)
                tail_start = len(seen)
                end = time.time() + 1.0
                while time.time() < end:
                    self._pump_once([], bad)
                for m in seen:
                    self.assertTrue(math.isfinite(m.linear.x) and math.isfinite(m.angular.z),
                                    f'non-finite output for {label}: {m}')
                tail = seen[tail_start:]
                self.assertTrue(tail, f'no guard output observed after the first iterations for {label}')
                for m in tail:
                    self.assertTrue(self._is_zero(m), f'non-zero output for {label}: {m}')
        finally:
            self.node.destroy_subscription(sub)
