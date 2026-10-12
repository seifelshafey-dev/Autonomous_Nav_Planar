# test_mock_bringup.py -- launch_testing gate for the robot_bringup mock-hardware odometry source.
#
# Brings the package up with `use_mock_hardware:=true` (no simulator, headless) and asserts the
# odometry contract the planner consumes:
#   1. joint_state_broadcaster AND diff_drive_controller both reach 'active'
#      (polled via /controller_manager/list_controllers).
#   2. /odom (nav_msgs/Odometry) publishes within a bounded wait.
#   3. odom->base_link TF is broadcast (tf2 can_transform).
#   4. A SUSTAINED forward Twist on /diff_drive_controller/cmd_vel_unstamped advances /odom's
#      pose x. The diff_drive_controller has a 0.5 s cmd_vel timeout, so the command MUST be
#      published repeatedly -- a single publish decays to zero before odom integrates.
#
# Runs with the other robot_bringup tests (`colcon test --packages-select robot_bringup`).
# Timeouts are generous: the mock is deterministic, so these waits are sized for
# CI scheduling latency, not for flaky physics.
import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory('robot_bringup')
    robot_launch = os.path.join(pkg_share, 'launch', 'mock_bringup.launch.py')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={'use_mock_hardware': 'true'}.items(),
    )

    return (
        launch.LaunchDescription([
            bringup,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestMockBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_mock_bringup_node')
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _controllers_active(self, names, timeout=60.0):
        """Poll /controller_manager/list_controllers until every name in `names` is 'active'.

        Returns (ok, last_observed_state_dict) so a failure carries a useful message.
        """
        client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        last_state = {}
        try:
            end = time.time() + timeout
            while time.time() < end:
                if not client.service_is_ready():
                    self._spin(0.5)
                    continue
                future = client.call_async(ListControllers.Request())
                call_end = time.time() + 5.0
                while not future.done() and time.time() < call_end:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                if not future.done():
                    continue
                result = future.result()
                if result is None:
                    continue
                last_state = {c.name: c.state for c in result.controller}
                if all(last_state.get(n) == 'active' for n in names):
                    return True, last_state
                self._spin(0.5)
            return False, last_state
        finally:
            self.node.destroy_client(client)

    def test_1_controllers_active(self):
        ok, state = self._controllers_active(
            ['joint_state_broadcaster', 'diff_drive_controller'], timeout=60.0)
        self.assertTrue(
            ok, f'controllers did not both reach active; observed states: {state}')

    def test_2_odom_published(self):
        msgs = []
        sub = self.node.create_subscription(
            Odometry, '/odom', lambda m: msgs.append(m), 10)
        try:
            end = time.time() + 30.0
            while time.time() < end and not msgs:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            self.assertTrue(msgs, '/odom (nav_msgs/Odometry) never published')
            self.assertEqual(msgs[-1].header.frame_id, 'odom',
                             '/odom header frame_id should be "odom"')
        finally:
            self.node.destroy_subscription(sub)

    def test_3_odom_base_link_tf(self):
        # Give TF a moment to start flowing, then require the transform to resolve.
        ok = False
        end = time.time() + 30.0
        while time.time() < end:
            self._spin(0.5)
            if self.tf_buffer.can_transform(
                    'odom', 'base_link', rclpy.time.Time()):
                ok = True
                break
        self.assertTrue(ok, 'odom->base_link TF was not broadcast')

    def test_4_sustained_cmd_advances_odom(self):
        """Publish a SUSTAINED forward Twist and assert /odom pose x advances.

        The diff_drive_controller times commands out after ~0.5 s, so a single publish would
        decay before the mock integrates it -- and, critically, a STARVED publisher (CPU
        contention when the planner test suites run concurrently) lets inter-message gaps
        exceed that timeout, which zeroes the command and repeatedly stops the base. We defeat
        that two ways: (a) publish at a high, wall-clock-paced ~50 Hz -- gaps stay ~10x under
        the 0.5 s timeout even when a scheduling hiccup steals a few dozen ms -- and (b) drive
        a clearly-nonzero speed over a long sustained window so the real, integrated advance
        clears the threshold with a wide margin. The spin is decoupled from the publish cadence
        so odom callbacks are still serviced without slowing the fixed-rate publisher.
        """
        latest = {'odom': None}
        sub = self.node.create_subscription(
            Odometry, '/odom', lambda m: latest.__setitem__('odom', m), 10)
        cmd_pub = self.node.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        try:
            # Ensure we have a baseline /odom sample.
            end = time.time() + 10.0
            while time.time() < end and latest['odom'] is None:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            self.assertIsNotNone(latest['odom'], 'no /odom sample before command')
            x0 = latest['odom'].pose.pose.position.x

            cmd = Twist()
            cmd.linear.x = 0.5  # m/s forward, well under vmax

            # Publish at a fixed wall-clock ~50 Hz for 6 s. We do NOT let spin_once pace the
            # loop: a non-blocking spin (timeout_sec=0.0) services queued odom callbacks, then
            # we sleep to the next 20 ms publish slot. Under contention a slot may slip, but the
            # 20 ms nominal gap has ~25x headroom before the 0.5 s cmd_vel timeout, so even a
            # badly starved slot rarely reaches it -- the command stays live and the base keeps
            # moving. Nominal advance: 0.5 m/s * 6 s = 3.0 m.
            publish_period = 0.02  # s  -> ~50 Hz
            duration = 6.0         # s  sustained window
            end = time.time() + duration
            next_pub = time.time()
            while time.time() < end:
                cmd_pub.publish(cmd)
                rclpy.spin_once(self.node, timeout_sec=0.0)
                next_pub += publish_period
                sleep_for = next_pub - time.time()
                if sleep_for > 0.0:
                    time.sleep(sleep_for)
                else:
                    # Fell behind (starved): reset the schedule so we don't burst-publish to
                    # catch up, and immediately continue publishing.
                    next_pub = time.time()

            # Stop and let the last /odom sample settle.
            cmd_pub.publish(Twist())
            self._spin(1.0)
            x1 = latest['odom'].pose.pose.position.x

            # 0.5 m/s over ~6 s of sustained command nominally moves ~3.0 m; even with heavy
            # contention stopping the base for a large fraction of the window, the net advance
            # stays well clear of 0.5 m. The 0.5 m threshold gives real margin, yet still fails
            # hard if the base does not actually move (a stalled base advances 0).
            self.assertGreater(
                x1 - x0, 0.5,
                f'/odom pose x did not advance under sustained forward cmd_vel '
                f'(x0={x0:.3f}, x1={x1:.3f})')
        finally:
            self.node.destroy_publisher(cmd_pub)
            self.node.destroy_subscription(sub)
