# controller_tracking_test.py -- closed-loop tracking gate for the pure-pursuit follower.
#
# This is the ONLY test that exercises the whole follower seam the pure-law unit/property tests cannot:
# TF lookup, PlanTrajectory conversion, timer cadence, and the real feedback loop
#   controller -> /cmd_vel -> diff_drive_controller -> mock odom -> TF -> controller.
#
# It composes (no SLAM):
#   * robot_bringup mock_bringup.launch.py (use_mock_hardware:=true) -- supplies the DYNAMIC
#     odom->base_link TF, /odom, and the diff_drive that consumes
#     /diff_drive_controller/cmd_vel_unstamped. This edge is the real feedback the loop moves.
#   * a static map->odom IDENTITY TF -- stands in for SLAM. The controller tracks in the odom
#     frame and needs BOTH frame edges: onPlan looks up map->odom (this static identity,
#     valid at every stamp) to reproject the plan into odom, and onTimer looks up odom->base_link
#     (the mock's dynamic edge) for the per-tick pose. With map==odom the odom-frame plan and pose
#     equal the map-frame values, so the "moves" and terminal-goal checks read identically to the
#     old single map->base_link lookup. Mirrors lifecycle_bringup_test's fake.
#   * the controller node (ros2 run planner controller) with /cmd_vel remapped onto
#     /diff_drive_controller/cmd_vel_unstamped, planning_frame:map, odom_frame:odom,
#     robot_frame:base_link.
#
# The test node publishes a synthetic PlanTrajectory on /plan (transient_local): a straight run
# then a gentle left turn, all FWD, poses spaced <= the 0.75 m lookahead, index-aligned
# mode/speed (the alignment invariant mode.size()==speed.size()==poses.size()).
#
# Lives in robot_bringup (test_depends on planner and path_planner_msgs); ctest label
# `integration_closedloop`; does NOT gate the planner's GoogleTest/RapidCheck sweep. Timeouts are generous -- the
# mock is deterministic, so convergence is real, not flaky.
import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry, Path
from path_planner_msgs.msg import PlanTrajectory
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener


def _yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# Synthetic plan (map frame, base starts at origin 0,0,0): a straight run along +x to (2.0, 0),
# then a gentle left arc curving up to (3.0, 0.6). Poses are spaced ~0.2 m -- well under the
# 0.75 m lookahead and under the rasterizer sample spacing, so the carrot is always resolvable.
def _make_plan_poses():
    poses = []
    # Straight leg: x from 0.2 to 2.0 at y=0, heading 0.
    x = 0.2
    while x <= 2.0 + 1e-9:
        poses.append((x, 0.0, 0.0))
        x += 0.2
    # Gentle left arc: quarter-ish sweep, center (2.0, 0.6), radius 0.6, from angle -pi/2 upward.
    cx, cy, r = 2.0, 0.6, 0.6
    steps = 8
    for k in range(1, steps + 1):
        a = -math.pi / 2.0 + (math.pi / 2.0) * (k / steps)  # -pi/2 -> 0
        px = cx + r * math.cos(a)
        py = cy + r * math.sin(a)
        heading = a + math.pi / 2.0  # tangent, curving left
        poses.append((px, py, heading))
    return poses


PLAN_POSES = _make_plan_poses()
GOAL_XY = (PLAN_POSES[-1][0], PLAN_POSES[-1][1])


@pytest.mark.launch_test
def generate_test_description():
    robot_share = get_package_share_directory('robot_bringup')
    robot_launch = os.path.join(robot_share, 'launch', 'mock_bringup.launch.py')
    planner_share = get_package_share_directory('planner')
    controller_params = os.path.join(
        planner_share, 'config', 'controller', 'controller.yaml')
    robot_params = os.path.join(
        planner_share, 'config', 'robot', 'robot_params.yaml')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={'use_mock_hardware': 'true'}.items(),
    )

    # Static map->odom identity TF stands in for SLAM (map == odom). It supplies the controller's
    # onPlan map->odom lookup (static => valid at the /plan stamp AND latest); the mock's dynamic
    # odom->base_link supplies onTimer's odom->base_link lookup. Their composition == the old
    # single map->base_link, so tracking is unchanged.
    map_to_odom = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='fake_map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    controller = launch_ros.actions.Node(
        package='planner', executable='controller',
        name='controller_node', output='screen',
        parameters=[{
            'ctrl_rate': 20.0,
            'planning_frame': 'map',
            'odom_frame': 'odom',
            'robot_frame': 'base_link',
            'controller_params_path': controller_params,
            'robot_params_path': robot_params,
        }],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    return (
        launch.LaunchDescription([
            bringup,
            map_to_odom,
            controller,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestControllerTracking(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('controller_tracking_test_node')
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        # Latched /plan matching the controller's transient_local subscription.
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.plan_pub = cls.node.create_publisher(PlanTrajectory, '/plan', qos)
        cls.latest_odom = {'msg': None}
        cls.odom_sub = cls.node.create_subscription(
            Odometry, '/odom', lambda m: cls.latest_odom.__setitem__('msg', m), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _build_plan(self):
        traj = PlanTrajectory()
        traj.plan_id = 1
        traj.speed_scale = 1.0
        traj.header.frame_id = 'map'
        traj.header.stamp = self.node.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = traj.header.stamp
        for (x, y, yaw) in PLAN_POSES:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = _yaw_to_quat(yaw)
            path.poses.append(ps)
            traj.mode.append(PlanTrajectory.FWD)
            traj.speed.append(1.0)  # signed reference speed; magnitude is not tracked (D-i)
        traj.path = path
        # Enforce the alignment invariant locally too, so a malformed test plan fails loudly.
        assert len(traj.mode) == len(traj.speed) == len(traj.path.poses)
        return traj

    def _wait_prerequisites(self):
        """Wait until /odom is flowing and map->base_link resolves before publishing the plan."""
        end = time.time() + 90.0
        while time.time() < end:
            self._spin(0.5)
            if self.latest_odom['msg'] is not None and \
                    self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                return True
        return False

    def _base_pose(self):
        """Current base pose in map (== odom, identity), (x, y, yaw) or None."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform
        yaw = math.atan2(
            2.0 * (t.rotation.w * t.rotation.z + t.rotation.x * t.rotation.y),
            1.0 - 2.0 * (t.rotation.y ** 2 + t.rotation.z ** 2))
        return (t.translation.x, t.translation.y, yaw)

    def test_closed_loop_tracks_plan(self):
        self.assertTrue(self._wait_prerequisites(),
                        'prerequisites (/odom + map->base_link TF) never became available')

        start = self._base_pose()
        self.assertIsNotNone(start, 'could not read base pose before publishing plan')

        # Publish and keep republishing the latched plan (transient_local, but re-asserting is
        # harmless and covers any subscriber-join race).
        plan = self._build_plan()
        self.plan_pub.publish(plan)
        self._spin(1.0)

        # --- Assertion 1: the loop is live -- the base actually MOVES after the plan lands. ---
        moved = False
        end = time.time() + 15.0
        while time.time() < end:
            self.plan_pub.publish(plan)
            self._spin(0.5)
            cur = self._base_pose()
            if cur is not None:
                d = math.hypot(cur[0] - start[0], cur[1] - start[1])
                if d > 0.15:
                    moved = True
                    break
        self.assertTrue(
            moved,
            'base did not move after /plan was published -- the closed loop '
            '(controller->cmd_vel->diff_drive->odom->TF->controller) is not running')

        # --- Assertion 2: tracking convergence -- the base SETTLES at the plan's final pose. ---
        # We assert on the SETTLED pose, not a fly-by minimum: the controller ramps into the goal
        # and commands zero on arrival, so a healthy follower comes to rest near the goal and stays
        # there. Poll until the base both (a) stops moving between samples and (b) is within
        # tolerance of the goal; then re-check the settled distance. A defective follower either
        # never stops near the goal (times out) or settles far off (fails the distance check).
        best = float('inf')
        settled_dist = None
        prev = None
        end = time.time() + 90.0
        while time.time() < end:
            self.plan_pub.publish(plan)
            self._spin(0.5)
            cur = self._base_pose()
            if cur is None:
                continue
            dist = math.hypot(cur[0] - GOAL_XY[0], cur[1] - GOAL_XY[1])
            best = min(best, dist)
            # "Stopped" = negligible motion since the previous sample.
            stopped = prev is not None and \
                math.hypot(cur[0] - prev[0], cur[1] - prev[1]) < 0.01
            prev = cur
            if stopped and dist <= self.CONVERGENCE_TOL:
                settled_dist = dist
                break

        self.assertIsNotNone(
            settled_dist,
            f'base never settled within {self.CONVERGENCE_TOL} m of the plan goal {GOAL_XY} '
            f'(closest approach was {best:.3f} m) -- the follower did not converge')
        self.assertLessEqual(
            settled_dist, self.CONVERGENCE_TOL,
            f'base settled {settled_dist:.3f} m from the plan goal {GOAL_XY}, '
            f'outside the {self.CONVERGENCE_TOL} m tolerance')

    # Convergence tolerance. The follower's own goal_pos_tol is 0.10 m (controller.yaml); at that
    # acceptance it commands zero. Observed settled distance on this plan is ~0.09 m (the follower
    # halts just inside its own tolerance). 0.25 m keeps a ~2.7x margin over that settled value --
    # loose enough to absorb pursuit corner-cutting on the arc and mock-integration jitter, tight
    # enough to fail a genuine tracking defect (a broken follower leaves the base >0.5 m off, at the
    # arc entry, or never moving).
    CONVERGENCE_TOL = 0.25
