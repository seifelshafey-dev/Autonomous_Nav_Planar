# planning_control_pipeline_test.py -- whole software pipeline, map provided, no perception:
# goal -> planner -> controller -> robot.
#
# Unlike controller_tracking_test.py (which injects a SYNTHETIC plan and skips the planner), this
# test exercises the ENTIRE software pipeline with the map provided directly and no perception layer
# (no Gazebo, no SLAM):
#
#   /goal_pose  -> Navigator     -> PlannerNode (hybrid A*)  -> /plan (PlanTrajectory)
#               -> ControllerNode -> /cmd_vel -> diff_drive_controller (mock)
#               -> mock /odom -> odom->base_link TF -> back into planner/controller
#
# and asserts the robot actually reaches the goal. SLAM is replaced by a faked static /map
# (transient_local OccupancyGrid) + a static map->odom IDENTITY TF (map == odom at start), so
# map->base_link resolves and the planner's kin_model derives its resolution from the fake map.
#
# The faked /map is NOT all-free: it carries ONE rectangular wall at the SAME WORLD coordinates as
# the shared offline-viz scenario (test/integration/scenarios/e2e_demo/e2e_demo.yaml) -- world
# x in [1.3, 1.7], y in [-0.3, 2.0] -- so the planner routes a real detour below the wall (dipping
# to ~y=-1.7) and the render is literally "the path the test drives." Only the wall's WORLD coords
# are shared; the E-E map keeps its own 12x12 m extents / (-6,-6) origin / 0.05 m resolution.
#
# An OPT-IN dump (E2E_DUMP_DIR env var; OFF by default) records the executed trajectory (base pose
# sampled ~20 Hz) and the planned /plan path, written to that dir on teardown for a plan-vs-execution
# overlay. Unset in normal CI -> nothing is recorded or written into the source tree.
#
# Composition (all in the map frame):
#   * robot_bringup mock_bringup.launch.py (use_mock_hardware:=true) -- odom->base_link, /odom,
#     diff_drive consuming /diff_drive_controller/cmd_vel_unstamped.
#   * static map->odom identity TF -- SLAM stand-in.
#   * planning_container (map_server + planner_node + navigator) + nav2_lifecycle_manager -> both
#     lifecycle nodes 'active' (bonds).
#   * controller -- /cmd_vel remapped onto the diff_drive sink, planning_frame:map, robot_frame:base_link.
#   * this test node -- publishes the faked static /map, publishes ONE goal on /goal_pose, reads
#     the navigator's trajectory from /plan, then asserts.
#
# REGRESSION-GUARDED co-located-pivot case (do NOT loosen Assertion 2): every planner-generated
# plan begins with in-place PIVOT poses (heading alignment) that share the start position (0,0) with
# the first FWD pose. If advanceCursor() were position-only, the cursor could never leave the
# co-located leading poses: the follower would pivot forever (cmd_vel oscillating v=0,
# omega=+/-omega_pivot) and the base would never translate. The follower guards against this
# (advanceCursor() retires leading pivots on heading convergence and co-located drive points, guarded
# so the cursor never jumps onto the final pose and zeroes the endpoint ramp). Assertion 2 PASSES;
# this end-to-end gate is the whole-stack guard against that stall regressing, backed by the pure
# unit regressions in test/unit/core/control/pure_pursuit_test.cpp (PurePursuitDefectB.*).
#
# GOAL-TRIGGER PATH: a single PoseStamped on /goal_pose, consumed by the navigator inside
# planning_container. The navigator requests the plan from planner_node and publishes it on /plan
# (transient_local) for the controller; this test reads that /plan for its plan assertions. This
# covers goal -> navigator -> planner -> plan -> controller -> robot -> goal.
#
# The goal is published exactly once, after the lifecycle nodes are active and the navigator's
# /goal_pose subscription has matched, so the base drives once into the goal.
#
# Registered under the ctest label `integration_pipeline`. This test is heavy (whole stack), so timeouts are deliberately generous;
# correctness (a non-empty plan + the base settling near the goal) is never loosened.
import csv
import json
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
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from path_planner_msgs.msg import PlanTrajectory
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


# --- Fake map + goal geometry -------------------------------------------------------------------
# A 12 m x 12 m grid at 0.05 m resolution, origin at (-6, -6) so the map interior spans x,y in
# [-6, +6]. The robot starts at map (0,0) (== grid CENTER), and the goal is (3.0, 0.5). This map's
# extents/origin/resolution are the E-E test's own; the ONLY thing shared with the offline-viz
# scenario (test/integration/scenarios/e2e_demo/) is the WALL's WORLD coordinates, so the render is
# literally "the path the test drives."
MAP_RESOLUTION = 0.05
MAP_WIDTH = 240          # cells -> 12.0 m
MAP_HEIGHT = 240         # cells -> 12.0 m
MAP_ORIGIN_X = -6.0
MAP_ORIGIN_Y = -6.0

GOAL_X = 3.0
GOAL_Y = 0.5
GOAL_YAW = 0.0

# --- Shared-scenario wall -----------------------------------------------------------------------
# ONE rectangular wall between start and goal, at the SAME WORLD coordinates as the shared offline
# scenario (test/integration/scenarios/e2e_demo/e2e_demo.yaml): world x in [1.3, 1.7], y in
# [-0.3, 2.0]. The wall blocks the straight start->goal line, so the planner must detour below it
# (into y < -0.3, where there is ample free space) -- the offline plan is the cost-6.52 detour
# dipping to ~y=-1.7. Start stays (0,0) (robot spawn); goal stays (3.0, 0.5).
#
# Cell math (this map's OWN origin/resolution): cell (i,j) center = origin + (i+0.5, j+0.5)*res.
# _paint_wall paints exactly the cells whose CENTER falls in the world span (ceil the low edge, floor
# the high edge), so on this map's (-6, -6) origin the wall lands at:
#   cols i in [146, 153]  (centers x in [1.325, 1.675]  <= within world x [1.3, 1.7])
#   rows j in [114, 159]  (centers y in [-0.275, 1.975] <= within world y [-0.3, 2.0])
# an 8 x 46-cell box. This is the SAME WORLD REGION the shared scenario paints via its cells
# [86, 54, 94, 100] on origin (-3, -3) (world x[1.325,1.725], y[-0.275,2.025]); the two agree to
# within one boundary cell (~5 cm), because the scenario includes the x=1.725 / y=2.025 boundary
# cells whose centers sit just outside the [1.3,1.7]x[-0.3,2.0] span. Functionally identical wall:
# same world extents, blocks the same straight line, forces the same below-the-wall detour.
WALL_WORLD_X = (1.3, 1.7)
WALL_WORLD_Y = (-0.3, 2.0)
OCCUPIED = 100           # OccupancyGrid OCCUPIED cell value


def _world_to_cell_range(w0, w1, origin, res):
    """Inclusive [lo, hi] cell indices whose CENTERS fall in the world span [w0, w1]."""
    lo = math.ceil((w0 - origin) / res - 0.5)
    hi = math.floor((w1 - origin) / res - 0.5)
    return lo, hi


def _paint_wall(data, width, height):
    """Paint the shared-scenario wall (OCCUPIED) into a row-major OccupancyGrid data list in place."""
    i_lo, i_hi = _world_to_cell_range(WALL_WORLD_X[0], WALL_WORLD_X[1], MAP_ORIGIN_X, MAP_RESOLUTION)
    j_lo, j_hi = _world_to_cell_range(WALL_WORLD_Y[0], WALL_WORLD_Y[1], MAP_ORIGIN_Y, MAP_RESOLUTION)
    i_lo = max(0, i_lo); i_hi = min(width - 1, i_hi)
    j_lo = max(0, j_lo); j_hi = min(height - 1, j_hi)
    for j in range(j_lo, j_hi + 1):
        base = j * width
        for i in range(i_lo, i_hi + 1):
            data[base + i] = OCCUPIED


def _yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _seg_point_dist(px, py, ax, ay, bx, by):
    """Perpendicular distance from point (px,py) to the segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 <= 1e-12:                      # degenerate segment -> point distance
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg2
    t = max(0.0, min(1.0, t))             # clamp to the segment
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def _max_cross_track(executed, planned):
    """Max over executed points of the distance to the planned polyline (the pure-pursuit corner-cut).

    Returns 0.0 if there is nothing to measure (no executed samples, or a planned path with < 2
    points). This is the plan-vs-executed deviation: for each driven base pose, the shortest distance
    to any planned segment; the worst such value over the whole run is the corner-cut.
    """
    if not executed or len(planned) < 2:
        return 0.0
    worst = 0.0
    for (px, py) in executed:
        best = min(
            _seg_point_dist(px, py, planned[k][0], planned[k][1],
                            planned[k + 1][0], planned[k + 1][1])
            for k in range(len(planned) - 1))
        worst = max(worst, best)
    return worst


@pytest.mark.launch_test
def generate_test_description():
    robot_share = get_package_share_directory('robot_bringup')
    robot_launch = os.path.join(robot_share, 'launch', 'mock_bringup.launch.py')

    planner_share = get_package_share_directory('planner')
    lifecycle_params = os.path.join(
        planner_share, 'config', 'lifecycle_manager', 'lifecycle_manager.yaml')
    robot_params = os.path.join(planner_share, 'config', 'robot', 'robot_params.yaml')
    planner_params = os.path.join(planner_share, 'config', 'planner', 'planner.yaml')
    cost_policy = os.path.join(planner_share, 'config', 'cost_policy', 'cost_policy.yaml')
    controller_params = os.path.join(planner_share, 'config', 'controller', 'controller.yaml')
    navigator_params = os.path.join(planner_share, 'config', 'navigation', 'navigator.yaml')

    # 1) Mock diff-drive base: odom->base_link, /odom, /diff_drive_controller/cmd_vel_unstamped sink.
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={'use_mock_hardware': 'true'}.items(),
    )

    # 2) Static map->odom identity TF stands in for SLAM (map == odom), so map->base_link resolves.
    map_to_odom = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='fake_map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    # 3) planning_container (map_server + planner_node + navigator). No name= : the executable hardcodes
    # all internal node names; a launch name= would remap and collide them (see lifecycle_bringup_test).
    container = launch_ros.actions.Node(
        package='planner', executable='planning_container',
        output='screen',
        parameters=[{
            'map_topic': '/map', 'edt_max_dist': 5.0,
            'planning_frame': 'map', 'robot_frame': 'base_link',
            'robot_params_path': robot_params, 'diff_drive_params_path': robot_params,
            'astar_config_path': planner_params, 'heuristic_config_path': planner_params,
            'cost_policy_path': cost_policy,
            'controller_params_path': controller_params,
            'navigator_params_path': navigator_params,
        }],
    )
    # The shared lifecycle_manager.yaml uses the nav2 default bond_timeout (4.0 s). That is fine for
    # lifecycle_bringup_test (container + manager only), but THIS test also brings up robot_bringup
    # (ros2_control + spawners + xacro), the navigator, and the controller concurrently. That
    # startup load spike can starve the container's bond heartbeat for several seconds, tripping the
    # 4 s timeout and making the manager kill map_server (observed: a false "map_server IS DOWN"
    # ~6 s after activation). Override bond_timeout to 30 s HERE (test-only; the shared production
    # yaml is untouched) so a transient load spike never kills a healthy node. This is a timing
    # robustness knob, not a correctness relaxation.
    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager', output='screen',
        parameters=[lifecycle_params, {'bond_timeout': 30.0}],
    )

    # 4) Pure-pursuit follower: consumes /plan, drives the base via the diff_drive sink.
    controller = launch_ros.actions.Node(
        package='planner', executable='controller',
        name='controller_node', output='screen',
        parameters=[{
            'ctrl_rate': 20.0,
            'planning_frame': 'map',
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
            container,
            lifecycle_manager,
            controller,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestFullPipelineE2E(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('planning_control_pipeline_test_node')
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)

        # Faked static /map: all-free grid, transient_local so late subscribers (map_server on
        # configure) still receive it. Republished each spin so any subscriber-join race is covered.
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.map_pub = cls.node.create_publisher(OccupancyGrid, '/map', latched)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = MAP_RESOLUTION
        grid.info.width = MAP_WIDTH
        grid.info.height = MAP_HEIGHT
        grid.info.origin.position.x = MAP_ORIGIN_X
        grid.info.origin.position.y = MAP_ORIGIN_Y
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (MAP_WIDTH * MAP_HEIGHT)   # 0 == FREE base fill
        _paint_wall(grid.data, MAP_WIDTH, MAP_HEIGHT)  # ONE shared-scenario wall between start/goal
        cls.grid = grid

        # The navigator's /plan (transient_local, depth 1). Keep the first real plan: plan_id != 0
        # and non-empty poses (an empty plan_id 0 message is the follower stop command).
        cls.planned_traj = {'msg': None}
        cls.plan_sub = cls.node.create_subscription(
            PlanTrajectory, '/plan', cls._plan_cb, latched)
        # Navigator's goal input.
        cls.goal_pub = cls.node.create_publisher(PoseStamped, '/goal_pose', 10)

        # --- Executed-trajectory recording + opt-in dump ----------------------------------------
        # The EXECUTED trajectory (base pose sampled while the controller drives) is recorded
        # IN-MEMORY unconditionally: it feeds the corner-cut re-observe (max cross-track vs the
        # planned path, asserted below). Separately, when E2E_DUMP_DIR is set we ALSO write the
        # executed trajectory + the planned path to that dir on teardown for a plan-vs-execution
        # overlay. When E2E_DUMP_DIR is UNSET (normal CI) nothing is written to disk, so the CI test
        # never writes into the source tree -- only the in-memory samples are kept.
        cls.dump_dir = os.environ.get('E2E_DUMP_DIR') or None
        cls.executed_xy = []          # list of (x, y) base-pose samples (always populated while driving)

    @classmethod
    def _plan_cb(cls, msg):
        if cls.planned_traj['msg'] is None and msg.plan_id != 0 and len(msg.path.poses) > 0:
            cls.planned_traj['msg'] = msg

    @classmethod
    def tearDownClass(cls):
        # Write the opt-in dump BEFORE tearing down rclpy. No-op when E2E_DUMP_DIR is unset.
        if cls.dump_dir:
            cls._write_dump()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _write_dump(cls):
        """Write executed trajectory (CSV x,y) + planned path (JSON list of {x,y}) to E2E_DUMP_DIR."""
        os.makedirs(cls.dump_dir, exist_ok=True)

        exec_path = os.path.join(cls.dump_dir, 'e2e_executed_trajectory.csv')
        with open(exec_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y'])
            for x, y in cls.executed_xy:
                w.writerow([f'{x:.6f}', f'{y:.6f}'])

        planned_xy = []
        msg = cls.planned_traj['msg']
        if msg is not None:
            for ps in msg.path.poses:
                p = ps.pose.position
                planned_xy.append({'x': p.x, 'y': p.y})
        plan_path = os.path.join(cls.dump_dir, 'e2e_planned_path.json')
        with open(plan_path, 'w') as f:
            json.dump(planned_xy, f, indent=2)

        cls.node.get_logger().info(
            f'[E2E dump] wrote executed trajectory ({len(cls.executed_xy)} samples) -> {exec_path}')
        cls.node.get_logger().info(
            f'[E2E dump] wrote planned path ({len(planned_xy)} poses) -> {plan_path}')

    # --- spin / republish helpers ---------------------------------------------------------------
    def _spin(self, seconds, publish_map=False, record=False):
        """High-rate non-blocking spin; optionally re-assert the latched map each pass.

        When record=True, samples the base pose (map->base_link) each pass (~20 Hz, the spin_once
        cadence) into cls.executed_xy. These in-memory samples drive the corner-cut re-observe
        (max cross-track vs the planned path) and, when E2E_DUMP_DIR is set, the plan-vs-execution
        overlay file. Recording is in-memory only; the CSV/JSON dump stays gated on E2E_DUMP_DIR, so
        CI still writes nothing into the source tree.
        """
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if publish_map:
                self.grid.header.stamp = self.node.get_clock().now().to_msg()
                self.map_pub.publish(self.grid)
            if record:
                pose = self._base_pose()
                if pose is not None:
                    self.executed_xy.append((pose[0], pose[1]))

    def _make_goal_msg(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = GOAL_X
        goal.pose.position.y = GOAL_Y
        goal.pose.orientation = _yaw_to_quat(GOAL_YAW)
        return goal

    def _publish_goal_and_get_plan(self, sub_timeout=30.0, plan_timeout=90.0):
        """Publish ONE goal on /goal_pose once the navigator subscribes; return the first real /plan."""
        end = time.time() + sub_timeout
        while self.goal_pub.get_subscription_count() < 1 and time.time() < end:
            self._spin(0.2, publish_map=True)
        self.assertGreaterEqual(
            self.goal_pub.get_subscription_count(), 1,
            '/goal_pose has no subscriber (navigator in planning_container never came up)')
        self.goal_pub.publish(self._make_goal_msg())
        end = time.time() + plan_timeout
        while self.planned_traj['msg'] is None and time.time() < end:
            self._spin(0.2, publish_map=True)
        return self.planned_traj['msg']

    def _call(self, client, request, timeout=10.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout),
                        f'service {client.srv_name} did not appear')
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(future.done(), f'service {client.srv_name} call timed out')
        return future.result()

    def _wait_active(self, node_name, timeout=45.0):
        client = self.node.create_client(GetState, f'/{node_name}/get_state')
        end = time.time() + timeout
        while time.time() < end:
            # Keep the latched map alive while the manager sequences configure/activate: map_server
            # subscribes on configure, and planner_node needs the snapshot to activate cleanly.
            self._spin(0.5, publish_map=True)
            if not client.service_is_ready():
                continue
            result = self._call(client, GetState.Request(), timeout=5.0)
            if result is not None and result.current_state.label == 'active':
                return True
        return False

    def _base_pose(self):
        """Current base pose in map (== odom, identity TF), (x, y, yaw) or None."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform
        yaw = math.atan2(
            2.0 * (t.rotation.w * t.rotation.z + t.rotation.x * t.rotation.y),
            1.0 - 2.0 * (t.rotation.y ** 2 + t.rotation.z ** 2))
        return (t.translation.x, t.translation.y, yaw)

    def _wait_prerequisites(self):
        """Wait until map->base_link resolves (odom->base_link from the mock + map->odom static)."""
        end = time.time() + 90.0
        while time.time() < end:
            self._spin(0.5, publish_map=True)
            if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                return True
        return False

    # --- the whole-pipeline test ----------------------------------------------------------------
    def test_goal_to_planner_to_controller_to_robot(self):
        # 0) Managed nodes reach 'active' (map_server + planner_node), the manager settles bonds.
        self.assertTrue(self._wait_active('map_server'),
                        'map_server did not reach active')
        self.assertTrue(self._wait_active('planner_node'),
                        'planner_node did not reach active')

        mgr = self.node.create_client(Trigger, '/lifecycle_manager/is_active')
        self._spin(2.0, publish_map=True)
        mgr_res = self._call(mgr, Trigger.Request(), timeout=15.0)
        self.assertIsNotNone(mgr_res, 'is_active service returned no result')
        self.assertTrue(mgr_res.success, 'lifecycle_manager reported is_active == false')

        # 1) TF prerequisites: map->base_link must resolve before the planner can read the start.
        self.assertTrue(self._wait_prerequisites(),
                        'map->base_link TF never became available')

        start = self._base_pose()
        self.assertIsNotNone(start, 'could not read base pose before sending goal')

        # 2) Publish ONE goal on /goal_pose; the navigator plans and publishes /plan.
        plan = self._publish_goal_and_get_plan()
        self.assertIsNotNone(plan, 'navigator never published a non-empty /plan (plan_id != 0) -- '
                             'hybrid A* found no path or the goal was lost '
                             '(map too small / goal blocked / start off-map?)')

        # --- Assertion 1: the planner ran -- a non-empty PlanTrajectory came back. ---
        # Proves goal -> planner_node (hybrid A*) -> PlanTrajectory. Also checks the index-alignment
        # invariant mode.size() == speed.size() == path.poses.size().
        self.assertGreater(len(plan.path.poses), 0,
                           'planner returned success but the PlanTrajectory path is empty')
        self.assertEqual(
            len(plan.mode), len(plan.path.poses),
            'PlanTrajectory index-alignment broken: mode.size() != path.poses.size()')
        self.assertEqual(
            len(plan.speed), len(plan.path.poses),
            'PlanTrajectory index-alignment broken: speed.size() != path.poses.size()')

        self._spin(1.0, publish_map=True, record=True)

        # --- Assertion 2: the robot reached the goal -- map->base_link SETTLES near the goal. ---
        # We assert on the SETTLED pose, not a fly-by: the controller ramps into the goal and
        # commands zero on arrival, so a healthy stack comes to rest near the goal and stays there.
        # Poll until the base both (a) stops moving between samples and (b) is within tolerance of
        # the goal; then re-check the settled distance. A defective stack either never stops near the
        # goal (times out) or settles far off (fails the distance check).
        best = float('inf')
        settled_dist = None
        prev = None
        # Detour settle window: the wall forces a ~y=-1.7 dip, so the driven path is materially
        # longer than a straight shot. Give the base plenty of time to track the whole
        # detour and come to rest -- this widens the SETTLE window generously; it does NOT loosen
        # GOAL_REACH_TOL (the correctness gate stays at 0.30 m).
        end = time.time() + 240.0
        while time.time() < end:
            # Keep the map latched while the controller drives the base.
            # record=True samples the executed base pose at ~20 Hz into executed_xy (in-memory) for
            # the corner-cut re-observe; the on-disk dump stays gated on E2E_DUMP_DIR.
            self._spin(0.5, publish_map=True, record=True)
            cur = self._base_pose()
            if cur is None:
                continue
            dist = math.hypot(cur[0] - GOAL_X, cur[1] - GOAL_Y)
            best = min(best, dist)
            stopped = prev is not None and \
                math.hypot(cur[0] - prev[0], cur[1] - prev[1]) < 0.01
            prev = cur
            if stopped and dist <= self.GOAL_REACH_TOL:
                settled_dist = dist
                break

        # --- Corner-cut re-observe -----------------------------------------------------------------
        # Max cross-track between the EXECUTED base path and the PLANNED path -- the real closed-loop
        # pure-pursuit corner-cut (with TF + mock-robot drift). With L derived to 0.45 (and
        # clearance_radius 0.70 / decay 4.5) it must fall within the tracking budget
        # (e_track ~= 0.22 m kinematic / ~0.27 m with drift), well below the ~0.41 m a larger L=0.75
        # would produce. Measured + logged UNCONDITIONALLY (even if the settle gate above failed) so
        # the number is always visible; asserted after the settle gate.
        planned_xy = [(ps.pose.position.x, ps.pose.position.y) for ps in plan.path.poses]
        max_cross_track = _max_cross_track(self.executed_xy, planned_xy)
        self.node.get_logger().info(
            f'[E2E corner-cut] executed {len(self.executed_xy)} samples vs planned '
            f'{len(planned_xy)} poses -> max cross-track = {max_cross_track:.4f} m '
            f'(budget {self.CORNER_CUT_TOL} m)')
        self.node.get_logger().info(
            f'[E2E settle] settled_dist = '
            f'{("%.4f" % settled_dist) if settled_dist is not None else "None"} m, '
            f'closest approach = {best:.4f} m (goal budget {self.GOAL_REACH_TOL} m)')

        self.assertIsNotNone(
            settled_dist,
            f'base never settled within {self.GOAL_REACH_TOL} m of the goal '
            f'({GOAL_X}, {GOAL_Y}) (closest approach was {best:.3f} m) -- the pipeline did not '
            f'drive the robot to the goal')
        self.assertLessEqual(
            settled_dist, self.GOAL_REACH_TOL,
            f'base settled {settled_dist:.3f} m from the goal ({GOAL_X}, {GOAL_Y}), '
            f'outside the {self.GOAL_REACH_TOL} m tolerance')

        # --- Assertion 3: the corner-cut is bounded. ---
        # A non-trivial cut must be present (there IS a sharp detour turn) but stay within the
        # tracking budget. A regression to a large L (or a clearance that stopped biasing the
        # centreline out) would blow past this.
        self.assertGreater(
            max_cross_track, 0.0,
            'measured zero corner-cut -- executed trajectory was not recorded (measurement broken)')
        self.assertLess(
            max_cross_track, self.CORNER_CUT_TOL,
            f'executed corner-cut {max_cross_track:.4f} m exceeds the tracking budget '
            f'{self.CORNER_CUT_TOL} m (derived L=0.45, clearance_radius=0.70) -- the corner is '
            f'sharper than modelled or the clearance bias shifted the plan')

    # Goal-reach tolerance. This is a COMPOUND-error budget, so it is set slightly looser than
    # controller_tracking_test's 0.25 m: the planner's own goal_pos_tol is 0.10 m (planner.yaml), the
    # controller's goal_pos_tol is 0.10 m (controller.yaml), and pure-pursuit corner-cutting plus
    # mock-integration jitter add on top. 0.30 m keeps a ~3x margin over each 0.10 m acceptance band
    # while staying inside a 0.25-0.35 m band. It is loose enough to absorb the
    # planner-endpoint + follower-settle + integration errors that stack in the full loop, but tight
    # enough to fail a robot that stalls, drives past, or ends far off (any of which leaves it > 0.5 m
    # away, at the wrong arc entry, or never moving).
    GOAL_REACH_TOL = 0.30

    # Corner-cut tolerance. Pure pursuit tracks smooth arcs exactly; the cut lives only at
    # curvature/heading junctions, e_corner = (L/2)*sin(theta/2). The theoretical worst junction
    # (tightest 0.15 m arc -> straight, ~156 deg) would cut ~0.22 m kinematic / ~0.27 m with drift at
    # L=0.45; a larger L=0.75 would cut ~0.41 m.
    # MEASURED on THIS fixed scenario (map/goal/config are constant -> the plan and its cut are
    # deterministic): max cross-track = 0.1786 m (its sharpest junction is ~105 deg, gentler than the
    # theoretical worst, so it sits below the 0.22 m kinematic bound). Tightened from the generic
    # 0.30 m budget to 0.25 m = observed 0.1786 + ~0.07 headroom (~40%) for mock-integration/TF jitter.
    # Still far below the ~0.41 m large-L regression signal, so a regression to a large L (or a
    # clearance that stopped biasing the centreline out) blows past it. NOTE: 0.25 is scenario-specific
    # (this detour's junction angles); if the fixed map/goal ever change to a sharper turn, re-observe
    # and lift back toward the 0.30 m generic e_track+headroom budget.
    CORNER_CUT_TOL = 0.25
