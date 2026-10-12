# lifecycle_bringup_test.py — ROS-level bring-up gate for the SLAM + lifecycle-manager wiring.
# CI has no laser/odometry, so SLAM's inputs are faked: a static /map (transient_local
# OccupancyGrid) and a static odom->base_link TF. The real lifecycle_manager drives the two
# container nodes through configure -> activate; the test asserts both reach 'active' and the
# manager reports is_active == true.
#
# Two bring-up details are load-bearing (omitting either reproduces the real-launch failure):
#   1. The container Node MUST pass the installed config paths (not '' — on_configure
#      YAML::LoadFile()s them and an empty path throws, so planner_node never configures).
#   2. The container Node MUST NOT set name=. The 'planning_container' executable internally
#      constructs three nodes hardcoded as 'map_server', 'planner_node' and 'navigator'; a
#      launch-level name= remaps __node:= onto ALL of them, colliding them so their
#      /<node>/get_state services never appear.
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory('planner')
    lifecycle_params = os.path.join(pkg_share, 'config', 'lifecycle_manager', 'lifecycle_manager.yaml')

    # The container's planner_node YAML::LoadFile()s these in on_configure; empty strings
    # throw and configure fails, so the test MUST pass the real installed config files (same
    # as the launch). diff_drive/heuristic share a file with their sibling loader.
    robot_params = os.path.join(pkg_share, 'config', 'robot', 'robot_params.yaml')
    planner_params = os.path.join(pkg_share, 'config', 'planner', 'planner.yaml')
    cost_policy = os.path.join(pkg_share, 'config', 'cost_policy', 'cost_policy.yaml')
    # The navigator node in the same container exits the process without these two.
    controller_params = os.path.join(pkg_share, 'config', 'controller', 'controller.yaml')
    navigator_params = os.path.join(pkg_share, 'config', 'navigation', 'navigator.yaml')

    # NOTE: no name= here. The executable already names its three internal nodes
    # (map_server, planner_node, navigator); a launch name= would remap all and collide them.
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
    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager', output='screen',
        parameters=[lifecycle_params],
    )
    static_tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher', name='fake_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen',
    )
    return (
        launch.LaunchDescription([
            container,
            lifecycle_manager,
            static_tf,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'container': container, 'lifecycle_manager': lifecycle_manager},
    )


class TestBringUp(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('lifecycle_bringup_test_node')
        # Fake /map: a small static grid, transient_local so late subscribers still get it.
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.map_pub = cls.node.create_publisher(OccupancyGrid, '/map', qos)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.05
        grid.info.width = 20
        grid.info.height = 20
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (20 * 20)
        cls.grid = grid

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self.grid.header.stamp = self.node.get_clock().now().to_msg()
            self.map_pub.publish(self.grid)

    def _call(self, client, request, timeout=10.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout),
                        f'service {client.srv_name} did not appear')
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(future.done(), f'service {client.srv_name} call timed out')
        return future.result()

    def _wait_active(self, node_name, timeout=30.0):
        client = self.node.create_client(GetState, f'/{node_name}/get_state')
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.5)
            if not client.service_is_ready():
                continue
            result = self._call(client, GetState.Request(), timeout=5.0)
            if result is not None and result.current_state.label == 'active':
                return True
        return False

    def test_map_server_active(self):
        self.assertTrue(self._wait_active('map_server'),
                        'map_server did not reach active')

    def test_planner_node_active(self):
        self.assertTrue(self._wait_active('planner_node'),
                        'planner_node did not reach active')

    def test_manager_is_active(self):
        client = self.node.create_client(Trigger, '/lifecycle_manager/is_active')
        # Give the manager time to sequence both nodes + settle bonds before asserting.
        self._spin(2.0)
        result = self._call(client, Trigger.Request(), timeout=15.0)
        self.assertIsNotNone(result, 'is_active service returned no result')
        self.assertTrue(result.success, 'lifecycle_manager reported is_active == false')
