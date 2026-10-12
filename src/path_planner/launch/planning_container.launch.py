import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('planner')

    # PlannerNode::on_configure YAML::LoadFile's each of these paths, so they must be real installed
    # files — an empty path makes on_configure throw and the lifecycle configure step fail.
    # (diff_drive/heuristic each share a file with another section.)
    robot_params = os.path.join(pkg_share, 'config', 'robot', 'robot_params.yaml')
    planner_params = os.path.join(pkg_share, 'config', 'planner', 'planner.yaml')
    cost_policy = os.path.join(pkg_share, 'config', 'cost_policy', 'cost_policy.yaml')
    controller_params = os.path.join(pkg_share, 'config', 'controller', 'controller.yaml')
    navigator_params = os.path.join(pkg_share, 'config', 'navigation', 'navigator.yaml')

    return LaunchDescription([
        Node(
            # No name= here: this executable hosts THREE nodes (lifecycle map_server and planner_node,
            # plain navigator) with names hardcoded in C++. A name= injects `__node:=` which renames
            # ALL of them to the same name, colliding them and hiding map_server/planner_node from
            # the manager.
            package='planner',
            executable='planning_container',
            output='screen',
            parameters=[{
                'map_topic': '/map',
                'edt_max_dist': 5.0,
                # planner config-file paths:
                'robot_params_path': robot_params,
                'diff_drive_params_path': robot_params,
                'astar_config_path': planner_params,
                'heuristic_config_path': planner_params,
                'cost_policy_path': cost_policy,
                'controller_params_path': controller_params,
                'navigator_params_path': navigator_params,
            }],
        ),
    ])
