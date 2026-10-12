from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('planner')
    slam_params = os.path.join(pkg_share, 'config', 'slam', 'mapper_params.yaml')
    lifecycle_params = os.path.join(pkg_share, 'config', 'lifecycle_manager', 'lifecycle_manager.yaml')

    # PlannerNode::on_configure YAML::LoadFile's each of these paths, so they must be real installed
    # files — an empty path makes on_configure throw and the lifecycle configure step fail.
    # (diff_drive/heuristic each share a file with another section.)
    robot_params = os.path.join(pkg_share, 'config', 'robot', 'robot_params.yaml')
    planner_params = os.path.join(pkg_share, 'config', 'planner', 'planner.yaml')
    cost_policy = os.path.join(pkg_share, 'config', 'cost_policy', 'cost_policy.yaml')
    controller_params = os.path.join(pkg_share, 'config', 'controller', 'controller.yaml')
    scan_guard_params = os.path.join(pkg_share, 'config', 'safety', 'scan_safety_guard.yaml')
    navigator_params = os.path.join(pkg_share, 'config', 'navigation', 'navigator.yaml')

    slam_mode = LaunchConfiguration('slam_mode')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Typed as float so it overrides mapper_params.yaml's double transform_publish_period cleanly
    # (a bare substitution would resolve to a string and clash with slam_toolbox's double param).
    slam_transform_publish_period = ParameterValue(
        LaunchConfiguration('slam_transform_publish_period'), value_type=float)

    return LaunchDescription([
        # Single launch-scoped time-source knob: applies use_sim_time to EVERY node declared below,
        # no per-node duplication. Flip the one launch arg for sim<->real; real/mock runs keep 'false'.
        SetParameter('use_sim_time', use_sim_time),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            description="SLAM startup mode: 'mapping' (full SLAM) or "
                        "'localization' (deserialize map_file and localize).",
        ),
        DeclareLaunchArgument(
            'map_file', default_value='',
            description='Serialized pose-graph file (no extension) to load in '
                        'localization mode; ignored in mapping mode.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the /clock time source (true under simulation) for the controller.',
        ),
        DeclareLaunchArgument(
            'slam_transform_publish_period', default_value='0.02',
            description='slam_toolbox map->odom TF publish period (s); set 0.0 to let an external '
                        'localizer own map->odom. Default 0.02 preserves standalone SLAM behavior.',
        ),
        Node(
            package='slam_toolbox', executable='map_and_localization_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=[slam_params, {'mode': slam_mode, 'map_file_name': map_file,
                                      'transform_publish_period': slam_transform_publish_period}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager', output='screen',
            parameters=[lifecycle_params],
        ),
        Node(
            # No name= here: this executable hosts THREE nodes (lifecycle map_server and planner_node,
            # plain navigator) with names hardcoded in C++. A name= injects `__node:=` which renames
            # ALL of them to the same name, colliding them and hiding map_server/planner_node from
            # the manager.
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
        ),
        Node(
            # Standalone pure-pursuit follower. Consumes the latched /plan and drives the base. Its
            # /cmd_vel feeds the ScanSafetyGuard (/cmd_vel_raw) instead of the drive sink
            # directly; the guard gates it and forwards onto diff_drive_controller.
            package='planner', executable='controller', name='controller_node',
            output='screen',
            parameters=[{
                'ctrl_rate': 20.0,
                'planning_frame': 'map', 'robot_frame': 'base_link',
                'controller_params_path': controller_params,
                'robot_params_path': robot_params,
            }],
            remappings=[('/cmd_vel', '/cmd_vel_raw')],
        ),
        Node(
            # Reactive /scan safety guard interposed between the controller's command and the drive
            # sink: passes /cmd_vel_raw through as /diff_drive_controller/cmd_vel_unstamped, or
            # safe-stops on an in-footprint /scan return. Map/localization-independent (needs only
            # the static scan-frame->base_link TF).
            package='planner', executable='scan_safety_guard', name='scan_safety_guard',
            output='screen',
            parameters=[{
                'robot_frame': 'base_link',
                'robot_params_path': robot_params,
                'guard_params_path': scan_guard_params,
            }],
            remappings=[
                ('scan', '/scan'),
                ('cmd_vel_in', '/cmd_vel_raw'),
                ('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),
            ],
        ),
    ])
