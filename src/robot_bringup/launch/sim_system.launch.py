"""Reference full-loop composer: Fortress sim + the planning system, one command.

Runtime-include-only composer. It IncludeLaunchDescription's two existing launches and adds
nothing but arg forwarding and an optional RViz:
  robot_bringup/sim_bringup.launch.py   -- robot + Fortress sim + diff_drive + /odom + /scan+/clock bridge
  planner/planning_system.launch.py     -- slam_toolbox + lifecycle_manager + planning_container
                                           (map_server + planner_node) + navigator + controller

Together they close the loop: sim /scan -> slam /map -> planner; a goal (RViz "2D Goal Pose" ->
/goal_pose) -> navigator -> PlanPath -> /plan (+ /plan_path for RViz) -> controller -> /cmd_vel
-> diff_drive -> /odom -> TF.

This file lives in robot_bringup (the reference-integration package that carries the sim
exec_depends), NOT in the planner product. Both includes resolve the sibling package's share dir at
launch time via get_package_share_directory (ament runtime index) -- there is NO find_package and NO
package.xml <depend>/<exec_depend> edge added in either direction. If a sibling package is absent,
only this composer fails to launch; neither package's own build or tests are affected.

use_sim_time is the single sim<->real knob: it is forwarded to BOTH included launches (overriding
planning_system's own 'false' default), and planning_system's launch-scoped SetParameter propagates
it to every planner node.

Launch args:
  world        (default cluttered_demo.sdf) -> sim_bringup: Gazebo world to load.
  gui          (default false)           -> sim_bringup: Fortress GUI vs headless server.
  slam_mode    (default mapping)         -> planning_system: 'mapping' or 'localization'.
  map_file     (default '')              -> planning_system: serialized pose-graph for localization.
  use_sim_time (default true)            -> both: bridged /clock as the time source.
  rviz         (default false)           -> launch rviz2 with the bundled planning.rviz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot_bringup_share = get_package_share_directory('robot_bringup')
    planner_share = get_package_share_directory('planner')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    slam_mode = LaunchConfiguration('slam_mode')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    use_ekf = LaunchConfiguration('use_ekf')
    ekf_config = LaunchConfiguration('ekf_config')

    rviz_config = os.path.join(robot_bringup_share, 'rviz', 'planning.rviz')
    # Default EKF config; overridable via the `ekf_config` arg to try another EKF file
    # without touching the base file.
    ekf_params = os.path.join(robot_bringup_share, 'config', 'ekf.yaml')

    # map->odom ownership toggle (use_ekf). true (default): the EKF owns map->odom and slam's own
    # broadcast is suppressed (period 0.0). false: the EKF is not launched and slam_toolbox owns
    # map->odom directly (period 0.02) -- the known-good no-EKF topology, used to isolate the rest
    # of the stack from the EKF (e.g. validating churn hysteresis, or EKF-on-vs-off A/B).
    slam_tf_period = PythonExpression(
        ["'0.0' if '", use_ekf, "' == 'true' else '0.02'"])

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_share, 'launch', 'sim_bringup.launch.py')
        ),
        launch_arguments={
            'world': world,
            'gui': gui,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    planning_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_share, 'launch', 'planning_system.launch.py')
        ),
        launch_arguments={
            'slam_mode': slam_mode,
            'map_file': map_file,
            'use_sim_time': use_sim_time,
            # Suppress slam_toolbox's own map->odom TF broadcast so the EKF below owns that edge
            # This relies only on the arg NAME contract that planning_system.launch.py
            # declares 'slam_transform_publish_period' and wires it into the slam node's
            # transform_publish_period param; '0.0' hits slam_toolbox's `if (period == 0) return;`
            # early-exit -- localization, /pose, and /map are unaffected, only the TF side-effect stops.
            # 0.0 when use_ekf (EKF owns map->odom); 0.02 when not (slam owns it directly).
            'slam_transform_publish_period': slam_tf_period,
        }.items(),
    )

    # Single-instance robot_localization EKF (world_frame: map) that fuses /odom (robot_bringup's
    # diff_drive stack) + /pose (planning_system's slam_toolbox) and publishes map->odom in place of
    # slam_toolbox's suppressed broadcast. sim_system is the ONLY composition where both topics are
    # alive, so this reference-integration bridge node lives here, not in the planner package.
    # use_sim_time is forwarded via the parameters list (rviz_node precedent below) since this node is
    # not covered by planning_system's launch-scoped SetParameter.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_ekf),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='cluttered_demo.sdf',
            description='Gazebo world to load (forwarded to sim_bringup).',
        ),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Fortress GUI window (true) vs headless server (false); forwarded to sim_bringup.',
        ),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            description="SLAM mode 'mapping' or 'localization' (forwarded to planning_system).",
        ),
        DeclareLaunchArgument(
            'map_file', default_value='',
            description='Serialized pose-graph (no extension) for localization mode; forwarded to '
                        'planning_system, ignored in mapping mode.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Single sim<->real knob: bridged /clock as time source; forwarded to both '
                        'included launches.',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Launch rviz2 with the bundled planning.rviz.',
        ),
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description="true: robot_localization EKF owns map->odom (slam's own broadcast "
                        "suppressed). false: EKF not launched, slam_toolbox owns map->odom "
                        "(no-EKF topology; isolates the stack from the EKF).",
        ),
        DeclareLaunchArgument(
            'ekf_config', default_value=ekf_params,
            description='Path to the ekf_node config YAML. Defaults to the installed '
                        'config/ekf.yaml; override to try another EKF file.',
        ),
        sim_bringup,
        planning_system,
        ekf_node,
        rviz_node,
    ])
