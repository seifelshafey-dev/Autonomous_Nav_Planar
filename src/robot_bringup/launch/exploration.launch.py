"""Manual drive-and-map exploration stack: sim + SLAM + optional RViz, NO planning.

This brings up only what you need to drive the robot around by hand and build a map:
  robot_bringup/sim_bringup.launch.py  -- robot + Fortress sim + diff_drive + /odom + /scan bridge
  slam_toolbox (mapping mode)          -- /scan + odom -> /map + map->odom, live SLAM
  rviz2 (optional, rviz:=true)         -- watch /map + robot + scan build up

It deliberately omits the planner, controller, navigator, and lifecycle_manager: nothing
publishes to /cmd_vel, so nothing fights your teleop. It is also how a map for localization mode is made:
drive the world, then serialize the pose graph into robot_bringup/maps/.

Teleop is NOT launched here: teleop_twist_keyboard reads the keyboard from stdin and needs an
interactive TTY, which a ros2 launch Node does not get (keypresses would be lost). Run it yourself
in a SEPARATE terminal:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped

(the remap targets robot_bringup's diff_drive_controller sink, use_stamped_vel:false.)

Map-serialization workflow:
  1. ros2 launch robot_bringup exploration.launch.py rviz:=true
  2. drive the robot around with the teleop command above until the map looks complete
  3. serialize the pose graph into robot_bringup/maps/ (srv: slam_toolbox/srv/SerializePoseGraph,
     request field `filename`; writes <name>.posegraph + <name>.data):

        ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \\
            "{filename: '<abs path>/src/robot_bringup/maps/cluttered_demo'}"

  Later, localize on it: sim_system.launch.py slam_mode:=localization map_file:=cluttered_demo.

The slam_toolbox node here mirrors planning_system.launch.py EXACTLY (same executable, node name,
and the planner's mapper_params.yaml resolved at runtime via the ament index) so the serialized map
is compatible with what localization mode expects -- only `mode` is hardcoded to 'mapping'.

Launch args:
  world        (default cluttered_demo.sdf) -> sim_bringup: Gazebo world to load.
  gui          (default false)           -> sim_bringup: Fortress GUI vs headless server.
  use_sim_time (default true)            -> sim_bringup + slam + rviz: bridged /clock time source.
  rviz         (default false)           -> launch rviz2 with the bundled planning.rviz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_bringup_share = get_package_share_directory('robot_bringup')
    # Reuse the planner package's SLAM tuning (mapper_params.yaml) so the map built here matches
    # what localization mode expects. Runtime ament lookup, same cross-package pattern as the
    # sim_system composer -- no <depend>/<exec_depend> manifest edge is created.
    planner_share = get_package_share_directory('planner')
    slam_params = os.path.join(planner_share, 'config', 'slam', 'mapper_params.yaml')
    rviz_config = os.path.join(robot_bringup_share, 'rviz', 'planning.rviz')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

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

    # Mirrors planning_system.launch.py's slam_toolbox node (executable, name, params file) so the
    # serialized map is drop-in compatible with localization mode; mode is hardcoded 'mapping'.
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='map_and_localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'mode': 'mapping', 'use_sim_time': use_sim_time}],
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
            'use_sim_time', default_value='true',
            description='Use the bridged /clock as the time source (sim_bringup + slam + rviz).',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Launch rviz2 with the bundled planning.rviz.',
        ),
        sim_bringup,
        slam_toolbox,
        rviz_node,
    ])
