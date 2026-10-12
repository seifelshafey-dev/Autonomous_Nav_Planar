"""Gazebo Fortress sim bring-up for the reference diff-drive base.

Full sim actuation loop, headless:
  ign gazebo server (the world given by `world`)             -- physics + gpu_lidar render
  robot_state_publisher (use_mock_hardware:=false)   -- URDF w/ IgnitionSystem
  ros_gz_sim create                                  -- spawns the robot; its
      ign_ros2_control-system plugin then hosts the controller_manager in-sim
  parameter_bridge (ros_gz_bridge.yaml)              -- /scan + /clock  gz -> ROS
  spawner joint_state_broadcaster                    -- /joint_states
  spawner diff_drive_controller                      -- odom + odom->base_link TF
  topic_tools relay                                  -- /diff_drive_controller/odom -> /odom

All ROS nodes run with use_sim_time:=true (the bridged /clock is the time source).

Launch args:
  world        (default cluttered_demo.sdf) -- Gazebo world to load. A bare
      filename or relative path resolves against the package's installed
      worlds/ share dir; an absolute path is used as-is.
  use_sim_time (default true)  -- bridged /clock as the time source.
  gui          (default false) -- false: headless offscreen server
      (--headless-rendering; needs no display). true: the Ignition Fortress GUI window (server + client)
      for interactive human validation.

The controller_manager mechanism:
  * The controller_manager lives INSIDE the ign_ros2_control-system plugin, so it
    cannot be topic-remapped from this launch (unlike the mock path's standalone
    ros2_control_node). A topic_tools relay exposes the controller's odom on the
    global /odom the planner expects; the odom->base_link TF (enable_odom_tf) is
    on /tf regardless. No standalone ros2_control_node here.

Launch ordering: server -> (t+10s) spawn robot -> on spawn exit, joint_state_
broadcaster -> on its exit, diff_drive_controller + odom relay. This defers the
spawners until the plugin-hosted controller_manager exists. (10 s gives
the gz server headroom when it starts concurrently with the full planning_system
under sim_system -- see the spawn_after_server comment below.)
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_bringup")
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    urdf_path = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
    bridge_config = os.path.join(pkg_share, "config", "ros_gz_bridge.yaml")

    # IGN_GAZEBO_SYSTEM_PLUGIN_PATH must include ign_ros2_control's lib dir so Gazebo's
    # SystemLoader finds libign_ros2_control-system.so (not on the default plugin path). Any
    # inherited value is preserved. Set as a launch env var BEFORE the gz_sim.launch.py include
    # below (that launcher prepends its own paths onto os.environ). GL backend is left to the
    # shell; no override here.
    ros2_control_lib = os.path.join(get_package_prefix("ign_ros2_control"), "lib")
    inherited_plugin_path = os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")
    system_plugin_path = ros2_control_lib + (
        os.pathsep + inherited_plugin_path if inherited_plugin_path else ""
    )
    set_plugin_path = SetEnvironmentVariable(
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH", system_plugin_path
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="cluttered_demo.sdf",
        description=(
            "Gazebo world to load. A bare filename or relative path resolves "
            "against the package's installed worlds/ share dir; an absolute "
            "path is used as-is."
        ),
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use the bridged /clock as the time source (Gazebo sim).",
    )

    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description=(
            "false: headless offscreen server (--headless-rendering) for the "
            "launch_testing scenarios and display-less runs. true: the Ignition "
            "Fortress GUI window for interactive validation."
        ),
    )

    # Simulation URDF: IgnitionSystem hardware + the ign_ros2_control-system plugin.
    robot_description = ParameterValue(
        Command(["xacro ", urdf_path, " use_mock_hardware:=false"]),
        value_type=str,
    )

    # gz server via ros_gz_sim's maintained launcher (gz_sim.launch.py). It runs the Fortress
    # ruby wrapper the supported way and, with on_exit_shutdown:=true, makes a server death tear
    # the whole launch down loudly instead of leaving `ros_gz_sim create` to loop forever. This is
    # process-management hygiene only, NOT the GPU-render-crash fix (that is environmental).
    # gz_args depends on the `world` + `gui` args, so it is assembled at launch time in an
    # OpaqueFunction: headless server (-s --headless-rendering) for gui:=false; server + Fortress
    # window (-r, no -s) for gui:=true.
    def gz_server_actions(context):
        world_value = LaunchConfiguration("world").perform(context)
        world_path = (
            world_value
            if os.path.isabs(world_value)
            else os.path.join(pkg_share, "worlds", world_value)
        )
        gui_value = LaunchConfiguration("gui").perform(context)
        gz_args = (
            f"-r {world_path}"
            if gui_value == "true"
            else f"-s -r --headless-rendering {world_path}"
        )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")
                ),
                launch_arguments={
                    "gz_args": gz_args,
                    "gz_version": "6",
                    "on_exit_shutdown": "true",
                }.items(),
            )
        ]

    gz_server = OpaqueFunction(function=gz_server_actions)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "reference_robot", "-z", "0.05"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    # Expose the plugin-hosted diff_drive_controller odom on the global /odom.
    odom_relay = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/diff_drive_controller/odom", "/odom"],
        output="screen",
    )

    # Ordering: spawn the robot once the server is up; the plugin-hosted
    # controller_manager appears with it, so spawn controllers only after.
    # Period 10 s (was 4 s): under the composed sim_system bring-up the gz server starts
    # CONCURRENTLY with the whole planning_system (slam + lifecycle + container + navigator
    # + controller + scan_safety_guard), so at t+4 s it is often not yet render/physics-ready and
    # the spawn->jsb->diff_drive chain fires before the plugin-hosted controller_manager's hardware
    # interfaces exist -> "Failed to configure controller" + a settling /clock jump-back that splits
    # TF. 10 s gives the server headroom against that concurrent load. (Standalone sim_bringup only
    # pays a slower startup.) A readiness-gated spawn would be the robust replacement.
    spawn_after_server = TimerAction(period=10.0, actions=[spawn_robot])
    jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    ddc_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner, odom_relay],
        )
    )

    return LaunchDescription(
        [
            declare_world,
            declare_use_sim_time,
            declare_gui,
            # Single launch-scoped time-source knob (mirrors planning_system.launch.py), applied to
            # robot_state_publisher / bridge / both spawners / odom_relay.
            # NOTE: the controller_manager is hosted INSIDE the ign_ros2_control plugin (not a launch_ros
            # Node), so this SetParameter does NOT reach it — it relies on the plugin's own sim-time
            # behavior (ign_ros2_control forces CM use_sim_time=true). Verify at runtime:
            # `ros2 param get /controller_manager use_sim_time`.
            SetParameter("use_sim_time", use_sim_time),
            set_plugin_path,
            gz_server,
            robot_state_publisher,
            bridge,
            spawn_after_server,
            jsb_after_spawn,
            ddc_after_jsb,
        ]
    )
