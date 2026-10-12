"""Mock-hardware bring-up for the reference diff-drive base (robot_bringup).

Launches, with no simulator:
  robot_state_publisher   -- publishes the URDF TF tree from robot_description
  controller_manager      -- ros2_control_node loading controllers.yaml + the URDF
  spawner joint_state_broadcaster  -- /joint_states
  spawner diff_drive_controller    -- /odom + odom->base_link

Launch argument:
  use_mock_hardware (default 'true')
    'true'  selects mock_components/GenericSystem -- ideal frictionless
            hardware, needs no simulator; yields /odom + odom->base_link on any
            machine, headless.
    'false' selects the ign_ros2_control/IgnitionSystem branch, which
            does nothing useful without a running Gazebo Fortress simulation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_bringup")
    urdf_path = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
    controllers_path = os.path.join(pkg_share, "config", "controllers.yaml")

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_mock_hardware = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description=(
            "'true' selects mock_components/GenericSystem (no simulator); "
            "'false' selects the ign_ros2_control/IgnitionSystem branch."
        ),
    )

    # Central time-source knob, mirroring planning_system.launch.py. The mock path has no
    # simulator, so it stays wall-clock (default 'false'); a single launch-scoped SetParameter
    # applies it to every node below instead of hardcoding it in controllers.yaml.
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the /clock time source (true under simulation); mock path stays wall-clock.",
    )

    # robot_description = xacro(robot.urdf.xacro use_mock_hardware:=<arg>)
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_path,
                " use_mock_hardware:=",
                use_mock_hardware,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            controllers_path,
        ],
        # Expose the diff_drive_controller's odometry on the global /odom topic
        # (the planner's odometry input). The cmd_vel sink keeps its resolved
        # name /diff_drive_controller/cmd_vel_unstamped (the pure-pursuit follower's
        # remap target), so it is intentionally left un-remapped.
        remappings=[("/diff_drive_controller/odom", "/odom")],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_mock_hardware,
            declare_use_sim_time,
            SetParameter("use_sim_time", use_sim_time),
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )
