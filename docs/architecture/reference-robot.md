# Reference robot

`src/robot_bringup` is an example robot for trying the planner in simulation: a differential-drive base, a lidar, Gazebo Fortress worlds, an optional EKF and the launch files that join them to the planner stack. The planner does not depend on it. `package.xml` lists the planner packages only as `test_depend`, and `sim_system.launch.py` finds the `planner` share directory at launch time. A real robot replaces this package. What the planner nodes expect from a robot is listed under [What the planner stack expects](#what-the-planner-stack-expects).

The nodes of the planner stack are in [ros.md](ros.md). Usage, arguments and the test list are in [`src/robot_bringup/README.md`](../../src/robot_bringup/README.md).

## What the planner stack expects

| Needs | Provided here by |
|---|---|
| `scan` (`sensor_msgs/LaserScan`) on `/scan` | The Gazebo `gpu_lidar` through `ros_gz_bridge`. |
| `/diff_drive_controller/cmd_vel_unstamped` (plain `Twist`) as the drive input | `diff_drive_controller` with `use_stamped_vel: false`. |
| TF `odom` -> `base_link` | `diff_drive_controller` (`enable_odom_tf: true`). |
| TF `map` -> `odom` | `slam_toolbox` or the EKF, see [Localisation](#localisation). |
| A static TF from the scan frame to `base_link` | `robot_state_publisher` from the URDF. |
| Footprint and kinematics in `robot_params.yaml` | The URDF chassis is sized to fit inside the planner footprint. |

## Robot model

The model is in `urdf/`. `robot.urdf.xacro` includes the properties, links, joints, `ros2_control` and lidar files.

| Part | Value | File |
|---|---|---|
| Chassis | box 0.5 m x 0.5 m x 0.10 m | `robot_properties.xacro`, `robot_links.xacro` |
| Driven wheels | radius 0.075 m, at x = 0, y = +/-0.275 m, z = -0.075 m from `base_link` | `robot_joints.xacro` |
| Wheel separation | 0.55 m | `robot_properties.xacro` |
| Castor | fixed sphere of radius 0.025 m at x = 0.25 m, z = -0.125 m | `robot_joints.xacro`, `robot_links.xacro` |
| Lidar mount | `lidar_link`, fixed at x = 0.20 m, z = 0.20 m from `base_link` | `lidar.xacro` |
| Lidar sensor | `gpu_lidar`, 10 Hz, 360 samples over [-3.14159, 3.14159] rad, range 0.15 m to 20 m, header frame `lidar_link` | `lidar.xacro` |

The planner footprint in `src/path_planner/config/robot/robot_params.yaml` is the square with vertices (+/-0.3, +/-0.3) m, which contains the 0.5 m chassis.

The wheel separation and wheel radius appear three times: in the URDF, in `robot_joints.xacro` and in `config/controllers.yaml`. Nothing checks that they agree. If they differ, `/odom` integrates a base of the wrong size.

### Hardware plugin

The xacro argument `use_mock_hardware` (default `true`) selects the plugin in `ros2_control.xacro`:

| Value | Plugin | Used by |
|---|---|---|
| `true` | `mock_components/GenericSystem` with `calculate_dynamics: true` | `mock_bringup.launch.py` |
| `false` | `ign_ros2_control/IgnitionSystem`, with the `ign_ros2_control-system` Gazebo plugin hosting the `controller_manager` inside Gazebo | `sim_bringup.launch.py` |

With the mock plugin, `calculate_dynamics: true` makes each wheel's position state follow its commanded velocity. Without it the wheel position stays at zero and `/odom` does not move.

### Controllers

`config/controllers.yaml` configures `controller_manager` (`update_rate` 30 Hz), `joint_state_broadcaster` and `diff_drive_controller`.

| `diff_drive_controller` setting | Value |
|---|---|
| `left_wheel_names`, `right_wheel_names` | `left_wheel_joint`, `right_wheel_joint` |
| `wheel_separation`, `wheel_radius` | 0.55 m, 0.075 m |
| `publish_rate` | 30 Hz |
| `base_frame_id`, `odom_frame_id`, `enable_odom_tf` | `base_link`, `odom`, `true` |
| `use_stamped_vel` | `false`: the controller subscribes to `~/cmd_vel_unstamped`, a plain `geometry_msgs/Twist`. The publisher and this flag must agree. |
| `linear.x` velocity | -0.6 m/s to +0.6 m/s |
| `linear.x` acceleration | +1.0 m/s^2, -1.5 m/s^2 |
| `angular.z` velocity | -2.0 rad/s to +2.0 rad/s |
| `angular.z` acceleration | +/-2.5 rad/s^2 |
| `pose_covariance_diagonal`, `twist_covariance_diagonal` | [0.001, 0.001, 1e6, 1e6, 1e6, 0.003] |

Relations to the planner stack:

- The linear velocity limit of 0.6 m/s is above the planner's `vmax` and the follower's `v_cruise`, both 0.5 m/s in the shipped files, so the follower's own cap governs. The reverse limit is not zero, because `REV` segments need it.
- `linear_decel` 1.5 m/s^2 and `angular_decel` 2.5 rad/s^2 in `scan_safety_guard.yaml` equal the deceleration limits above. No code checks this.
- The acceleration limits ramp a velocity jump over several control ticks instead of applying it as one step.
- The covariance diagonals are non-zero so that an EKF fusing `/odom` does not read the covariance as exact. The large values mark z, roll, pitch and sideways velocity as unobservable.
- `use_sim_time` is not set in this file, because the Gazebo plugin loads the same file. The launch files set it.

## Launch files

Launch files are in `launch/`.

| File | Starts | Arguments (default) |
|---|---|---|
| `mock_bringup.launch.py` | `robot_state_publisher`, `ros2_control_node`, spawners for `joint_state_broadcaster` and `diff_drive_controller`; remaps `/diff_drive_controller/odom` to `/odom`. No simulator, no `/scan`. | `use_mock_hardware` (`true`), `use_sim_time` (`false`) |
| `sim_bringup.launch.py` | Gazebo Fortress server for `world`, `robot_state_publisher` (with `use_mock_hardware:=false`), `parameter_bridge`, the robot spawn, the two controller spawners and a `topic_tools` relay from `/diff_drive_controller/odom` to `/odom`. | `world` (`cluttered_demo.sdf`), `gui` (`false`), `use_sim_time` (`true`) |
| `exploration.launch.py` | `sim_bringup.launch.py`, `slam_toolbox` in mapping mode with the planner's `mapper_params.yaml`, optional RViz. No planner, controller or navigator, so nothing publishes velocity commands. | `world`, `gui`, `use_sim_time` (`true`), `rviz` (`false`) |
| `sim_system.launch.py` | `sim_bringup.launch.py`, the planner's `planning_system.launch.py`, optional `ekf_filter_node`, optional RViz. | `world`, `gui`, `slam_mode` (`mapping`), `map_file` (empty), `use_sim_time` (`true`), `rviz` (`false`), `use_ekf` (`true`), `ekf_config` (installed `config/ekf.yaml`) |

`world` is resolved against the installed `worlds/` directory unless it is an absolute path.

### Simulator start-up

`sim_bringup.launch.py` runs the following sequence:

1. It sets `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` to include the `ign_ros2_control` library directory, keeping any inherited value.
2. It starts the server through `ros_gz_sim`'s `gz_sim.launch.py` (`gz_version` 6, `on_exit_shutdown` true). With `gui:=false` the arguments are `-s -r --headless-rendering <world>`; with `gui:=true` they are `-r <world>`.
3. Ten seconds later it spawns the robot from the `robot_description` topic under the name `reference_robot` at z = 0.05 m. The robot's x, y and yaw are not set, so it appears at the world origin with yaw 0.
4. When the spawn process exits it starts the `joint_state_broadcaster` spawner. When that exits it starts the `diff_drive_controller` spawner and the odometry relay. Each spawner waits up to 60 s for `/controller_manager`.

The `controller_manager` lives inside the Gazebo plugin, so it is not a launch node and its topics cannot be remapped from the launch file; the relay is what puts the odometry on `/odom`. The `odom` -> `base_link` TF is published on `/tf` directly.

### Bridged topics

`config/ros_gz_bridge.yaml`, all from Gazebo to ROS:

| Topic | Type |
|---|---|
| `/scan` | `sensor_msgs/LaserScan` |
| `/clock` | `rosgraph_msgs/Clock` |
| `/ground_truth/odom` | `nav_msgs/Odometry` |

`/ground_truth/odom` is the true pose of the model in the Gazebo `world` frame, published at 50 Hz in simulated time by the `OdometryPublisher` system in `ros2_control.xacro` (two dimensions, `robot_base_frame` `base_link`). It adds nothing to `/tf`. It exists in the simulator only, not with mock hardware. No node of the planner stack subscribes to it; the scripts in `tools/` do.

Because the robot spawns at the world origin with yaw 0 and the SLAM `map` frame starts at the robot's first pose, the world frame and `map` coincide up to SLAM error.

## Worlds

`worlds/`, selected with `world:=<file>.sdf`:

| File | Content |
|---|---|
| `cluttered_demo.sdf` | A walled room built from boxes 10.2 m and 8.2 m long, with pillars and angled baffles. Default. |
| `phase2_demo.sdf` | One wall on a ground plane. In my runs SLAM did not hold the robot's pose along that wall, so I use it only to check that the stack starts. |
| `tb4_maze.sdf`, `tb4_warehouse.sdf`, `tb4_depot.sdf` | Copied from the TurtleBot 4 simulator (`turtlebot/turtlebot4_simulator`, branch `humble`, Apache-2.0; notice in `worlds/THIRD_PARTY_LICENSE.turtlebot4`). The only change is the world-level `Sensors` plugin (`ogre2`) that the lidar needs. |

`tb4_warehouse.sdf` has 27 lines that name a Gazebo Fuel model and `tb4_depot.sdf` has one; the other three worlds have none. Gazebo fetches Fuel models on first load and caches them under `~/.ignition/fuel`. The first load of the warehouse on my connection took about 220 s and downloaded about 51 MB.

## Localisation

Two transforms are needed on top of the planner: `odom` -> `base_link` and `map` -> `odom`.

| Transform | Published by | Condition |
|---|---|---|
| `odom` -> `base_link` | `diff_drive_controller` | every launch file here that starts the controller (`enable_odom_tf: true`) |
| `map` -> `odom` | `ekf_filter_node` (`robot_localization`, `world_frame: map`) | `sim_system.launch.py` with `use_ekf:=true` (default) |
| `map` -> `odom` | `slam_toolbox` | `use_ekf:=false`, and in `exploration.launch.py` and `planning_system.launch.py` run alone |

`sim_system.launch.py` sets `slam_transform_publish_period` to `0.0` when the EKF runs and to `0.02` otherwise. The value `0.0` lets an external localiser own `map` -> `odom`. The override reaches `slam_toolbox` because `mapper_params.yaml` is keyed by `/**` (see [config.md](config.md)).

### EKF

`config/ekf.yaml`, node `ekf_filter_node`. The EKF fuses two inputs and publishes `map` -> `odom` only.

| Setting | Value | Reason |
|---|---|---|
| `odom0` | `/odom`, `odom0_differential: true`, fields x, y, yaw, vx, vy, vyaw | Wheel odometry enters as relative motion, so the SLAM pose owns the absolute correction. |
| `pose0` | `/pose` (`slam_toolbox`), `pose0_differential: false`, fields x, y, yaw | The absolute correction. |
| `pose0_rejection_threshold` | 3.0, the `robot_localization` default | A Mahalanobis gate that drops a SLAM pose inconsistent with the filter's belief. |
| `two_d_mode` | `true` | Planar base; z, roll and pitch stay zero. |
| `world_frame`, `map_frame`, `odom_frame`, `base_link_frame` | `map`, `map`, `odom`, `base_link` | With `world_frame: map` the filter publishes `map` -> `odom` and needs `odom` -> `base_link` from another node. |
| `frequency` | 20 Hz | Publish rate of the transform. |
| `sensor_timeout` | 0.5 s | Stale-input limit. |
| `transform_timeout` | 0.2 s | Same value as `mapper_params.yaml`. |
| `process_noise_covariance` | not set | The `robot_localization` default is used. |

`use_sim_time` is passed by the launch file and is not in `ekf.yaml`. `ekf_config` replaces the file for an A/B comparison without editing it.

### SLAM settings

`slam_toolbox` runs from `src/path_planner/config/slam/mapper_params.yaml` in all launch files here. The values below are the deliberate ones; the file comments hold the rest.

| Key | Value | Reason |
|---|---|---|
| `base_frame` | `base_link` | The planner looks up `base_link`; the `slam_toolbox` default is `base_footprint`. |
| `max_laser_range`, `scan_buffer_maximum_scan_distance` | 19.5 m | Kept below the lidar's 20 m so a beam at the sensor maximum does not clear free space. |
| `min_laser_range` | 0.15 m | Equals the lidar minimum. |
| `map_update_interval` | 0.5 s | The map provider reacts to a new `/map`, so this sets how soon a new obstacle can trigger a replan. |
| `minimum_travel_distance`, `minimum_travel_heading` | 0.2 m, 0.2 rad | Keyframes close enough that the odometry change between two scans stays inside the correlation search window (`correlation_search_space_dimension` 0.5 m). |
| `distance_variance_penalty`, `angle_variance_penalty` | 0.2, 0.3 | In Karto a smaller value steepens the penalty, so these increase the weight of the odometry prior in the scan match. |
| `minimum_angle_penalty`, `minimum_distance_penalty` | 0.5, 0.3 | Let the penalties lower the score of a match that disagrees with the odometry. |
| `transform_publish_period` | 0.02 s | Default of the file; overridden by the launch argument. |

## Known behaviour

- In the container the simulator runs headless with software rendering, and GPU rendering is not supported there. See [environment.md](environment.md).
- `gui:=true` opens the Gazebo window and needs a display.
