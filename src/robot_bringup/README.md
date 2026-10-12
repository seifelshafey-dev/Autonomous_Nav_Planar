# robot_bringup

An example robot for trying the planner in simulation: a differential-drive base, a lidar, Gazebo Fortress worlds and the launch files that wire them to the planner stack. The planner does not depend on this package; bring your own robot for real use.

## What is inside

The robot model is in `urdf/` (`robot.urdf.xacro` includes the links, joints, properties, lidar and `ros2_control` files). It is a box chassis with two driven rear wheels, one passive front castor and a lidar that publishes `/scan` in simulation.

| Launch file | Use it to |
|---|---|
| `mock_bringup.launch.py` | Bring up the robot with mock hardware and no simulator. It gives you `/odom`, `odom` to `base_link` TF and `/joint_states` on any machine. Argument: `use_mock_hardware` (default `true`). |
| `sim_bringup.launch.py` | Start Gazebo Fortress with the robot in a world. It bridges `/scan` and `/clock` and publishes `/odom`. Arguments: `world`, `use_sim_time`, `gui`. |
| `sim_system.launch.py` | Run the full stack: the simulator, SLAM, the planner nodes and an optional EKF and RViz. |
| `exploration.launch.py` | Run the simulator and SLAM with no planner, so you can drive by hand and build a map. |

```bash
ros2 launch robot_bringup sim_system.launch.py world:=cluttered_demo.sdf
```

Arguments of `sim_system.launch.py`:

| Argument | Default | Meaning |
|---|---|---|
| `world` | `cluttered_demo.sdf` | World file, resolved against the installed `worlds/` folder; an absolute path is used as given. |
| `gui` | `false` | `true` opens the Gazebo window; `false` runs the server headless. |
| `rviz` | `false` | Open RViz with the bundled `planning.rviz`. |
| `slam_mode` | `mapping` | `mapping`, or `localization` to load `map_file`. |
| `map_file` | empty | Serialized pose graph (no extension) for localization mode. |
| `use_ekf` | `true` | `true`: a `robot_localization` EKF owns `map` to `odom`. `false`: SLAM Toolbox owns it. |

To drive by hand and build a map, start `exploration.launch.py` (it takes `world`, `gui`, `use_sim_time` and `rviz`) and run the teleop node in a second terminal:

```bash
ros2 launch robot_bringup exploration.launch.py rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

## Worlds

| File | What it is |
|---|---|
| `cluttered_demo.sdf` | A room about 10 m by 8 m with pillars and angled baffles. The default. |
| `tb4_maze.sdf` | Maze, copied from the TurtleBot 4 simulator. |
| `tb4_warehouse.sdf` | Warehouse, copied from the TurtleBot 4 simulator. |
| `tb4_depot.sdf` | Depot, copied from the TurtleBot 4 simulator. |
| `phase2_demo.sdf` | A single wall on a ground plane. Useful only to check that everything starts. SLAM has nothing to hold on to there, so do not use it for navigation. |

## Configuration

| File | Controls |
|---|---|
| `config/controllers.yaml` | `controller_manager`, the joint state broadcaster and `diff_drive_controller` (wheel separation, wheel radius, odometry covariance). |
| `config/ekf.yaml` | The EKF that fuses wheel odometry with the SLAM pose and publishes `map` to `odom`. |
| `config/ros_gz_bridge.yaml` | Which Gazebo topics are bridged into ROS (`/scan`, `/clock` and a ground-truth odometry topic for diagnosis). |

## Tests

| File | Checks |
|---|---|
| `test/test_mock_bringup.py` | With mock hardware, both controllers become active, `/odom` and the TF are published, and a sustained forward command moves the odometry pose. |
| `test/controller_tracking_test.py` | Closed-loop tracking of the pure-pursuit follower against the mock base, with a synthetic plan and no SLAM. |
| `test/planning_control_pipeline_test.py` | Goal to planner to follower to robot with a fixed map and no perception; asserts that the robot reaches the goal. |

Run them with `colcon test --packages-select robot_bringup`.

## Rendering

In the container the simulator runs headless with software rendering. On a host install, add `gui:=true rviz:=true` to see it.

Apache-2.0, see [LICENSE](../../LICENSE). The `tb4_*` worlds are copied from the TurtleBot 4 simulator, also Apache-2.0; their notice is in `worlds/THIRD_PARTY_LICENSE.turtlebot4`.
