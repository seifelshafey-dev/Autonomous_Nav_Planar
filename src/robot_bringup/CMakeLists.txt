cmake_minimum_required(VERSION 3.20)
project(robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The package builds no compiled target -- it installs the URDF, controllers config, launch
# files, RViz config, and (operator-populated) maps into the package share tree.
install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # Mock bring-up launch test: brings the package up with use_mock_hardware:=true and
  # asserts the odometry contract (controllers active, /odom, odom->base_link TF, and a
  # sustained cmd_vel advancing /odom). Runs only under `colcon test --packages-select
  # robot_bringup` -- isolated from the planner sweep.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_mock_bringup.py
    TIMEOUT 180
  )

  # System launch tests: the planner stack + controller driving the mock robot. They need the
  # `planner` and `path_planner_msgs` packages (test_depend in package.xml). RUN_SERIAL keeps the
  # heavyweight, timing-sensitive stacks from starving each other of CPU.
  # Closed-loop controller tracking of a synthetic plan.
  add_launch_test(
    test/controller_tracking_test.py
    TARGET controller_tracking_test
    TIMEOUT 240
    LABELS integration_closedloop
  )
  # Whole pipeline: /goal_pose -> navigator -> PlannerNode -> /plan -> controller -> mock robot.
  # Generous timeout: the robot drives a detour around a wall.
  add_launch_test(
    test/planning_control_pipeline_test.py
    TARGET planning_control_pipeline_test
    TIMEOUT 720
    LABELS integration_pipeline
  )
  set_tests_properties(
    controller_tracking_test
    planning_control_pipeline_test
    PROPERTIES RUN_SERIAL TRUE
  )
endif()

ament_package()
