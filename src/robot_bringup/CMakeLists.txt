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
endif()

ament_package()
