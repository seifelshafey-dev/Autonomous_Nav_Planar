cmake_minimum_required(VERSION 3.20)
project(path_planner_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanTrajectory.msg"
  "msg/PlanProgress.msg"
  "action/PlanPath.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
