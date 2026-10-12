cmake_minimum_required(VERSION 3.20)
project(planner CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_cmake is optional: present under colcon/ROS, absent in plain-CMake CI/non-ROS builds.
# All ament-only pieces (install rules, ament_package()) are guarded by this check so the core
# library and unit tests build without any ROS toolchain installed.
find_package(ament_cmake QUIET)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(absl REQUIRED)

# yaml-cpp target-name portability: the namespaced `yaml-cpp::yaml-cpp` target only exists in
# newer yaml-cpp (>= 0.8). Older installs (e.g. the system Ubuntu package) export an unversioned
# `yaml-cpp` target instead. Normalize to the namespaced name so the rest of the build can link a
# single, stable target regardless of which install find_package resolved.
if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

# Environment guard: in mixed conda/system toolchains, linking yaml-cpp from a conda prefix drags
# that prefix's lib dir (and its versionless libcurl) onto the link line. OpenCV's transitive deps
# (libgdal/libnetcdf) need curl_*@CURL_OPENSSL_4 — version nodes the conda libcurl does not export,
# so any fresh link of an OpenCV consumer fails with undefined `curl_*@CURL_OPENSSL_4` references.
# Pull in the system libcurl (which carries the CURL_OPENSSL_4 nodes) so it satisfies that transitive
# need. PUBLIC so every consumer (tests/tools/integration) inherits it. Harmless when curl is absent.
find_package(CURL QUIET)

add_library(path_planner
    src/core/robot/robot_params.cpp
    src/core/config/config_loader.cpp
    src/core/map/rasterize.cpp
    src/core/map/edt.cpp
    src/core/map/map_signature.cpp
    src/core/motion/motion_context.cpp
    src/core/motion/world_robot_geometry.cpp
    src/core/motion/kinematic_model.cpp
    src/core/collision_checker/collision_checker.cpp
    src/core/control/pure_pursuit.cpp
    src/core/control/stop_zone.cpp
    src/core/navigation/navigation_mode.cpp
    src/core/planner/cost/costmap.cpp
    src/core/planner/primitives/build_diff_prim_lib.cpp
    src/core/planner/hybrid_astar/search.cpp
    src/core/planner/hybrid_astar/build_hybrid_astar.cpp
    src/core/planner/hybrid_astar/obstacle_heuristic.cpp
    src/core/planner/validation/path_validity.cpp
    src/core/planner/escape/escape_search.cpp
    src/core/scenario/scenario_loader.cpp
    src/core/scenario/scenario_planner.cpp
)

# Core is scenario-agnostic: it carries no corpus path. The scenario test corpus lives with the
# targets that consume it (test/integration/scenarios/), and each consumer defines its own private
# TEST_SCENARIO_CORPUS_DIR pointing there. See test/integration/CMakeLists.txt and
# test/tools/viz/CMakeLists.txt.

target_include_directories(path_planner
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(path_planner
    PUBLIC
        Eigen3::Eigen
        absl::flat_hash_map
    PRIVATE
        yaml-cpp::yaml-cpp
        ${OpenCV_LIBS}
)

# See the CURL find_package note above: satisfy libgdal/libnetcdf's curl_*@CURL_OPENSSL_4 with the
# system libcurl. PUBLIC so downstream executables inherit it ahead of any conda-prefix libcurl.
if(CURL_FOUND)
    target_link_libraries(path_planner PUBLIC CURL::libcurl)
endif()

# Optional ROS2 layer: only built when rclcpp_lifecycle is available, so the core library stays
# buildable in pure C++ environments (CI, unit tests, embedded toolchains).
find_package(rclcpp QUIET)
find_package(rclcpp_lifecycle QUIET)
find_package(nav_msgs QUIET)

if(rclcpp_FOUND AND rclcpp_lifecycle_FOUND AND nav_msgs_FOUND)
    find_package(rclcpp_action REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(path_planner_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(bondcpp REQUIRED)

    # Fail at configure time instead of deep in the build: the ROS layer uses std::atomic<std::shared_ptr>.
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 12)
        message(FATAL_ERROR "The planner ROS layer needs GCC 12 or newer (std::atomic<std::shared_ptr>); found ${CMAKE_CXX_COMPILER_VERSION}. Configure with -DCMAKE_C_COMPILER=gcc-12 -DCMAKE_CXX_COMPILER=g++-12.")
    endif()

    add_library(planner_ros
        src/ros/map_provider/map_provider_server.cpp
        src/ros/planner_server/planner_node.cpp
        src/ros/planner_server/plan_conversions.cpp
        src/ros/navigator/navigator.cpp
    )
    target_include_directories(planner_ros
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(planner_ros
        PUBLIC
            path_planner
            rclcpp::rclcpp
            rclcpp_lifecycle::rclcpp_lifecycle
            ${nav_msgs_TARGETS}
            rclcpp_action::rclcpp_action
            tf2_ros::tf2_ros
            ${tf2_TARGETS}
            ${tf2_geometry_msgs_TARGETS}
            ${geometry_msgs_TARGETS}
            ${path_planner_msgs_TARGETS}
            ${visualization_msgs_TARGETS}
            ${std_msgs_TARGETS}
            bondcpp::bondcpp
    )

    add_executable(planning_container src/ros/app/planning_container_main.cpp)
    target_link_libraries(planning_container
        PRIVATE
            planner_ros
            rclcpp::rclcpp
            rclcpp_lifecycle::rclcpp_lifecycle
    )

    # Pure-pursuit follower: thin ROS adapter over the ROS-free PurePursuit core (in path_planner).
    add_library(controller_lib src/ros/controller/controller_node.cpp)
    target_include_directories(controller_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(controller_lib
        PUBLIC
            path_planner
            rclcpp::rclcpp
            tf2_ros::tf2_ros
            ${tf2_TARGETS}
            ${tf2_geometry_msgs_TARGETS}
            ${geometry_msgs_TARGETS}
            ${nav_msgs_TARGETS}
            ${path_planner_msgs_TARGETS}
    )

    add_executable(controller src/ros/app/controller_main.cpp)
    target_link_libraries(controller PRIVATE controller_lib)

    # Reactive /scan safety guard: thin ROS adapter over the ROS-free footprint geometry
    # (RobotParams/HalfPlane in path_planner). Point-vs-polygon only; no grid/EDT/rasterizer.
    add_library(scan_safety_guard_lib src/ros/safety/scan_safety_guard.cpp)
    target_include_directories(scan_safety_guard_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(scan_safety_guard_lib
        PUBLIC
            path_planner
            rclcpp::rclcpp
            tf2_ros::tf2_ros
            ${tf2_TARGETS}
            ${tf2_geometry_msgs_TARGETS}
            ${geometry_msgs_TARGETS}
            ${sensor_msgs_TARGETS}
    )

    add_executable(scan_safety_guard src/ros/app/scan_safety_guard_main.cpp)
    target_link_libraries(scan_safety_guard PRIVATE scan_safety_guard_lib)

    if(ament_cmake_FOUND)
        install(TARGETS planning_container controller scan_safety_guard
            RUNTIME DESTINATION lib/${PROJECT_NAME}
        )
        install(DIRECTORY config/
            DESTINATION share/${PROJECT_NAME}/config
        )
    endif()
endif()

option(BUILD_TOOLS "Build non-ROS visualization/debug tools" OFF)

if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

option(BUILD_TESTING "Build unit tests" OFF)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)

    # RapidCheck is fetched in test/CMakeLists.txt (skipped when PLANNER_BUILD_PROPERTY_TESTS=OFF).
    enable_testing()
    include(GoogleTest)
    add_subdirectory(test)

    # ROS bring-up gate (launch_testing). Registered HERE, after enable_testing(), NOT inside the
    # ROS guard at the top of this file: add_launch_test() calls add_test(), which requires
    # enable_testing() to have already run, and that guard block executes long before this point.
    # The ROS find_package _FOUND vars set in that guard persist into this scope, so we re-guard on
    # them (plus ament_cmake) here — a pure-C++ test build (no ROS toolchain) simply skips the case.
    if(rclcpp_lifecycle_FOUND AND ament_cmake_FOUND)
        find_package(launch_testing_ament_cmake REQUIRED)

        # Reactive /scan safety guard gate. Composes a tf2_ros static_transform_publisher
        # for the STATIC lidar_link->base_link TF, plus fake /scan + /cmd_vel_raw
        # publishers and the guard under test, and asserts the guard safe-stops on an in-footprint
        # return -- deliberately with NO map frame in the TF tree, proving the guard is decoupled
        # from SLAM/localization. Carries a DISTINCT ctest label (integration_safety) so it is
        # separable from the core sweep and the other launch gates, and never gates the core
        # GoogleTest/RapidCheck gate.
        add_launch_test(
            test/integration/ros/scan_safety_guard_test.py
            TARGET scan_safety_guard_test
            TIMEOUT 240
            LABELS integration_safety
        )

        # Serialize the heavyweight / timing-sensitive launch_testing gates: running them concurrently
        # under `colcon test` starves them of CPU. RUN_SERIAL tells ctest to never run these in parallel
        # with each other or any other test.
        set_tests_properties(
            scan_safety_guard_test
            PROPERTIES RUN_SERIAL TRUE
        )
        # Lane: launch tests are `full`.
        set_property(TEST
            scan_safety_guard_test
            APPEND PROPERTY LABELS full)
    endif()
endif()

# ament_package() must be the last CMake call. Guard ensures it is a no-op in plain-CMake builds.
if(ament_cmake_FOUND)
    ament_package()
endif()

