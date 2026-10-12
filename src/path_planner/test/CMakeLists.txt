# Lanes. Every planner ctest target carries exactly one of two labels:
#   fast : example unit tests, scenario_plan_test, planner_lifecycle_test
#   full : property executables, viz smoke, launch tests
# Descriptive labels (integration, integration_safety, ...) are kept as additional labels.
#   ctest -L fast     # no RapidCheck needed (configure with -DPLANNER_BUILD_PROPERTY_TESTS=OFF)
#   ctest -L full
option(PLANNER_BUILD_PROPERTY_TESTS "Build the RapidCheck property-test executables (props_core, props_planner, ...)" ON)

function(add_planner_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE path_planner GTest::gtest_main)
    add_test(NAME ${name} COMMAND ${name})
    set_property(TEST ${name} APPEND PROPERTY LABELS fast unit)
endfunction()

# Property-based tests additionally link RapidCheck and its gtest integration. Several sources may be
# listed to build one executable from several test files. No-op when PLANNER_BUILD_PROPERTY_TESTS=OFF.
function(add_planner_property_test name)
    if(NOT PLANNER_BUILD_PROPERTY_TESTS)
        return()
    endif()
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE path_planner GTest::gtest_main rapidcheck rapidcheck_gtest)
    add_test(NAME ${name} COMMAND ${name})
    set_property(TEST ${name} APPEND PROPERTY LABELS full prop)
endfunction()

if(PLANNER_BUILD_PROPERTY_TESTS)
    # RapidCheck (property-based testing). Not a system package, so pull it via FetchContent.
    # GIT_TAG is pinned for reproducibility; bump deliberately.
    # GIT_SHALLOW FALSE: a bare commit SHA can't always be fetched from a shallow clone, so force a
    # full clone to guarantee the pinned SHA resolves.
    include(FetchContent)
    FetchContent_Declare(rapidcheck
        GIT_REPOSITORY https://github.com/emil-e/rapidcheck.git
        GIT_TAG b2d9ed2dddefc4b84318d664b4f221eb792d89c7
        GIT_SHALLOW FALSE
    )
    set(RC_ENABLE_GTEST ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(rapidcheck)
endif()

add_planner_test(wrap_angle_test unit/core/types/math/wrap_angle_test.cpp)


add_planner_test(grid_coords_test unit/core/types/grid_coords_test.cpp)

add_planner_test(robot_params_test unit/core/robot/robot_params_test.cpp)

add_planner_test(grid_test unit/core/map/grid_metadata_test.cpp unit/core/map/grid_occupancy_test.cpp)

add_planner_test(edt_test unit/core/map/edt_test.cpp)

add_planner_test(map_signature_test unit/core/map/map_signature_test.cpp)


add_planner_test(rasterize_test unit/core/map/rasterize_test.cpp)

add_planner_test(world_robot_geometry_test unit/core/motion/world_robot_geometry_test.cpp)

add_planner_test(distance_functions_test unit/core/motion/distance_functions_test.cpp)


add_planner_test(motion_context_test unit/core/motion/motion_context_test.cpp unit/core/motion/kinematic_model_test.cpp)

add_planner_test(diff_drive_test unit/core/motion/kinematics/diff_drive_test.cpp)

add_planner_test(collision_checker_test unit/core/collision_checker/collision_checker_test.cpp)
add_planner_property_test(collision_checker_property_test unit/core/collision_checker/collision_checker_property_test.cpp)

# Pure-pursuit follower: example tests pin the sign/magnitude conventions.
add_planner_test(pure_pursuit_test unit/core/control/pure_pursuit_test.cpp)

# Stop-zone test of the scan guard: example table plus brute-force cover/bound/monotone properties.
add_planner_test(stop_zone_test unit/core/control/stop_zone_test.cpp)
add_planner_property_test(stop_zone_property_test unit/core/control/stop_zone_property_test.cpp)

add_planner_test(navigation_mode_test unit/core/navigation/navigation_mode_test.cpp)
add_planner_test(forward_clock_test unit/core/navigation/forward_clock_test.cpp)
add_planner_test(navigation_checks_test unit/core/navigation/navigation_checks_test.cpp)

add_planner_test(costmap_test unit/core/planner/cost/costmap_test.cpp)

add_planner_test(build_diff_prim_lib_test unit/core/planner/primitives/build_diff_prim_lib_test.cpp)

add_planner_test(closed_set_test unit/core/planner/hybrid_astar/closed_set_test.cpp)

add_planner_test(open_set_test unit/core/planner/hybrid_astar/open_set_test.cpp)

add_planner_test(heuristics_test unit/core/planner/hybrid_astar/heuristics_test.cpp)

add_planner_test(obstacle_heuristic_test unit/core/planner/hybrid_astar/obstacle_heuristic_test.cpp)

add_planner_test(build_hybrid_astar_test unit/core/planner/hybrid_astar/build_hybrid_astar_test.cpp)

add_planner_test(search_test unit/core/planner/hybrid_astar/search_test.cpp)

add_planner_test(escape_search_test unit/core/planner/escape/escape_search_test.cpp)
add_planner_property_test(escape_search_property_test unit/core/planner/escape/escape_search_property_test.cpp)
add_planner_test(path_validity_test unit/core/planner/validation/path_validity_test.cpp)

add_planner_test(config_loader_test unit/core/config/config_loader_test.cpp)

# Property executables. Each is built from several test files (one source per original test).
add_planner_property_test(props_core
    unit/core/types/math/wrap_angle_property_test.cpp
    unit/core/map/grid_transform_property_test.cpp
    unit/core/map/edt_property_test.cpp
    unit/core/navigation/forward_clock_property_test.cpp
    unit/core/map/rasterize_property_test.cpp
    unit/core/motion/world_robot_geometry_property_test.cpp
    unit/core/motion/distance_functions_property_test.cpp
    unit/core/motion/motion_context_property_test.cpp
    unit/core/motion/kinematics/diff_drive_property_test.cpp
)
add_planner_property_test(swept_cover_property_test
    unit/core/planner/primitives/swept_cover_property_test.cpp
    unit/core/planner/primitives/agreement_cover_property_test.cpp)
add_planner_property_test(exact_cover_property_test unit/core/planner/primitives/exact_cover_property_test.cpp)

# Fixed-seed loop of 300 returned paths; every returned path must pass the plain path monitor.
add_planner_test(path_monitor_agreement_test unit/core/planner/validation/path_monitor_agreement_test.cpp)

# pure_pursuit_property_test iterates step -> DiffDriveStateSpace::propagate to require closed-loop
# convergence in both forward and reverse.
add_planner_property_test(props_planner
    unit/core/control/pure_pursuit_property_test.cpp
    unit/core/planner/cost/costmap_property_test.cpp
    unit/core/planner/primitives/build_diff_prim_lib_property_test.cpp
    unit/core/planner/hybrid_astar/closed_set_property_test.cpp
    unit/core/planner/hybrid_astar/obstacle_heuristic_property_test.cpp
    unit/core/planner/hybrid_astar/search_property_test.cpp
)

# Scenario image-occupancy decode. The test generates its grayscale PNG fixture at run time with
# cv::imwrite, so it needs OpenCV directly: path_planner links OpenCV PRIVATE, so the include dirs +
# libs are NOT inherited by consumers and must be attached to this test target explicitly.
add_planner_test(scenario_loader_test unit/core/scenario/scenario_loader_test.cpp)
target_include_directories(scenario_loader_test PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(scenario_loader_test PRIVATE ${OpenCV_LIBS})

# ROS lifecycle idempotency regression: exercises PlannerNode/ROSMapProvider on_configure/
# on_cleanup directly (configure -> cleanup -> configure), so it only builds when the optional
# ROS layer (planner_ros) is present. Needs its own main() to bracket node construction with
# rclcpp::init/shutdown, so it links GTest::gtest rather than the gtest_main convenience target
# the other tests above use.
if(TARGET planner_ros)
    add_executable(planner_lifecycle_test unit/ros/planner_lifecycle_test.cpp)
    target_link_libraries(planner_lifecycle_test PRIVATE planner_ros GTest::gtest)
    add_test(NAME planner_lifecycle_test COMMAND planner_lifecycle_test)
    set_property(TEST planner_lifecycle_test APPEND PROPERTY LABELS fast unit)

    # Navigator node against a scripted plan_path server (one process, one executor). Topic and
    # action names are graph-global, so it never runs alongside another ROS test.
    add_executable(navigator_test unit/ros/navigator_test.cpp)
    target_link_libraries(navigator_test PRIVATE planner_ros GTest::gtest)
    add_test(NAME navigator_test COMMAND navigator_test)
    set_property(TEST navigator_test APPEND PROPERTY LABELS fast unit)
    set_tests_properties(navigator_test PROPERTIES RUN_SERIAL TRUE TIMEOUT 120)
endif()

# Data-driven whole-planner integration tier (scenario corpus). Owns its own macro + runner; see
# integration/CMakeLists.txt. Kept in a subdirectory so its `integration`-labeled ctest cases stay
# selectable apart from the unit suite above.
add_subdirectory(integration)

# Tooling tests. The viz trace-dump smoke test (viz_trace_dump_smoke) lives here so it builds under
# BUILD_TESTING regardless of BUILD_TOOLS; see tools/viz/CMakeLists.txt.
add_subdirectory(tools/viz)