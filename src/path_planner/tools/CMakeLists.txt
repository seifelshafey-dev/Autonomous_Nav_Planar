# Non-ROS visualization/debug tools. Built only when BUILD_TOOLS=ON; never part of the core library.
# nlohmann/json is header-only (system include); path_planner pulls Eigen/absl/OpenCV transitively.
# The scenario to visualize is an explicit argv to search_viz_dump — the tool references no corpus.

# SearchTrace serialization, factored into a small static lib so the trace-dump pipeline is shared by
# any consumer (currently just search_viz_dump). PUBLIC link of path_planner propagates the include
# dirs; PUBLIC include of viz/src lets consumers resolve "search_trace.hpp". The CI smoke test does
# NOT depend on this target (it lives under BUILD_TESTING, independent of BUILD_TOOLS, and compiles
# search_trace.cpp directly — see test/tools/viz).
add_library(viz_search_trace STATIC viz/src/search_trace.cpp)
target_include_directories(viz_search_trace PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/viz/src)
target_link_libraries(viz_search_trace PUBLIC path_planner)

add_executable(search_viz_dump viz/src/search_viz_dump.cpp)
target_link_libraries(search_viz_dump PRIVATE viz_search_trace)

# Planner benchmark: ROS-free, not a test and not installed.
add_executable(plan_benchmark plan_benchmark.cpp)
target_link_libraries(plan_benchmark PRIVATE path_planner)

# Figure data for the swept-cell cover page: the cached cover of one motion and the cells a fan of
# start headings touches. ROS-free, not a test and not installed.
add_executable(swept_cover_dump viz/src/swept_cover_dump.cpp)
target_link_libraries(swept_cover_dump PRIVATE path_planner)

# Figure data for the control-primitives page: the motions of the robot from one pose and the states
# reached after one, two and three motions. ROS-free, not a test and not installed.
add_executable(primitives_dump viz/src/primitives_dump.cpp)
target_link_libraries(primitives_dump PRIVATE path_planner)

# Figure data for the cost and heuristic page: the cost of travel and the obstacle-aware cost-to-goal
# over a scenario map, with the planned path. ROS-free, not a test and not installed.
add_executable(cost_field_dump viz/src/cost_field_dump.cpp)
target_link_libraries(cost_field_dump PRIVATE path_planner)

# Figure data for the navigation page: the escape move found for a start inside a wall, with the
# start check at each step. ROS-free, not a test and not installed.
add_executable(escape_move_dump viz/src/escape_move_dump.cpp)
target_link_libraries(escape_move_dump PRIVATE path_planner)

# Figure data for the stop zone page: where a single scan return holds the robot, for three
# commands. ROS-free, not a test and not installed.
add_executable(stop_zone_dump viz/src/stop_zone_dump.cpp)
target_link_libraries(stop_zone_dump PRIVATE path_planner)
