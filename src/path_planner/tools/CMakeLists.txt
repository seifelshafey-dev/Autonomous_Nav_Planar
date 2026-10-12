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
