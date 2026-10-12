# Viz trace-dump CI smoke test. Loads the seed scenario, runs the dump pipeline, serializes a
# SearchTrace to JSON, then reparses to assert it is well-formed and carries the required keys.
#
# This test lives under BUILD_TESTING, which is independent of BUILD_TOOLS — the viz_search_trace
# static lib (built only when BUILD_TOOLS=ON) is therefore NOT guaranteed to exist here. To stay
# self-contained, the smoke exe compiles tools/viz/src/search_trace.cpp directly (one duplicate
# compile of a small file) and links path_planner for the public include tree. The viz/src include
# dir resolves "search_trace.hpp". The seed scenario path is this target's own PRIVATE define
# (TEST_SCENARIO_CORPUS_DIR), pointing at the integration corpus — the core library carries no
# corpus path.
add_executable(search_viz_smoke
    search_viz_smoke.cpp
    ${CMAKE_SOURCE_DIR}/tools/viz/src/search_trace.cpp
)
target_include_directories(search_viz_smoke PRIVATE ${CMAKE_SOURCE_DIR}/tools/viz/src)
target_link_libraries(search_viz_smoke PRIVATE path_planner)
target_compile_definitions(search_viz_smoke
    PRIVATE
        TEST_SCENARIO_CORPUS_DIR="${CMAKE_SOURCE_DIR}/test/integration/scenarios"
)

add_test(
    NAME viz_trace_dump_smoke
    COMMAND search_viz_smoke ${CMAKE_CURRENT_BINARY_DIR}/search_viz_smoke_trace.json
)
set_tests_properties(viz_trace_dump_smoke PROPERTIES LABELS "tools;full")
