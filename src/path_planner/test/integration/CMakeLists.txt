# Integration tier: whole-planner, data-driven scenario tests. Unlike the unit tier these exercise the
# full buildHybridAstar factory chain against a corpus of scenario YAMLs (runtime-globbed from
# TEST_SCENARIO_CORPUS_DIR, a PRIVATE compile define owned by this test target — the core library is
# scenario-agnostic). The corpus lives alongside this CMake at test/integration/scenarios/. Each
# registered ctest is labeled `integration` (and `fast`) so it can be selected/excluded independently of the unit
# suite, e.g.
#   ctest -L integration        # only integration tests
#   ctest -LE integration       # everything except integration tests
function(add_planner_integration_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE path_planner GTest::gtest_main)
    target_compile_definitions(${name}
        PRIVATE
            TEST_SCENARIO_CORPUS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scenarios"
    )
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES LABELS "integration;fast")
endfunction()

# Single data-driven runner: one ctest case per scenario is produced at run time by
# INSTANTIATE_TEST_SUITE_P globbing the corpus, so dropping a new YAML into scenarios/ is picked up
# with no CMake reconfigure.
add_planner_integration_test(scenario_plan_test scenario_plan_test.cpp)

# Loads the shipped config/ YAMLs and plans with them over a synthetic map.
add_planner_integration_test(shipped_config_test shipped_config_test.cpp)
