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

# Property executables. Each is built from several test files (one source per original test).
add_planner_property_test(props_core
    unit/core/types/math/wrap_angle_property_test.cpp
)
