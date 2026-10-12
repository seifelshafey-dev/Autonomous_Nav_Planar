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
    src/core/map/rasterize.cpp
    src/core/map/edt.cpp
    src/core/map/map_signature.cpp
    src/core/motion/motion_context.cpp
    src/core/motion/world_robot_geometry.cpp
    src/core/motion/kinematic_model.cpp
    src/core/collision_checker/collision_checker.cpp
    src/core/planner/cost/costmap.cpp
    src/core/planner/primitives/build_diff_prim_lib.cpp
    src/core/planner/hybrid_astar/search.cpp
    src/core/planner/hybrid_astar/obstacle_heuristic.cpp
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

option(BUILD_TESTING "Build unit tests" OFF)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)

    # RapidCheck is fetched in test/CMakeLists.txt (skipped when PLANNER_BUILD_PROPERTY_TESTS=OFF).
    enable_testing()
    include(GoogleTest)
    add_subdirectory(test)
endif()

# ament_package() must be the last CMake call. Guard ensures it is a no-op in plain-CMake builds.
if(ament_cmake_FOUND)
    ament_package()
endif()

