# Standalone Motion::Planner build (motion-planner sketch §7): host-only,
# no firmware, no Python in the build/test path. Configure/build/test:
#
#   cmake -S src/firm/motion/planner -B src/firm/motion/planner/build
#   cmake --build src/firm/motion/planner/build --target planner_tests
#
# `planner_tests` builds every test executable and runs them via ctest,
# exiting nonzero on any failure. The `motionplanner` shared library is the
# ctypes surface (src/tests/bench/planner_harness.py, relocated 128-010
# from this directory's own py/).
cmake_minimum_required(VERSION 3.16)
project(motion_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra)

add_library(planner STATIC
  profile.cpp
  shape.cpp
  estimation.cpp
  planner.cpp
)
# Two include roots: this tree, and src/firm for the ONE sanctioned shared
# header -- "types/robot_state.h" resolves to src/firm/types/robot_state.h
# (the 124 blackboard; the planner's own types/ mirror was deleted at the
# joint checkpoint, 2026-07-26). firm/types is dependency-free by that
# sprint's own contract, so this adds no firmware coupling.
target_include_directories(planner PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

add_library(motionplanner SHARED capi.cpp)
target_link_libraries(motionplanner PRIVATE planner)

enable_testing()
set(PLANNER_TESTS profile_test shape_test estimation_test
                  planner_scenarios_test planner_noise_test ratio_lock_test
                  planner_lifecycle_test)
foreach(test_name IN LISTS PLANNER_TESTS)
  add_executable(${test_name} tests/${test_name}.cpp)
  target_link_libraries(${test_name} PRIVATE planner)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

# pose_ownership_test (128-016, SUC-002): the ONLY test in this suite that
# needs Motion::Odometry (RobotState::pose's sole writer, post-128-016) --
# not part of the `planner` library above, so this target adds
# odometry.cpp/body_kinematics.cpp directly rather than widening every
# sibling test's link graph.
add_executable(pose_ownership_test tests/pose_ownership_test.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../odometry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../kinematics/differential.cpp)
target_link_libraries(pose_ownership_test PRIVATE planner)
# odometry.cpp/body_kinematics.cpp use the qualified #include "motion/..."
# spelling every other src/firm/motion consumer uses (root CMakeLists.txt,
# src/firm/platform/host/CMakeLists.txt) -- src/ (two levels up from this file), not
# src/firm/motion, is the include root that resolves it.
target_include_directories(pose_ownership_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
add_test(NAME pose_ownership_test COMMAND pose_ownership_test)
list(APPEND PLANNER_TESTS pose_ownership_test)

add_custom_target(planner_tests
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS ${PLANNER_TESTS}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
