# Standalone Motion::Planner build (motion-planner sketch §7): host-only,
# no firmware, no Python in the build/test path. Configure/build/test:
#
#   cmake -S src/motion/planner -B src/motion/planner/build
#   cmake --build src/motion/planner/build --target planner_tests
#
# `planner_tests` builds every test executable and runs them via ctest,
# exiting nonzero on any failure. The `motionplanner` shared library is the
# ctypes surface (py/planner_harness.py).
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
  wheel_pid.cpp
  wheel_trim.cpp
  planner.cpp
)
# Two include roots: this tree, and src/firm for the ONE sanctioned shared
# header -- "types/robot_state.h" resolves to src/firm/types/robot_state.h
# (the 124 blackboard; the planner's own types/ mirror was deleted at the
# joint checkpoint, 2026-07-26). firm/types is dependency-free by that
# sprint's own contract, so this adds no firmware coupling.
target_include_directories(planner PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../firm)

add_library(motionplanner SHARED capi.cpp)
target_link_libraries(motionplanner PRIVATE planner)

enable_testing()
set(PLANNER_TESTS profile_test shape_test estimation_test
                  planner_scenarios_test planner_noise_test ratio_lock_test
                  wheel_trim_test wheel_pid_test planner_duty_scenarios_test)
foreach(test_name IN LISTS PLANNER_TESTS)
  add_executable(${test_name} tests/${test_name}.cpp)
  target_link_libraries(${test_name} PRIVATE planner)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

add_custom_target(planner_tests
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS ${PLANNER_TESTS}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
