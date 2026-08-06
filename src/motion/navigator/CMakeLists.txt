# Standalone Motion::ArcSolver build (135-002), mirroring
# src/motion/planner/CMakeLists.txt's own standalone-project pattern:
# host-only, no firmware, no Python in the build/test path. Configure/
# build/test:
#
#   cmake -S src/motion/navigator -B src/motion/navigator/build
#   cmake --build src/motion/navigator/build --target navigator_tests
#
# `navigator_tests` builds every test executable and runs them via ctest,
# exiting nonzero on any failure.
#
# A SEPARATE CMake project from src/motion/CMakeLists.txt's own
# `motion_tests` (body_kinematics/odometry only -- explicitly does not
# depend on planner/) and from src/motion/planner/CMakeLists.txt's own
# `motion_planner` project. Chosen (135-002 ticket's own build-list
# decision, see that ticket's Completion Notes) over folding arc_solver.cpp
# into `motion_tests` because ticket 003's Motion::Navigator, the next
# occupant of this directory, depends on Motion::Planner's move()/
# TickResult -- a dependency `motion_tests` deliberately does not carry
# (src/motion/DESIGN.md §4: "Separate from, and does not depend on,
# planner/'s own CMake project"). A sibling standalone project here can add
# that dependency (e.g. via add_subdirectory on planner/) when ticket 003
# needs it, without disturbing motion_tests' own narrower scope.
#
# THIS ticket's own arc_solver.cpp has zero dependency on Motion::Planner
# or Types::RobotState -- no src/firm include root is needed here, unlike
# planner/CMakeLists.txt's own (which adds src/firm for the one sanctioned
# types/robot_state.h include).
cmake_minimum_required(VERSION 3.16)
project(motion_navigator CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra)

add_library(navigator STATIC
  arc_solver.cpp
)
target_include_directories(navigator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

enable_testing()
set(NAVIGATOR_TESTS arc_solver_test)
foreach(test_name IN LISTS NAVIGATOR_TESTS)
  add_executable(${test_name} tests/${test_name}.cpp)
  target_link_libraries(${test_name} PRIVATE navigator)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

add_custom_target(navigator_tests
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS ${NAVIGATOR_TESTS}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
