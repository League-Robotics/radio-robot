# Standalone Motion::ArcSolver / Motion::Navigator build (135-002/135-003),
# mirroring src/firm/motion/planner/CMakeLists.txt's own standalone-project
# pattern: host-only, no firmware, no Python in the build/test path.
# Configure/build/test:
#
#   cmake -S src/firm/motion/navigator -B src/firm/motion/navigator/build
#   cmake --build src/firm/motion/navigator/build --target navigator_tests
#
# `navigator_tests` builds every test executable and runs them via ctest,
# exiting nonzero on any failure.
#
# A SEPARATE CMake project from src/firm/motion/CMakeLists.txt's own
# `motion_tests` (body_kinematics/odometry only -- explicitly does not
# depend on planner/) and from src/firm/motion/planner/CMakeLists.txt's own
# `motion_planner` project. Chosen (135-002 ticket's own build-list
# decision, see that ticket's Completion Notes) over folding arc_solver.cpp
# into `motion_tests` because Motion::Navigator (135-003, this ticket's own
# addition below) depends on Motion::Planner's move()/TickResult -- a
# dependency `motion_tests` deliberately does not carry (src/firm/motion/
# DESIGN.md §4: "Separate from, and does not depend on, planner/'s own
# CMake project").
#
# arc_solver.cpp itself has zero dependency on Motion::Planner or
# Types::RobotState. navigator.cpp does (see navigator.h's own header
# comment for the exact boundary), which is why this file now also builds
# a `motion_planner_lib` static library from planner/'s own sources
# directly -- 135-003's own build-list decision, per ticket 002's note
# ("link its sources directly the way planner/'s own pose_ownership_test
# target links odometry.cpp/body_kinematics.cpp") -- rather than
# add_subdirectory(../planner), which would nest a second project() and a
# second enable_testing()/CTest registration inside this one for no
# benefit ticket 003 needs.
cmake_minimum_required(VERSION 3.16)
project(motion_navigator CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra)

# Motion::Planner, built from planner/'s own sources directly (135-003) --
# navigator.cpp/navigator_test link against this for Motion::Planner/
# Types::RobotState. Same source list as planner/CMakeLists.txt's own
# `planner` library.
add_library(motion_planner_lib STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../planner/profile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../planner/shape.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../planner/estimation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../planner/planner.cpp
)
target_include_directories(motion_planner_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../planner
  # types/robot_state.h -- the one sanctioned src/firm include, same as
  # planner/CMakeLists.txt's own `planner` library.
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

add_library(navigator STATIC
  arc_solver.cpp
  navigator.cpp
)
target_include_directories(navigator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(navigator PUBLIC motion_planner_lib)

enable_testing()
set(NAVIGATOR_TESTS arc_solver_test navigator_test)
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
