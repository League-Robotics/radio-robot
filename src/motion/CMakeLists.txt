# src/motion/CMakeLists.txt -- the standalone motion_tests build (sprint
# 122 ticket 001, Design Rationale Decision 4). Plain CMake, no
# libfirmware_host, no ctypes, no Python anywhere in the build/run path --
# this is the whole point of the split (SUC-001): a fast, hardware- and
# Python-free iteration loop for the motion-control logic, independent of
# the sim harness/pytest.
#
# This is a SEPARATE CMake project from the repo root's own CMakeLists.txt
# (the ARM firmware build) and src/sim/CMakeLists.txt (the libfirmware_host
# HOST_BUILD sim library) -- configure/build it on its own:
#
#   cmake -S src/motion -B src/motion/build
#   cmake --build src/motion/build
#   cmake --build src/motion/build --target motion_tests   # build + run
#
# or, equivalently, once configured/built once:
#
#   (cd src/motion/build && ctest --output-on-failure)
#
# No Python process anywhere in that sequence.
cmake_minimum_required(VERSION 3.16)
project(motion_tests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Matches the project's actual compiled standard (gnu++20, see the repo
# root CMakeLists.txt's own override comment) closely enough for a host
# build; HOST_BUILD mirrors every other host-side harness's own define
# even though none of these three modules currently branch on it.
add_definitions(-DHOST_BUILD=1)

# ---------------------------------------------------------------------------
# Repo root: grandparent of this CMakeLists.txt's directory
# (src/motion/CMakeLists.txt -> src/motion -> src -> repo root is TWO
# levels up from CMAKE_SOURCE_DIR, which IS this directory) -- same
# convention as src/sim/CMakeLists.txt's own REPO_ROOT.
# ---------------------------------------------------------------------------
get_filename_component(REPO_ROOT "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)
set(MOTION_DIR "${CMAKE_SOURCE_DIR}")
set(HARNESS_DIR "${REPO_ROOT}/src/tests/sim/unit")

# ---------------------------------------------------------------------------
# The motion library itself -- ALL THREE moved modules (stop_condition,
# velocity_shaper, body_kinematics), proving the whole src/motion tree
# compiles standalone, not just whichever one module a given test harness
# happens to need. wheel_sink.h (the boundary header) is header-only and
# not yet load-bearing (122-001: no consumer includes it yet -- ticket 002
# is its first), so there is no .cpp for it to add here; its own compile
# check is implicit in the fact that nothing in this tree emits an error
# just from its presence, and ticket 002 will be the first TU that
# actually #includes it.
# ---------------------------------------------------------------------------
add_library(motion STATIC
    "${MOTION_DIR}/stop_condition.cpp"
    "${MOTION_DIR}/velocity_shaper.cpp"
    "${MOTION_DIR}/body_kinematics.cpp"
)

# "${REPO_ROOT}/src" resolves this tree's own qualified `#include
# "motion/stop_condition.h"` / `#include "motion/velocity_shaper.h"` style
# includes (the SAME include text these headers used back when they lived
# under src/firm/motion/ -- see the root CMakeLists.txt / src/sim/
# CMakeLists.txt's own matching comment). "${REPO_ROOT}/src/firm" resolves
# body_kinematics.h's ONE allowed src/firm dependency, `#include
# "messages/common.h"` -- the single exception the sprint's boundary rule
# permits (messages/ only, never anything else from src/firm).
target_include_directories(motion PUBLIC
    "${REPO_ROOT}/src"
    "${REPO_ROOT}/src/firm"
)

enable_testing()

# ---------------------------------------------------------------------------
# StopCondition / VelocityShaper scenario coverage -- reuses the EXISTING
# harness .cpp files under src/tests/sim/unit/ as-is (not copied/moved;
# sprint.md's Open Question 3 -- module boundary is normative, harness file
# layout is not). Each harness already carries its own main() (hand-rolled
# PASS/FAIL assertions, nonzero exit on failure) -- the same shape
# test_motion_stop_condition.py/test_motion_velocity_shaper.py's own
# pytest-subprocess compile already exercises; this target is the SAME
# coverage, reachable with no Python at all.
# ---------------------------------------------------------------------------
add_executable(motion_stop_condition_tests
    "${HARNESS_DIR}/motion_stop_condition_harness.cpp"
)
target_link_libraries(motion_stop_condition_tests PRIVATE motion)
add_test(NAME motion_stop_condition_tests COMMAND motion_stop_condition_tests)

add_executable(motion_velocity_shaper_tests
    "${HARNESS_DIR}/motion_velocity_shaper_harness.cpp"
)
target_link_libraries(motion_velocity_shaper_tests PRIVATE motion)
add_test(NAME motion_velocity_shaper_tests COMMAND motion_velocity_shaper_tests)

# motion_tests -- the one named target SUC-001 asks a developer to build
# and run with plain cmake/make, no Python: depends on both test binaries,
# then runs them via ctest as its own recipe, so `cmake --build . --target
# motion_tests` alone builds AND runs every scenario in this ticket's
# scope, exiting nonzero on any failure (ctest's own exit code). Ticket
# 002 adds the end-to-end two-chained-moves model-plant scenario to this
# same target once move_queue/wheel_plant land here.
add_custom_target(motion_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS motion_stop_condition_tests motion_velocity_shaper_tests
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
