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
# PLANT_DIR (122-002): the model plant the new end-to-end chained-moves
# scenario (motion_move_queue_chained_harness.cpp, below) links against --
# TestSim::WheelPlant is test-only physics scaffolding, reused as-is from
# src/tests/sim/plant/, never folded into the `motion` library itself.
set(PLANT_DIR "${REPO_ROOT}/src/tests/sim/plant")

# ---------------------------------------------------------------------------
# The motion library itself. 122-001 landed the three already-pure leaves
# (stop_condition, velocity_shaper, body_kinematics); 122-002 (this ticket)
# adds the remaining three modules moved off src/firm/app/ behind the
# velocity-sink boundary -- move_queue.cpp (now calls
# BodyKinematics::inverse() itself for the twist path -- the decomposition
# that used to live in App::Drive::setTwist()), odometry.cpp (the
# dead-reckoning integration half; App::applyOtosSample(), the OTOS-sampling
# half odometry.cpp used to also define, stayed base-side as a new
# src/firm/app/otos_sample.cpp), and state_estimator.cpp (now takes a plain
# Motion::StateEstimator::Input instead of App::Telemetry::Frame). wheel_sink.h
# (the boundary header) is header-only -- no .cpp to add for it; move_queue.cpp
# is its first real consumer (holds a WheelSink&).
# ---------------------------------------------------------------------------
add_library(motion STATIC
    "${MOTION_DIR}/stop_condition.cpp"
    "${MOTION_DIR}/velocity_shaper.cpp"
    "${MOTION_DIR}/body_kinematics.cpp"
    "${MOTION_DIR}/move_queue.cpp"
    "${MOTION_DIR}/odometry.cpp"
    "${MOTION_DIR}/state_estimator.cpp"
    # 125-003: relocated from src/firm/devices/velocity_pid.{h,cpp}
    # (Devices::MotorVelocityPid -> Motion::WheelVelocityPid) -- the
    # closed-loop velocity control law, verbatim except its Gains
    # parameter type (motion-local now, not Devices::Gains -- isolation
    # invariant). App::Drive holds the interim instances this sprint;
    # ticket 005 relocates ownership to Motion::MoveQueue per sprint.md
    # Decision 1.
    "${MOTION_DIR}/wheel_velocity_pid.cpp"
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

# ---------------------------------------------------------------------------
# 122-002's own end-to-end scenario (SUC-001): two chained WHEELS Moves
# through the REAL Motion::MoveQueue/Motion::Odometry against the REAL model
# plant (TestSim::WheelPlant, PLANT_DIR/wheel_plant.cpp -- reused verbatim,
# never copied). wheel_plant.cpp is test-only physics scaffolding, linked
# directly into this one executable rather than folded into the `motion`
# library itself (mirrors every src/tests/sim/unit/*.py wrapper's own
# ad hoc-compile convention for this same file).
# ---------------------------------------------------------------------------
add_executable(motion_move_queue_chained_tests
    "${HARNESS_DIR}/motion_move_queue_chained_harness.cpp"
    "${PLANT_DIR}/wheel_plant.cpp"
)
target_include_directories(motion_move_queue_chained_tests PRIVATE "${PLANT_DIR}")
target_link_libraries(motion_move_queue_chained_tests PRIVATE motion)
add_test(NAME motion_move_queue_chained_tests COMMAND motion_move_queue_chained_tests)

# motion_tests -- the one named target SUC-001 asks a developer to build
# and run with plain cmake/make, no Python: depends on all three test
# binaries, then runs them via ctest as its own recipe, so `cmake --build .
# --target motion_tests` alone builds AND runs every scenario in this
# ticket's scope, exiting nonzero on any failure (ctest's own exit code).
add_custom_target(motion_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS motion_stop_condition_tests motion_velocity_shaper_tests
            motion_move_queue_chained_tests
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
