# src/firm/motion/CMakeLists.txt -- the standalone motion_tests build (sprint
# 122 ticket 001, Design Rationale Decision 4). Plain CMake, no
# libfirmware_host, no ctypes, no Python anywhere in the build/run path --
# this is the whole point of the split (SUC-001): a fast, hardware- and
# Python-free iteration loop for the motion-control logic, independent of
# the sim harness/pytest.
#
# This is a SEPARATE CMake project from the repo root's own CMakeLists.txt
# (the ARM firmware build), src/firm/platform/host/CMakeLists.txt (the libfirmware_host
# HOST_BUILD sim library), and src/firm/motion/planner/CMakeLists.txt (the
# Motion::Planner standalone build, its own project entirely --
# self-contained, no dependency on this directory's `motion` library) --
# configure/build it on its own:
#
#   cmake -S src/firm/motion -B src/firm/motion/build
#   cmake --build src/firm/motion/build
#   cmake --build src/firm/motion/build --target motion_tests   # build + run
#
# or, equivalently, once configured/built once:
#
#   (cd src/firm/motion/build && ctest --output-on-failure)
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
# (src/firm/motion/CMakeLists.txt -> src/firm/motion -> src -> repo root is TWO
# levels up from CMAKE_SOURCE_DIR, which IS this directory) -- same
# convention as src/firm/platform/host/CMakeLists.txt's own REPO_ROOT.
# ---------------------------------------------------------------------------
get_filename_component(REPO_ROOT "${CMAKE_SOURCE_DIR}/../../.." ABSOLUTE)
set(MOTION_DIR "${CMAKE_SOURCE_DIR}")

# ---------------------------------------------------------------------------
# The motion library itself: the modules that remain in this tree after
# sprint 128 ticket 014 deleted the dead MoveQueue-generation stack
# (wheel_sink.h, move_queue.{h,cpp}, stop_condition.{h,cpp},
# velocity_shaper.{h,cpp} -- zero callers; Motion::Planner already wrote
# Types::RobotState::Wheel::cmdVelocity directly and RobotLoop::cycle()
# already consumed it directly, so the WheelSink boundary interface those
# three modules existed to drive was never actually used) and ticket 015
# deleted the zero-instantiation wheel_velocity_pid.{h,cpp} (125-003's
# relocated closed-loop velocity control law -- superseded before it ever
# had a caller; see src/firm/motion/DESIGN.md's "wheel control generations"
# note). Ticket 016 deleted state_estimator.{h,cpp} outright (Motion::
# StateEstimator retired from the loop, robot-state-pose-needs-exactly-
# one-writer.md -- a per-cycle computation with no consumer). What's
# left: odometry (dead-reckoning integration, RobotState::pose's sole
# writer). body_kinematics left this tree entirely in the platform/
# hardware/hal reorganization: it is src/firm/kinematics/ now, behind
# Kinematics::Model, because chassis geometry belongs to the drivetrain
# model rather than to the motion library. Odometry still needs the
# differential map to integrate, so this target links it from there.
# ---------------------------------------------------------------------------
set(KINEMATICS_DIR "${REPO_ROOT}/src/firm/kinematics")
add_library(motion STATIC
    "${KINEMATICS_DIR}/kinematics.cpp"
    "${KINEMATICS_DIR}/differential.cpp"
    "${KINEMATICS_DIR}/mecanum.cpp"
    "${MOTION_DIR}/odometry.cpp"
)

# "${REPO_ROOT}/src" resolves this tree's own qualified `#include
# "motion/..."` style includes (the SAME include text these headers used
# back when they lived under src/firm/motion/ -- see the root
# CMakeLists.txt / src/firm/platform/host/CMakeLists.txt's own matching comment).
# "${REPO_ROOT}/src/firm" resolves the ONE allowed src/firm dependency
# these modules have, `#include "messages/common.h"` (and, for the
# kinematics sources, their own `kinematics/...` self-includes) -- the
# single exception the boundary rule permits (messages/ only, never
# anything else from src/firm).
target_include_directories(motion PUBLIC
    "${REPO_ROOT}/src"
    "${REPO_ROOT}/src/firm"
)

enable_testing()

# motion_tests -- the one named target SUC-001 asks a developer to build
# and run with plain cmake/make, no Python. Sprint 128 ticket 014 deleted
# the three ctest executables this target used to depend on
# (motion_stop_condition_tests, motion_velocity_shaper_tests,
# motion_move_queue_chained_tests) along with the MoveQueue-generation
# code they exercised -- there is currently no standalone (non-pytest,
# non-ctypes) harness for the modules that remain in the `motion`
# library above (body_kinematics/odometry are exercised today through the
# pytest ctypes harness under src/tests/sim/unit/ --
# app_odometry_harness.cpp -- not through this CMake project).
# `motion_tests` therefore currently runs ctest over zero registered tests
# (a valid, if trivial, green result) rather than being removed outright:
# SUC-001's own "fast, Python-free iteration loop" contract for this tree
# is a real, live requirement even though there is nothing in THIS
# ticket's scope to attach it to (this ticket authorizes no new tests) --
# a future ticket adding standalone coverage for either remaining module
# has this target ready to attach to.
add_custom_target(motion_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
