cmake_minimum_required(VERSION 3.16)
project(radio_robot_host CXX)

# C++ standard matched to the firmware (the ARM build's root
# CMakeLists.txt appends -std=c++20 over the vendored codal target's
# c++11) -- this host-sim build must match so shared source/ code compiles
# identically here and on hardware.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# HOST_BUILD gates the host-only forks that already exist in source/ -- the
# same define every tests/sim/*.py host-build harness compiles with (see
# e.g. tests/sim/system/test_straight_twist.py's own subprocess compile
# invocation, which this CMakeLists.txt's source list mirrors).
add_definitions(-DHOST_BUILD=1)

# ---------------------------------------------------------------------------
# Repo root: grandparent of this CMakeLists.txt's directory
# (src/sim/CMakeLists.txt -> src/sim -> src -> repo root is TWO levels up
# from CMAKE_SOURCE_DIR, which IS this directory).
# ---------------------------------------------------------------------------
get_filename_component(REPO_ROOT "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)
set(SOURCE_DIR "${REPO_ROOT}/src/firm")
set(SUPPORT_DIR "${REPO_ROOT}/src/tests/sim/support")
set(PLANT_DIR "${REPO_ROOT}/src/tests/sim/plant")
set(RUCKIG_DIR "${REPO_ROOT}/vendor/ruckig")

# ---------------------------------------------------------------------------
# Explicit source list -- no glob-then-filter. Mirrors
# tests/sim/system/test_straight_twist.py's own compile recipe (the sibling
# test_*.py scenario harnesses under tests/sim/system/ all compile this
# same dependency graph, ad hoc, via subprocess -- this CMakeLists.txt is
# the SAME graph, standing as a real, reusable shared library instead of a
# one-off throwaway binary per test). A file missing from this list
# surfaces as a link error (an unresolved symbol), not a silent omission.
#
# Present:
#   app/*.cpp                 -- Comms/Deadman/Drive/Odometry/Preamble/
#                                 RobotLoop/Telemetry (the single-loop
#                                 rebuild's whole App:: graph)
#   devices/velocity_pid.cpp, devices/nezha_motor.cpp, devices/otos.cpp,
#   devices/color_sensor.cpp, devices/line_sensor.cpp
#                              -- the device leaves SimPlant answers wire
#                                 transactions for
#   messages/wire.cpp, messages/wire_runtime.cpp
#                              -- the generated + hand-written codec
#                                 primitives
#   kinematics/body_kinematics.cpp
#                              -- pure math, no CODAL deps
#   motion/jerk_trajectory.cpp -- Motion::JerkTrajectory (109-001), the
#                                 restored Ruckig<1> wrapper; HOST_BUILD-pure,
#                                 no CODAL deps
#   vendor/ruckig/src/*.cpp -- the vendored Ruckig solver sources
#                                 Motion::JerkTrajectory links against, listed
#                                 explicitly (not globbed) matching this
#                                 file's own convention
#   sim_ctypes.cpp             -- this ticket's C ABI (108-005)
#   sim_plant.cpp               -- ticket 108-002's SimPlant (tests/_infra/sim/)
#   sim_clock.cpp                -- ticket 108-010's TestSim::SimClock/
#                                 SimSleeper (tests/_infra/sim/), the
#                                 Devices::Clock/Sleeper fakes
#   tests/sim/support/wire_test_codec.cpp
#                              -- TestSupport::armor*Command()/decodeOutboundLine()
#   tests/sim/plant/{wheel,otos}_plant.cpp
#                              -- the reused, deterministic physics models
#
# Absent (deliberately):
#   devices/microbit_clock.cpp, devices/microbit_i2c_bus.cpp
#                              -- the REAL CODAL-only forks (MicroBit.h)
#   com/*.cpp                 -- real serial/radio transport (CODAL-only);
#                                 SimHarness uses TestSupport::FakeTransport
#                                 instead
#   config/boot_config.cpp    -- bakes in the ACTIVE ROBOT JSON at ARM
#                                 build time; not part of the sim graph
#   main.cpp                  -- the ARM entry point (constructs uBit,
#                                 the real I2C bus/transports -- none of
#                                 which link host-side)
#   messages/layout_checks.cpp -- a static_assert-only TU anchor with zero
#                                 runtime symbols; not needed for a
#                                 SHARED-library link the way it is for a
#                                 from-scratch harness binary's own build
#                                 line (harmless either way -- omitted here
#                                 purely because nothing in this library's
#                                 link needs it).
# ---------------------------------------------------------------------------
set(APP_SOURCES
    "${SOURCE_DIR}/app/comms.cpp"
    "${SOURCE_DIR}/app/deadman.cpp"
    "${SOURCE_DIR}/app/drive.cpp"
    "${SOURCE_DIR}/app/heading_source.cpp"
    "${SOURCE_DIR}/app/odometry.cpp"
    "${SOURCE_DIR}/app/pilot.cpp"
    "${SOURCE_DIR}/app/preamble.cpp"
    "${SOURCE_DIR}/app/robot_loop.cpp"
    "${SOURCE_DIR}/app/telemetry.cpp"
)
set(DEVICE_SOURCES
    "${SOURCE_DIR}/devices/velocity_pid.cpp"
    "${SOURCE_DIR}/devices/nezha_motor.cpp"
    "${SOURCE_DIR}/devices/otos.cpp"
    "${SOURCE_DIR}/devices/color_sensor.cpp"
    "${SOURCE_DIR}/devices/line_sensor.cpp"
)
set(MESSAGE_SOURCES
    "${SOURCE_DIR}/messages/wire.cpp"
    "${SOURCE_DIR}/messages/wire_runtime.cpp"
)
set(KINEMATICS_SOURCES
    "${SOURCE_DIR}/kinematics/body_kinematics.cpp"
)
set(MOTION_SOURCES
    "${SOURCE_DIR}/motion/jerk_trajectory.cpp"
    "${SOURCE_DIR}/motion/executor.cpp"
)
# Explicit list (not a glob), matching this file's own house style --
# mirrors the vendored library's 11 core state-to-state solver sources
# (see vendor/ruckig/README.vendored.md's "What was vendored" list).
set(RUCKIG_SOURCES
    "${RUCKIG_DIR}/src/brake.cpp"
    "${RUCKIG_DIR}/src/position_first_step1.cpp"
    "${RUCKIG_DIR}/src/position_first_step2.cpp"
    "${RUCKIG_DIR}/src/position_second_step1.cpp"
    "${RUCKIG_DIR}/src/position_second_step2.cpp"
    "${RUCKIG_DIR}/src/position_third_step1.cpp"
    "${RUCKIG_DIR}/src/position_third_step2.cpp"
    "${RUCKIG_DIR}/src/velocity_second_step1.cpp"
    "${RUCKIG_DIR}/src/velocity_second_step2.cpp"
    "${RUCKIG_DIR}/src/velocity_third_step1.cpp"
    "${RUCKIG_DIR}/src/velocity_third_step2.cpp"
)
set(SIM_SOURCES
    "${CMAKE_SOURCE_DIR}/sim_ctypes.cpp"
    "${CMAKE_SOURCE_DIR}/sim_plant.cpp"
    "${CMAKE_SOURCE_DIR}/sim_clock.cpp"
    "${SUPPORT_DIR}/wire_test_codec.cpp"
    "${PLANT_DIR}/wheel_plant.cpp"
    "${PLANT_DIR}/otos_plant.cpp"
)

add_library(firmware_host SHARED
    ${APP_SOURCES}
    ${DEVICE_SOURCES}
    ${MESSAGE_SOURCES}
    ${KINEMATICS_SOURCES}
    ${MOTION_SOURCES}
    ${RUCKIG_SOURCES}
    ${SIM_SOURCES}
)

# Every #include in the source list above is directory-qualified relative
# to SOURCE_DIR ("app/...", "devices/...", "messages/...") -- see
# test_straight_twist.py's identical single -I source flag. SUPPORT_DIR/
# PLANT_DIR/CMAKE_SOURCE_DIR (this directory, for sim_plant.h/sim_harness.h)
# resolve the tests/sim/support, tests/sim/plant, and tests/_infra/sim
# bare-filename includes (fake_transport.h, wire_test_codec.h,
# {wheel,otos}_plant.h, sim_plant.h/sim_harness.h) the way
# test_straight_twist.py's three additional -I flags do.
target_include_directories(firmware_host PRIVATE
    "${SOURCE_DIR}"
    "${SUPPORT_DIR}"
    "${PLANT_DIR}"
    "${CMAKE_SOURCE_DIR}"
    "${RUCKIG_DIR}/include"
)
