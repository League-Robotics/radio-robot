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
# 122-001 (motion-library extraction): src/motion is a SIBLING of src/firm,
# not a child (sprint 122 architecture, Design Rationale Decision 3) --
# MOTION_DIR is therefore its own path, not SOURCE_DIR-relative.
set(MOTION_DIR "${REPO_ROOT}/src/motion")

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
#   app/*.cpp                 -- Comms/Drive/MoveQueue/Odometry/Preamble/
#                                 RobotLoop/Telemetry (the single-loop
#                                 rebuild's whole App:: graph; 116-006:
#                                 App::MoveQueue replaces the deleted
#                                 App::Deadman)
#   src/motion/stop_condition.cpp -- Motion::StopCondition (116-002), the
#                                 per-Move stop-condition object MoveQueue
#                                 owns and drives. Moved out of SOURCE_DIR
#                                 (122-001: src/motion is now a SIBLING of
#                                 src/firm, not a child -- see MOTION_DIR
#                                 above) -- linked in unchanged otherwise.
#   src/motion/velocity_shaper.cpp -- Motion::VelocityShaper (decel-into-the-
#                                 goal campaign), the pure per-tick
#                                 commanded-speed shaping function MoveQueue
#                                 calls every Continue tick. Also under
#                                 MOTION_DIR now (122-001).
#   devices/nezha_motor.cpp, devices/otos.cpp, devices/color_sensor.cpp,
#   devices/line_sensor.cpp
#                              -- the device leaves SimPlant answers wire
#                                 transactions for
#   src/motion/wheel_velocity_pid.cpp -- Motion::WheelVelocityPid (125-003:
#                                 relocated from devices/velocity_pid.cpp --
#                                 App::Drive's interim closed loop this
#                                 sprint). Under MOTION_DIR now.
#   messages/wire.cpp, messages/wire_runtime.cpp
#                              -- the generated + hand-written codec
#                                 primitives
#   src/motion/body_kinematics.cpp
#                              -- pure math, no CODAL deps. Moved out of
#                                 SOURCE_DIR/kinematics/ (122-001: that
#                                 directory is retired, folded flat into
#                                 src/motion) -- under MOTION_DIR now.
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
    "${SOURCE_DIR}/app/drive.cpp"
    "${SOURCE_DIR}/app/preamble.cpp"
    "${SOURCE_DIR}/app/robot_loop.cpp"
    "${SOURCE_DIR}/app/telemetry.cpp"
)
# motion/stop_condition.cpp (116-002) -- Motion::MoveQueue (MOTION_SOURCES,
# below) owns and drives one Motion::StopCondition per active Move; this TU
# must link into the host lib too. motion/velocity_shaper.cpp (decel-into-
# the-goal campaign) -- MoveQueue's own per-tick commanded-speed shaping,
# the SAME "MoveQueue links it" reasoning as stop_condition.cpp. Both live
# under MOTION_DIR (122-001: src/motion, a sibling of src/firm), not
# SOURCE_DIR.
#
# move_queue.cpp/odometry.cpp/state_estimator.cpp (122-002, motion-library
# extraction ticket 2): moved from SOURCE_DIR/app/ into MOTION_DIR, behind
# the velocity-sink boundary (motion/wheel_sink.h) -- App::Drive
# (APP_SOURCES, above) is now the one WheelSink implementation these link
# against. The OTOS-perception half odometry.cpp used to also define
# (applyOtosSample()) is now a private RobotLoop method (robot_loop.cpp,
# APP_SOURCES) -- Motion::Odometry may not depend on Devices::Otos/
# App::Telemetry::Frame.
set(MOTION_SOURCES
    "${MOTION_DIR}/stop_condition.cpp"
    "${MOTION_DIR}/velocity_shaper.cpp"
    "${MOTION_DIR}/move_queue.cpp"
    "${MOTION_DIR}/odometry.cpp"
    "${MOTION_DIR}/state_estimator.cpp"
    # 125-003: relocated from devices/velocity_pid.cpp
    # (Devices::MotorVelocityPid -> Motion::WheelVelocityPid) -- App::Drive
    # (APP_SOURCES, above) holds the interim instances this sprint.
    "${MOTION_DIR}/wheel_velocity_pid.cpp"
)
set(DEVICE_SOURCES
    "${SOURCE_DIR}/devices/nezha_motor.cpp"
    "${SOURCE_DIR}/devices/otos.cpp"
    "${SOURCE_DIR}/devices/color_sensor.cpp"
    "${SOURCE_DIR}/devices/line_sensor.cpp"
)
# config/persisted_tuning.cpp (114-004) -- App::RobotLoop (APP_SOURCES,
# above) now #includes config/persisted_tuning.h and calls its pure
# serializeSnapshot()/Config::TuningStore seam directly, so this TU must
# link into the host lib too -- UNLIKE config/boot_config.cpp (still
# absent, see this file's own "Absent" note below: that one bakes in the
# active robot JSON at ARM build time and has no sim role at all).
# persisted_tuning.cpp's own #ifndef HOST_BUILD block (the ARM-only
# MicroBitStorage adapter) compiles out entirely under this file's
# -DHOST_BUILD=1, mirroring app/comms.cpp's identical SerialTransport/
# RadioTransport guard.
set(CONFIG_SOURCES
    "${SOURCE_DIR}/config/persisted_tuning.cpp"
)
set(MESSAGE_SOURCES
    "${SOURCE_DIR}/messages/wire.cpp"
    "${SOURCE_DIR}/messages/wire_runtime.cpp"
)
# body_kinematics.cpp also moved under MOTION_DIR (122-001) -- it lived at
# SOURCE_DIR/kinematics/ (a child of src/firm) before this ticket; the
# kinematics/ directory itself is retired (folded flat into src/motion).
# KINEMATICS_SOURCES keeps its own variable name (nothing downstream needs
# renaming to know it's now motion-owned).
set(KINEMATICS_SOURCES
    "${MOTION_DIR}/body_kinematics.cpp"
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
    ${MOTION_SOURCES}
    ${DEVICE_SOURCES}
    ${CONFIG_SOURCES}
    ${MESSAGE_SOURCES}
    ${KINEMATICS_SOURCES}
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
#
# "${REPO_ROOT}/src" (122-001): src/motion is now a SIBLING of src/firm, not
# a child -- a qualified `#include "motion/stop_condition.h"` (the SAME
# text these headers used when they lived under src/firm/motion/) resolves
# via THIS root, since "motion" is a top-level dir under src/ now instead
# of a child of SOURCE_DIR. Zero #include text churn in src/firm callers
# (app/move_queue.h etc.) that already said `#include "motion/...h"`.
target_include_directories(firmware_host PRIVATE
    "${SOURCE_DIR}"
    "${SUPPORT_DIR}"
    "${PLANT_DIR}"
    "${CMAKE_SOURCE_DIR}"
    "${REPO_ROOT}/src"
)
