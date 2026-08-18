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
# Repo root: FOUR levels above this CMakeLists.txt's directory
# (src/firm/platform/host -> src/firm/platform -> src/firm -> src -> repo
# root), CMAKE_SOURCE_DIR being this directory itself. It was TWO levels
# while this tree lived at src/sim; the host platform is a child of
# src/firm now, so the walk-up is correspondingly longer.
# ---------------------------------------------------------------------------
get_filename_component(REPO_ROOT "${CMAKE_SOURCE_DIR}/../../../.." ABSOLUTE)
set(SOURCE_DIR "${REPO_ROOT}/src/firm")
set(SUPPORT_DIR "${REPO_ROOT}/src/tests/sim/support")
set(PLANT_DIR "${REPO_ROOT}/src/tests/sim/plant")
# src/firm/motion/ is GONE (the DifferentialDrive-kernel exploratory
# rewrite): planner/navigator/odometry/estimation folded into
# Control::DifferentialDrive and the application layer -- there is no
# MOTION_DIR/MOTION_SOURCES any more.

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
#   core/*.cpp                -- Comms/Configurator/Preamble/RobotLoop/
#                                 Telemetry (the Core:: graph)
#   control/differential_drive.cpp
#                              -- Control::DifferentialDrive, the wheel
#                                 kernel: both motors, the encoder
#                                 split-phase schedule, the (velocity,
#                                 twist) control law, and its own fiber
#                                 (exploratory worktree,
#                                 differentialdrive-one-class-one-fiber).
#   core/boot_wiring.cpp, core/boot_calibration.cpp
#                              -- 130-002 (unify-sim-and-robot-composition-
#                                 roots.md): the ONE shared composition
#                                 root (Core::composeRobot()/RobotGraph) and
#                                 its Config::boot_config-reading
#                                 calibration helpers -- TestSim::
#                                 SimHarness (sim_harness.h) calls the SAME
#                                 composeRobot() src/firm/main.cpp does.
#   hardware/nezha/nezha_motor.cpp, hardware/generic/real_otos.cpp, hardware/planetx/color_sensor.cpp,
#   hardware/planetx/line_sensor.cpp
#                              -- the device leaves SimPlant answers wire
#                                 transactions for
#   messages/wire.cpp, messages/wire_runtime.cpp
#                              -- the generated + hand-written codec
#                                 primitives
#   src/firm/kinematics/differential.cpp
#                              -- pure math, no CODAL deps. Still its own
#                                 subsystem (kinematics_harness.cpp/
#                                 test_kinematics.py exercise it directly)
#                                 even though RobotLoop no longer calls it
#                                 (pose is OTOS-only post-motion-deletion).
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
#   platform/microbit/microbit_serial_port.cpp, microbit_radio_link.cpp,
#   microbit_banner.cpp, microbit_boot_identity.cpp
#                              -- real serial/radio transport + boot-display
#                                 UI (CODAL-only; 136-005 dissolved the
#                                 former com/*.cpp here); SimHarness uses
#                                 TestSupport::FakeTransport instead
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
#
# REVERSED by 130-002 (used to be listed "Absent"):
#   config/boot_config.cpp    -- bakes in the ACTIVE ROBOT JSON. Used to be
#                                 excluded ("not part of the sim graph");
#                                 now linked here too so both composition
#                                 roots bake the SAME robot-JSON calibration
#                                 by default (unify-sim-and-robot-
#                                 composition-roots.md work item 2). A sim
#                                 scenario needing a synthetic-plant config
#                                 still overrides via the existing
#                                 SimLoop.configure_from_robot()/
#                                 sim_configure_drive() config-push path.
# ---------------------------------------------------------------------------
set(APP_SOURCES
    "${SOURCE_DIR}/core/comms.cpp"
    "${SOURCE_DIR}/core/configurator.cpp"
    # debug.cpp (129-003): Core::debugf()'s only implementation -- ALWAYS
    # live here (HOST_BUILD is unconditionally defined for this whole
    # library, above), so SimHarness::Core::setDebugSink(&comms_) needs a
    # real definition, not the ARM-release inline no-op stub.
    "${SOURCE_DIR}/core/debug.cpp"
    # control/differential_drive.cpp (136-006, "control law out of core/ --
    # new control/ layer"): Control::DifferentialDrive, relocated verbatim
    # out of core/ -- the control law itself unchanged, only its namespace
    # and directory.
    "${SOURCE_DIR}/diffdrive/differential_drive.cpp"
    "${SOURCE_DIR}/core/preamble.cpp"
    "${SOURCE_DIR}/core/robot_loop.cpp"
    "${SOURCE_DIR}/core/telemetry.cpp"
    # 130-002 -- the shared composition root, see this file's own "Present"
    # comment above.
    "${SOURCE_DIR}/core/boot_wiring.cpp"
    "${SOURCE_DIR}/core/boot_calibration.cpp"
)
set(DEVICE_SOURCES
    "${SOURCE_DIR}/hardware/nezha/nezha_motor.cpp"
    "${SOURCE_DIR}/hardware/generic/real_otos.cpp"
    "${SOURCE_DIR}/hardware/planetx/color_sensor.cpp"
    "${SOURCE_DIR}/hardware/planetx/line_sensor.cpp"
)
# config/persisted_tuning.cpp (114-004) -- Core::RobotLoop (APP_SOURCES,
# above) now #includes config/persisted_tuning.h and calls its pure
# serializeSnapshot()/Config::TuningStore seam directly, so this TU must
# link into the host lib too. persisted_tuning.cpp's own #ifndef HOST_BUILD
# block (the ARM-only MicroBitStorage adapter) compiles out entirely under
# this file's -DHOST_BUILD=1 -- the same shape core/comms.cpp's own
# #ifndef HOST_BUILD block used to have before 136-005 deleted it outright
# (Core::SerialTransport/RadioTransport are gone, not guarded, now that
# Platform::MicroBitSerialPort/MicroBitRadioLink implement Hal::Transport
# directly).
#
# config/boot_config.cpp (130-002, REVERSED from this file's pre-130-002
# "Absent" note): bakes in the active robot JSON at ARM build time -- now
# linked host-side too, so Core::composeRobot() bakes the SAME calibration
# in both the sim and hardware composition roots by default. Has zero
# CODAL includes (see boot_config.h's own header), so nothing prevents
# linking it here.
set(CONFIG_SOURCES
    "${SOURCE_DIR}/config/persisted_tuning.cpp"
    "${SOURCE_DIR}/config/boot_config.cpp"
)
set(MESSAGE_SOURCES
    "${SOURCE_DIR}/messages/wire.cpp"
    "${SOURCE_DIR}/messages/wire_runtime.cpp"
)
# kinematics/ is a real subsystem again (the platform/hardware/hal
# reorganization): Kinematics::Model, the swappable twist<->wheel-speed map,
# with Differential (the former BodyKinematics free functions,
# unchanged math) and Mecanum behind it. The 122-era arrangement
# -- body_kinematics.cpp folded flat into src/motion -- is gone. Chassis
# geometry (track width, wheelbase) lives HERE and nowhere below it.
set(KINEMATICS_SOURCES
    "${SOURCE_DIR}/kinematics/differential.cpp"
    "${SOURCE_DIR}/kinematics/kinematics.cpp"
    "${SOURCE_DIR}/kinematics/mecanum.cpp"
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
# "${REPO_ROOT}/src" stays on the include path for any header that resolves
# via the repo's src/ root rather than SOURCE_DIR (src/firm) -- e.g. shared
# blackboard headers under src/firm/types/ reached as "firm/types/...".
target_include_directories(firmware_host PRIVATE
    "${SOURCE_DIR}"
    "${SUPPORT_DIR}"
    "${PLANT_DIR}"
    "${CMAKE_SOURCE_DIR}"
    "${REPO_ROOT}/src"
)
