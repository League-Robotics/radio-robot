cmake_minimum_required(VERSION 3.16)
project(radio_robot_host CXX)

# C++ standard matched to the firmware (-std=c++11 -fno-exceptions
# -fno-rtti -- see source/types/command_types.h's file header, and every
# tests/sim/unit/*_harness.cpp's own _CXX_STANDARD = "c++11").
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# HOST_BUILD gates the host-only forks (I2CBus, Types::systemClockNow, the
# PhysicsWorld/SimOdometer std::mt19937 members) — the same define every
# tests/sim/unit/*_harness.cpp compiles with. ROBOT_DEV_BUILD=1 mirrors
# codal.json's "config" object (the value the ARM firmware build itself
# uses) so the DEV command family (source/commands/dev_commands.*,
# source/dev_loop.*) compiles in here exactly as it does on-target.
add_definitions(-DHOST_BUILD=1 -DROBOT_DEV_BUILD=1)

# ---------------------------------------------------------------------------
# Root of the firmware repo: great-grandparent of this CMakeLists.txt's
# directory. CMAKE_SOURCE_DIR is tests/_infra/sim/; repo root is three
# levels up.
# ---------------------------------------------------------------------------
get_filename_component(REPO_ROOT "${CMAKE_SOURCE_DIR}/../../.." ABSOLUTE)
set(SOURCE_DIR "${REPO_ROOT}/source")

# ---------------------------------------------------------------------------
# Explicit source list — NO glob-then-filter (tests_old/_infra/sim/
# CMakeLists.txt's file(GLOB ...) + list(FILTER ... EXCLUDE REGEX ...) shape
# is deliberately NOT reused here; see clasi/sprints/081-.../architecture-
# update.md Step 4's dependency graph, which this list was cross-checked
# against file-by-file). A file missing from this list surfaces as a LINK
# error (an unresolved symbol), not a silent omission — that is the point:
# every addition below is a deliberate, reviewable decision, not whatever a
# glob happened to sweep up.
#
# Present (mirrors the architecture doc's Step 4 dependency graph):
#   kinematics/*.cpp                        -- pure math, no CODAL deps
#   motion/*.cpp                            -- 084-001's VelocityRamp/
#                                               evaluateStopCondition (pure
#                                               math, no CODAL deps)
#   subsystems/drivetrain.cpp               -- the Drivetrain faceplate
#   subsystems/sim_hardware.cpp             -- ticket 003's SimHardware
#   subsystems/pose_estimator.cpp           -- 082-002/082-003's PoseEstimator
#   subsystems/planner.cpp                  -- 084-001's goal-closure engine
#   estimation/ekf_tiny.cpp                 -- 082-001's EkfTiny (PoseEstimator's EKF core)
#   telemetry/tlm_frame.cpp                 -- 082-004's pure TLM frame formatter
#   commands/{arg_parse,command_processor,config_commands,dev_commands,
#             motion_commands,otos_commands,pose_commands,system_commands,
#             telemetry_commands}.cpp -- the command dispatch + DEV +
#                                               motion (S/T/D/STOP, 084-002)
#                                               + telemetry (STREAM/SNAP) +
#                                               config (SET/GET, 084-006) +
#                                               pose-set (SI/ZERO, 084-007) +
#                                               OTOS (OI/OZ/OR/OP/OV/OL/OA,
#                                               084-008) families
#   dev_loop.cpp                            -- ticket 002's shared loop body,
#                                               082-003's pose-estimation step,
#                                               082-004's periodic TLM step
#   hal/sim/*.cpp                           -- PhysicsWorld/SimMotor/SimOdometer
#   hal/velocity_pid.cpp                    -- ticket 001's shared PID
#   types/clock_host.cpp                    -- HOST_BUILD fake clock
#   com/i2c_bus_host.cpp                    -- HOST_BUILD scripted-fake I2CBus
#   sim_api.cpp                             -- this ticket's C ABI
#
# Absent (deliberately, per this ticket's acceptance criteria):
#   com/i2c_bus.cpp                 -- the REAL MicroBitI2C fork (CODAL-only)
#   subsystems/communicator.*       -- serial+radio comms (MicroBit.h, CODAL-only)
#   subsystems/nezha_hardware.cpp   -- the REAL I2C brick flip-flop (needs
#                                      the real I2CBus/NezhaMotor, not the sim's)
#   hal/nezha/*.cpp                 -- NezhaMotor (I2C register protocol, CODAL-only)
#   types/clock.cpp                 -- the REAL vendor-clock fork
#   main.cpp                        -- the ARM entry point (constructs uBit,
#                                      Communicator, NezhaHardware -- none of
#                                      which exist/link host-side)
# ---------------------------------------------------------------------------
file(GLOB KINEMATICS_SOURCES "${SOURCE_DIR}/kinematics/*.cpp")
file(GLOB MOTION_SOURCES     "${SOURCE_DIR}/motion/*.cpp")
file(GLOB HAL_SIM_SOURCES    "${SOURCE_DIR}/hal/sim/*.cpp")

set(FIRMWARE_SOURCES
    ${KINEMATICS_SOURCES}
    ${MOTION_SOURCES}
    "${SOURCE_DIR}/subsystems/drivetrain.cpp"
    "${SOURCE_DIR}/subsystems/sim_hardware.cpp"
    "${SOURCE_DIR}/subsystems/pose_estimator.cpp"
    "${SOURCE_DIR}/subsystems/planner.cpp"
    "${SOURCE_DIR}/estimation/ekf_tiny.cpp"
    "${SOURCE_DIR}/telemetry/tlm_frame.cpp"
    "${SOURCE_DIR}/commands/arg_parse.cpp"
    "${SOURCE_DIR}/commands/command_processor.cpp"
    "${SOURCE_DIR}/commands/config_commands.cpp"
    "${SOURCE_DIR}/commands/dev_commands.cpp"
    "${SOURCE_DIR}/commands/motion_commands.cpp"
    "${SOURCE_DIR}/commands/otos_commands.cpp"
    "${SOURCE_DIR}/commands/pose_commands.cpp"
    "${SOURCE_DIR}/commands/system_commands.cpp"
    "${SOURCE_DIR}/commands/telemetry_commands.cpp"
    "${SOURCE_DIR}/dev_loop.cpp"
    ${HAL_SIM_SOURCES}
    "${SOURCE_DIR}/hal/velocity_pid.cpp"
    "${SOURCE_DIR}/types/clock_host.cpp"
    "${SOURCE_DIR}/com/i2c_bus_host.cpp"
)

add_library(firmware_host SHARED
    ${FIRMWARE_SOURCES}
    "${CMAKE_SOURCE_DIR}/sim_api.cpp"
)

# source/commands/command_processor.h (and source/types/command_types.h)
# #include their types/ siblings by bare filename ("protocol.h",
# "command_types.h", "arg_schema.h") rather than a path-qualified
# "types/protocol.h" -- the real CMake build's RECURSIVE_FIND_DIR adds every
# header-bearing directory individually (see the root CMakeLists.txt's
# comment near "finally, find sources and includes"). SOURCE_DIR alone
# resolves every OTHER directory-qualified include in this source list
# ("hal/capability/motor.h", "subsystems/hardware.h", "messages/motor.h",
# "commands/dev_commands.h", ...) — TYPES_DIR is needed only for the bare
# forms above. Mirrors tests/sim/unit/test_dev_command_outbox.py's two -I
# flags exactly.
#
# libraries/tinyekf: header-only EKF linear-algebra backend (mirrors the root
# CMakeLists.txt's own include_directories() for the ARM build) --
# estimation/ekf_tiny.h's #include <tinyekf.h> needs this on the path.
target_include_directories(firmware_host PRIVATE
    "${SOURCE_DIR}"
    "${SOURCE_DIR}/types"
    "${REPO_ROOT}/libraries/tinyekf"
)
