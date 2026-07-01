cmake_minimum_required(VERSION 3.16)
project(radio_robot_host CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DHOST_BUILD=1)


# ---------------------------------------------------------------------------
# Root of the firmware repo: great-grandparent of this CMakeLists.txt's directory.
# CMAKE_SOURCE_DIR is tests/_infra/sim/; repo root is three levels up.
# ---------------------------------------------------------------------------
get_filename_component(REPO_ROOT "${CMAKE_SOURCE_DIR}/../../.." ABSOLUTE)

# ---------------------------------------------------------------------------
# ROBOT_RUN_MODE (039-005): explicit build-target run mode replacing the old
# glob-then-filter "exclude hal/mock" pattern.
#
#   SIM    (default here) — host build: hal/sim/ device impls (MockHAL etc.);
#          hal/real/ device impls are NOT globbed (NezhaHAL/I2CBus/Motor.cpp are
#          CODAL-only and would not link host-side).
#   REAL   — firmware build (driven from the root CMakeLists.txt + build.py);
#          hal/real/ device impls, hal/sim/ excluded.
#   REPLAY — SIM source set plus the hal/ReplayHAL stub (Phase F TLM replay).
#
# PRODUCTION_BUILD and BENCH_OTOS_ENABLED stay ORTHOGONAL to ROBOT_RUN_MODE —
# they are not folded into this variable.
# ---------------------------------------------------------------------------
set(ROBOT_RUN_MODE "SIM" CACHE STRING "Build-target run mode: REAL | SIM | REPLAY")

# ---------------------------------------------------------------------------
# Collect common firmware sources (commands/, control/, robot/), then exclude
# CODAL-dependent files that can't be stubbed at file level.
# DebugCommands.cpp guards all CODAL-dependent includes/handlers with
# #ifndef HOST_BUILD so it can be compiled in the host build.
# ---------------------------------------------------------------------------
# Command handlers + dispatch infrastructure live in source/commands/
# (CommandProcessor, CommandQueue, the *Commands handler sets, DebugCommands,
# MotionCommand). The firmware (CODAL) build discovers these via
# RECURSIVE_FIND_FILE over source/.
file(GLOB COMMANDS_SOURCES "${REPO_ROOT}/source/commands/*.cpp")
file(GLOB CONTROL_SOURCES "${REPO_ROOT}/source/control/*.cpp")
# Kinematics (body/wheel geometry math) moved out of control/ into its own dir.
# Pure math, no CODAL deps, so it builds host-side. The firmware (CODAL) build
# discovers these via RECURSIVE_FIND_FILE over source/.
file(GLOB KINEMATICS_SOURCES "${REPO_ROOT}/source/kinematics/*.cpp")
# source/robot/ includes NezhaHAL.cpp and MecanumHAL.cpp (055-001: moved from
# hal/real/) which are CODAL-dependent and excluded below.
file(GLOB ROBOT_SOURCES   "${REPO_ROOT}/source/robot/*.cpp")
# 041-001 (Phase C): source/state/ holds the EKF (moved from control/) and the
# new PhysicalStateEstimate wrapper. The control/ glob no longer picks up
# EKF.cpp (the file is deleted from control/), so it must be globbed here.
file(GLOB STATE_SOURCES   "${REPO_ROOT}/source/state/*.cpp")
# 042-001 (Phase D): source/superstructure/ holds Superstructure.cpp (Seam 3).
file(GLOB SUPERSTRUCTURE_SOURCES "${REPO_ROOT}/source/superstructure/*.cpp")
# 043-001 (Phase E): source/subsystems/ holds the thin sensor subsystems
# (LineSensor/ColorSensor/Ports). Globbed recursively so drive/ and gripper/
# subdirs added in later Phase E tickets are picked up too. The firmware (CODAL)
# build discovers these via RECURSIVE_FIND_FILE; the host build needs them here.
file(GLOB_RECURSE SUBSYSTEM_SOURCES "${REPO_ROOT}/source/subsystems/*.cpp")

# WedgeTest.cpp and LoopScheduler.cpp live in source/robot/; both are
# CODAL-dependent (MicroBit fiber / diagnostic APIs) and can't build host-side,
# so exclude them from the robot glob.
list(FILTER ROBOT_SOURCES    EXCLUDE REGEX ".*/WedgeTest\\.cpp$")
list(FILTER ROBOT_SOURCES    EXCLUDE REGEX ".*/LoopScheduler\\.cpp$")
# NezhaHAL.cpp and MecanumHAL.cpp moved to source/robot/ (055-001); they are
# CODAL-dependent hardware drivers and must not compile host-side.
list(FILTER ROBOT_SOURCES EXCLUDE REGEX ".*/NezhaHAL\\.cpp$")
list(FILTER ROBOT_SOURCES EXCLUDE REGEX ".*/MecanumHAL\\.cpp$")

# ---------------------------------------------------------------------------
# Device-layer source set by ROBOT_RUN_MODE.
#
# SIM / REPLAY: glob the host (sim) device impls under source/hal/sim/. The
# hal/real/ tree (I2CBus.cpp, Motor.cpp, MotorBusDiagnostics.cpp, the comms
# files, ...) is deliberately NOT globbed — those are CODAL-only and would
# fail to link host-side. NezhaHAL/MecanumHAL moved to source/robot/ (055-001)
# and are excluded by the FILTER lines above. BenchOtosSensor.cpp is the one
# hal/real/ file the host build needs (the sim_bench_otos_* hooks), so it is
# added explicitly below.
#
# REPLAY additionally compiles the hal/ReplayHAL stub (Phase F TLM replay; no-op
# for now). It is a header-only no-op HAL whose .cpp anchors RobotMode::REPLAY.
# ---------------------------------------------------------------------------
file(GLOB IO_DEVICE_SOURCES "${REPO_ROOT}/source/hal/sim/*.cpp")

set(REPLAY_SOURCES "")
if(ROBOT_RUN_MODE STREQUAL "REPLAY")
    list(APPEND REPLAY_SOURCES "${REPO_ROOT}/source/hal/ReplayHAL.cpp")
endif()

add_library(firmware_host SHARED
    ${COMMANDS_SOURCES}
    ${CONTROL_SOURCES}
    ${KINEMATICS_SOURCES}
    ${ROBOT_SOURCES}
    ${STATE_SOURCES}
    ${SUPERSTRUCTURE_SOURCES}
    ${SUBSYSTEM_SOURCES}
    ${IO_DEVICE_SOURCES}
    ${REPLAY_SOURCES}
    # BenchOtosSensor lives in source/hal/real/ (not hal/sim/) so it is not globbed
    # by IO_DEVICE_SOURCES. Add it explicitly so the host build includes it — the
    # sim_bench_otos_* hooks drive it directly. It has no CODAL/I2CBus link deps.
    "${REPO_ROOT}/source/hal/real/BenchOtosSensor.cpp"
    # NOTE (039-001): MotorBusDiagnostics.cpp is deliberately NOT added to the
    # host build. It is a REAL/firmware-only adapter (architecture §3 places it
    # in hal/real/) that forwards to I2CBus, whose definitions (I2CBus.cpp) are
    # not compiled host-side — linking it here fails with undefined I2CBus
    # symbols. The host build never constructs it (only NezhaHAL does, which is
    # firmware-only); MockHAL leaves MotorController::_busDiag null, so the
    # enc_wedged stats read zeros exactly as before. The firmware (CODAL) build
    # picks it up via RECURSIVE_FIND_FILE over source/.
    "${CMAKE_SOURCE_DIR}/sim_api.cpp"
    # 056-003: C-ABI shims for generated message type tests + generated-side
    # static_assert layout-compat checks (bridging messages/common.h types).
    "${CMAKE_SOURCE_DIR}/message_test_api.cpp"
    # 057-003: C-ABI shims for Sensors subsystem isolation tests.
    "${CMAKE_SOURCE_DIR}/sensors_api.cpp"
    # 057-004: C-ABI shims for Drive subsystem isolation tests.
    "${CMAKE_SOURCE_DIR}/drive_api.cpp"
    # 059-001: C-ABI shims for Planner subsystem isolation tests.
    "${CMAKE_SOURCE_DIR}/planner_api.cpp"
    # 059-003: C-ABI shims for BusDrain (command-queue bus drain/route) tests.
    "${CMAKE_SOURCE_DIR}/bus_drain_api.cpp"
    # 059-004: C-ABI shims for bottom-up config routing tests (Robot + CfgCtx wiring).
    "${CMAKE_SOURCE_DIR}/config_routing_api.cpp"
)

target_include_directories(firmware_host PRIVATE
    "${REPO_ROOT}/source"
    "${REPO_ROOT}/source/commands"
    "${REPO_ROOT}/source/control"
    "${REPO_ROOT}/source/kinematics"
    "${REPO_ROOT}/source/hal"
    "${REPO_ROOT}/source/hal/capability"
    "${REPO_ROOT}/source/hal/real"
    "${REPO_ROOT}/source/hal/sim"
    "${REPO_ROOT}/source/com"
    "${REPO_ROOT}/source/robot"
    "${REPO_ROOT}/source/state"
    "${REPO_ROOT}/source/superstructure"
    # 043-001 (Phase E): subsystem dirs so each subsystem .cpp resolves its own
    # same-directory header (#include "LineSensor.h" etc.).
    "${REPO_ROOT}/source/subsystems/sensors"
    # 043-002/060-006: Drive subsystem dir (Drive2.h renamed to Drive.h in 060-006).
    "${REPO_ROOT}/source/subsystems/drive"
    # 043-003 (Phase E): Gripper subsystem dir (#include "Gripper.h").
    "${REPO_ROOT}/source/subsystems/gripper"
    "${REPO_ROOT}/source/types"
    # 049-002: cmon-pid float-adapted vendored header (Sprint 049, Phase A).
    "${REPO_ROOT}/libraries/cmon-pid"
    # 050-002: TinyEKF header-only LA backend (Sprint 050, Phase B).
    "${REPO_ROOT}/libraries/tinyekf"
    # 056-003: generated C++11 POD message headers (source/messages/*.h).
    "${REPO_ROOT}/source/messages"
)
