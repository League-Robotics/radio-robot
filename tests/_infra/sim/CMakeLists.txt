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
#   SIM    (default here) — host build: io/sim/ device impls (MockHAL etc.);
#          io/real/ device impls are NOT globbed (NezhaHAL/I2CBus/Motor.cpp are
#          CODAL-only and would not link host-side).
#   REAL   — firmware build (driven from the root CMakeLists.txt + build.py);
#          io/real/ device impls, io/sim/ excluded.
#   REPLAY — SIM source set plus the io/ReplayHAL stub (Phase F TLM replay).
#
# PRODUCTION_BUILD and BENCH_OTOS_ENABLED stay ORTHOGONAL to ROBOT_RUN_MODE —
# they are not folded into this variable.
# ---------------------------------------------------------------------------
set(ROBOT_RUN_MODE "SIM" CACHE STRING "Build-target run mode: REAL | SIM | REPLAY")

# ---------------------------------------------------------------------------
# Collect common firmware sources (app/, control/, robot/), then exclude
# CODAL-dependent files that can't be stubbed at file level.
# DebugCommandable.cpp guards all CODAL-dependent includes/handlers with
# #ifndef HOST_BUILD so it can be compiled in the host build.
# ---------------------------------------------------------------------------
file(GLOB APP_SOURCES     "${REPO_ROOT}/source/app/*.cpp")
file(GLOB CONTROL_SOURCES "${REPO_ROOT}/source/control/*.cpp")
file(GLOB ROBOT_SOURCES   "${REPO_ROOT}/source/robot/*.cpp")
# 041-001 (Phase C): source/state/ holds the EKF (moved from control/) and the
# new PhysicalStateEstimate wrapper. The control/ glob no longer picks up
# EKF.cpp (the file is deleted from control/), so it must be globbed here.
file(GLOB STATE_SOURCES   "${REPO_ROOT}/source/state/*.cpp")
# 042-001 (Phase D): source/superstructure/ holds Superstructure.cpp (Seam 3).
file(GLOB SUPERSTRUCTURE_SOURCES "${REPO_ROOT}/source/superstructure/*.cpp")

list(FILTER APP_SOURCES     EXCLUDE REGEX ".*/WedgeTest\\.cpp$")
list(FILTER CONTROL_SOURCES EXCLUDE REGEX ".*/LoopScheduler\\.cpp$")

# ---------------------------------------------------------------------------
# Device-layer source set by ROBOT_RUN_MODE.
#
# SIM / REPLAY: glob the host (sim) device impls under source/io/sim/. The
# io/real/ tree (NezhaHAL.cpp, I2CBus.cpp, Motor.cpp, MotorBusDiagnostics.cpp,
# the comms files, ...) is deliberately NOT globbed — those are CODAL-only and
# would fail to link host-side. BenchOtosSensor.cpp is the one io/real/ file the
# host build needs (the sim_bench_otos_* hooks), so it is added explicitly below.
#
# REPLAY additionally compiles the io/ReplayHAL stub (Phase F TLM replay; no-op
# for now). It is a header-only no-op HAL whose .cpp anchors RobotMode::REPLAY.
# ---------------------------------------------------------------------------
file(GLOB IO_DEVICE_SOURCES "${REPO_ROOT}/source/io/sim/*.cpp")

set(REPLAY_SOURCES "")
if(ROBOT_RUN_MODE STREQUAL "REPLAY")
    list(APPEND REPLAY_SOURCES "${REPO_ROOT}/source/io/ReplayHAL.cpp")
endif()

add_library(firmware_host SHARED
    ${APP_SOURCES}
    ${CONTROL_SOURCES}
    ${ROBOT_SOURCES}
    ${STATE_SOURCES}
    ${SUPERSTRUCTURE_SOURCES}
    ${IO_DEVICE_SOURCES}
    ${REPLAY_SOURCES}
    # BenchOtosSensor lives in source/io/real/ (not io/sim/) so it is not globbed
    # by IO_DEVICE_SOURCES. Add it explicitly so the host build includes it — the
    # sim_bench_otos_* hooks drive it directly. It has no CODAL/I2CBus link deps.
    "${REPO_ROOT}/source/io/real/BenchOtosSensor.cpp"
    # NOTE (039-001): MotorBusDiagnostics.cpp is deliberately NOT added to the
    # host build. It is a REAL/firmware-only adapter (architecture §3 places it
    # in io/real/) that forwards to I2CBus, whose definitions (I2CBus.cpp) are
    # not compiled host-side — linking it here fails with undefined I2CBus
    # symbols. The host build never constructs it (only NezhaHAL does, which is
    # firmware-only); MockHAL leaves MotorController::_busDiag null, so the
    # enc_wedged stats read zeros exactly as before. The firmware (CODAL) build
    # picks it up via RECURSIVE_FIND_FILE over source/.
    "${CMAKE_SOURCE_DIR}/sim_api.cpp"
)

target_include_directories(firmware_host PRIVATE
    "${REPO_ROOT}/source"
    "${REPO_ROOT}/source/app"
    "${REPO_ROOT}/source/control"
    "${REPO_ROOT}/source/io"
    "${REPO_ROOT}/source/io/capability"
    "${REPO_ROOT}/source/io/real"
    "${REPO_ROOT}/source/io/sim"
    "${REPO_ROOT}/source/robot"
    "${REPO_ROOT}/source/state"
    "${REPO_ROOT}/source/superstructure"
    "${REPO_ROOT}/source/types"
)
