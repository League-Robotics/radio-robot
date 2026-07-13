cmake_minimum_required(VERSION 3.16)
project(radio_robot_drive_host CXX)

# C++ standard matched to the firmware (c++20) -- mirrors tests/_infra/sim/
# CMakeLists.txt's own rationale: shared source/ code (here, source/drive/)
# and vendored Ruckig must compile identically here and on hardware.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Root of the firmware repo: great-grandparent of this CMakeLists.txt's
# directory (tests/_infra/drive/ -> tests/_infra/ -> tests/ -> repo root).
# ---------------------------------------------------------------------------
get_filename_component(REPO_ROOT "${CMAKE_SOURCE_DIR}/../../.." ABSOLUTE)
set(SOURCE_DIR "${REPO_ROOT}/source")

# ---------------------------------------------------------------------------
# Explicit source list -- source/drive/ compiles STANDALONE (ticket 100-002's
# own isolation boundary, SUC-008, enforced by tests/sim/unit/
# test_drive_isolation.py: nothing under source/drive/ may #include anything
# outside source/drive/, libc/libm, or libraries/ruckig). Unlike tests/
# _infra/sim/CMakeLists.txt's much larger firmware_host build (kinematics/,
# motion/, subsystems/, hal/, commands/, runtime/, messages/, ...), this
# build needs ONLY the six source/drive/*.cpp files, the vendored Ruckig
# sources, and drive_api.cpp itself -- no HOST_BUILD/ROBOT_DEV_BUILD defines,
# no codegen step (just build-drive, the justfile recipe, runs no gen_*.py
# scripts, unlike build-sim).
#
# Present:
#   drive/{arc_math,master_profile,drivetrain,motion_plan,policy,tracker}.cpp
#                                            -- ticket 100-002..005's drive::
#                                               core (pure math + the
#                                               Drivetrain/MotionPlan facade,
#                                               no CODAL deps)
#   libraries/ruckig/src/*.cpp              -- the vendored jerk-limited
#                                               trajectory solver
#                                               (master_profile.h's
#                                               ruckig::Ruckig<1> member)
#   drive_api.cpp                           -- this ticket's C ABI
#
# Absent (deliberately -- everything else in source/ that a Drive:: caller
# would eventually need, e.g. the wire adapter, is OUTSIDE source/drive/ and
# is ticket 100-007's concern, not tier-0's):
#   messages/*, subsystems/*, hal/*, commands/*, runtime/*, motion/*,
#   kinematics/*, com/*, types/*, main.cpp -- none of these are reachable
#   from source/drive/'s own headers (the isolation grep test's whole point).
# ---------------------------------------------------------------------------
set(DRIVE_SOURCES
    "${SOURCE_DIR}/drive/arc_math.cpp"
    "${SOURCE_DIR}/drive/master_profile.cpp"
    "${SOURCE_DIR}/drive/drivetrain.cpp"
    "${SOURCE_DIR}/drive/motion_plan.cpp"
    "${SOURCE_DIR}/drive/policy.cpp"
    "${SOURCE_DIR}/drive/tracker.cpp"
)
file(GLOB RUCKIG_SOURCES "${REPO_ROOT}/libraries/ruckig/src/*.cpp")

add_library(drive_host SHARED
    ${DRIVE_SOURCES}
    ${RUCKIG_SOURCES}
    "${CMAKE_SOURCE_DIR}/drive_api.cpp"
)

# -fno-exceptions -fno-rtti: the firmware's EXACT build constraints
# (source/drive/'s own header comments -- e.g. master_profile.h's "the same
# viability jerk_trajectory.h's own class comment documents for the SAME
# vendored library under this firmware's exact flags"), matching every
# tests/sim/unit/test_drive_*.py ad hoc compile-and-run harness's own
# _CONSTRAINT_FLAGS. Catches any accidental exception/RTTI dependency
# regression that a plain default-flags build would silently let through.
target_compile_options(drive_host PRIVATE -fno-exceptions -fno-rtti)

# libraries/ruckig/include: vendored jerk-limited trajectory generator
# (master_profile.h's #include "ruckig/ruckig.hpp").
target_include_directories(drive_host PRIVATE
    "${SOURCE_DIR}"
    "${REPO_ROOT}/libraries/ruckig/include"
)
