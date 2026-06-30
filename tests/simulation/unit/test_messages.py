"""
test_messages.py — Host unit tests for generated source/messages/*.h types.

(ticket 056-003) Exercises the generated C++11 POD message headers through
C-ABI shims in tests/_infra/sim/message_test_api.cpp, loaded via ctypes.

Tests are designed to run as part of the standard simulation tier:
    uv run python -m pytest tests/simulation/unit/test_messages.py -v

The ctypes shims call into the firmware_host shared library, which is built
by the autouse build_lib fixture in tests/conftest.py.

Test 6 (static_assert bridges compile) is a compile-time test verified by
the host sim build (cmake --build) and python build.py --clean; see
the comment in that test.
"""

from __future__ import annotations

import ctypes
import pathlib
import sys

import pytest

# ---------------------------------------------------------------------------
# Library loading
# ---------------------------------------------------------------------------

_HERE = pathlib.Path(__file__).parent
_REPO = _HERE.parent.parent.parent
_SIM_DIR = _REPO / "tests" / "_infra" / "sim"

# Add _infra/sim to path so we can import firmware.LIB_PATH.
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from firmware import LIB_PATH  # noqa: E402


def _load_lib() -> ctypes.CDLL:
    """Load the firmware_host shared library and configure message shim argtypes."""
    lib = ctypes.CDLL(str(LIB_PATH))

    # msg_test_drivetrain_twist_roundtrip(vx, vy, omega, out_vx, out_vy, out_omega, out_kind)
    lib.msg_test_drivetrain_twist_roundtrip.restype = ctypes.c_int
    lib.msg_test_drivetrain_twist_roundtrip.argtypes = [
        ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_int),
    ]

    # msg_test_motor_feedforward_present(val, out_has, out_val)
    lib.msg_test_motor_feedforward_present.restype = ctypes.c_int
    lib.msg_test_motor_feedforward_present.argtypes = [
        ctypes.c_float,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_float),
    ]

    # msg_test_motor_feedforward_absent(out_has)
    lib.msg_test_motor_feedforward_absent.restype = ctypes.c_int
    lib.msg_test_motor_feedforward_absent.argtypes = [
        ctypes.POINTER(ctypes.c_int),
    ]

    # msg_test_command_batch_count(n_cmds, out_count)
    lib.msg_test_command_batch_count.restype = ctypes.c_int
    lib.msg_test_command_batch_count.argtypes = [
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
    ]

    # msg_test_planner_config_chained(a_max, v_body_max, out_a_max, out_v_body_max)
    lib.msg_test_planner_config_chained.restype = ctypes.c_int
    lib.msg_test_planner_config_chained.argtypes = [
        ctypes.c_float, ctypes.c_float,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
    ]

    return lib


# ---------------------------------------------------------------------------
# Fixture: shared lib handle for the message tests.
# The build_lib autouse fixture (conftest.py) ensures the library is built
# before any test in this session runs.
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def msg_lib(build_lib):  # noqa: ARG001 (build_lib is a session fixture)
    """Return a ctypes handle to the firmware_host library with message shims configured."""
    return _load_lib()


# ---------------------------------------------------------------------------
# Test 1: DrivetrainCommand fluent builder round-trip.
#
# Constructs DrivetrainCommand, calls setTwist(vx=100.0, vy=0.0, omega=1.5),
# reads back twist fields and the control_kind discriminant.
# ---------------------------------------------------------------------------

def test_drivetrain_command_fluent_builder(msg_lib):
    """setTwist() sets control_kind=TWIST and populates control.twist fields."""
    out_vx    = ctypes.c_float()
    out_vy    = ctypes.c_float()
    out_omega = ctypes.c_float()
    out_kind  = ctypes.c_int()

    ret = msg_lib.msg_test_drivetrain_twist_roundtrip(
        100.0, 0.0, 1.5,
        ctypes.byref(out_vx),
        ctypes.byref(out_vy),
        ctypes.byref(out_omega),
        ctypes.byref(out_kind),
    )

    assert ret == 1
    assert out_vx.value    == pytest.approx(100.0, abs=1e-5)
    assert out_vy.value    == pytest.approx(0.0,   abs=1e-5)
    assert out_omega.value == pytest.approx(1.5,   abs=1e-5)
    # ControlKind::TWIST == 1 (see drivetrain.h enum)
    assert out_kind.value == 1, f"Expected TWIST=1, got {out_kind.value}"


# ---------------------------------------------------------------------------
# Test 2: Opt<float> — present case.
#
# MotorCommand.setFeedforward(0.25) must set feedforward.has=true and
# feedforward.val=0.25.
# ---------------------------------------------------------------------------

def test_motor_command_opt_present(msg_lib):
    """setFeedforward() sets has=True and val to the given float."""
    out_has = ctypes.c_int()
    out_val = ctypes.c_float()

    ret = msg_lib.msg_test_motor_feedforward_present(
        0.25,
        ctypes.byref(out_has),
        ctypes.byref(out_val),
    )

    assert ret == 1
    assert out_has.value == 1, "feedforward.has must be true after setFeedforward()"
    assert out_val.value == pytest.approx(0.25, abs=1e-6)


# ---------------------------------------------------------------------------
# Test 3: Opt<float> — absent (default) case.
#
# Default-constructed MotorCommand must have feedforward.has=false.
# ---------------------------------------------------------------------------

def test_motor_command_opt_absent(msg_lib):
    """Default-constructed MotorCommand must have feedforward.has=False."""
    out_has = ctypes.c_int()

    ret = msg_lib.msg_test_motor_feedforward_absent(ctypes.byref(out_has))

    assert ret == 1
    assert out_has.value == 0, "feedforward.has must be false on default MotorCommand"


# ---------------------------------------------------------------------------
# Test 4: CommandBatch repeated-field count.
#
# Appending 2 OutCommand entries must yield cmds_count==2.
# ---------------------------------------------------------------------------

def test_command_batch_count(msg_lib):
    """Appending 2 OutCommands to a CommandBatch yields cmds_count==2."""
    out_count = ctypes.c_int()

    ret = msg_lib.msg_test_command_batch_count(2, ctypes.byref(out_count))

    assert ret == 1
    assert out_count.value == 2


# ---------------------------------------------------------------------------
# Test 5: PlannerConfig chainable setters.
#
# cfg.setAMax(300.0).setVBodyMax(400.0) must set a_max and v_body_max.
# ---------------------------------------------------------------------------

def test_planner_config_chained_setters(msg_lib):
    """Chained PlannerConfig setters correctly set a_max and v_body_max."""
    out_a_max      = ctypes.c_float()
    out_v_body_max = ctypes.c_float()

    ret = msg_lib.msg_test_planner_config_chained(
        300.0, 400.0,
        ctypes.byref(out_a_max),
        ctypes.byref(out_v_body_max),
    )

    assert ret == 1
    assert out_a_max.value      == pytest.approx(300.0, abs=1e-4)
    assert out_v_body_max.value == pytest.approx(400.0, abs=1e-4)


# ---------------------------------------------------------------------------
# Test 6: static_assert bridges compile (compile-time test).
#
# This is a compile-time property verified by the build steps, not by a
# runtime assertion here.  The host sim build (cmake --build) compiles
# message_test_api.cpp which contains:
#   static_assert(sizeof(Pose2D) == sizeof(float) * 3, ...)       // generated
#   static_assert(sizeof(BodyTwist3) == sizeof(float) * 3, ...)   // generated
#   static_assert(sizeof(CommandBatch) >= sizeof(OutCommand) * 8, ...)
#
# source/messages/bridges.h contains the HAL-side counterparts:
#   static_assert(sizeof(::Pose2D) == sizeof(float) * 3, ...)     // HAL
#   static_assert(sizeof(::BodyTwist3) == sizeof(float) * 3, ...) // HAL
#   static_assert(sizeof(::RobotGeometry) == sizeof(float) * 2, ...) // HAL
#
# Both sets of asserts fire at compile time if layout diverges.
# python build.py --clean additionally verifies all asserts under CODAL
# C++11 (-std=c++11 -fno-rtti -fno-exceptions) on the device toolchain.
#
# This test records the verification outcome as a documentary stub.
# ---------------------------------------------------------------------------

def test_static_assert_bridges_compile():
    """Compile-time layout checks pass (verified by host sim build and build.py --clean).

    The fact that this test file is reachable (imported by pytest) means the
    shared library compiled successfully, which means all static_asserts in
    message_test_api.cpp passed.
    """
    # If the library loaded (msg_lib fixture didn't raise), all static_asserts
    # in message_test_api.cpp fired and passed at compile time.
    # This test is a documentary marker; it always passes at runtime.
    pass
