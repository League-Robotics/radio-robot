"""Off-hardware acceptance proof for ticket 082-004 (SUC-004), extended by
087-008 (SUC-001/SUC-002/SUC-006) and re-scoped by 097-008
(architecture-update-r2.md Decision 9, pure-binary firmware):
Telemetry::tick() (source/telemetry/tlm_frame.{h,cpp}, bb -> TlmFrameInput,
reads a bare Rt::Blackboard directly with no live subsystem behind any
cell) and Telemetry::buildTelemetryMessage() (096-003, TlmFrameInput ->
msg::Telemetry). Telemetry::buildTlmFrame() -- the pure, stateless TEXT
frame-formatting function the now-deleted text STREAM/SNAP command family
used to build on -- was deleted by 097-008 along with its own scenarios in
``tlm_frame_harness.cpp`` (see that file's own header comment); tick()'s
own field-assembly is now proven directly against TlmFrameInput's fields
rather than via a formatted text line.

Compiles ``tlm_frame_harness.cpp`` together with ``source/telemetry/
tlm_frame.cpp`` and ``source/kinematics/body_kinematics.cpp`` (087-008:
Telemetry::tick()'s one pure-math dependency, for twist=) using the system
C++ compiler, runs the resulting binary, and asserts it exits 0. Mirrors
``test_velocity_pid.py``'s compile-and-run pattern (081-001): no hardware,
no CODAL, no CMake -- tlm_frame.{h,cpp}, body_kinematics.{h,cpp}, and the
header-only runtime/blackboard.h + messages/*.h it pulls in, compiled
standalone, since neither Telemetry::tick() nor Telemetry::buildTelemetryMessage()
has any DevLoop/Hardware/Drivetrain/PoseEstimator/CommandRouter dependency
at all (the remaining impure wiring -- reply-channel resolution, the seq=
counter's mutation, the binary armor/encode -- lives in commands/
telemetry_commands.cpp instead, exercised end-to-end via the ctypes sim
harness in test_binary_channel.py's `stream` section).

Collected under ``tests/sim/unit/`` alongside the existing
``test_velocity_pid.py``/``test_dev_loop_pose_estimator.py`` -- already
within ``pyproject.toml``'s ``testpaths = ["tests/sim", "tests/unit"]``, no
configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_tlm_frame.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "tlm_frame_harness.cpp"
_TLM_FRAME_SRC = _SOURCE_DIR / "telemetry" / "tlm_frame.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"

# messages/common.h documents its own target as "CODAL C++11" -- build the
# host harness to the same standard so it exercises exactly the language
# subset the firmware itself uses.
_CXX_STANDARD = "c++20"


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++."""
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def test_tlm_frame_harness_compiles_and_passes(tmp_path):
    """Compile the Telemetry::buildTlmFrame() harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _TLM_FRAME_SRC.is_file(), f"tlm_frame.cpp missing: {_TLM_FRAME_SRC}"
    assert _BODY_KINEMATICS_SRC.is_file(), f"body_kinematics.cpp missing: {_BODY_KINEMATICS_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "tlm_frame_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-I",
            str(_SOURCE_DIR),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
            str(_TLM_FRAME_SRC),
            str(_BODY_KINEMATICS_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "tlm_frame_harness.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "tlm_frame_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
