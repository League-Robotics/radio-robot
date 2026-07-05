"""Off-hardware acceptance proof for ticket 079-005 (SUC-004/SUC-005/SUC-006/
SUC-007).

Compiles ``dev_command_outbox_harness.cpp`` together with the REAL
``source/commands/dev_commands.cpp``, ``source/commands/command_processor.cpp``,
``source/commands/arg_parse.cpp``, ``source/subsystems/drivetrain.cpp``,
``source/kinematics/body_kinematics.cpp``, ``source/hal/nezha/nezha_motor.cpp``,
and ``source/subsystems/nezha_hardware.cpp``, plus ticket 001's HOST_BUILD scripted-fake
``source/com/i2c_bus_host.cpp``, against the SAME headers every ARM build
compiles, with ``-DHOST_BUILD`` (sheds nezha_motor.cpp's MicroBit.h dependency,
see nezha_flipflop_harness.cpp) AND ``-DROBOT_DEV_BUILD=1`` (codal.json's value --
compiles in the DEV command family, see dev_commands.h's file header). Mirrors
test_nezha_flipflop.py/test_drivetrain.py's shape exactly: compile with the
system C++ compiler, run the resulting binary, assert it exits 0.

Collected under ``tests/sim/unit/`` alongside the other harness wrappers --
already within ``pyproject.toml``'s ``testpaths = ["tests/sim"]``, no
configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_dev_command_outbox.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
# source/commands/command_processor.h (and source/types/command_types.h)
# #include their types/types.h siblings by bare filename ("protocol.h",
# "command_types.h", "arg_schema.h") rather than a path-qualified
# "types/protocol.h" -- the real CMake build's RECURSIVE_FIND_DIR adds every
# header-bearing directory individually (see CMakeLists.txt's comment near
# "finally, find sources and includes"); this second -I replicates that for
# the host compiler.
_TYPES_DIR = _SOURCE_DIR / "types"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "dev_command_outbox_harness.cpp"
_HOST_FAKE_SRC = _SOURCE_DIR / "com" / "i2c_bus_host.cpp"
_NEZHA_MOTOR_SRC = _SOURCE_DIR / "hal" / "nezha" / "nezha_motor.cpp"
_NEZHA_HARDWARE_SRC = _SOURCE_DIR / "subsystems" / "nezha_hardware.cpp"
_DRIVETRAIN_SRC = _SOURCE_DIR / "subsystems" / "drivetrain.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"
_DEV_COMMANDS_SRC = _SOURCE_DIR / "commands" / "dev_commands.cpp"
_COMMAND_PROCESSOR_SRC = _SOURCE_DIR / "commands" / "command_processor.cpp"
_ARG_PARSE_SRC = _SOURCE_DIR / "commands" / "arg_parse.cpp"

_SOURCES = [
    _HARNESS_SRC,
    _HOST_FAKE_SRC,
    _NEZHA_MOTOR_SRC,
    _NEZHA_HARDWARE_SRC,
    _DRIVETRAIN_SRC,
    _BODY_KINEMATICS_SRC,
    _DEV_COMMANDS_SRC,
    _COMMAND_PROCESSOR_SRC,
    _ARG_PARSE_SRC,
]

# messages/common.h documents its own target as "CODAL C++11" -- build the
# host harness to the same standard so it exercises exactly the language
# subset the firmware itself uses.
_CXX_STANDARD = "c++11"


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++."""
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def test_dev_command_outbox_harness_compiles_and_passes(tmp_path):
    """Compile the DEV command outbox harness and assert every scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "dev_command_outbox_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD",
            "-DROBOT_DEV_BUILD=1",
            "-I",
            str(_SOURCE_DIR),
            "-I",
            str(_TYPES_DIR),
            "-o",
            str(binary),
        ]
        + [str(src) for src in _SOURCES],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "dev_command_outbox_harness.cpp / dev_commands.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "dev_command_outbox_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
