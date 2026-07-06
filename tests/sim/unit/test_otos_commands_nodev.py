"""Off-hardware acceptance proof for ticket 084-008 (SUC-007): every one of
the seven OTOS verbs (``OI``/``OZ``/``OR``/``OP``/``OV``/``OL``/``OA``)
replies ``ERR nodev <verb>`` against the REAL ``Subsystems::NezhaHardware``
(whose ``odometer()`` is still ``nullptr`` this program — no real-hardware
OTOS driver exists, ``clasi/issues/
nezha-hardware-otos-driver-for-new-source-tree.md``).

Compiles ``otos_commands_harness.cpp`` together with the REAL
``source/hal/nezha/nezha_motor.cpp``, ``source/hal/velocity_pid.cpp``,
``source/subsystems/nezha_hardware.cpp`` (the SAME trio
``test_hardware_seam.py`` compiles), ticket 001's HOST_BUILD scripted-fake
``source/com/i2c_bus_host.cpp``, PLUS this ticket's own
``source/commands/otos_commands.cpp``, ``source/commands/
command_processor.cpp``, and ``source/commands/arg_parse.cpp`` — the full
dispatch path from wire text to ``ERR nodev``, not just a bare
``odometer() == nullptr`` check. ``-DROBOT_DEV_BUILD=1`` is required in
addition to ``-DHOST_BUILD`` here (unlike ``test_hardware_seam.py``):
``commands/otos_commands.{h,cpp}`` are wrapped in ``#if ROBOT_DEV_BUILD``
(matching every other command-family file in this tree), while
``subsystems/hardware.h``/``nezha_hardware.cpp`` are not gated at all. A
second include dir (``source/types``) is also required, unlike
``test_hardware_seam.py``: ``commands/command_processor.h``/``types/
command_types.h`` include their ``types/`` siblings by bare filename
(``"protocol.h"``, ``"command_types.h"``) — mirrors ``test_dev_command_
outbox.py``'s own two ``-I`` flags for the identical reason (that file's own
doc comment / ``tests/_infra/sim/CMakeLists.txt``'s file header explain it).

``libfirmware_host`` cannot be reused for this proof: ``tests/_infra/sim/
CMakeLists.txt`` deliberately never compiles ``subsystems/nezha_hardware.cpp``
(its own file header's "Absent" list — it needs the REAL I2CBus/NezhaMotor,
not the sim's), so the ctypes-backed ``Sim`` wrapper (``firmware.py``) can
only ever exercise ``Subsystems::SimHardware``. This ad hoc harness (078's
"no CMake needed yet" precedent, reused by ``test_hardware_seam.py``/
``test_sim_hardware.py``) is this repo's only way to drive
``Subsystems::NezhaHardware`` at all.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_otos_commands_nodev.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_TYPES_DIR = _SOURCE_DIR / "types"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "otos_commands_harness.cpp"
_HOST_FAKE_SRC = _SOURCE_DIR / "com" / "i2c_bus_host.cpp"
_NEZHA_MOTOR_SRC = _SOURCE_DIR / "hal" / "nezha" / "nezha_motor.cpp"
_VELOCITY_PID_SRC = _SOURCE_DIR / "hal" / "velocity_pid.cpp"
_NEZHA_HARDWARE_SRC = _SOURCE_DIR / "subsystems" / "nezha_hardware.cpp"
_OTOS_COMMANDS_SRC = _SOURCE_DIR / "commands" / "otos_commands.cpp"
_COMMAND_PROCESSOR_SRC = _SOURCE_DIR / "commands" / "command_processor.cpp"
_ARG_PARSE_SRC = _SOURCE_DIR / "commands" / "arg_parse.cpp"

_SOURCES = [
    _HARNESS_SRC,
    _HOST_FAKE_SRC,
    _NEZHA_MOTOR_SRC,
    _VELOCITY_PID_SRC,
    _NEZHA_HARDWARE_SRC,
    _OTOS_COMMANDS_SRC,
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


def test_otos_commands_nodev_harness_compiles_and_passes(tmp_path):
    """Compile the OTOS nodev harness and assert every one-of-seven scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "otos_commands_harness"

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
        "otos_commands_harness.cpp / otos_commands.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "otos_commands_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
