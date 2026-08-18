"""src/tests/sim/unit/test_configurator_applyfield.py -- ticket 132-012's own
acceptance test (Generic applyField(target, field, value) setter +
SetConfigField wire command, sprint 132 "configuration discipline"):
Configurator::applyField() -- boot-only ERR_NOT_LIVE rejection, the
isfinite()-before-validateBounds() NaN/Inf guard (ERR_BADARG), unknown-field
ERR_BADARG, out-of-bounds ERR_RANGE, and a valid push landing at the correct
offset and reaching install(target).

Compiles ``configurator_applyfield_harness.cpp`` together with
configurator.cpp, drive.cpp, boot_calibration.cpp, persisted_tuning.cpp,
config/boot_config.cpp, messages/wire.cpp, messages/wire_runtime.cpp,
hardware/generic/real_otos.cpp, and the standalone Motion::Planner sources
(planner.cpp/profile.cpp/estimation.cpp/shape.cpp), runs the resulting
binary, and asserts it exits 0 -- printing its own human-readable
per-scenario trace. No I2C bus, no sim plant: every Hal::Motor/
Hal::Otos this harness touches is a small in-file test double
implementing the pure interface directly (mirrors
configurator_applygroup_harness.cpp's own source-list shape, 132-008).

    uv run python -m pytest src/tests/sim/unit/test_configurator_applyfield.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_configurator_applyfield.py -> unit -> sim -> tests -> src -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_UNIT_DIR = pathlib.Path(__file__).resolve().parent

_HARNESS_SRC = _UNIT_DIR / "configurator_applyfield_harness.cpp"

_APP_SOURCES = [
    _SOURCE_DIR / "core" / "configurator.cpp",
    _SOURCE_DIR / "diffdrive" / "differential_drive.cpp",
    _SOURCE_DIR / "core" / "boot_calibration.cpp",
]
# _MOTION_SOURCES -- DELETED (exploratory-kernel rewrite, 2026-08-15):
# src/firm/motion/ no longer exists.
_MOTION_SOURCES = []
_CONFIG_SOURCES = [
    # Config::default*Config()/defaultPlannerLimits() -- bootPlannerLimits()
    # (boot_calibration.cpp, linked above but unused by this harness's own
    # scenarios) still calls Config::defaultPlannerLimits() at link time
    # (same four-source-list trap test_configure_entry_points.py's own
    # comment documents).
    _SOURCE_DIR / "config" / "boot_config.cpp",
    # Configurator::persistTuningIfChanged()/reapplyPersistedTuning()
    # reference Config::serializeSnapshot()/deserializeSnapshot() at link
    # time even though this harness always passes tuningStore=nullptr and
    # never exercises the persisted-tuning path.
    _SOURCE_DIR / "config" / "persisted_tuning.cpp",
]
_MESSAGE_SOURCES = [
    # The generated wire codec -- Configurator::applyField() (132-012)
    # writes through msg::wire::setField(<Group>&, ...), which needs both
    # the generated engine (wire.cpp, including its 132-012 setScalarField()
    # addition) and its byte-level primitives (wire_runtime.cpp) linked.
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_DEVICE_SOURCES = [
    # Hal::Otos's virtual destructor is declared out-of-line (otos.h)
    # and defined here -- RecordingOtos's own dtor chain needs the symbol
    # even though this harness never constructs a RealOtos.
    _SOURCE_DIR / "hardware" / "generic" / "real_otos.cpp",
]

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


def _all_sources():
    return (
        [_HARNESS_SRC]
        + _APP_SOURCES
        + _MOTION_SOURCES
        + _CONFIG_SOURCES
        + _MESSAGE_SOURCES
        + _DEVICE_SOURCES
    )


def test_configurator_applyfield(tmp_path):
    """Compile configurator_applyfield_harness.cpp + its dependency graph;
    assert every boot-only/NaN-Inf/unknown-field/out-of-bounds/valid-push/
    install(target)-reuse scenario passes."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "configurator_applyfield_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD",
            "-I",
            str(_SOURCE_DIR),
            "-I",
            str(_SOURCE_DIR / "platform" / "host"),  # host_fiber.h
            "-I",
            str(_REPO_ROOT / "src"),
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "configurator_applyfield_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "configurator_applyfield_harness reported a failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
