"""Host-side activation proof for sprint 117 ticket 001 (SUC-056).

Ticket 001's own Acceptance Criteria: "A host-side activation test (sim-
first; bench if a live robot is reachable) drives a real ``ClockSync``
instance's ``ping_burst()`` against the firmware's new reply and asserts
``best_offset()`` is non-``None`` after one burst -- i.e. ``ClockSync`` is
proven to actually activate against this firmware, not just parse a
hand-written fixture string."

``src/tests/unit/test_clock_sync.py`` already covers ``ClockSync`` at the
pure-Python unit level, but every reply line it feeds ``ping_burst()`` is
hand-typed in Python (``f"OK pong t={t}"``). This test is different in
kind, not degree: it compiles and runs ``clock_sync_activation_harness.cpp``
(this directory), which links the REAL ``App::Comms::pumpTransport()``
(``src/firm/app/comms.cpp``) -- the exact PING handler
``RobotLoop::cycle()`` calls on real/simulated firmware -- and prints its
actual reply lines to stdout. Those lines, never touched by Python, are
then fed straight into ``ClockSync.ping_burst()``'s own ``send_fn``
contract.

Sim-first per this project's convention and per the ticket's own guidance
("sim-first; bench if a live robot is reachable"): the harness IS the sim
side (no hardware, no serial port) -- see this file's own header for why a
live/bench round trip through ``NezhaProtocol.send()`` is currently blocked
by a separate, pre-existing host-side gap
(``io/serial_conn.py``'s corr-id-suffixed ``send()`` breaks an exact-string
match like ``PING``'s -- see ``src/host/robot_radio/DESIGN.md`` §6), flagged
there rather than worked around here.

Collected under ``src/tests/sim/unit/`` -- already within
``pyproject.toml``'s ``testpaths``.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_clock_sync_activation.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_TESTS_SIM_DIR = _REPO_ROOT / "src" / "tests" / "sim"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "clock_sync_activation_harness.cpp"
_COMMS_SRC = _SOURCE_DIR / "app" / "comms.cpp"
_WIRE_SRC = _SOURCE_DIR / "messages" / "wire.cpp"
_WIRE_RUNTIME_SRC = _SOURCE_DIR / "messages" / "wire_runtime.cpp"

# Matches every other src/tests/sim/unit harness's own compiled standard.
_CXX_STANDARD = "c++20"

_EXPECTED_PING_COUNT = 5  # clock_sync_activation_harness.cpp's own kPingCount


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++."""
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def _run_harness(tmp_path: pathlib.Path) -> list[str]:
    """Compile clock_sync_activation_harness.cpp and return its stdout lines
    (one real "OK pong t=<ms>" reply per line, from the actual compiled
    App::Comms code -- see this file's own module docstring)."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _COMMS_SRC.is_file(), f"comms.cpp missing: {_COMMS_SRC}"
    assert _WIRE_SRC.is_file(), f"wire.cpp missing (run scripts/gen_messages.py?): {_WIRE_SRC}"
    assert _WIRE_RUNTIME_SRC.is_file(), f"wire_runtime.cpp missing: {_WIRE_RUNTIME_SRC}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "clock_sync_activation_harness"

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
            str(_TESTS_SIM_DIR),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
            str(_COMMS_SRC),
            str(_WIRE_SRC),
            str(_WIRE_RUNTIME_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "clock_sync_activation_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        f"clock_sync_activation_harness exited {run_result.returncode}:\n"
        f"{run_result.stdout}\n{run_result.stderr}"
    )

    lines = [line for line in run_result.stdout.splitlines() if line]
    assert len(lines) == _EXPECTED_PING_COUNT, (
        f"expected {_EXPECTED_PING_COUNT} reply lines, got {len(lines)}: {lines}"
    )
    return lines


def test_clock_sync_activates_against_real_firmware_ping_reply(tmp_path):
    """ClockSync.ping_burst(), driven by the REAL App::Comms PING reply
    (not a hand-written fixture), converges to a non-None best_offset()."""
    from robot_radio.robot.clock_sync import ClockSync, _parse_pong_t

    reply_lines = _run_harness(tmp_path)

    # Sanity: every captured line really does carry the new t=<ms> field --
    # if 001's own PING-handler change ever regressed, this fails LOUDLY
    # here rather than ping_burst() silently recording zero samples.
    for line in reply_lines:
        assert line.startswith("OK pong t="), f"unexpected harness reply shape: {line!r}"
        assert _parse_pong_t(line) is not None, f"_parse_pong_t() could not parse: {line!r}"

    replies = iter(reply_lines)

    def send_fn(cmd: str) -> str | None:
        assert cmd == "PING"
        return next(replies, None)

    cs = ClockSync()
    cs.ping_burst(send_fn, n=_EXPECTED_PING_COUNT)

    assert cs.sample_count == _EXPECTED_PING_COUNT, (
        "every one of the harness's real firmware replies should have recorded a sample"
    )
    assert cs.best_offset() is not None, (
        "ClockSync failed to activate (best_offset() stayed None) against the "
        "REAL firmware PING reply -- see clock_sync_activation_harness.cpp"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
