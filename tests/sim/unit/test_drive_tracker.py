"""Off-hardware acceptance proof for ticket 100-004 (SUC-004).

Compiles ``drive_tracker_harness.cpp`` together with ``source/drive/
{tracker,motion_plan,drivetrain,master_profile,arc_math}.cpp`` and the
vendored Ruckig sources (``libraries/ruckig/src/*.cpp``) using the system
C++ compiler under the firmware's EXACT build constraints (``gnu++20
-fno-exceptions -fno-rtti``, mirroring ``test_drive_plan.py``/``test_jerk_
trajectory.py``), runs the resulting binary, and asserts it exits 0.
Mirrors those files' compile-and-run pattern: no CMake, no ARM toolchain,
no hardware. ``tracker.h`` only needs ``motion_plan.h`` for ``RefState``,
which drags in ``master_profile.h``/Ruckig transitively at the header
level (no MotionPlan/MasterProfile object is ever instantiated by the
harness) -- the extra sources/includes below are linked purely so that
transitive header inclusion resolves cleanly, matching test_drive_plan.py's
own precedent rather than relying on an unlinked-but-untested assumption.

The harness exercises the trim law's clamp behavior across all four error
quadrants, the reverse-travel signed-v_ref cross term, the pivot-mode
literal-zero-v rule (with an UNCLAMPED heading trim), the trimSaturated
exact-true-iff-clamped contract (independently for each of trimVMax/
trimOmegaMax), and the one-sided forward-arc wheel clamp as a wide-grid
property test. (100-006 reconciliation: this harness ORIGINALLY also carried
a minimal closed-loop convergence smoke test (arc + pivot) against a
ticket-scoped first-order plant stub, documented there as "superseded once
ticket 100-006's real plant model lands" -- ticket 100-006 landed the real
tier-0 plant model and its own closed-loop convergence tests
(tests/sim/drive/test_drive_closed_loop.py), so that scenario and its
PlantState/stepPlant stub were REMOVED from this harness rather than kept as
a duplicate.)

A second, non-compiled check greps tracker.{h,cpp} for a derivative/
integral term, mirroring test_drive_isolation.py's forbidden-token
approach -- the P-only outer-loop rule (architecture-update.md, the
issue's "k_d = 0 -- not shipped" rationale) is enforced structurally, not
just by convention.
"""

import pathlib
import re
import subprocess
import sys

import pytest

# tests/sim/unit/test_drive_tracker.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_DRIVE_DIR = _SOURCE_DIR / "drive"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "drive_tracker_harness.cpp"
_TRACKER_HEADER = _DRIVE_DIR / "tracker.h"
_TRACKER_SRC = _DRIVE_DIR / "tracker.cpp"
_DRIVE_SOURCES = [
    _TRACKER_SRC,
    _DRIVE_DIR / "motion_plan.cpp",
    _DRIVE_DIR / "policy.cpp",  # motion_plan.cpp's step() calls Drive::evaluate() (ticket 100-005)
    _DRIVE_DIR / "drivetrain.cpp",
    _DRIVE_DIR / "master_profile.cpp",
    _DRIVE_DIR / "arc_math.cpp",
]
_RUCKIG_INCLUDE = _REPO_ROOT / "libraries" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "libraries" / "ruckig" / "src"

# Match the firmware build EXACTLY (test_jerk_trajectory.py's own precedent).
_CXX_STANDARD = "gnu++20"
_CONSTRAINT_FLAGS = ["-fno-exceptions", "-fno-rtti"]


def _find_cxx_compiler() -> str:
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def test_drive_tracker_harness_compiles_and_passes(tmp_path):
    assert _HARNESS_SRC.is_file(), f"harness missing: {_HARNESS_SRC}"
    for src in _DRIVE_SOURCES:
        assert src.is_file(), f"source missing: {src}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "drive_tracker_harness"

    compile_cmd = [
        cxx,
        f"-std={_CXX_STANDARD}",
        *_CONSTRAINT_FLAGS,
        "-O2",
        "-Wall",
        "-I", str(_SOURCE_DIR),
        "-I", str(_RUCKIG_INCLUDE),
        "-o", str(binary),
        str(_HARNESS_SRC),
        *[str(s) for s in _DRIVE_SOURCES],
        *[str(s) for s in ruckig_srcs],
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, (
        "drive_tracker_harness.cpp failed to compile under -std=gnu++20 "
        f"-fno-exceptions -fno-rtti:\nstdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
    )

    run = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run.returncode == 0, (
        f"drive_tracker_harness reported a scenario failure (exit {run.returncode}):\n"
        f"{run.stdout}\n{run.stderr}"
    )
    assert "OK: all Drive:: tracker scenarios passed" in run.stdout, run.stdout


# --- Structural: no derivative, no integral (P-only outer loop rule) ---

_DERIVATIVE_TOKENS = ("k_d", "kD", "kDeriv", "derivative")
_INTEGRAL_TOKENS = ("integral", "accumulator", "k_i", "kI", "integrator")


def _strip_line_comment(line: str) -> str:
    """Drop everything from ``//`` to end-of-line (mirrors test_drive_
    isolation.py's own ``_strip_line_comment``) -- this file's OWN doc
    comments legitimately discuss "no k_d" / "no integral" in prose; the
    check below only scans CODE."""
    return line.split("//", 1)[0]


def _scan_forbidden_tokens(path: pathlib.Path, tokens) -> list:
    violations = []
    for lineno, raw_line in enumerate(path.read_text().splitlines(), start=1):
        code = _strip_line_comment(raw_line)
        for token in tokens:
            if token in code:
                violations.append(f"{path.name}:{lineno}: forbidden token {token!r}: {code.strip()!r}")
    return violations


def test_tracker_has_no_derivative_term():
    violations = []
    for path in (_TRACKER_HEADER, _TRACKER_SRC):
        violations.extend(_scan_forbidden_tokens(path, _DERIVATIVE_TOKENS))
    assert not violations, (
        "tracker.{h,cpp} must carry NO k_d/derivative term (P-only outer-loop rule, "
        "architecture-update.md; the issue's 'k_d = 0 -- not shipped' rationale):\n"
        + "\n".join(violations)
    )


def test_tracker_has_no_integral_or_accumulator_field():
    violations = []
    for path in (_TRACKER_HEADER, _TRACKER_SRC):
        violations.extend(_scan_forbidden_tokens(path, _INTEGRAL_TOKENS))
    assert not violations, (
        "tracker.{h,cpp} must carry NO integral/accumulator field anywhere "
        "(the P-only outer-loop rule):\n" + "\n".join(violations)
    )


# A stricter structural pin: TrackerOutput/the `track()` function must not
# accumulate state across calls -- no `static` (non-const) locals, which
# would be the classic way to smuggle in an integrator without naming it
# "integral"/"accumulator" literally.
_STATIC_MUTABLE_RE = re.compile(r"\bstatic\s+(?!const\b)(?!constexpr\b)")


def test_tracker_cpp_has_no_static_mutable_state():
    violations = []
    for lineno, raw_line in enumerate(_TRACKER_SRC.read_text().splitlines(), start=1):
        code = _strip_line_comment(raw_line)
        if _STATIC_MUTABLE_RE.search(code):
            violations.append(f"{_TRACKER_SRC.name}:{lineno}: {code.strip()!r}")
    assert not violations, (
        "tracker.cpp must carry no mutable `static` local (a de-facto integrator/"
        "accumulator hidden from the integral/accumulator token grep):\n" + "\n".join(violations)
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
