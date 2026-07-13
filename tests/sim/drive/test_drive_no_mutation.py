"""Static source check (ticket 100-006 AC): confirms no method in
source/drive/ mutates a `const MotionPlan&` or reads global/static mutable
state.

Two grep-based checks, mirroring tests/sim/unit/test_drive_tracker.py's own
forbidden-token / no-static-mutable-local precedent (and test_drive_
isolation.py's `_strip_line_comment` pattern), applied across ALL of
source/drive/'s *.h/*.cpp files rather than just tracker.{h,cpp}:

1. No `const_cast` anywhere in source/drive/ -- the only mechanism that
   could turn a `const MotionPlan&` parameter (Drivetrain::replan()'s own
   signature) into a mutable one.
2. No mutable `static` declaration (a non-const/constexpr `static` at
   namespace, class, or function scope) anywhere in source/drive/ -- the
   grep-verifiable absence of global/static mutable state. (`static
   constexpr`/`static const` -- compile-time constants, e.g. policy.cpp's
   own kSustainHold/kDwellHold table -- are explicitly allowed; they carry
   no mutable state, matching test_drive_tracker.py's own
   `_STATIC_MUTABLE_RE`.)

As of this ticket, BOTH checks find zero matches anywhere in source/drive/
(confirmed directly: `grep -n static source/drive/*.h source/drive/*.cpp`
and `grep -n const_cast ...` both return nothing) -- there is currently no
`static` keyword of ANY kind (mutable, const, or a plain free function) in
the directory, so this test is not merely permissive-by-construction, it is
presently vacuously true and will start actually discriminating the moment
a first `static` is ever added.

Manual code-review note (the AC's own "or ... an explicit code-review note"
fallback, kept here for completeness even though the grep check above is not
impractical): the one place a mutable reference to a MotionPlan COULD be
smuggled in structurally is `MotionPlan::step(const StepInput&, StepState*)`
-- but that is an explicit, documented OUT parameter (`StepState*`, never a
`const MotionPlan&`), motion_plan.h's own class comment names it the
subsystem's ONE statelessness residue, and it is caller-owned, never
Drive::-owned -- not a violation of the const-correctness rule this test
enforces.
"""
from __future__ import annotations

import pathlib
import re

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_DRIVE_DIR = _REPO_ROOT / "source" / "drive"

_STATIC_MUTABLE_RE = re.compile(r"\bstatic\s+(?!const\b)(?!constexpr\b)")


def _drive_files() -> list[pathlib.Path]:
    assert _DRIVE_DIR.is_dir(), f"source/drive/ does not exist: {_DRIVE_DIR}"
    files = sorted(_DRIVE_DIR.glob("*.h")) + sorted(_DRIVE_DIR.glob("*.cpp"))
    assert files, f"no files found directly under {_DRIVE_DIR} -- nothing to check"
    return files


def _strip_line_comment(line: str) -> str:
    """Drop everything from `//` to end-of-line (mirrors test_drive_
    isolation.py's own `_strip_line_comment`)."""
    return line.split("//", 1)[0]


def test_source_drive_has_no_const_cast():
    violations = []
    for path in _drive_files():
        for lineno, raw_line in enumerate(path.read_text().splitlines(), start=1):
            code = _strip_line_comment(raw_line)
            if "const_cast" in code:
                violations.append(f"{path.name}:{lineno}: {code.strip()!r}")
    assert not violations, (
        "source/drive/ must never const_cast a const reference (the only "
        "mechanism that could mutate Drivetrain::replan()'s `const MotionPlan&` "
        "parameter, or any other const-ref parameter in the directory):\n"
        + "\n".join(violations)
    )


def test_source_drive_has_no_mutable_static_state():
    violations = []
    for path in _drive_files():
        for lineno, raw_line in enumerate(path.read_text().splitlines(), start=1):
            code = _strip_line_comment(raw_line)
            if _STATIC_MUTABLE_RE.search(code):
                violations.append(f"{path.name}:{lineno}: {code.strip()!r}")
    assert not violations, (
        "source/drive/ must carry no mutable static/global state anywhere -- "
        "every method is a pure function of its arguments; StepState is the ONE "
        "documented, caller-owned statelessness residue (motion_plan.h's own "
        "class comment):\n" + "\n".join(violations)
    )


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
