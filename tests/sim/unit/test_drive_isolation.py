"""Structural isolation test for source/drive/ (ticket 100-002, SUC-008).

architecture-update.md (100) M2: source/drive/ is a self-contained
subsystem that "refers to nothing outside it (copying code where needed)".
This test enforces that boundary structurally, FOREVER -- it must keep
passing as later tickets (100-003 through 100-013) add tracker.{h,cpp},
policy.{h,cpp}, drivetrain.{h,cpp}, motion_plan.{h,cpp} to this directory.

Two checks, over every ``*.h``/``*.cpp`` file directly under
``source/drive/``:

1. **Forbidden tokens** -- ``msg::``, ``Hal::``, ``Subsystems::``,
   ``MicroBit``, ``kOutputHops``, ``kDeadTime`` must never appear in CODE.
   ``//`` line comments are stripped before scanning (mirrors test_jerk_
   trajectory.py's own ``_strip_line_comments`` precedent for the identical
   situation): this directory's own doc comments legitimately NAME these
   tokens to document the boundary they are excluded by (e.g. this file's
   own module docstring, master_profile.h's seeding-contract note) -- that
   is the point, not a violation of it.
2. **#include boundary** -- every ``#include`` must be either an
   angle-bracket standard-library header (``<...>`` -- "libc/libm", read
   broadly as the C/C++ standard library, per this codebase's own existing
   usage in jerk_trajectory.cpp/body_kinematics.cpp), a quoted
   ``"drive/..."`` header (another file in this same directory, addressed
   the same way every other source/ file addresses its own siblings, e.g.
   motion/jerk_trajectory.h's ``#include "motion/jerk_trajectory.h"``
   style), or a quoted ``"ruckig/..."`` header (the vendored library,
   master_profile.h's own ``#include "ruckig/ruckig.hpp"``). Anything else
   (``"messages/...", "subsystems/...", "hal/..."``, etc.) fails.

Failures name the exact offending file and line number.
"""

import pathlib
import re

import pytest

# tests/sim/unit/test_drive_isolation.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_DRIVE_DIR = _REPO_ROOT / "source" / "drive"

_FORBIDDEN_TOKENS = ("msg::", "Hal::", "Subsystems::", "MicroBit", "kOutputHops", "kDeadTime")

_ALLOWED_QUOTED_INCLUDE_PREFIXES = ("drive/", "ruckig/")

_INCLUDE_RE = re.compile(r'^\s*#include\s*([<"])([^>"]+)[>"]')


def _drive_files():
    assert _DRIVE_DIR.is_dir(), f"source/drive/ does not exist: {_DRIVE_DIR}"
    files = sorted(_DRIVE_DIR.glob("*.h")) + sorted(_DRIVE_DIR.glob("*.cpp"))
    assert files, f"no files found directly under {_DRIVE_DIR} -- nothing to check"
    return files


def _strip_line_comment(line: str) -> str:
    """Drop everything from ``//`` to end-of-line -- see module docstring
    check (1)."""
    return line.split("//", 1)[0]


def _scan(path: pathlib.Path) -> list:
    """Return a list of violation strings ("path:line: message") for one file."""
    rel = path.relative_to(_REPO_ROOT)
    violations = []
    for lineno, raw_line in enumerate(path.read_text().splitlines(), start=1):
        code = _strip_line_comment(raw_line)

        for token in _FORBIDDEN_TOKENS:
            if token in code:
                violations.append(
                    f"{rel}:{lineno}: forbidden token {token!r} referenced in code: "
                    f"{code.strip()!r}"
                )

        include_match = _INCLUDE_RE.match(raw_line)
        if include_match:
            delimiter, target = include_match.groups()
            if delimiter == "<":
                continue  # angle-bracket standard-library header: always allowed
            if not target.startswith(_ALLOWED_QUOTED_INCLUDE_PREFIXES):
                violations.append(
                    f"{rel}:{lineno}: #include \"{target}\" is outside source/drive/, "
                    f"libc/libm, and libraries/ruckig"
                )

    return violations


def test_source_drive_isolation():
    all_violations = []
    for path in _drive_files():
        all_violations.extend(_scan(path))

    assert not all_violations, (
        "source/drive/ isolation boundary violated (architecture-update.md (100) "
        "M2, SUC-008):\n" + "\n".join(all_violations)
    )


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
