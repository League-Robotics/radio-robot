"""test_lint.py — static source-code lint checks for the radio-robot-c project.

Checks that are fast, grep-based, and should never regress.
"""
from __future__ import annotations

import pathlib
import subprocess


# __file__ is tests/simulation/unit/test_lint.py; repo root is four levels up.
_REPO = pathlib.Path(__file__).resolve().parents[3]
_SOURCE = _REPO / "source"
_SIM_DIR = _REPO / "tests" / "_infra" / "sim"


def test_no_must_mirror_comment() -> None:
    """Assert that no 'MUST mirror' comment exists in source/ or tests/sim/.

    The pattern 'MUST mirror' flags hand-mirrored code that must stay in sync
    between the sim and the firmware.  Sprint 026-001 removed the last such
    instance; this test prevents the pattern from being reintroduced.
    """
    # Exclude the lint test file itself (the pattern appears in its own docstring
    # and grep argument) and compiled __pycache__ bytecode.  All other files
    # under source/ and tests/sim/ are scanned.
    result = subprocess.run(
        [
            "grep", "-rn",
            "--exclude=test_lint.py",
            "--exclude-dir=__pycache__",
            "MUST mirror",
            str(_SOURCE),
            str(_SIM_DIR),
        ],
        capture_output=True,
        text=True,
    )
    matches = result.stdout.strip()
    assert matches == "", (
        "Found 'MUST mirror' comment(s) — remove the hand-mirrored pattern "
        "and replace with a shared abstraction:\n" + matches
    )
