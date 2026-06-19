"""Vendor-confinement grep-gate canary (038-004).

Greps source/app/, source/control/, source/robot/, source/types/ for forbidden
vendor tokens and asserts the hit-set has not grown vs the committed baseline
tests/_infra/vendor_baseline.txt.

CODAL-only files excluded from the host build are skipped — their vendor
includes are intentional device-layer references, not leaks above hal/.

To update the baseline after an intentional Phase-A seal:
    python3 -c "..." > tests/_infra/vendor_baseline.txt
See the generation snippet in ticket 038-004.
"""
from __future__ import annotations

import pathlib
import re

import pytest

REPO_ROOT = pathlib.Path(__file__).parents[3]
SOURCE_DIR = REPO_ROOT / "source"
BASELINE_FILE = REPO_ROOT / "tests" / "_infra" / "vendor_baseline.txt"

INSPECT_DIRS = ["app", "control", "robot", "types"]

# CODAL-only files excluded from the host build — vendor deps here are expected.
CODAL_ONLY = {
    "WedgeTest.cpp",
    "WedgeTest.h",
    "LoopScheduler.cpp",
    "LoopScheduler.h",
    "Icons.h",
    "SystemCommands.cpp",
    "Robot.cpp",
}

FORBIDDEN_PATTERNS = [
    r"#include.*MicroBit\.h",
    r"\bI2CBus\b",
    r"\bmicrobit_random\b",
]


def collect_hits() -> set[str]:
    hits: set[str] = set()
    for d in INSPECT_DIRS:
        for ext in ("*.cpp", "*.hpp", "*.h"):
            for f in (SOURCE_DIR / d).rglob(ext):
                if f.name in CODAL_ONLY:
                    continue
                for lineno, line in enumerate(
                    f.read_text(errors="replace").splitlines(), 1
                ):
                    for pat in FORBIDDEN_PATTERNS:
                        if re.search(pat, line):
                            rel = f.relative_to(REPO_ROOT)
                            hits.add(f"{rel}:{lineno}: {line.strip()[:80]}")
    return hits


def test_vendor_confinement_no_new_leaks():
    """Assert no new vendor tokens appear above source/hal/ vs Phase 0 baseline.

    Existing baseline entries may disappear (Phase A sealing them) without
    failing the gate — only NEW entries cause failure.
    """
    baseline: set[str] = set()
    if BASELINE_FILE.exists():
        baseline = {l for l in BASELINE_FILE.read_text().splitlines() if l.strip()}

    current = collect_hits()
    new_leaks = current - baseline

    assert not new_leaks, (
        f"New vendor leaks found above source/hal/ ({len(new_leaks)} new):\n"
        + "\n".join(sorted(new_leaks))
        + "\nUpdate tests/_infra/vendor_baseline.txt ONLY if these are intentional."
    )
