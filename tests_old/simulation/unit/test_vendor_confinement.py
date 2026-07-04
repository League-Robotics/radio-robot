"""Vendor-confinement grep-gate canary (038-004).

Greps source/commands/, source/control/, source/robot/, source/state/, source/types/
for forbidden vendor tokens and asserts the hit-set has not grown vs the committed
baseline tests/_infra/vendor_baseline.txt.

CODAL-only files excluded from the host build are skipped — their vendor
includes are intentional device-layer references, not leaks above io/.

039-005: the device layer moved source/hal/ -> source/io/ (capability/ real/
sim/). The grep scope below (app/control/robot/types) is unchanged — those are
the layers ABOVE the IO boundary that must stay vendor-free. The boundary is now
source/io/ rather than source/hal/.

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

# 041-002 (Phase C): source/state/ added to scope. After Commandable is stripped
# from Odometry, the estimator layer (PhysicalStateEstimate, EKF) is dependency-
# clean and must stay vendor-free — no MicroBit.h / I2CBus / microbit_random.
# 042-001 (Phase D): source/superstructure/ added to scope. The Superstructure
# seam (Goal enum + requestGoal) must stay vendor-free — it depends only on
# Config/Protocol and forward-declares MotionController/HaltController/Robot.
# 043-001 (Phase E): source/subsystems/ added to scope. The thin sensor/drive/
# gripper subsystems wrap capability interfaces only — no MicroBit.h / I2CBus /
# microbit_random may appear in source/subsystems/.
INSPECT_DIRS = ["commands", "control", "robot", "state", "subsystems", "superstructure", "types"]

# CODAL-only files excluded from the host build — vendor deps here are expected.
# NezhaHAL and MecanumHAL moved from hal/real/ to source/robot/ in 055-001;
# they are CODAL-dependent hardware drivers and their vendor tokens are expected.
CODAL_ONLY = {
    "WedgeTest.cpp",
    "WedgeTest.h",
    "LoopScheduler.cpp",
    "LoopScheduler.h",
    "Icons.h",
    "SystemCommands.cpp",
    "Robot.cpp",
    "NezhaHAL.cpp",
    "NezhaHAL.h",
    "MecanumHAL.cpp",
    "MecanumHAL.h",
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


def _baseline() -> set[str]:
    if BASELINE_FILE.exists():
        return {l for l in BASELINE_FILE.read_text().splitlines() if l.strip()}
    return set()


def test_vendor_confinement_no_new_leaks():
    """Assert no new vendor tokens appear above source/io/ vs the baseline.

    Existing baseline entries may disappear (sealing them) without failing the
    gate — only NEW entries cause failure.

    044-003 (Phase F): the DebugCommands I2CBus leak is sealed and the
    baseline is now EMPTY, so this assertion is equivalent to "zero hits above
    source/io/" — the migration's final vendor-confinement criterion. The
    explicit test below pins that to a hard zero so the gate stays tight.
    """
    baseline = _baseline()
    current = collect_hits()
    new_leaks = current - baseline

    assert not new_leaks, (
        f"New vendor leaks found above source/io/ ({len(new_leaks)} new):\n"
        + "\n".join(sorted(new_leaks))
        + "\nUpdate tests/_infra/vendor_baseline.txt ONLY if these are intentional."
    )


def test_vendor_confinement_zero_hits_empty_baseline():
    """Hard gate (044-003, FINAL): zero vendor hits above source/io/.

    After Phase F sealed the last leak (DebugCommands's I2CBus*, via
    IBusDiagnostics + IRawBusAccess), the baseline is empty. The vendor-
    confinement grep must return ZERO hits across the layers above the IO
    boundary. If a future edit reintroduces a vendor token (MicroBit.h, I2CBus,
    microbit_random) above source/io/, this fails immediately — there is no
    longer any baseline to absorb it.

    The baseline file MUST stay empty (or contain only explicitly-documented,
    firmware-only-tool exemptions with a rationale comment — none exist today).
    """
    baseline = _baseline()
    # Allow only commented-out rationale lines as "exemptions"; any real entry
    # (a non-comment line) is a regression of the final criterion.
    real_entries = {l for l in baseline if not l.lstrip().startswith("#")}
    assert not real_entries, (
        "vendor_baseline.txt must be empty after Phase F (044-003) — the final "
        "vendor-confinement criterion is ZERO hits above source/io/. Found "
        f"{len(real_entries)} baseline entries:\n" + "\n".join(sorted(real_entries))
    )

    current = collect_hits()
    assert not current, (
        f"Vendor tokens found above source/io/ ({len(current)} hits) — the "
        "final vendor-confinement criterion (ZERO hits) is violated:\n"
        + "\n".join(sorted(current))
    )
