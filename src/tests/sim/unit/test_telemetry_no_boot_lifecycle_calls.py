"""Grep-based regression guard for ticket 125-006 (telemetry-emit-policy-
rebuild-spec.md Part 8, sim acceptance criterion #12).

125-002 (issue Part 1, items 1-4) DELETED App::Telemetry's whole boot-
arming lifecycle: ``kBootStableCycles``, ``markBootComplete()``,
``tickBootSettle()``, and the hidden ``changeReportingArmed_``/
``bootSettling_``/``stableCycles_``/``lastSeenFlags_`` members. A freshly
constructed ``App::Telemetry`` now behaves identically to one that has run
for an hour -- there is no "arm the reporting" call left anywhere, in
production OR in tests. Criterion #12 requires this to be enforced as a
literal, automated check ("so this stays true going forward, not just at
this ticket's completion") rather than a one-time manual confirmation --
a future test author re-adding a ``markBootComplete()``-shaped warm-up call
(exactly the mistake this whole sprint exists to correct) must fail CI, not
merely fail code review.

Scope: only ``src/tests/`` -- the production symbols themselves are already
gone from ``src/firm/`` (checked here too, belt-and-suspenders), but the
NORMATIVE constraint (Part 8 #12's own wording) is specifically that "no
test may re-add a Telemetry lifecycle call."
"""

import pathlib
import subprocess

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_TESTS_DIR = _REPO_ROOT / "src" / "tests"
_FIRM_DIR = _REPO_ROOT / "src" / "firm"
_SELF = pathlib.Path(__file__).resolve()

_BANNED_PATTERN = r"markBootComplete\|tickBootSettle"


def _grep(root: pathlib.Path) -> str:
    result = subprocess.run(
        [
            "grep",
            "-rn",
            "--binary-files=without-match",
            "--exclude-dir=__pycache__",
            f"--exclude={_SELF.name}",
            _BANNED_PATTERN,
            str(root),
        ],
        capture_output=True,
        text=True,
    )
    # grep exit code 1 == "no matches" (success, for this test's purposes);
    # exit code 0 == matches found; >1 == a real grep error.
    assert result.returncode in (0, 1), (
        f"grep itself failed against {root}: exit {result.returncode}\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )
    return result.stdout


def test_no_telemetry_boot_lifecycle_calls_in_tests():
    """issue Part 8 #12: no test under src/tests/ may call markBootComplete()
    or tickBootSettle() -- both were deleted outright by 125-002; App::
    Telemetry has no boot-arming lifecycle left to re-invoke."""
    hits = _grep(_TESTS_DIR)
    assert hits == "", (
        "found a re-added Telemetry boot-lifecycle call under src/tests/ -- "
        "125-002 deleted markBootComplete()/tickBootSettle() outright "
        "(telemetry-emit-policy-rebuild-spec.md Part 1, items 2-3); a "
        "freshly constructed App::Telemetry needs no warm-up call at all:\n"
        f"{hits}"
    )


def test_no_telemetry_boot_lifecycle_calls_in_firmware():
    """Belt-and-suspenders: the production symbols themselves must stay
    deleted from src/firm/ too (the primary guarantee is 125-002's own
    deletion; this just confirms nothing resurrected them)."""
    hits = _grep(_FIRM_DIR)
    assert hits == "", (
        "found a re-added markBootComplete()/tickBootSettle() symbol under "
        f"src/firm/ -- these were deleted outright by 125-002:\n{hits}"
    )
