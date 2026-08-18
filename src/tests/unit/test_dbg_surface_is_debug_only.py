"""src/tests/unit/test_dbg_surface_is_debug_only.py -- 133-003: the DBG
live-tuning surface must not exist in a shipped image.

WHY THIS IS A SOURCE-STRUCTURE TEST AND NOT A COMPILE TEST. The obvious
proof would be to compile `comms.cpp` with `ROBOT_DEBUG` undefined and
assert `DBG:vmin 60` falls through to `malformedCount()`. That cannot be
done on the host: `app/debug.h` DEFINES `ROBOT_DEBUG` whenever `HOST_BUILD`
is defined (its own file header says so), and without `HOST_BUILD` the
translation unit pulls in `platform/microbit/microbit_serial_port.h` ->
`MicroBit.h`, which does
not compile off-target. So the shipped-image arm has no host-compilable
form at all, and `src/tests/sim/unit/test_app_comms.py`'s harness -- which
DOES exercise every parser arm for real -- necessarily runs with the debug
surface live.

What this test can do, and does, is hold the guard structure itself: every
token that reaches a live tuning setter must sit inside an `#ifdef
ROBOT_DEBUG` region. That is a real regression guard -- an arm added
outside the guard, or a guard accidentally closed early by an edit above
it, both fail here -- and it is checkable with no ARM toolchain. It is NOT
a claim that the release image was built and inspected; the ARM release
build is a build-time check, run by ticket 004's own flash.

The `#if` scanner below is deliberately simple (it tracks nesting and
whether `ROBOT_DEBUG` is among the enclosing conditions) rather than a real
preprocessor: these two files use only plain `#ifdef ROBOT_DEBUG` guards,
and a scanner that silently handled more than it was tested on would be the
less trustworthy of the two.
"""
from __future__ import annotations

import pathlib
import re

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_COMMS_CPP = _REPO_ROOT / "src" / "firm" / "core" / "comms.cpp"
_ROBOT_LOOP_CPP = _REPO_ROOT / "src" / "firm" / "core" / "robot_loop.cpp"
_ROBOT_LOOP_H = _REPO_ROOT / "src" / "firm" / "core" / "robot_loop.h"
_DRIVE_H = _REPO_ROOT / "src" / "firm" / "control" / "differential_drive.h"

_IF_OPEN = re.compile(r"^\s*#\s*(if|ifdef|ifndef)\b(.*)$")
_ELSE = re.compile(r"^\s*#\s*(else|elif)\b")
_ENDIF = re.compile(r"^\s*#\s*endif\b")


def _lines_guarded_by(path: pathlib.Path, macro: str) -> "list[bool]":
    """One bool per source line: is this line inside an `#if*` region whose
    condition mentions `macro`?

    An `#else` on a guarded `#if` flips that level to unguarded -- the
    shipped-image arm is exactly such an else, and code there must NOT be
    counted as protected.
    """
    guarded: "list[bool]" = []
    stack: "list[bool]" = []
    for line in path.read_text().splitlines():
        opened = _IF_OPEN.match(line)
        if opened:
            stack.append(macro in opened.group(2))
            guarded.append(any(stack))
            continue
        if _ENDIF.match(line):
            if stack:
                stack.pop()
            guarded.append(any(stack))
            continue
        if _ELSE.match(line) and stack:
            stack[-1] = False
            guarded.append(any(stack))
            continue
        guarded.append(any(stack))
    return guarded


def _unguarded_hits(path: pathlib.Path, token: str,
                    macro: str = "ROBOT_DEBUG") -> "list[tuple[int, str]]":
    guarded = _lines_guarded_by(path, macro)
    return [(number, line)
            for number, (line, is_guarded) in enumerate(
                zip(path.read_text().splitlines(), guarded), start=1)
            if token in line and not is_guarded]


# ---------------------------------------------------------------------------
# The scanner itself -- tested, because everything below trusts it
# ---------------------------------------------------------------------------

def test_guard_scanner_tracks_nesting_and_else(tmp_path):
    source = tmp_path / "sample.cpp"
    source.write_text(
        "outside\n"                 # 1 unguarded
        "#ifdef ROBOT_DEBUG\n"      # 2
        "inside\n"                  # 3 guarded
        "#ifdef OTHER\n"            # 4
        "nested\n"                  # 5 guarded (outer still applies)
        "#endif\n"                  # 6
        "still_inside\n"            # 7 guarded
        "#else\n"                   # 8 -- the shipped-image arm
        "release_arm\n"             # 9 NOT guarded
        "#endif\n"                  # 10
        "after\n")                  # 11 unguarded
    guarded = _lines_guarded_by(source, "ROBOT_DEBUG")
    assert guarded[0] is False        # outside
    assert guarded[2] is True         # inside
    assert guarded[4] is True         # nested
    assert guarded[6] is True         # still_inside
    assert guarded[8] is False        # release_arm -- the whole point
    assert guarded[10] is False       # after


# ---------------------------------------------------------------------------
# comms.cpp -- the parser arms
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("token", ["kVmin", "kGain", "kASteady", "kPos",
                                   "classifyDbgArg"])
def test_comms_tuning_parser_arms_are_debug_only(token):
    hits = _unguarded_hits(_COMMS_CPP, token)
    # Doc-comment mentions are fine and expected -- the grammar is documented
    # above the guard. Only real code must be inside it.
    code_hits = [(number, line) for number, line in hits
                 if not line.lstrip().startswith("//")]
    assert not code_hits, (
        f"{token} appears OUTSIDE #ifdef ROBOT_DEBUG in comms.cpp at "
        f"{[number for number, _ in code_hits]} -- a shipped image must not "
        f"carry a live-tuning parser")


def test_comms_dbg_interception_is_debug_only():
    """Without the guard, an inbound `DBG:` line on a shipped image would be
    parsed instead of counted malformed -- fault injection and live tuning
    cannot exist on a robot in the field."""
    assert not _unguarded_hits(_COMMS_CPP, "pushDbgAction(classifyDbgArg")


# ---------------------------------------------------------------------------
# robot_loop -- the apply arms and their baseline state
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("token", ["setSpeedFloor", "setASteady",
                                   "setPositionErrorMax",
                                   "captureTuningBaseline",
                                   "applyDbgAction"])
def test_robot_loop_tuning_apply_arms_are_debug_only(token):
    for path in (_ROBOT_LOOP_CPP, _ROBOT_LOOP_H):
        hits = [(number, line) for number, line in _unguarded_hits(path, token)
                if not line.lstrip().startswith("//")]
        assert not hits, (
            f"{token} appears OUTSIDE #ifdef ROBOT_DEBUG in {path.name} at "
            f"{[number for number, _ in hits]}")


@pytest.mark.parametrize("token", ["dbgTuningBaselined_",
                                   "dbgBoundsBaseline_",
                                   "dbgDutyPerSpeedBaseLeft_",
                                   "dbgDutyPerSpeedBaseRight_"])
def test_robot_loop_tuning_baseline_members_cost_a_shipped_image_nothing(token):
    """RAM on this part is permanently near-full by design, so a debug-only
    member that leaks into the release build is a real cost, not a
    tidiness point."""
    hits = [(number, line) for number, line
            in _unguarded_hits(_ROBOT_LOOP_H, token)
            if not line.lstrip().startswith("//")]
    assert not hits, f"{token} is not inside #ifdef ROBOT_DEBUG in robot_loop.h"


# ---------------------------------------------------------------------------
# drive.h -- the setters themselves are NOT guarded, deliberately
# ---------------------------------------------------------------------------

def test_drive_setters_are_unguarded_on_purpose():
    """`DifferentialDrive::setSpeedFloor()`/`setAdaptation()`/
    `setPositionErrorMax()` are ordinary public setters and stay compiled in
    every build.

    That is deliberate, not an oversight: they are plain assignments into the
    kernel's own `Config` (which `setConfig()` writes at boot anyway), they
    are what a WHEEL_CONTROL wire push calls, and guarding them would make
    the header read differently depending on a macro that has nothing to do
    with the drive. What must be debug-only is the wire SURFACE that reaches
    them -- which is what every test above holds. This test exists so the
    distinction is recorded rather than rediscovered as a suspected
    inconsistency.

    SIGNATURES UPDATED for the DifferentialDrive kernel rework: the setters
    are CHAINABLE now, so they return `DifferentialDrive&`, not `void`
    (config surface 1 of 3: construct empty, chain setters). `setASteady()`
    no longer exists on its own -- aSteady is one of the three Stage C
    parameters folded into the grouped `setAdaptation(biasMax, tauAdapt,
    aSteady)`, grouped precisely because they must move together against the
    fiber's per-cycle config snapshot.
    """
    text = _DRIVE_H.read_text()
    for setter in ("DifferentialDrive& setSpeedFloor(",
                   "DifferentialDrive& setAdaptation(",
                   "DifferentialDrive& setPositionErrorMax("):
        assert setter in text, f"{setter} missing from differential_drive.h"
    guarded = _lines_guarded_by(_DRIVE_H, "ROBOT_DEBUG")
    for number, line in enumerate(text.splitlines()):
        if ("DifferentialDrive& setSpeedFloor(" in line
                or "DifferentialDrive& setAdaptation(" in line):
            assert not guarded[number], (
                "the DifferentialDrive setters are intentionally unguarded -- see this "
                "test's own docstring before changing that")
