"""src/tests/sim/system/test_dbg_tuning_verbs.py -- 133-003: the four live
tuning DBG verbs, end to end through the REAL compiled firmware.

This is the round trip `src/tests/bench/velocity_profile_gate.py`'s
`_assert_tuning()` depends on, exercised without a robot:

    inject `DBG:vmin 60` as a cleartext wire line
      -> App::Comms::dispatchLine() intercepts it (ROBOT_DEBUG)
      -> classifyDbgArg() stages a DbgAction on the ring
      -> App::RobotLoop::applyDbgAction() drains it
      -> App::Drive::setSpeedFloor() applies it
      -> App::debugf() formats the echo
      -> App::Comms::sendDebug() emits `DBG:vmin 60.000 applied`
      -> SimLoop.drain_pending_debug() hands it back here

The echo is the contract, not a nicety. The gate REFUSES to report a run
until it sees `applied`, precisely so a verb that applies silently -- or
applies a different value than the one typed -- cannot produce a
measurement attributed to gains that were never set. That makes the exact
echo text a tested interface.

Why the value formatting is checked so specifically: newlib-nano has no
printf float support, so each echo splits a rounded integer milli-unit
into whole and thousandths parts. Truncating instead of rounding would
report `1.019` for a pushed `1.02` (`static_cast<int>(1.02f * 1000.0f)` is
1019) and send an operator hunting a firmware bug that is really a printf.

`src/tests/sim/unit/app_comms_harness.cpp` covers the PARSER arms
exhaustively (including every malformed form). This file covers the part
that harness cannot: that the staged action actually reaches a Drive setter
and comes back out on the wire.

Requires the compiled `src/firm/platform/host/build/libfirmware_host.{dylib,so}`; skips
cleanly if absent, matching every other file in this tier.
"""
from __future__ import annotations

import pathlib
import sys

import pytest

# -> system -> sim -> tests -> src -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_SIM_LIB_PATH = _REPO_ROOT / "src" / "firm" / "platform" / "host" / "build" / _LIB_NAME

pytestmark = pytest.mark.skipif(
    not _SIM_LIB_PATH.exists(),
    reason="sim lib not built -- just build-sim (or cmake --build src/firm/platform/host/build)",
)


def _make_loop():
    """A bare, headless SimLoop -- deterministic manual stepping, no tick
    thread, no Qt. This tier's established helper shape
    (test_sim_debug_channel.py)."""
    from robot_radio.io.sim_loop import SimLoop

    loop = SimLoop(lib_path=_SIM_LIB_PATH)
    loop.connect(start_tick_thread=False)
    return loop


def _push(loop, verb: str, *, cycles: int = 6) -> "list[str]":
    """Inject one `DBG:<verb>` cleartext wire line, step the loop, and
    return every debug line it produced.

    `cycles` is generous on purpose: pump() runs inside the loop's existing
    settle windows, and applyDbgAction() drains the ring once per cycle, so
    the echo lands a cycle or two after the injection rather than
    synchronously.
    """
    loop.drain_pending_debug()  # discard anything from an earlier push
    loop.inject_command(f"DBG:{verb}".encode())
    loop.step(cycles)
    return loop.drain_pending_debug()


def _latest_duty_per_speed(loop, *, cycles: int = 6):
    """The most recent `(left, right)` duty_per_speed pair telemetry
    reported -- App::Drive's own installed conversion scale, which is what
    `DBG:gain` rewrites and therefore the only way to observe the verb's
    effect rather than merely its echo.

    Telemetry is silent at IDLE by design, so a bare `TLM` (kFrame -- an
    unconditional forced emit, honored in every mode) is what makes a parked
    sim produce a frame at all. Passively stepping and hoping would give an
    empty list here.
    """
    loop.drain_pending_tlm()
    loop.inject_command(b"TLM")
    loop.step(cycles)
    frames = loop.drain_pending_tlm()
    pairs = [(f.duty_per_speed_left, f.duty_per_speed_right) for f in frames
             if f.duty_per_speed_left is not None]
    assert pairs, "no telemetry frame carried duty_per_speed"
    return pairs[-1]


@pytest.fixture
def loop():
    sim = _make_loop()
    sim.step(30)  # let boot finish; boot() emits its own banner/READY traffic
    yield sim
    sim.disconnect()


# ---------------------------------------------------------------------------
# Each verb applies and echoes
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("verb,expected", [
    ("vmin 60", "vmin 60.000 applied"),
    ("vmin 0", "vmin 0.000 applied"),
    ("vmin 99.7", "vmin 99.700 applied"),
    ("asteady 250", "asteady 250.000 applied"),
    ("pos 5", "pos 5.000 applied"),
    ("pos 0", "pos 0.000 applied"),
])
def test_scalar_tuning_verb_echoes_the_value_that_landed(loop, verb, expected):
    lines = _push(loop, verb)
    assert any(expected in line for line in lines), (
        f"DBG:{verb} produced {lines!r}, expected a line containing "
        f"{expected!r}")


def test_gain_echoes_both_multipliers(loop):
    lines = _push(loop, "gain 1.02 0.98")
    assert any("gain L=1.020 R=0.980 applied" in line for line in lines), lines


def test_gain_rounds_rather_than_truncates(loop):
    """`static_cast<int>(1.02f * 1000.0f)` is 1019, not 1020. A truncating
    echo would misreport the value that landed by a thousandth -- small,
    but it is the ONE number an operator uses to decide whether the push
    took."""
    lines = _push(loop, "gain 1.02 1.02")
    joined = " ".join(lines)
    assert "1.020" in joined, joined
    assert "1.019" not in joined, joined


def test_every_verb_uses_the_literal_token_applied(loop):
    """The gate waits for `applied` and nothing else. If this token is ever
    reworded, `_assert_tuning()` starts timing out on a robot that is in
    fact correctly tuned -- and the failure looks like broken hardware."""
    for verb in ("vmin 60", "asteady 250", "pos 5", "gain 1 1"):
        lines = _push(loop, verb)
        assert any("applied" in line for line in lines), (verb, lines)


# ---------------------------------------------------------------------------
# Rejection: a bad push must NOT echo `applied`
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("verb", [
    "vmin", "vmin oops", "vmin -1", "vmin 60mm",
    "asteady -0.5", "pos nan",
    "gain 1.02", "gain 0 1", "gain -1 -1",
])
def test_a_rejected_push_never_reports_applied(loop, verb):
    """The single most damaging failure this surface could have: a value
    that did not land, reported as though it did. The gate would then
    attribute a whole run's distance to gains the robot never had."""
    lines = _push(loop, verb)
    assert not any("applied" in line for line in lines), (verb, lines)
    assert any("unrecognized" in line for line in lines), (
        f"DBG:{verb} should be echoed back as unrecognized so the operator "
        f"sees the typo, got {lines!r}")


# ---------------------------------------------------------------------------
# Idempotence and restore -- the two properties an operator relies on
# ---------------------------------------------------------------------------

def test_gain_is_idempotent(loop):
    """`gain` multiplies the BOOT-INSTALLED duty_per_speed pair, not the
    current one, so re-asserting the same value does not compound. Without
    this an operator who re-pushed their settings between profiles would
    silently get a different robot each time -- and the gate re-asserts
    tuning on every reconnect, so that would happen routinely."""
    baseline_left, baseline_right = _latest_duty_per_speed(loop)

    first = _push(loop, "gain 1.02 0.98")
    assert any("gain L=1.020 R=0.980 applied" in line for line in first), first
    once_left, once_right = _latest_duty_per_speed(loop)

    second = _push(loop, "gain 1.02 0.98")
    assert any("gain L=1.020 R=0.980 applied" in line for line in second), second
    twice_left, twice_right = _latest_duty_per_speed(loop)

    # The second push lands on the SAME value as the first. Multiplying the
    # current value instead of the boot baseline would give 1.02^2 = 1.0404
    # here, a silent 2% error that reads as a worse robot.
    assert twice_left == pytest.approx(once_left, rel=1e-6)
    assert twice_right == pytest.approx(once_right, rel=1e-6)
    assert once_left == pytest.approx(baseline_left * 1.02, rel=1e-5)
    assert once_right == pytest.approx(baseline_right * 0.98, rel=1e-5)


def test_clear_restores_the_boot_installed_gains(loop):
    """`DBG:clear`'s documented contract is "clear every injected override".
    A sweep must be undoable without a reflash or a power cycle, or the last
    value pushed silently rides into the next measurement."""
    baseline_left, baseline_right = _latest_duty_per_speed(loop)

    _push(loop, "gain 1.20 0.80")
    tuned_left, tuned_right = _latest_duty_per_speed(loop)
    assert tuned_left == pytest.approx(baseline_left * 1.20, rel=1e-5)

    lines = _push(loop, "clear")
    assert any("clear" in line for line in lines), lines

    restored_left, restored_right = _latest_duty_per_speed(loop)
    assert restored_left == pytest.approx(baseline_left, rel=1e-6)
    assert restored_right == pytest.approx(baseline_right, rel=1e-6)
