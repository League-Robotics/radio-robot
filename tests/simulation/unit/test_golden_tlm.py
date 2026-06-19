"""test_golden_tlm.py — Golden-TLM frame canary with committed capture (038-006).

Drives the sim through a fixed, deterministic command sequence (stepped time,
no wall-clock), captures the resulting TLM frame strings, and asserts they
match a committed golden capture ``tests/_infra/golden_tlm_capture.json``.

Any difference — EKF output change, odometry integration change, TLM frame
layout change — causes this test to fail. This is the behavior-preservation
oracle for the entire Phase 0 → Phase A migration.

Fixed command sequence (documented so it can be regenerated):

    t=0ms:    SET sTimeout=60000    (via sim fixture)
    t=0ms:    STREAM 50             (TLM every 50 ms)
    tick 200ms (step 10ms)          → 3 frames: seq=0..2, mode=I, robot idle
    t=200ms:  T 100 100 10000       (drive both wheels at 100 mm/s for 10 s)
    tick 500ms (step 10ms)          → 10 frames: seq=3..12, mode=V, robot moving
    t=700ms:  X                     (stop)
    tick 100ms (step 10ms)          → 2 frames: seq=13..14, mode=I, robot stopped
    Total: 15 frames

To regenerate the golden capture after an INTENTIONAL firmware change:
    python3 -c "
    import sys, json
    sys.path.insert(0, 'tests/_infra/sim')
    from firmware import Sim
    s = Sim()
    s.send_command('SET sTimeout=60000')
    s.send_command('STREAM 50')
    frames  = s.tick_collect_tlm(total_ms=200, step_ms=10)
    s.send_command('T 100 100 10000')
    frames += s.tick_collect_tlm(total_ms=500, step_ms=10)
    s.send_command('X')
    frames += s.tick_collect_tlm(total_ms=100, step_ms=10)
    print(json.dumps(frames, indent=2))
    " > tests/_infra/golden_tlm_capture.json
"""
from __future__ import annotations

import json
import pathlib

import pytest

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

# __file__ is tests/simulation/unit/test_golden_tlm.py
# parents[3] = repo root
GOLDEN = pathlib.Path(__file__).parents[3] / "tests" / "_infra" / "golden_tlm_capture.json"


# ---------------------------------------------------------------------------
# Sequence runner
# ---------------------------------------------------------------------------

def _run_fixed_sequence(sim) -> list[str]:
    """Drive sim through the fixed sequence and return list of TLM frame strings.

    Uses only stepped time (tick_collect_tlm) — no time.sleep, no wall-clock
    timestamps in assertions.  The sim's fixed-seed LCG makes this deterministic.

    Returns a list of raw TLM line strings (no trailing newlines).
    """
    frames: list[str] = []

    # t=0: enable telemetry stream at 50 ms interval
    r = sim.send_command("STREAM 50")
    assert "period=50" in r, f"STREAM 50 rejected: {repr(r)}"

    # Phase 1: 200 ms idle — 3 frames (t=50, 100, 150)
    frames.extend(sim.tick_collect_tlm(total_ms=200, step_ms=10))

    # t=200ms: start driving both wheels at 100 mm/s for 10 s
    r = sim.send_command("T 100 100 10000")
    assert "OK drive" in r, f"T command rejected: {repr(r)}"

    # Phase 2: 500 ms driving — 10 frames (t=200..650)
    frames.extend(sim.tick_collect_tlm(total_ms=500, step_ms=10))

    # t=700ms: stop
    sim.send_command("X")

    # Phase 3: 100 ms after stop — 2 frames (t=700, 750)
    frames.extend(sim.tick_collect_tlm(total_ms=100, step_ms=10))

    return frames


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_golden_tlm_unchanged(sim):
    """TLM frames from the fixed sequence match the committed golden capture.

    Uses the ``sim`` fixture (function-scoped: fresh Sim per test, watchdog
    extended to 60 s).  Uses stepped time only — no wall-clock.

    Determinism: the sim uses a fixed-seed LCG for noise; running this test
    twice in a row produces identical results.
    """
    assert GOLDEN.exists(), (
        f"Golden capture missing: {GOLDEN}\n"
        "Regenerate with: see docstring at top of this file."
    )
    golden: list[str] = json.loads(GOLDEN.read_text())
    actual: list[str] = _run_fixed_sequence(sim)

    # Frame count must match first (better error message than zip exhaustion).
    assert len(actual) == len(golden), (
        f"TLM frame count differs: golden={len(golden)} actual={len(actual)}\n"
        f"  actual frames: {actual}"
    )

    # Each frame must match exactly.
    for i, (g, a) in enumerate(zip(golden, actual)):
        assert g == a, (
            f"TLM frame {i} differs:\n"
            f"  golden: {g!r}\n"
            f"  actual: {a!r}"
        )
