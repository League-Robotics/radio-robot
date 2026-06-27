---
id: '006'
title: Golden-TLM frame canary with committed capture
status: done
use-cases:
- SUC-005
depends-on:
- '003'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Golden-TLM frame canary with committed capture

## Description

Add `tests/simulation/unit/test_golden_tlm.py` — a pytest test that drives the sim through
a fixed, deterministic command sequence (stepped time, no wall-clock), captures the
resulting TLM frame(s), and asserts they match a committed golden capture
`tests/_infra/golden_tlm_capture.json`. Any difference causes the test to fail with a diff.

This is the behavior-preservation oracle for the entire migration. If a source reorganization
accidentally changes EKF output, odometry integration, or the TLM frame layout, this test
catches it even if all unit tests still pass individually.

## Design requirements

**Deterministic**: The command sequence must produce identical output on every run.
- Use the `sim` fixture (fresh Sim, stepped time via `sim.tick_for(ms, step_ms)`).
- No `time.sleep`. No wall-clock timestamps in assertions (use sim time, not system time).
- The sim already uses a fixed-seed LCG for noise — no extra seeding needed.

**Stable**: The sequence should exercise meaningful firmware behavior without being fragile.
- A reasonable sequence: send a `STREAM 50` command, tick for 100 ms (2 TLM frames),
  send a `S speed=100` command, tick for 500 ms (10 frames), send `X` to stop.
- Capture the full TLM text output from the sim's response buffer.

**Self-updating**: Provide a `--update-golden` mechanism or a separate generation script
so the programmer can regenerate the baseline after an intentional behavior change.

## Command sequence (programmer may adjust, but must document the choice)

```
Suggested fixed sequence:
  t=0ms:    SET sTimeout=60000
  t=0ms:    STREAM 50          # TLM every 50 ms
  tick 200ms (step 10ms)       # 4 TLM frames: seq=1..4 with encoder/pose/vel fields
  t=200ms:  S speed=100        # begin streaming at 100 mm/s
  tick 500ms (step 10ms)       # 10 more TLM frames: robot moving, pose changing
  t=700ms:  X                  # stop
  tick 100ms (step 10ms)       # final 2 frames: robot stopped
```

Capture all TLM lines emitted during this sequence. Parse into structured dicts per
field group (enc, pose, vel, otos, twist, etc.). Save as JSON list.

## Golden capture format (`tests/_infra/golden_tlm_capture.json`)

```json
[
  {"seq": 1, "t": 50, "enc": {"l": 0.0, "r": 0.0}, "pose": {"x": 0.0, "y": 0.0, "h": 0}, ...},
  {"seq": 2, "t": 100, ...},
  ...
]
```

Parse TLM lines from `sim.read_lines()` or `sim.send_command("SNAP")` responses.
Include all TLM fields emitted. Use exact field values — do not round.

## Test structure (`test_golden_tlm.py`)

```python
# Pseudocode — programmer implements:
import json, pathlib, pytest

GOLDEN = pathlib.Path(__file__).parents[3] / "tests" / "_infra" / "golden_tlm_capture.json"

def run_fixed_sequence(sim):
    """Drive the sim through the fixed sequence and return parsed TLM frames."""
    frames = []
    sim.send_command("STREAM 50")
    for step in range(20):  # 200ms in 10ms steps
        sim.tick(10)
        frames.extend(parse_tlm(sim.read_lines()))
    sim.send_command("S speed=100")
    for step in range(50):  # 500ms
        sim.tick(10)
        frames.extend(parse_tlm(sim.read_lines()))
    sim.send_command("X")
    for step in range(10):  # 100ms
        sim.tick(10)
        frames.extend(parse_tlm(sim.read_lines()))
    return frames

def test_golden_tlm_unchanged(sim):
    golden = json.loads(GOLDEN.read_text())
    actual = run_fixed_sequence(sim)
    assert len(actual) == len(golden), f"Frame count: golden={len(golden)} actual={len(actual)}"
    for i, (g, a) in enumerate(zip(golden, actual)):
        assert g == a, f"Frame {i} differs:\n  golden: {g}\n  actual: {a}"
```

## Golden capture generation procedure

1. Run the sequence once with capture mode (e.g., a `--update-golden` flag or a one-off
   script `tests/_infra/gen_golden_tlm.py`).
2. Write the output to `tests/_infra/golden_tlm_capture.json`.
3. Run the test — must pass green immediately.
4. Commit both `test_golden_tlm.py` and `golden_tlm_capture.json` together.

## Files to Create

- `tests/simulation/unit/test_golden_tlm.py`
- `tests/_infra/golden_tlm_capture.json` (generated, then committed)
- Optionally: `tests/_infra/gen_golden_tlm.py` (baseline generation script)

## Acceptance Criteria

- [x] `test_golden_tlm.py` exists in `tests/simulation/unit/`.
- [x] `tests/_infra/golden_tlm_capture.json` is committed with ≥ 8 TLM frames (15 frames captured).
- [x] `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -v` passes.
- [x] The test uses the `sim` fixture and stepped time only (no `time.sleep`, no system clock).
- [x] The test is deterministic: running it twice in a row produces the same pass/fail.
- [x] The overall simulation suite still passes ≥ 1954 tests with the new canary added.
- [x] `git diff source/` is empty.

## Testing Plan

```bash
# Full suite including new canary:
uv run --with pytest python -m pytest -q

# Canary alone (must pass):
uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -v

# Determinism check (run twice — both must pass):
uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -v
uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -v
```
