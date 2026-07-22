---
id: 008
title: Sim system tests for MOVE protocol
status: open
use-cases: [SUC-050, SUC-051, SUC-052, SUC-053, SUC-054, SUC-055]
depends-on: ['006']
github-issue: ''
issue: protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim system tests for MOVE protocol

## Description

New `src/tests/sim/system/test_move_protocol.py` (+ harness), following
the existing `sim/system/` pairing convention (`straight_twist_harness.cpp`/
`test_straight_twist.py`, `scripted_twist_demo_harness.cpp`/
`test_scripted_twist_demo.py`). Exercises the full MOVE protocol end to
end through `SimHarness` driving the real firmware via `SimPlant` — per
the project's "one Sim object shared by tests and TestGUI" convention, not
a mock of `RobotLoop`/`MoveQueue`. This is the sim-executable half of the
protocol set-point issue's Verification section (stop conditions,
chaining, replace, `ERR_FULL`, timeout fault, no-deadman drain) — the
scenarios that don't strictly require real hardware to validate, per
sprint.md's Migration Concerns ("every MOVE-protocol scenario ... is a
hard, sim-executable acceptance criterion regardless of hardware
presence").

## Acceptance Criteria

- [ ] TIME/DISTANCE/ANGLE stop conditions each reach completion within
      tolerance, driven through `SimHarness::injectMove()` and stepped via
      `SimHarness::step()` (SUC-050).
- [ ] A DISTANCE/ANGLE MOVE whose target the sim plant cannot reach within
      `timeout` ends at `timeout` with `kFlagFaultMoveTimeout` set
      (SUC-054).
- [ ] Chaining: MOVE B (`replace=false`) sent while A runs hands off
      seamlessly at A's expiry — no cycle with zero commanded velocity in
      between (SUC-051).
- [ ] `replace=true` preempts mid-motion on the same cycle it arrives
      (SUC-051).
- [ ] A 5th pending MOVE is rejected `ERR_FULL`; the existing active + 4
      pending contents are unchanged (SUC-052).
- [ ] An empty-queue MOVE expiry stops motors within one cycle, with zero
      further commands injected by the test after the expiring MOVE
      (SUC-053).
- [ ] A CONFIG patch injected mid-MOVE does not change the active MOVE's
      completion outcome (SUC-055).
- [ ] All scenarios run against the real firmware (`RobotLoop`/`MoveQueue`/
      `StopCondition`/`Drive`/`Odometry`), not a test double.

## Testing

- **Existing tests to run**: `src/tests/sim/system/test_straight_twist.py`,
  `test_scripted_twist_demo.py`, `test_sim_api.py` (confirm the sim
  harness itself is unaffected by the cutover).
- **New tests to write**: `test_move_protocol.py` per the acceptance
  criteria above.
- **Verification command**: `python build.py && uv run python -m pytest
  src/tests/sim/system/`
