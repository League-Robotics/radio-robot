---
id: '002'
title: 'Wheel-frozen fault flag: telemetry bits + host decode'
status: in-progress
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: wheel-frozen-fault-flag-in-telemetry.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wheel-frozen fault flag: telemetry bits + host decode

## Description

Sequenced right after ticket 001 (both touch `nezha_motor.{h,cpp}`; this
ticket is also the direct regression guard for 001's safety fix — if a
duty write is ever silently lost again, this is what makes it visible in
seconds instead of requiring another incident). Stakeholder, 2026-07-31:
*"If you've commanded an encoder and it's been commanded to move for the
last cycle and it hasn't moved, then it's frozen... that should be on the
telemetry, and if it is, then the test program should be throwing big red
errors."*

`Devices::NezhaMotor::wedgeSuspect()` — the **gated** stall detector
(commanded nonzero duty for N consecutive cycles with no encoder change)
— already exists but has zero consumers. Use `wedgeSuspect()`, not the
ungated `wedgeLatched_` (which fires on healthy moves too and would cry
wolf on every leg — this distinction matters, see the issue's own
correction note).

1. `telemetry.h` — `kFlagFaultWheelFrozenLeft = 1u << 19`,
   `kFlagFaultWheelFrozenRight = 1u << 20` (bits 19/20 are the next free
   slots — confirmed by grep against the current flag table).
2. `robot_state.h`'s `Health` struct — `wheelFrozenLeft`/`wheelFrozenRight`.
3. `robot_loop.cpp` — publish `motorL_.wedgeSuspect()`/
   `motorR_.wedgeSuspect()` into `Health` each cycle.
4. Host `protocol.py` — decode the two new flags.
5. NOT IN THIS SPRINT: the TestGUI red banner and the host drive
   loop's abort-on-flag. TestGUI is owned by another agent right now
   (stakeholder, 2026-08-01) — this ticket stops at the decoded flag on
   `TLMFrame`. The GUI consumer rides a later sprint.

**Session note (2026-07-31):** on picking this ticket up, items 1-4 above
(`telemetry.h` flags, `robot_state.h::Health` fields, `robot_loop.cpp`'s
`publishWheels()` publish point, `protocol.py`'s decode properties +
`wheel_frozen_reason()` helper) were already present on this branch,
landed by an earlier session's work in commit `4002fe895` (the same
commit that rescoped this ticket off the GUI) — confirmed sourcing
`wedgeSuspect()` correctly throughout, matching the gated-vs-ungated
correction the issue calls out. The firmware-side gating test
(`src/tests/sim/system/faults/fault_knobs_harness.cpp`'s
`scenarioWheelFrozenGatedFlagSetsOnlyAfterThresholdLeftOnly`) and the
telemetry-projection test
(`src/tests/sim/unit/app_telemetry_harness.cpp`'s
`scenarioWheelFrozenFlagsAreGatedAndIndependentOfWedgeLatch`) were
likewise already present and pass. This session's own work: the missing
host unit test (`test_twist_stop_ack_matcher.py`), the `just build-clean`
+ flash + bench verification (healthy leg, plus a best-effort/negative
stall-probe run), and closing out the acceptance criteria below.

## Acceptance Criteria

- [ ] Physically stall one wheel on the stand: the correct per-wheel flag
      sets within ~0.5 s and is observable on the decoded `TLMFrame`
      (a short bench script, NOT the TestGUI). **NOT COMPLETED — see note
      below.** `src/tests/bench/wheel_frozen_bench.py --stall-wheel
      left|right` is written and bench-run 2026-07-31 against `tovez`
      (`/dev/cu.usbmodem2121102`, fw v0.20260731.14, clean build): the
      `move_wheels()`/ack/telemetry-poll plumbing all work end to end, but
      this run applied NO physical resistance to the wheel (an autonomous
      programmer-agent session has no physical actuator — this is a HITL
      check per `src/tests/CLAUDE.md`'s own `bench/` description, "a
      person is present to hand-load wheels"), so, as expected on an
      unloaded free-spinning wheel under closed-loop velocity control, the
      flag never set — that run is a negative control (confirms free
      spinning does NOT falsely trip the flag), not a positive
      reproduction. Needs a human operator to physically hold the named
      wheel while the script runs; script and instructions are ready.
- [x] A full healthy 700 mm leg raises **neither** flag — verify
      explicitly; a false positive here is worse than no flag at all.
      Bench-verified 2026-07-31 on `tovez`: `wheel_frozen_bench.py`
      (default/negative-case mode) drove a 700 mm leg at 200 mm/s
      (`enc` 0->(774,731)), 143 telemetry frames observed, neither
      `fault_wheel_frozen_left` nor `fault_wheel_frozen_right` ever set.
- [x] A host unit test (`src/tests/unit/`, NOT `src/tests/testgui/`)
      covers decode of both flags, positive and negative case.
      `src/tests/unit/test_twist_stop_ack_matcher.py` — added
      `test_from_pb2_decodes_wheel_frozen_left_only`,
      `..._right_only`, `..._both`,
      `test_from_pb2_wheel_frozen_negative_case_neither_flag_set` (5
      tests incl. the `wheel_frozen_reason()` helper); all pass.
- [x] `grep -n "wedgeSuspect\|wedgeLatched_" src/firm` confirms the
      published flag sources `wedgeSuspect()`, not `wedgeLatched_`.
      Confirmed: `robot_loop.cpp:404-405` — `state_.health.wheelFrozenLeft
      = motorL_.wedgeSuspect(); state_.health.wheelFrozenRight =
      motorR_.wedgeSuspect();` — `wedgeLatched_` never referenced outside
      `motor_armor.h` itself.

## Testing

- **Existing tests run**: `uv run python -m pytest -q` (full suite,
  437 s): 1560 passed, 3 skipped, 10 xfailed, 1 xpassed, 5 failed — all 5
  failures pre-existing and unrelated (confirmed via `git stash` back to
  the pre-session commit `4002fe89` and re-running one of them):
  `test_clock_sync_activation.py`/`test_fake_transport.py` (documented
  standalone-harness include-path failures) and three
  `src/tests/testgui/` tour-turn-angle-tolerance tests (a different
  sprint concern — the sprint's own speed-deficit issue — and untouched
  per the hard testgui exclusion). `src/tests/sim/unit/
  test_app_telemetry.py` + `src/tests/sim/system/faults/
  test_fault_knobs.py` (the two harnesses wrapping
  `app_telemetry_harness.cpp`/`fault_knobs_harness.cpp`) individually
  re-run and pass.
- **New tests written**: `test_from_pb2_decodes_wheel_frozen_left_only`/
  `..._right_only`/`..._both`/
  `test_from_pb2_wheel_frozen_negative_case_neither_flag_set` in
  `src/tests/unit/test_twist_stop_ack_matcher.py` (host decode, positive
  + negative). The firmware gating test
  (`scenarioWheelFrozenGatedFlagSetsOnlyAfterThresholdLeftOnly`) and
  telemetry-projection test
  (`scenarioWheelFrozenFlagsAreGatedAndIndependentOfWedgeLatch`) were
  already present (see session note above) and re-verified passing, not
  re-written.
- **Bench verification**: `src/tests/bench/wheel_frozen_bench.py`
  (new) — healthy-leg negative case run and PASSED (see Acceptance
  Criteria); stall positive case run but NOT completed — needs a human
  operator, see Acceptance Criteria note.
- **Verification command**: `uv run pytest`.

## Implementation Plan

- **Approach**: wire an existing, already-computed detector through to
  telemetry and the host decode — no new detection logic, only
  publication. No GUI work in this sprint.
- **Files to modify**: `src/firm/messages/telemetry.h`,
  `src/firm/types/robot_state.h`, `src/firm/app/robot_loop.cpp`,
  `src/host/robot_radio/robot/protocol.py`. NOTHING under
  `src/host/robot_radio/testgui/`.
- **Documentation updates**: `telemetry.h`'s flag-table comment block
  (bits 19/20); note the gated-vs-ungated distinction inline since it is
  the one correction the source issue explicitly flags as easy to get
  wrong again.
