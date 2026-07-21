---
id: 009
title: Test-suite sweep + green bar (build.py clean, pytest green)
status: open
use-cases: [SUC-045, SUC-047, SUC-048, SUC-049]
depends-on: ["008"]
github-issue: ''
issue: telemetry-frame-tightening-amendment-to-gut-s1.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Test-suite sweep + green bar (build.py clean, pytest green)

## Description

The closing ticket of the "one coherent unit; no intermediate state
compiles" excision (sprint.md Architecture Decision 1) — everything from
ticket 002 through 008 lands here as a certified green whole. Sweeps any
residual executor/pilot/tour/ruckig-dependent test file ticket 002
didn't already catch (that ticket's own bench-script list was a floor,
not a ceiling — "grep and confirm" was explicitly deferred to here where
the full picture is available), edits every survivor for the new
frame/blob shape, and runs the full non-hardware verification the gut
issue's own Test Strategy calls for. Also closes out the amendment
issue's own Verification/Tests requirements (wire round-trip, flags
semantics, reading-stamp monotonicity, single-ack overwrite,
`wait_for_ack` happy+timeout paths).

## Acceptance Criteria

- [ ] Residual grep sweep: `grep -rln "Motion::\|Executor\|Ruckig\|PlannerConfig\|HeadingSource"
      src/ src/tests/` (excluding generated `messages/` and the
      `pre-gut-motion-stack` tag itself, which is history, not tree)
      returns nothing outside of comments/doc-strings explaining what
      was removed and why (e.g. sprint.md itself, DESIGN.md historical
      notes) — no live code or test references any deleted symbol.
  - [ ] Any `src/tests/bench/*.py` script ticket 002 left un-triaged
      (see that ticket's own "grep each remaining script" acceptance
      criterion) is resolved here: deleted if executor/segment/tour-
      dependent, kept otherwise.
- [ ] Survivors edited and green: `app_comms_harness.cpp`/
      `test_app_comms.py`, `app_deadman_harness.cpp`/
      `test_app_deadman.py`, `app_drive_harness.cpp`/`test_app_drive.py`,
      `app_odometry_harness.cpp`/`test_app_odometry.py`,
      `app_preamble_harness.cpp`/`test_app_preamble.py`,
      `app_robot_loop_harness.cpp`/`test_app_robot_loop.py`,
      `app_telemetry_harness.cpp`/`test_app_telemetry.py`,
      `config_gate_harness.cpp`/`test_config_gate.py`,
      `devices_*_harness.cpp`/`test_devices_*.py` (all — should be
      unaffected, verify not accidentally broken),
      `persisted_tuning_harness.cpp`/`test_persisted_tuning.py`
      (ticket 004's own edit, re-verified here as part of the whole-suite
      pass), `sim_harness_configure_harness.cpp`/
      `test_sim_harness_configure.py` (ticket 006's own edit,
      re-verified), `wire_codec_harness.cpp`/`test_wire_codec.py`,
      `wire_differential_harness.cpp`/`test_wire_differential.py`,
      `test_wire_fuzz.py`, `test_wire_runtime.py`.
- [ ] Sim system-test green bar (the gut issue's own named post-gut
      survivors): `test_straight_twist.py`, `test_scripted_twist_demo.py`,
      `test_sim_api.py`, `test_sim_boot_config_parity.py`,
      `test_sim_configure_from_robot.py`.
- [ ] New wire round-trip tests added (per the amendment issue's
      Verification section, not yet covered by any ticket above):
      `EncoderReading`/`OtosReading`/full-frame round-trip; `flags`
      semantics exercised across all three bit groups
      (status/fault/event); reading `time` stamps monotonic and
      consistent with the frame's own `now`; single-ack overwrite
      behavior (a second ack within one primary period overwrites the
      first — the "ack-depth-1 tradeoff", stakeholder-accepted);
      `wait_for_ack` happy-path and timeout-retry paths.
- [ ] `uv run python -m pytest` green on the full surviving suite.
- [ ] `python build.py` builds firmware + host sim lib clean.
- [ ] Flash-freed confirmation: build output shows firmware flash usage
      dropped by approximately the expected ~164 KiB versus a pre-gut
      build (informational — record the actual number, don't block on
      hitting the estimate exactly).

## Implementation Plan

**Approach**: Run the full suite first to get a concrete failure list
(faster than guessing which files need touching), then work through
failures grouped by cause (residual dead references vs. shape mismatches
vs. genuinely missing new tests), then do the grep sweep as a final
confirmation pass (catches anything the failure list didn't surface
because a stale file simply wasn't being collected/run at all).

**Files to modify**: the survivor list in Acceptance Criteria above,
plus any newly-identified residual file from the grep sweep. New test
files for the round-trip/flags/ack coverage, colocated with the existing
wire-codec test suite (`src/tests/sim/unit/test_wire_codec.py` or a
sibling).

**Testing plan**: `uv run python -m pytest` (full suite) and `python
build.py` (both targets) are this ticket's own acceptance criteria —
see above.

**Documentation updates**: sweep `src/firm/DESIGN.md`,
`src/firm/app/DESIGN.md`, `src/firm/motion/DESIGN.md` (if the directory
still exists as a stub — otherwise its deletion in ticket 002 already
removed the doc with it), `src/firm/devices/DESIGN.md` for any remaining
reference to a deleted subsystem; update or remove as appropriate.
