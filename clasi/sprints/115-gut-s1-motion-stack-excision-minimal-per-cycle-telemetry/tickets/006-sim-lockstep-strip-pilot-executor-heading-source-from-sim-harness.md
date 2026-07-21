---
id: "006"
title: "Sim lockstep: strip pilot/executor/heading-source from sim_harness"
status: open
use-cases: [SUC-045, SUC-047]
depends-on: ["005"]
github-issue: ""
issue: ""
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim lockstep: strip pilot/executor/heading-source from sim_harness

## Description

Brings the host-buildable sim tree (`TestSim::SimHarness`, sprint-108
vintage, unaffected in its own I2CBus/Clock-interface architecture by
this sprint — see sprint.md's Impact section) back in sync with the
app-layer reshape ticket 005 just landed: `sim_harness.h` still
constructs `Pilot`/`Executor`/`HeadingSource` and calls
`configurePlanner()`, none of which exist anymore. This is the ticket
that makes `python build.py`'s host sim library (not just the ARM
target, already fixed by ticket 005) build clean, and is the
prerequisite for every sim-based system test (ticket 009) and the host
protocol work (ticket 007, which integration-tests against sim-emitted
frames).

## Acceptance Criteria

- [ ] `src/sim/sim_harness.h`: `Motion::Executor`/`App::Pilot`/
      `App::HeadingSource` includes, members, `configurePlanner()`, and
      their accessors removed. `SimHarness`'s remaining construction
      (real `App::RobotLoop` graph against a `SimPlant` in the
      `I2CBus&` slot) matches ticket 005's updated `RobotLoop`
      constructor signature (no `pilot` argument).
- [ ] `src/tests/_infra/sim/support/wire_test_codec.*`: MOVE
      encode/decode helpers removed; `EncoderReading`/`OtosReading`/
      `flags` decode helpers added, matching ticket 003's rewritten
      `telemetry.proto`. `Twist`/`Config`/`Stop` encode/decode
      untouched.
- [ ] `python build.py`'s host sim library (`build_host_sim()`) builds
      clean.
- [ ] `src/tests/sim/unit/sim_harness_configure_harness.cpp` /
      `test_sim_harness_configure.py` updated to compile and pass
      against the reshaped `SimHarness`.
- [ ] **Optional stretch** (sprint.md Open Questions #4 — not required
      for this ticket's completion): `src/sim/sim_plant.cpp`'s
      `OtosPlant` models real `v_x`/`v_y` instead of hard-zeroing them
      (verified today at sim_plant.cpp:220-226 — the comment block
      starts at :220, the hard-zero loop itself at :225). If attempted
      and it doesn't land cleanly, defer it rather than let it block
      this ticket — `OtosReading.v_x`/`v_y` already ride the wire
      correctly either way (0 in sim, real on hardware).

## Implementation Plan

**Approach**: Fix `sim_harness.h`'s construction first (mechanical —
remove what ticket 002 already deleted from the source tree, matching
ticket 005's new `RobotLoop` signature), then `wire_test_codec.*`
(matching ticket 003's new frame shape), then attempt the optional
`OtosPlant` stretch only after both compile and the existing sim system
tests that will be re-verified in ticket 009 give a smoke signal (run a
quick manual `pytest` pass on one surviving sim/system test here as a
sanity check, even though the full sweep is ticket 009's job).

**Files to modify**: `src/sim/sim_harness.h`,
`src/tests/_infra/sim/support/wire_test_codec.{h,cpp}`,
`src/tests/sim/unit/sim_harness_configure_harness.cpp`,
`src/tests/sim/unit/test_sim_harness_configure.py`. Optional:
`src/sim/sim_plant.cpp` (`OtosPlant` only).

**Testing plan**: `python build.py` (host sim lib) clean.
`uv run python -m pytest src/tests/sim/unit/test_sim_harness_configure.py`
green. As a sanity smoke test (not the full sweep):
`uv run python -m pytest src/tests/sim/system/test_straight_twist.py`
green — this is one of the gut issue's own named post-gut survivors and
exercises `SimHarness` end-to-end through a twist-drive scenario, a
reasonable canary that this ticket's changes actually compose correctly
before ticket 007/009 build further on top.

**Documentation updates**: none required — `SimHarness`'s own module
boundary (sprint 108's architecture doc) is unchanged by this ticket,
only its construction-time wiring.
