---
id: '006'
title: 'Sim lockstep: strip pilot/executor/heading-source from sim_harness'
status: done
use-cases:
- SUC-045
- SUC-047
depends-on:
- '005'
github-issue: ''
issue: ''
completes_issue: true
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

- [x] `src/sim/sim_harness.h`: `Motion::Executor`/`App::Pilot`/
      `App::HeadingSource` includes, members, `configurePlanner()`, and
      their accessors removed. `SimHarness`'s remaining construction
      (real `App::RobotLoop` graph against a `SimPlant` in the
      `I2CBus&` slot) matches ticket 005's updated `RobotLoop`
      constructor signature (no `pilot` argument).
- [x] `src/tests/sim/support/wire_test_codec.*` (ticket text says
      `src/tests/_infra/sim/support/` — that path is stale; the file
      actually lives at `src/tests/sim/support/`, matching
      `src/sim/CMakeLists.txt`'s own `SUPPORT_DIR`): MOVE encode/decode
      helpers removed; `EncoderReading`/`OtosReading`/`flags` decode
      helpers added, matching ticket 003's rewritten `telemetry.proto`.
      `Twist`/`Stop` encode/decode untouched (no `Config` encode existed
      before or after — nothing to touch there).
- [x] `python build.py`'s host sim library (`build_host_sim()`) builds
      clean.
- [x] `src/tests/sim/unit/sim_harness_configure_harness.cpp` /
      `test_sim_harness_configure.py` updated to compile and pass
      against the reshaped `SimHarness`.
- [x] **Optional stretch** (sprint.md Open Questions #4 — not required
      for this ticket's completion): `src/sim/sim_plant.cpp`'s
      `OtosPlant` models real `v_x`/`v_y` instead of hard-zeroing them
      (verified today at sim_plant.cpp:220-226 — the comment block
      starts at :220, the hard-zero loop itself at :225). If attempted
      and it doesn't land cleanly, defer it rather than let it block
      this ticket — `OtosReading.v_x`/`v_y` already ride the wire
      correctly either way (0 in sim, real on hardware). LANDED:
      `TestSim::OtosPlant::v_x()`/`v_y()` (finite-difference forward
      velocity from the same `distance`/`dt` `step()` already computes;
      `v_y()` stays 0 — no lateral-slip model) and
      `sim_plant.cpp::handleOtosRead()` now encodes real `rvx`/`rvy`
      instead of hard-zeroing bytes 6-9.

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
