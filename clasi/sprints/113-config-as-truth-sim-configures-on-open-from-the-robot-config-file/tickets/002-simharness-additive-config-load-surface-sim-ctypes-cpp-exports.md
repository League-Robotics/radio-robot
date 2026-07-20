---
id: '002'
title: SimHarness additive config-load surface + sim_ctypes.cpp exports
status: open
use-cases: [SUC-001, SUC-002, SUC-005]
depends-on: ['001']
github-issue: ''
issue: config-as-truth-sim-configure-on-open.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# SimHarness additive config-load surface + sim_ctypes.cpp exports

## Description

`TestSim::SimHarness` (`src/sim/sim_harness.h`) currently only ever configures
its `Motion::Executor`/`App::HeadingSource`/`App::Drive`/`App::Pilot`/motors
from its own private, hardcoded `makeExecutorConfig()`/`makeMotorConfig()`
(called once, from the constructor). There is no way for a caller (the host,
via ctypes) to supply a different, JSON-derived config after construction.

Three existing sim-only test hooks — `setYawRateMax()`, `setLeadCompensation()`,
`setDistanceKp()` — already demonstrate the exact shape needed: build a fresh
`msg::PlannerConfig` and re-call `executor_.configure(cfg);
headingSource_.configure(cfg); drive_.configure(cfg);
pilot_.configureHeading(cfg);` on all four existing consumers. This ticket
**generalizes that pattern** into two public, reusable methods instead of
one-hook-per-field, and exposes them over the ctypes ABI so the host
(ticket 005) can push a complete config in one call each for the planner and
per-motor pieces.

**Critical constraint**: `SimHarness`'s existing no-args constructor path
(`makeExecutorConfig()`/`makeMotorConfig()`, still hardcoded) **must not
change** — ~40 existing C++ test files under `src/tests/sim/unit/` and
`src/tests/sim/system/` construct `SimHarness` with no arguments and never
call the new methods; they must observe byte-for-byte identical behavior
after this ticket (SUC-005). This is purely additive capability.

`sim_create()` (`sim_ctypes.cpp`) calls `harness->boot()` immediately — there
is no window between handle creation and boot completion. The new methods
must therefore be safe to call **either before or after `boot()`** (boot
sequencing itself doesn't depend on `PlannerConfig`/`MotorConfig` values —
only Preamble's own sensor-probe retry logic runs during boot, which these
methods don't touch); the host (ticket 005/006) will call them immediately
after `sim_create()` returns, before injecting any twist/move.

## Acceptance Criteria

- [ ] `SimHarness` gains a public `void configurePlanner(const
      msg::PlannerConfig& cfg)` that calls `executor_.configure(cfg);
      headingSource_.configure(cfg); drive_.configure(cfg);
      pilot_.configureHeading(cfg);` — the identical 4-call fan-out the
      constructor and the three existing test hooks already use — and
      replaces the "last known config" the existing `setYawRateMax()`/
      `setLeadCompensation()`/`setDistanceKp()` hooks currently reconstruct
      piecemeal from `makeExecutorConfig()` plus remembered override fields
      (`lastYawRateMax_` etc.). Refactor those three hooks to build their
      override on top of whatever `configurePlanner()` last received
      (falling back to `makeExecutorConfig()` if it was never called) rather
      than always restarting from `makeExecutorConfig()` — otherwise a
      caller who first calls `configurePlanner()` with real robot values and
      then calls `setYawRateMax()` would silently lose everything else
      `configurePlanner()` set. Preserve each hook's existing public
      signature and behavior when `configurePlanner()` was never called
      (regression: existing callers of these three hooks must be unaffected).
- [ ] `SimHarness` gains a public `void configureMotor(uint32_t port, const
      Devices::MotorConfig& cfg)` that calls `armorL_.configure(cfg)` or
      `armorR_.configure(cfg)` depending on `port` (1=left, 2=right, matching
      every other port-keyed convention in this file — see
      `setEncScaleErr()`/`setEncTickQuant()` for the precedent).
- [ ] A test-only readback is added for verification (ticket 007 needs this):
      either a `const msg::PlannerConfig& plannerConfig() const` accessor
      exposing what `configurePlanner()` last set (note: `SimHarness`
      already exposes this indirectly via `pilotQueueDepth()`-style
      forwarding to `pilot_.plannerConfig()` — reuse/extend that existing
      forwarding rather than adding a parallel copy) — or a small snapshot
      dump. Choose whichever is less code; document the choice.
- [ ] `sim_ctypes.cpp` gains `sim_configure_planner(SimHandle h, <one float
      arg per msg::PlannerConfig field this sprint's Tier 2 covers: a_max,
      a_decel, v_body_max, yaw_rate_max, yaw_acc_max, j_max, yaw_jerk_max,
      min_speed, heading_kp, heading_kd, arrive_dwell, heading_source (int),
      heading_dwell_tol, heading_dwell_rate, heading_lead_bias, plan_lead,
      terminal_lead, actuation_lag, distance_kp, distance_tol,
      model_tau_lin, model_tau_ang>)` — builds a `msg::PlannerConfig` from
      the arguments and calls `configurePlanner()`. Follow `_bind_ctypes()`'s
      existing exhaustive-transcription convention (ticket 005 binds
      `argtypes`/`restype` on the Python side; this ticket only needs the C++
      export and its doc comment enumerating the export list addition,
      matching `sim_ctypes.cpp`'s own header-comment convention of listing
      every export).
- [ ] `sim_ctypes.cpp` gains `sim_configure_motor(SimHandle h, int port,
      float velFiltAlpha, int fwdSign)` — the two `Devices::MotorConfig`
      fields with no live wire arm (`vel_filt`, `fwd_sign` — everything else
      motor-related, `travel_calib`/PID gains, stays on the existing Tier-1
      wire path per `sprint.md`'s Design Rationale). Builds a `Devices::
      MotorConfig` (reading the CURRENT `wheelTravelCalib`/`velGains`/
      `slewRate` off the live motor first via whatever accessor already
      exists, so this call doesn't clobber values Tier 1 already pushed —
      mirrors `RobotLoop::handleConfig()`'s own "merge, don't clobber"
      convention for `MotorConfigPatch`) and calls `configureMotor()`.
- [ ] A small new C++ test (new file under `src/tests/sim/unit/`, e.g.
      `sim_harness_configure_harness.cpp` + `test_sim_harness_configure.py`,
      matching the existing `_harness.cpp`/`test_*.py` pairing convention)
      constructs a default `SimHarness`, calls `configurePlanner()`/
      `configureMotor()` with values that differ from the hardcoded
      defaults, and asserts (via the new readback) that they took effect —
      proving the additive surface works in isolation, without touching any
      existing harness file.
- [ ] None of the ~40 existing files under `src/tests/sim/unit/` or
      `src/tests/sim/system/` are modified by this ticket.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` (~6 min) — the
  gate for "no existing sim test changed behavior" (SUC-005). Also run the
  sim's own C++ test build (however this project currently invokes it —
  check `justfile`/`src/sim/build.py` for the harness-build-and-run step)
  to confirm the ~40 existing harnesses still compile and pass.
- **New tests to write**: `sim_harness_configure_harness.cpp` +
  `test_sim_harness_configure.py` as above.
- **Verification command**: `uv run python -m pytest src/tests/sim/ -v`
  for the sim-scoped subset first (faster iteration), then the full suite.

## Files to touch

- `src/sim/sim_harness.h` (`configurePlanner()`, `configureMotor()`, refactor
  of `setYawRateMax()`/`setLeadCompensation()`/`setDistanceKp()`, readback
  accessor)
- `src/sim/sim_ctypes.cpp` (`sim_configure_planner`, `sim_configure_motor`
  exports, header-comment export-list update)
- New: `src/tests/sim/unit/sim_harness_configure_harness.cpp` +
  `src/tests/sim/unit/test_sim_harness_configure.py`

## Depends On

- Ticket 001 (needs `msg::PlannerConfig.model_tau_lin`/`model_tau_ang` to
  exist before `sim_configure_planner()`'s signature can include them).
