---
id: '001'
title: Convert Planner to a live RobotConfig reference
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: set-config-not-propagated-to-planner.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Convert Planner to a live RobotConfig reference

## Description

`Planner` is the only subsystem in the control stack that holds a
`RobotConfig` by VALUE (`RobotConfig _cfg;` at `Planner.h:190`) instead of a
live reference. Every sibling subsystem — `Drive` (`_robCfg`),
`Superstructure` (`_cfg`), `MotorController` (`_cal`), `Ports` (`_cfg`),
`BodyVelocityController` (`_cfg`), `Motor` (`_cfg`), `OtosSensor` real and
sim (`_cfg`) — holds `const RobotConfig&`, bound at construction to the
single `RobotConfig` `Robot` owns, and therefore automatically observes
every committed `SET`, annotated or not.

`Planner`'s private copy is refreshed only by `Planner::configure()`, which
is itself only invoked when a `"planner"`-annotated key is SET, and even
then only patches an eleven-field whitelist (`msg::PlannerConfig`) — the
plain keys `rotSlip`, `tw`, `vWheelMax`, `rotGainPos`, `rotGainNeg`, and
`ctrlPeriod` are read directly off `_cfg` throughout `PlannerBegin.cpp`/
`Planner.cpp` and are frozen at the boot default forever. This is the root
cause of the reported bug (`SET rotSlip=1.0` replies `OK` but never changes
RT turn behavior).

This ticket converts `Planner::_cfg` to `const RobotConfig&`, matching the
pattern already proven correct by the seven sibling subsystems. This is a
structural fix, not a redesign: every current `_cfg.<field>` read site in
`PlannerBegin.cpp`/`Planner.cpp` is already read-only and compiles unchanged
against a reference — the fix is entirely in what memory `_cfg` denotes.

This ticket also fixes the sprint's own smoking-gun test,
`tests/simulation/unit/test_rt_slip.py`, which currently passes for the
wrong reason: its `_arc_after_rt()` helper sends a bare `sim.send_command
("ZERO")` with no token. `parseZero()`
(`source/commands/SystemCommands.cpp`) requires at least one of
`enc`/`pose`/`T`/`D` and replies `ERR badarg` to a bare `ZERO` — a reply the
test never checks. Encoder readings therefore accumulate, un-reset, across
the two sequential `RT 9000` calls each test function makes, faking a slip
effect that isn't real (instrumented and confirmed during sprint planning:
`_cfg.rotationalSlip` reads `0.920000` on every call regardless of any
prior `SET rotSlip=...`, and a corrected, isolated measurement shows the
arc is identical — 105.02 mm — at `rotSlip=1.0` and `rotSlip=0.5`).

See `architecture-update.md` Step 4-5 item 1 and Design Rationale
Decision 1 for the full audit and the alternatives considered (per-key
annotation was rejected as it leaves the same landmine in place for future
Planner fields).

## Acceptance Criteria

- [ ] `source/superstructure/Planner.h`: `RobotConfig _cfg;` (owned value)
      changed to `const RobotConfig& _cfg;` (reference). Header comment
      updated to describe the new live-reference contract.
- [ ] `source/superstructure/Planner.cpp`: constructor's `_cfg(cfg)`
      member-init is unchanged (reference members bind with the same
      syntax as value members). `Planner::configure()` loses the lines
      (~634-645 today) that assign into `_cfg.<field>` — these would not
      compile against a `const`-qualified reference and are no longer
      needed since `_cfg` is always current.
- [ ] `_planCfg = cfg;` (the separate, still-owned `msg::PlannerConfig`
      member written by `configure()`) is retained unchanged — it is
      confirmed-dead (never read anywhere) but out of this ticket's scope
      to remove (see architecture-update.md Design Rationale Decision 4;
      flagged as Open Question 2 for a future cleanup sprint).
- [ ] `source/control/PlannerBegin.cpp` requires no code changes — verify
      by inspection that every `_cfg.<field>` read there is read-only.
- [ ] `Robot`'s member declaration order still constructs `config` before
      `planner` (already required for `Drive`'s existing reference to be
      valid) — verify no reordering is needed.
- [ ] `tests/simulation/unit/test_rt_slip.py`: `_arc_after_rt()`'s bare
      `sim.send_command("ZERO")` changed to `sim.send_command("ZERO enc")`,
      with the reply checked (`assert "OK" in reply`) so a future rejected
      `ZERO` fails loudly instead of silently degrading into an
      accumulation artifact.
- [ ] `test_rt_slip.py`'s three existing tests pass for the right reason:
      with `_cfg` live, `SET rotSlip=<a>` vs `SET rotSlip=<b>` genuinely
      produce different RT arcs (confirm by temporarily asserting the arcs
      differ by more than a small epsilon, not just that the test doesn't
      error).
- [ ] `SET rotSlip=<x>` measurably changes the RT 9000 arc target on the
      next invocation, isolated from any prior RT call.
- [ ] `SET tw=<x>`, `SET vWheelMax=<x>`, `SET rotGainPos=<x>`,
      `SET rotGainNeg=<x>` each change Planner's use of that value without
      requiring any other key to be SET in the same command.
- [ ] `SET ctrlPeriod=<x>` changes Planner's own tick-throttle cadence.
- [ ] `test_059_config_routing.py::test_set_amax_routes_to_planner`
      continues to pass unchanged (it asserts the projected
      `PlannerConfig.a_max` reflects the SET value via the still-live
      whitelist path, which remains true after this change).
- [ ] Full default sim/unit test suite green.

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_rt_slip.py`,
  `tests/simulation/unit/test_059_config_routing.py`, full default suite
  via `uv run python -m pytest`.
- **New tests to write**: none required by this ticket beyond the
  `test_rt_slip.py` fix above — the dedicated propagation sweep is Ticket
  004. Optionally add a minimal isolated assertion in `test_rt_slip.py`
  that `SET tw=<x>` changes RT's arc too, if convenient alongside the
  `rotSlip` fix (not required; covered by Ticket 004 either way).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Change `Planner::_cfg`'s declared type from `RobotConfig` to
`const RobotConfig&`, matching the binding already used by
`Drive`/`Superstructure`/`MotorController`/`Ports`/
`BodyVelocityController`/`Motor`/`OtosSensor`. Remove `configure()`'s now
partially-invalid whitelist-copy assignments into `_cfg` (they will not
compile against a const reference member). Fix `test_rt_slip.py`'s
`ZERO`→`ZERO enc` bug separately (same ticket, since it's the test that
would otherwise mask a regression of this exact fix).

**Files to modify**:
- `source/superstructure/Planner.h` — `_cfg` field declaration + comment.
- `source/superstructure/Planner.cpp` — remove invalid assignments in
  `configure()`; leave `_planCfg = cfg;` in place.
- `tests/simulation/unit/test_rt_slip.py` — `_arc_after_rt()` fix.

**Testing plan**:
- Run `test_rt_slip.py` and confirm the three existing tests still pass,
  and confirm (by temporarily widening an assertion or adding a debug
  print, then reverting) that the arcs now genuinely differ between
  `rotSlip` values rather than passing by encoder-accumulation coincidence.
- Run `test_059_config_routing.py` to confirm the whitelist-projection path
  (`aMax`/`vBodyMax`/`yawRateMax`/`arriveTol` etc.) is unaffected.
- Run the full default suite (`uv run python -m pytest`) and confirm no
  new failures beyond the pre-existing baseline.

**Documentation updates**: none — `architecture-update.md` already
documents this change in full (Step 4-5 item 1, Design Rationale
Decision 1). No wire-protocol change, no `RobotConfig` schema change.
