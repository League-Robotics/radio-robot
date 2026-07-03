---
id: '072'
title: 'Motion terminal safety: signed DISTANCE stop, D-drive terminal completion,
  sim stiction/lag for testability'
status: planning-docs
branch: sprint/072-motion-terminal-safety-signed-distance-stop-d-drive-terminal-completion-sim-stiction-lag-for-testability
use-cases: []
issues:
- distance-stop-fabsf-accepts-backward-completion.md
- d-drive-terminal-instability-reversal-thrash.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 072: Motion terminal safety: signed DISTANCE stop, D-drive terminal completion, sim stiction/lag for testability

## Goals

Close two defects in the `D` (distance-drive) termination path — one a
safety defect, one a reliability defect — and add the sim plant capability
(motor stiction/breakaway, optionally response lag) needed to actually
regression-test the reliability fix, which is otherwise structurally
untestable against today's zero-lag/zero-stiction plant.

## Problem

1. **Safety**: `StopCondition::Kind::DISTANCE` (and `Kind::ROTATION`) gate on
   `fabsf(delta) >= target`, with no notion of which direction was
   commanded. A robot running away BACKWARD on a forward `D` self-reports
   `EVT done D reason=dist` once it has traveled the target magnitude the
   WRONG way — reproduced in a forced-stall sim experiment against the real
   firmware control code, which drove over a meter in full reverse before
   reporting success. See `clasi/issues/distance-stop-fabsf-accepts-backward-completion.md`.
2. **Reliability**: field recordings show 5 of 6 `D` drives landing 1-3 mm
   short of their target at near-zero speed, stalling, ramping backward,
   thrashing, and completing on a violent forward lunge — root-caused to the
   interaction of an asymptotic decel profile (`v_cap -> 0` exactly at the
   target), a strict `>=` crossing stop, a down-only speed ratchet, and the
   velocity controller's integrator-freeze deadband. See
   `clasi/issues/d-drive-terminal-instability-reversal-thrash.md`.
3. **Testability**: the sim plant (`PhysicsWorld`) is a purely algebraic,
   zero-lag, zero-stiction function of PWM — it structurally cannot "land
   short," so defect 2's failure mode is unreproducible in sim today except
   via an artificial forced-encoder-cap harness that bypasses the plant
   entirely. This is exactly the "response lag / coast" capability sprint
   069's architecture-update.md Open Question 3 deferred pending a concrete
   trigger; this sprint's field failures are that trigger.

## Solution

Four sequenced tickets (full design in `architecture-update.md`):

1. Add a motor stiction/breakaway gate (and optionally first-order response
   lag) to `PhysicsWorld`, exposed via `SIMSET`/`SIMGET` following sprint
   069's established pattern. Default is a no-op. This is the test vehicle
   for tickets 2-4.
2. Make `StopCondition::Kind::DISTANCE`/`Kind::ROTATION` direction-aware
   (`MotionBaseline` gains a commanded-direction sign) and add a new
   `Kind::SAFETY_MARGIN` wire-visible safety net that force-aborts (HARD
   stop, `EVT safety_stop`) a drive that runs away past a configurable
   margin in the wrong direction — faster than the existing multi-second
   TIME net.
3. Give the D-mode decel hook a terminal-completion guarantee: floor the
   terminal `v_cap` at `minWheelSpeed`, and add a bounded stall-confirm
   window that completes a drive stalled short of, but within tolerance of,
   its target — rather than letting the down-only ratchet hang indefinitely
   or re-approach (which risks reproducing the observed thrash).
4. Regression sweep against the full existing suite, updating the one test
   (`test_distance_fires_for_reverse`) that encoded the old, unsigned
   semantics as correct.

## Success Criteria

- A forward `D` that runs backward does not fire the DISTANCE stop from
  that backward travel, and instead force-aborts via the new
  `SAFETY_MARGIN`/`EVT safety_stop` path well before the old TIME net would.
- A reverse `D` (`D -200 -200 500`) still completes normally on backward
  travel — no regression on the legitimate case.
- Against a stiction-configured sim plant reproducing the field failure
  signature, a `D` drive lands within the configured arrive tolerance, at
  rest, without reversing or thrashing.
- Against the original zero-stiction plant, `D` drive behavior is provably
  unchanged (the new terminal-completion path is inert there).
- The full test suite (2621 tests, confirmed green pre-sprint) passes,
  except the one test deliberately updated to reflect the fixed semantics.
- **Confirmed (ticket 004):** pre-sprint baseline 2621 passed / 0 failed;
  post-ticket-001 2646 (+25); post-ticket-002 2646 (net 0: +6 new, but
  `test_distance_fires_for_reverse` unaffected — see ticket 002's
  Implementation Notes) plus two pre-existing tests updated in place for the
  3-internal-stop-slot budget change (no net count change); post-ticket-003
  2651 (+5); post-ticket-004 (this ticket, final) **2655 passed / 0 failed**
  (baseline 2651 − 1 removed `test_distance_fires_for_reverse` + 2 split
  replacements + 3 new consolidated regression tests), confirmed on two
  consecutive full-suite runs (`uv run python -m pytest`), preceded by a
  `--clean` sim rebuild (`tests/_infra/sim/build/` removed and reconfigured
  from scratch) per the project's stale-incremental-build-on-`/Volumes`
  gotcha.

## Scope

### In Scope

- `PhysicsWorld` stiction/breakaway plant model (+ optional first-order lag),
  `SIMSET`/`SIMGET`-exposed.
- Signed/direction-aware `DISTANCE` and `ROTATION` stop conditions.
- New `StopCondition::Kind::SAFETY_MARGIN` and `EVT safety_stop` runaway
  abort, scoped to `D` (not `G`/PURSUE/`RT` — see architecture-update.md
  Open Question 2).
- D-mode terminal-completion guarantee (`v_cap` floor + stall-confirm
  completion path).
- New `RobotConfig`/`SET`-able tunables for the safety margin, arrive
  tolerance, and stall-confirm window.
- Regression verification against the full existing suite.

### Out of Scope

- Real-hardware (HIL) validation — explicitly deferred; this sprint's
  acceptance is sim (with the new stiction plant) plus the safety logic
  itself. The stakeholder will bench-test separately.
- The "mean-of-wheels stop x encoder latch" compounding failure (one wedged
  wheel driving the healthy wheel far past target) — a related but distinct
  defect, not fixed here.
- Extending `SAFETY_MARGIN` to `G`/PURSUE/`RT` — flagged as an open question
  for a possible follow-up.
- TestGUI exposure of the new `SIMSET` stiction/lag knobs.
- The host-Python identifier-unit-rename split sprint 071 recommended
  slotting into "sprint 072" — that work needs a new home; this sprint
  number went to the motion-safety work instead.

## Test Strategy

Sim-only for this sprint (per the Out of Scope HIL deferral). Ticket 001
adds isolated `PhysicsWorld`/`SIMSET` unit tests for the new knobs plus one
end-to-end scenario test that reproduces the field failure signature against
the CURRENT (pre-fix) control code — the repro vehicle tickets 2-4 are
validated against, with ticket 4 flipping its assertion once the fix lands.
Ticket 002 adds direction/safety-net unit tests (forward-runs-backward does
not fire; reverse-D unaffected; SAFETY_MARGIN forces HARD + correct EVT;
ROTATION in both spin directions). Ticket 003 adds terminal-completion tests
against both the stiction-configured plant (fix validated) and the original
zero-stiction plant (behavior provably unchanged — the control test).
Ticket 004 runs the full suite and updates the one test
(`test_distance_fires_for_reverse`) that encoded the pre-fix semantics.

## Regression Sweep Results (Ticket 004)

**Full-suite counts.** Pre-sprint baseline: 2621 passed, 0 failed. Post-003
(entering ticket 004): 2651 passed, 0 failed. Final, post-004 (this
ticket), confirmed on two consecutive `uv run python -m pytest` runs after
a `--clean` sim rebuild: **2655 passed, 0 failed**. Delta from the 2651
baseline: −1 (`test_distance_fires_for_reverse` removed) +2
(`test_distance_fires_for_commanded_reverse`,
`test_distance_does_not_fire_for_wrong_direction_travel`, both in
`tests/simulation/unit/test_stop_condition.py`) +3 (the new consolidated
regression file, `tests/simulation/system/test_072_004_regression_sweep.py`
— one test per sprint guarantee: stiction-D-drive-completes-cleanly,
safety-stop-fires-on-runaway, nominal-zero-stiction-D-completes-via-dist).
`test_rotation_stop_terminates_spin`
(`tests/simulation/system/test_stop_condition_coverage.py`) confirmed still
passing unmodified.

**EVT/wire additions (all additive-only — new `reason=` token values on
already-recognized base labels; no base label renamed, no existing
`reason=` value repurposed):**

| Base EVT label | New `reason=` token | Existing tokens (unchanged) | Ticket |
|---|---|---|---|
| `EVT safety_stop` | `runaway` | `watchdog` | 002 |
| `EVT done D` | `arrive` | `dist`, `time` | 003 |

**`RobotConfig`/`SIMSET` field audit.** No existing field was renamed or
removed by this sprint. New fields added (all additive): `safetyMargin`
(`RobotConfig`, ticket 002), `distArriveTol`/`stallConfirm` (`RobotConfig`,
ticket 003), `stictionPwmL`/`stictionPwmR` (`SIMSET`-only, ticket 001).

**`reason=dist` host-grep re-confirmation.** Re-ran the grep
architecture-update.md's Migration Concerns performed at authoring time,
against the tree at ticket-004 execution time: `grep -rn "reason=" host/`
and a review of every `reason=dist`/`reason=` occurrence under `tests/`.
No hits of the shape "branches on `reason=dist` specifically and treats any
other value as an error." `host/robot_radio/robot/protocol.py`'s
`wait_for_evt_done()` (the one place that parses the `reason=` token off an
`EVT done`/`EVT safety_stop` line) extracts and returns the token generically
(`reason = r.kv.get("reason")`) without branching on its value; every
`tests/` occurrence of `reason=dist` is a positive assertion scoped to a
scenario that still completes via a clean crossing (unaffected by this
sprint), not a "reject anything else" check. No host code needs updating.

## Architecture Notes

See `architecture-update.md` for the full 7-step design, diagrams, and
Design Rationale. Key decisions: commanded direction lives on
`MotionBaseline` (not a `StopCondition` param); the safety net is a new
`StopCondition::Kind` dispatched via a `MotionCommand`-level special case
(not a bespoke Planner check); stiction is a stateless PWM dead-zone gate,
not a stateful friction model; the terminal-completion tunables are real
`RobotConfig` fields (not `SIMSET`-only), since the fix is a firmware
behavior change, not a sim-only one; "stalled-short-completes" was chosen
over "let the ratchet re-approach" specifically because re-approaching risks
reproducing the observed thrash rather than resolving it.

## GitHub Issues

(No GitHub issues linked yet — this sprint's issues are tracked as CLASI
issue files, listed in frontmatter.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

Ticket files created in `tickets/`, per architecture-update.md Step 4c:

| # | Title | Depends On |
|---|-------|------------|
| 001 | Sim stiction/breakaway plant + SIMSET knobs + repro test | — |
| 002 | Signed/direction-aware DISTANCE + ROTATION stop + EVT safety_stop | 001 |
| 003 | D-mode terminal completion guarantee | 001, 002 |
| 004 | Regression sweep + test update | 001, 002, 003 |

Tickets execute serially in the order listed.
