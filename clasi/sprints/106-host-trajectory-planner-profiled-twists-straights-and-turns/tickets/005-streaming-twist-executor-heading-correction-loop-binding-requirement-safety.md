---
id: '005'
title: Streaming twist executor + heading-correction loop + binding-requirement safety
status: open
use-cases:
- SUC-028
- SUC-029
depends-on:
- '002'
- '003'
- '004'
github-issue: ''
issue:
- host-planner-design-lessons-from-drive-v2-review.md
- heading-loop-output-clamp-and-velocity-resonance.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Streaming twist executor + heading-correction loop + binding-requirement safety

## Description

Build `host/robot_radio/planner/executor.py` (`StreamingExecutor`),
`host/robot_radio/planner/heading.py` (`HeadingCorrector`), and
`host/robot_radio/planner/model.py` (the sprint's live-tunable parameter
surface — streaming cadence, accel/decel limits, heading gains/clamp, and
the actuation-latency constant).

The executor walks ticket 004's profile setpoint sequence, sending one
`twist()` per streaming tick at `model.py`'s live-tunable pacing interval
(default ~150ms — the ONLY empirically soak-tested paced rate, per
`ack-ring-intermittent-delivery-gap.md` finding 2; NOT the new ~25Hz
telemetry cadence from ticket 001, which is an unrelated, unvalidated-for-
commands quantity — `architecture-update.md` Decision 6), re-arming the
deadman on every send, and continuously draining telemetry
(`SerialConnection.drain_binary_tlm()`) between sends rather than gating any
control decision on a bounded `wait_for_ack()` — per
`ack-ring-intermittent-delivery-gap.md`'s own explicit recommendation for
this exact use case (`architecture-update.md` Decision 5).

`HeadingCorrector` reads `otos_untrusted` from the active robot config to
select encoder-derived `Telemetry.pose` over `Telemetry.otos`, and computes
a CLAMPED omega trim via a reused `host/robot_radio/controllers/pid.py`
`PID` instance (`architecture-update.md` Decision 7) — the clamp directly
carries forward the deleted on-robot heading loop's own lesson
(`heading-loop-output-clamp-and-velocity-resonance.md` Part 1: an unclamped
correction over-drove the wheels into the ~140mm/s resonance band ticket 002
tames).

This ticket is where every one of the ten binding requirements from
`host-planner-design-lessons-from-drive-v2-review.md` is actually
implemented — see the disposition table below (reproduced from
`architecture-update.md` Step 6).

### Binding-requirements disposition (this ticket's own implementation)

| # | Requirement | Implementation in this ticket |
|---|---|---|
| 1 | Sign-aware completion, no `fabsf`-blind predicates | `executor.py`'s completion check compares signed measured quantities against the profile's own signed target sign — never `fabsf`; grep-verified. |
| 2 | No silent drops | Every validation reject/fault/timeout is logged loudly; Decision 5's continuous-drain design removes the one place (`wait_for_ack()` gating) most likely to silently stall. |
| 3 | Clock discipline across replans | One segment-global elapsed-time clock (captured at profile-run start) per run; never rebased mid-run. |
| 4 | Preemption invalidates chain state | `preempt()`/`stop_now()` always calls `NezhaProtocol.stop()` FIRST, discards remaining old setpoints, and replans (if continuing) from freshly-drained telemetry — never carried entry speed; unit-tested explicitly. |
| 5 | Validate wire inputs | `executor.py` re-validates every `twist()` magnitude against `model.py`'s ceilings immediately before send — independent of, and in addition to, `profile.py`'s own boundary validation (defense in depth). |
| 6 | Bounded overshoot | Completion check has an outer distance/angle bound in BOTH directions; exceeding it is a logged failure, not silently accepted. |
| 7 | Terminal-phase care, no zero-dwell reversal | The terminal setpoint of any run is always an explicit `stop()` call, never reliance on deadman timeout alone; `profile.py` already guarantees no sign-reversal shape, and the executor never reintroduces one. |
| 8 | Latency as a first-class parameter | `model.py`'s explicit, live-tunable latency constant (~130ms tau + link margin), consumed by the heading loop's own correction timing. |
| 9 | Everything tunable live | `model.py` holds every executor/heading parameter, live-editable with no code redeploy. |
| 10 | Heading-loop bandwidth verified empirically | This ticket's own gains are a starting point; ticket 006's bench session measures the ACTUAL achievable correction bandwidth over the post-ticket-001 ~25Hz link before any gain is treated as final. |

## Acceptance Criteria

- [ ] Executor's completion check is sign-aware (never `fabsf` on a signed
      measured quantity) with a bounded outer tolerance in BOTH directions
      (binding requirements #1, #6).
- [ ] No control decision anywhere in `executor.py` is gated on a bounded
      `wait_for_ack()` call — verified by code inspection/grep (binding
      requirement #2, Decision 5).
- [ ] The executor uses a single segment-global elapsed-time clock per
      profile run; a preemption starts a fresh clock, never rebasing a
      stale one (binding requirement #3).
- [ ] Preempting a running profile and starting a new one is unit-tested
      (fake transport/telemetry double) confirming the new profile plans
      from injected "current" state, not carried state from the interrupted
      one (binding requirement #4).
- [ ] Every `twist()` magnitude is validated against `model.py`'s configured
      ceilings immediately before sending, independent of `profile.py`'s own
      validation (binding requirement #5).
- [ ] The terminal setpoint of any profile run is an explicit `stop()` call,
      never reliance on deadman timeout alone; no phase commands a
      zero-dwell sign reversal (binding requirement #7).
- [ ] Streaming cadence, acceleration/deceleration limits, and every gain
      the executor/heading loop use are adjustable at runtime via
      `model.py` with no code redeploy (binding requirement #9).
- [ ] `HeadingCorrector` reads `otos_untrusted` from the active robot config
      and uses encoder-derived `pose`, never `otos`, when the flag is set —
      unit-tested with a fake config + fake telemetry frame.
- [ ] `HeadingCorrector`'s output is clamped to a stated, live-tunable
      ceiling — unit-tested: a large injected heading error never produces
      an omega trim above the ceiling.
- [ ] A fault bit observed mid-run (via drained telemetry) produces a
      logged stop, never silence (binding requirement #2, concrete case).
- [ ] Full unit test suite green; this ticket's own acceptance requires no
      hardware (bench verification is ticket 006's).

## Testing

- **Existing tests to run**: full `uv run python -m pytest`; existing
  `tests/unit/` coverage for `protocol.py`/`serial_conn.py` (mocked
  transport conventions this ticket's own tests should mirror) and
  `controllers/pid.py` (reused, unchanged).
- **New tests to write**: `tests/unit/test_planner_executor.py` (one test
  per binding-requirement acceptance criterion above — preemption, bounded
  overshoot, no-ack-gating via code inspection/grep test, terminal `stop()`,
  fault-bit-triggers-stop), `tests/unit/test_planner_heading.py`
  (`otos_untrusted` source selection, output clamp), and
  `tests/unit/test_planner_model.py` if `model.py`'s own load/override
  plumbing warrants direct coverage.
- **Verification command**: `uv run python -m pytest tests/unit/
  test_planner_executor.py tests/unit/test_planner_heading.py -v`, then the
  full suite.

## Implementation Plan

**Approach**:
- `planner/model.py`: a `PlannerParams` dataclass holding
  `streaming_interval` (default ~150ms), `v_max`/`a_max` (defaults
  consistent with ticket 004's own defaults), `heading_kp`/`heading_kd`,
  `heading_omega_clamp`, and `latency_tau`(~130ms)/link-latency margin —
  plus a simple, genuinely live-editable load/override mechanism (exact
  format — JSON file mirroring `data/robots/*.json`'s convention, or an
  env-overridable dataclass — is this ticket's own implementation call, per
  `architecture-update.md` Step 7 Open Question 5; any format satisfying
  "live-editable, no code redeploy" is acceptable).
- `planner/heading.py`: `HeadingCorrector` wraps a `controllers.pid.PID`
  instance constructed with `model.py`'s heading gains and
  `out_min`/`out_max=±heading_omega_clamp`; reads `robot_config`'s
  `geometry.otos_untrusted` once at construction to fix the pose source for
  the corrector's lifetime; exposes an update-style method taking the
  profile's commanded heading and the latest drained telemetry frame,
  returning the clamped omega trim.
- `planner/executor.py`: `StreamingExecutor` walks a profile's setpoint
  sequence; each tick: re-validates `|v_x|`/`|omega|` against
  `PlannerParams`' ceilings, computes the heading trim via
  `HeadingCorrector`, calls `NezhaProtocol.twist(v_x, omega+trim,
  duration=...)` (duration derived from `streaming_interval` plus a small
  latency margin so the deadman never expires between ticks), drains
  `SerialConnection.drain_binary_tlm()` and decodes it into the frame the
  NEXT tick's heading/completion check consumes, paces to
  `streaming_interval`, and repeats until the profile is exhausted or a
  bounded-overshoot/fault/timeout condition ends the run with an explicit
  `stop()`. `preempt(new_setpoints)`/`stop_now()` always call
  `NezhaProtocol.stop()` first, discard any remaining old setpoints, and (if
  continuing) resume against the NEXT freshly-drained telemetry frame —
  never carried state from the interrupted run.

**Files to create**:
- `host/robot_radio/planner/executor.py`
- `host/robot_radio/planner/heading.py`
- `host/robot_radio/planner/model.py`
- `tests/unit/test_planner_executor.py`
- `tests/unit/test_planner_heading.py`

**Files to modify**: none in existing `robot_radio` modules — this ticket
is a pure new caller of `NezhaProtocol`/`SerialConnection`/`controllers/
pid.py`/`config/robot_config.py`, all otherwise unchanged.

**Testing plan**: unit tests with a fake/mock `SerialConnection`/
`NezhaProtocol` double (mirroring this tree's existing conventions for
testing `protocol.py` callers) — one test per binding-requirement
acceptance criterion listed above, plus the `HeadingCorrector`-specific
source-selection and clamp tests. No hardware required for this ticket's
own gate; ticket 006 is the hardware proof.

**Documentation updates**: `executor.py`/`heading.py`/`model.py` module
docstrings each document their own slice of the binding-requirements
mapping (mirroring the table above) so a future reader does not need to
cross-reference the sprint's architecture doc to understand why a given
check exists.
