---
id: '005'
title: Streaming twist executor + heading-correction loop + binding-requirement safety
status: done
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

- [x] Executor's completion check is sign-aware (never `fabsf` on a signed
      measured quantity) with a bounded outer tolerance in BOTH directions
      (binding requirements #1, #6).
- [x] No control decision anywhere in `executor.py` is gated on a bounded
      `wait_for_ack()` call — verified by code inspection/grep (binding
      requirement #2, Decision 5).
- [x] The executor uses a single segment-global elapsed-time clock per
      profile run; a preemption starts a fresh clock, never rebasing a
      stale one (binding requirement #3).
- [x] Preempting a running profile and starting a new one is unit-tested
      (fake transport/telemetry double) confirming the new profile plans
      from injected "current" state, not carried state from the interrupted
      one (binding requirement #4).
- [x] Every `twist()` magnitude is validated against `model.py`'s configured
      ceilings immediately before sending, independent of `profile.py`'s own
      validation (binding requirement #5).
- [x] The terminal setpoint of any profile run is an explicit `stop()` call,
      never reliance on deadman timeout alone; no phase commands a
      zero-dwell sign reversal (binding requirement #7).
- [x] Streaming cadence, acceleration/deceleration limits, and every gain
      the executor/heading loop use are adjustable at runtime via
      `model.py` with no code redeploy (binding requirement #9).
- [x] `HeadingCorrector` reads `otos_untrusted` from the active robot config
      and uses encoder-derived `pose`, never `otos`, when the flag is set —
      unit-tested with a fake config + fake telemetry frame.
- [x] `HeadingCorrector`'s output is clamped to a stated, live-tunable
      ceiling — unit-tested: a large injected heading error never produces
      an omega trim above the ceiling.
- [x] A fault bit observed mid-run (via drained telemetry) produces a
      logged stop, never silence (binding requirement #2, concrete case).
- [x] Full unit test suite green; this ticket's own acceptance requires no
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

## Completion Notes

**Delivered exactly as planned**: `host/robot_radio/planner/model.py`
(`PlannerParams`), `host/robot_radio/planner/heading.py`
(`HeadingCorrector`), `host/robot_radio/planner/executor.py`
(`StreamingExecutor`), plus `tests/unit/test_planner_model.py`,
`tests/unit/test_planner_heading.py`, `tests/unit/test_planner_executor.py`.
No existing `robot_radio` module was modified — this ticket is a pure new
caller, as planned.

**Ten-item binding-requirements traceability** (architecture-update.md
Step 6 disposition table, reproduced in this ticket's own Description):

1. **Sign-aware completion, no `fabsf`-blind predicates** —
   `StreamingExecutor._within_bound()` (`executor.py`) builds a signed
   `[min(0,target)-tol, max(0,target)+tol]` interval and tests containment
   — never `abs()`/`fabsf()` on a measured value. AST-verified (not raw
   grep, since the module's own header docstring mentions `abs()`/`fabsf()`
   by name while explaining their absence — a substring search would
   false-positive) by
   `test_within_bound_never_calls_abs_or_fabs_on_a_signed_quantity`.
2. **No silent drops** — every clamp (`_clamp_ceiling()`), degraded-
   feedback condition (`HeadingCorrector.update()`), and fault-bit
   observation logs loudly (`logger.warning`/`logger.error`) before
   acting; the executor never calls `wait_for_ack()` anywhere (AST-
   verified by `test_no_wait_for_ack_call_anywhere_in_executor`).
3. **Clock discipline** — `self._run_start` is captured once in `begin()`
   and consumed unchanged by every `tick()` in that run; a preemption's
   own `begin()` call captures a fresh one.
   `test_run_start_clock_is_captured_once_at_begin_not_rebased_per_tick`/
   `test_preemption_captures_a_fresh_clock_never_rebasing_the_stale_one`.
4. **Preemption invalidates chain state** — `preempt()` calls
   `transport.stop()` FIRST, then `begin()` re-drains telemetry and
   rebuilds baseline/commanded-heading/index from that fresh frame.
   `test_preempt_stops_first_then_replans_from_fresh_telemetry_not_carried_state`.
5. **Validate wire inputs** — `_clamp_ceiling()` re-validates `|v_x|`/
   `|omega|` against `PlannerParams.v_max`/`omega_max` immediately before
   every `twist()` send, independent of `profile.py`'s own boundary
   validation.
6. **Bounded overshoot** — the SAME `_within_bound()` interval check is
   run every tick against `overshoot_bound_linear`/`_angular`; tripping it
   ends the run with a logged `RunOutcome.OVERSHOOT`.
7. **Terminal-phase care, no zero-dwell reversal** — the terminal setpoint
   always triggers an explicit `transport.stop()` call in `tick()`;
   `profile.py`'s own terminal setpoint already lands at exactly zero, so
   no reversal is ever reintroduced (`test_completion_never_reintroduces_a_sign_reversal`).
8. **Latency as a first-class parameter** — `PlannerParams.latency_tau` is
   genuinely CONSUMED, not merely declared: `tick()` computes
   `lead_heading = commanded_heading + setpoint.omega * latency_tau` (a
   first-order dead-time lead compensation — the twist sent this tick only
   actuates ~`latency_tau` later, so the corrector aims at where the
   profile will be by then) before calling `HeadingCorrector.update()`.
   Zero on a straight leg (`omega == 0`), so "hold heading" is unaffected.
   Covered by `test_latency_tau_zero_produces_no_lead_on_a_turn`/
   `test_latency_tau_nonzero_leads_the_commanded_heading_on_a_turn`/
   `test_latency_tau_lead_is_zero_on_a_straight_leg`.
9. **Everything tunable live** — every field `executor.py`/`heading.py`
   read comes from `self._params.<field>`, re-read fresh every call (the
   wrapped PID's own gains/clamp are re-synced from `params` inside
   `HeadingCorrector.update()` itself, not just at construction).
   `test_streaming_interval_change_is_reflected_in_next_ticks_duration`/
   `test_heading_gain_change_is_reflected_in_next_ticks_trim`/
   `test_clamp_mutated_after_construction_takes_effect_next_update`/
   `test_kp_mutated_after_construction_takes_effect_next_update`.
   `PlannerParams.load()` additionally layers a JSON file and/or
   `PLANNER_<FIELD>` env vars on top of the defaults, callable again at
   any time with no process restart.
10. **Heading-loop bandwidth verified empirically** — explicitly out of
    this ticket's scope; ticket 006's bench session is the empirical
    measurement. This ticket's `heading_kp=2.0`/`heading_kd=0.0`/
    `heading_omega_clamp=0.5` defaults are a documented starting point,
    not a final tuning.

**Design decisions not spelled out verbatim in the plan**:
- **Stepped `tick()` API, not a single blocking `execute()`.** The plan's
  prose describes a walking loop; this ticket implements it as
  `begin()`/`tick()`/`preempt()`/`stop_now()` plus a blocking `run()`
  convenience wrapper (`begin()` then `tick()` in a loop, pacing with an
  injectable `sleep_fn`). The stepped shape is what makes preemption,
  synthetic on-time/late/dropped-frame telemetry scripting, and the
  binding-requirement unit tests possible without real threads or real
  sleeping — `run()` is the production entry point; tests drive `tick()`
  directly.
- **`TwistTransport` is a `typing.Protocol`**, not a concrete
  `NezhaProtocol` import — structural typing so `tests/unit/
  test_planner_executor.py`'s `FakeTransport` (a plain
  `twist()`/`stop()`/`read_pending_binary_tlm_frames()` double, no real
  serial port or protobuf codec) satisfies the executor's dependency with
  zero adapter code. A real `NezhaProtocol` instance already satisfies
  the same Protocol as-is.
- **Straight-hold and turn-tracking are the SAME mechanism.** `tick()`
  advances `self._commanded_heading` by a trapezoidal integration of each
  setpoint's own `omega` (never the measured value) — for a straight leg
  (`omega == 0` throughout) this never moves, so "hold heading" falls out
  of the identical code path a turn's "track the planned trajectory"
  uses, with no special-casing.
- **`_progress()`/axis parameter.** `begin(setpoints, target, axis=...)`
  takes an explicit signed `target` (the same literal distance/angle the
  caller passed to `profile_for_distance()`/`profile_for_turn()`) and
  `axis` (`"linear"`/`"angular"`) so the bounded-overshoot check has a
  concrete signed quantity to compare against — `"linear"` reads the mean
  of `TLMFrame.enc`, `"angular"` reads `HeadingCorrector.measured_heading()`
  (the SAME `otos_untrusted`-selected source heading correction uses).

**Scope note — no live sim-transport integration in this ticket.** The
team-lead's dispatch asked for a SimApi-driven end-to-end sim proof
(profile → executor → `SimApi` → plant trace) in addition to this
ticket's own unit-test gate. That capability does not exist yet and is
not this ticket's to build: `host/robot_radio/io/sim_conn.py`
(`SimConnection`) targets a ctypes ABI (`tests/_infra/sim/`) that sprint
102 ticket 005 deleted — confirmed dead (`justfile`'s own `build-sim`
comment: "Sim mode is unavailable... testgui is parked until a later
sprint revives it"). The only live `SimApi` is the C++ harness
(`tests/sim/support/sim_api.{h,cpp}`), reachable only from a compiled C++
test binary, not from Python. Ticket 006's own Acceptance Criteria #1 and
Implementation Plan Phase 1 already own exactly this work (`tests/sim/
system/` profiled-straight/turn scenarios against `SimApi`, SUC-030) and
its own Phase 1 prose anticipates the same gap ("`architecture-update.md`
(105) Decision 4 explicitly deferred `io/sim_conn.py` to sprint 107, so
this ticket likely instead injects the SAME setpoint sequence
`planner/profile.py` would generate directly into `SimApi.injectTwist()`
calls"). Building a new Python-to-`SimApi` transport inside ticket 005
would duplicate/pre-empt ticket 006's own planned work and its
`architecture-update.md` Step 3 boundary ("outside — `SimApi` itself...
and the planner modules... does not reimplement their logic. Serves
SUC-030"). This ticket's own Testing section already states its gate is
unit tests only ("No hardware required for this ticket's own gate;
ticket 006 is the hardware proof") — flagging this rather than silently
skipping it, per this project's own no-silent-drops discipline.

**Test totals**: `uv run python -m pytest tests/unit/test_planner_executor.py
tests/unit/test_planner_heading.py tests/unit/test_planner_model.py -v` —
50 passed (8 model, 15 heading, 27 executor — one section per binding-
requirement acceptance criterion, plus the latency_tau traceability
section and the synthetic on-time/late/dropped-frame TLM stream tests).
Full project suite `uv run python -m pytest` — 667 passed, 0 failed, 0
skipped, no regressions in any existing suite.
