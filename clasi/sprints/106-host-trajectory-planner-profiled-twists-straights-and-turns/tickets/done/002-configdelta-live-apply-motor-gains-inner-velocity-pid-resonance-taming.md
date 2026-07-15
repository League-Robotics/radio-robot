---
id: '002'
title: ConfigDelta live-apply (motor gains) + inner velocity-PID resonance taming
status: done
use-cases:
- SUC-025
depends-on:
- '001'
github-issue: ''
issue:
- host-planner-design-lessons-from-drive-v2-review.md
- heading-loop-output-clamp-and-velocity-resonance.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# ConfigDelta live-apply (motor gains) + inner velocity-PID resonance taming

## Description

Two findings from this sprint's own architecture pass (Step 1), confirmed
against the live tree, block resonance work outright: (1)
`RobotLoop::cycle()`'s `CmdKind::CONFIG` case unconditionally acks
`ERR_UNIMPLEMENTED` — `ConfigDelta` decodes but is never applied — and (2)
`Devices::NezhaMotor` has NO runtime gain mutator at all, only a
constructor-time `const MotorConfig&`. The resonance issue's own prescribed
bench method (`SET pid.kp` on the stand) is pre-P4 text-protocol vocabulary
that no longer exists on the wire, so binding requirement #9 ("everything
tunable live") is currently unmet at the firmware boundary.

This ticket (a) gives `Devices::NezhaMotor` a live gain-apply method taking
ONLY `Devices`-local types (`Devices::Gains`, plain floats) — never the wire
`MotorConfigPatch` type, preserving `source/devices/`'s standing isolation
invariant (never `#include "messages/..."`); (b) wires `RobotLoop::cycle()`'s
`CONFIG` case to decode a `MotorConfigPatch`, translate its present fields
(`kp`/`ki`/`kff`/`i_max`/`kaw`/`travel_calib`) into that method's parameters
(the app-layer `RobotLoop` is the one legitimate translation boundary between
the wire type and the device-local type, mirroring `config.proto`'s own
documented `BinaryChannel` precedent for the SAME `MotorConfigPatch`), and
ack `OK` instead of `ERR_UNIMPLEMENTED` for that ONE patch type —
`DrivetrainConfigPatch`/`PlannerConfigPatch` stay `ERR_UNIMPLEMENTED`,
unchanged, deliberately out of scope (`PlannerConfigPatch`'s
`heading_kp`/`heading_kd` target `Motion::SegmentExecutor`, deleted post-102;
`DrivetrainConfigPatch`'s EKF fields have no on-robot fusion consumer this
sprint); and (c), with live tuning restored, characterizes and tames the
~140 mm/s inner-velocity-PID resonance (`heading-loop-output-clamp-and-
velocity-resonance.md` Part 2) using the on-stand step harness, EXHAUSTING
the already-wire-tunable `kp`/`ki`/`kff`/`iMax`/`kaw` surface first against
the `<~10%` overshoot bar (rise time preserved) before considering promoting
`velFiltAlpha` (currently reflash-only) to wire-tunable or adding a notch
filter — see `architecture-update.md` Decisions 2 and 4.

## Acceptance Criteria

- [x] `Devices::NezhaMotor` gains a live gain-apply method whose parameters
      are exclusively `Devices`-local types (`Devices::Gains`/plain floats)
      — confirmed by inspection that no `messages/...` include is added to
      `source/devices/`.
- [x] `RobotLoop::cycle()`'s `CONFIG` case decodes a `MotorConfigPatch`,
      applies every PRESENT field to both bound motors via the new method
      (matching `config.proto`'s own documented "applied to BOTH bound
      motors unconditionally" convention for `kp`/`ki`/`kff`/`i_max`/`kaw`,
      and per-side for `travel_calib`), and acks `ACK_STATUS_OK`.
- [x] `DrivetrainConfigPatch`/`PlannerConfigPatch` continue acking
      `ERR_UNIMPLEMENTED`, unchanged — confirmed by a test that a
      `ConfigDelta{drivetrain: ...}` or `{planner: ...}` still gets that
      error code.
- [x] A `config()` call carrying `pid.kp`/`ki`/`kff`/`iMax`/`kaw` measurably
      changes the robot's live step response on the SAME boot, with no
      reflash — bench-verified. Confirmed twice: `config()` acks `OK`
      (was `ERR_UNIMPLEMENTED`) same boot, no reflash; and the live gain
      sweep below (`kp`/`kff` trials) measurably changed the observed
      step-response overshoot on the SAME boot across every trial.
- [x] On-stand velocity-step harness (drive-arm step at 70/140/250 mm/s, per
      `heading-loop-output-clamp-and-velocity-resonance.md`'s own
      methodology) shows `<~10%` step overshoot across that range with rise
      time preserved, superseding the interim `vel_kp=0.0014` detuning
      currently shipped in `data/robots/tovez.json`. Met — see Completion
      Notes' before/after table (worst-case 9.3%/4.3%/5.0% across 3
      confirmation runs, rise time FASTER than the interim detune, not
      just preserved).
- [x] If constants-only tuning cannot hit the bar, Completion Notes document
      the attempt and either (a) promote `velFiltAlpha` to a new
      live-tunable `MotorConfigPatch` field (the smallest of the three
      original candidates), or (b) explicitly flag a fast-follow ticket —
      an empirical decision, not pre-assumed by this ticket. **Neither (a)
      nor (b) needed** — constants-only tuning (`kp`/`kff`) hit the bar;
      see Completion Notes for the empirical path (kff was the actual
      lever, not kp/ki/kaw as originally suspected).
- [x] Full project test suite green (`uv run python -m pytest`, 569 passed).
      Fully bench-verified per `.claude/rules/hardware-bench-testing.md` —
      sensors alive, wheels drive both directions, encoders increment,
      live `config()` round trip, full 70/140/250 mm/s step-response grid
      confirmed from a clean reflash (boot defaults only, no live
      `config()`).

## Testing

- **Existing tests to run**: full `uv run python -m pytest`; the existing
  `devices_motor_harness.cpp`-family unit tests
  (`tests/sim/unit/test_app_drive.py` and neighbors) that exercise
  `Devices::NezhaMotor`/`Devices::MotorVelocityPid`, since this ticket adds
  a new mutator to a class they already cover.
- **New tests to write**: a sim/unit test proving the new `NezhaMotor`
  method changes subsequent PID output (mirroring
  `devices_motor_harness.cpp`'s existing scenario 6 pattern — "PID-on chases
  a velocity target"); a `RobotLoop`-level test proving `CONFIG{motor:...}`
  acks `OK` and applies, while `CONFIG{drivetrain:...}`/`{planner:...}`
  still ack `ERR_UNIMPLEMENTED`.
- **Verification command**: `uv run python -m pytest`, plus the on-stand
  step-response bench sweep (manual, per
  `.claude/rules/hardware-bench-testing.md`).

## Implementation Plan

**Approach**: Add a `Devices`-local gain-apply method to
`Devices::NezhaMotor` (e.g. taking a `const Gains&` plus an optional
`travelCalib` float) that mutates `config_`'s relevant fields directly — no
I2C side effect required, since `MotorVelocityPid::compute()` already reads
`config_.velGains` fresh every tick. Wire `RobotLoop::cycle()`'s
`CmdKind::CONFIG` case: when the decoded `ConfigDelta`'s patch is
`MotorConfigPatch`, build a `Devices::Gains` from the PRESENT optional wire
fields (falling back to each motor's current `config_` value for any absent
field — the same `Opt<T>`-presence convention `config.proto` already
documents), call the new method on both `motorL_`/`motorR_` for
`kp`/`ki`/`kff`/`i_max`/`kaw`, and on the side-selected motor only for
`travel_calib`; ack `OK`. Leave the `Drivetrain`/`Planner` patch arms exactly
as today. Then run the on-stand velocity-step harness (drive-arm step,
`tests/bench/` — check `pid_hold_speed.py`/`velocity_chart.py` first for
reuse before writing anything new) against the now-live `config()` path
instead of a reflash loop; iterate `kp`/`ki`/`kff`/`iMax`/`kaw` against the
`<~10%` bar; update `data/robots/tovez.json` if the shipped tuning changes.

**Files to modify**:
- `source/devices/nezha_motor.{h,cpp}` — new live gain-apply method.
- `source/app/robot_loop.cpp` — `CONFIG` case's `MotorConfigPatch` handling.
- `source/devices/velocity_pid.{h,cpp}` — ONLY if constants-only tuning
  proves insufficient (Open Question 1); gain/filter VALUES, not a
  control-law shape change, unless the fallback notch/feedforward path is
  reached.
- `data/robots/tovez.json` — updated gain values if the shipped tuning
  changes.

**Files to create**: a bench step-response sweep script/extension under
`tests/bench/`, ONLY if no existing script already fits the P4 binary plane
for this purpose.

**Testing plan**: sim/unit coverage for the new `NezhaMotor` method and the
`RobotLoop` `CONFIG` dispatch (both patch-applied and still-unimplemented
paths); the on-stand step-response sweep is the real acceptance evidence,
captured numerically in Completion Notes, not merely asserted.

**Documentation updates**: `heading-loop-output-clamp-and-velocity-
resonance.md` updated with the final tamed numbers (or Part 2 marked
resolved) once the bar is met; Completion Notes record whichever of the two
outcomes in the last Acceptance Criterion above actually occurred.

## Completion Notes (2026-07-15)

### What shipped (done, sim + real-hardware confirmed)

1. **`Devices::NezhaMotor::applyGains(gains, travelCalib={})`** +
   `gains()` getter (`source/devices/nezha_motor.{h,cpp}`) — Devices-local
   types only, no isolation-invariant violation. Mutates `config_` directly;
   `MotorVelocityPid::compute()` reads it fresh every tick, so the change
   takes effect the SAME boot.
2. **`RobotLoop::cycle()`'s `CONFIG` case** (`source/app/robot_loop.cpp`)
   now decodes `MotorConfigPatch`, merges each motor's OWN current gains
   against whichever wire fields are PRESENT, applies `kp`/`ki`/`kff`/
   `i_max`/`kaw` to BOTH bound motors and `travel_calib` to the side-selected
   motor only, and acks `OK`. `DrivetrainConfigPatch`/`PlannerConfigPatch`/
   `watchdog`/`none` are unchanged (`ERR_UNIMPLEMENTED`).
3. **New sim/unit coverage**: `devices_motor_harness.cpp` (`applyGains()`
   changes subsequent PID output on the same instance, no reflash;
   `travelCalib` gates independently of gains), `app_robot_loop_harness.cpp`
   (CONFIG dispatch applies to both motors + acks OK for MOTOR, still
   `ERR_UNIMPLEMENTED` for DRIVETRAIN/PLANNER, verified via a raw-byte
   `AckEntry` fingerprint search since no `decode(ReplyEnvelope)` codec
   exists). Full suite: `uv run python -m pytest` — 569 passed.
4. **Bench-confirmed on real hardware** (`/dev/cu.usbmodem2121102`, robot
   `tovez`, this build): `config(**{"pid.kp": 0.0020, "pid.ki": 0.006})`
   acked `OK` (was `ERR_UNIMPLEMENTED` pre-ticket), same boot, no reflash —
   direct proof the live-apply path works end-to-end on the real robot, not
   just in sim. `tests/bench/rig_dev.py`'s own `config()` round-trip check
   (which sends a `sTimeout` WATCHDOG patch, correctly still
   `ERR_UNIMPLEMENTED`) also passed, confirming the DrivetrainConfigPatch/
   PlannerConfigPatch/watchdog-unchanged half.

### Drive-by fix (separate commit, see its own issue —
`secondary-telemetry-starved-by-106-001-cadence-retarget.md`, now archived
to `clasi/issues/done/`): resolved and bench-confirmed. `rig_dev.py`'s
"secondary telemetry received" check — which failed unconditionally
post-106-001 per that issue's own Evidence — passed on every run this
session, including runs affected by the hardware blocker below (the
blocker is drivetrain-specific; telemetry scheduling is unaffected by it).

### Hardware blocker — encountered, then resolved (update)

The hardware fault described in this ticket's earlier draft of these notes
(drivetrain stopped responding to `twist()` mid-session: correct acks/
`active=true`, but `vel`/`enc` pinned at `(0,0)` even at `v_x=500`, surviving
two independent CTRL-AP mass-erase + clean reflash cycles) was **real and
correctly diagnosed as external to this ticket's code** — confirmed by the
team-lead: the signature matched a known motor-power fault, unrelated to
firmware. Power was restored; a live twist afterward drove the wheels
normally (`enc` climbing to `(1099, 1066)`, acks clean). The characterization
below was completed once hardware was confirmed healthy
(`tests/bench/rig_dev.py` 8/8, real encoder motion both directions).

### Resonance-taming step-response sweep — COMPLETE

Method: `tests/bench/velocity_step_response.py` (new this ticket), live
`config(pid.*)` gain application between trials — **zero reflashes** during
the sweep itself, exercising this ticket's own live-apply deliverable as the
tuning mechanism. Two script robustness fixes were needed along the way and
are part of this ticket's own deliverable, since the direct-USB CDC link's
characterized flakiness (`ack-ring-intermittent-delivery-gap.md`) affects
any bench tool driving it: (1) retry the `config()`/`twist()` SEND itself
(not just the ack wait) when neither an ack nor real motion is observed —
an occasional dropped outbound command, not just a dropped reply; (2)
confirm a genuine standstill (poll for `|vel| <= 5` for up to 1s) before
starting the next step, so a not-yet-decayed tail from the previous step
can't fool the next step's own retry-break check into accepting a dropped
command as landed.

**Trial log** (all live, same boot, `vel_ki=0.005`/`vel_kaw=20.0` held
constant throughout — the fix lived entirely in `kp`/`kff`):

| # | kp | kff | 70 mm/s ovL/ovR | 140 mm/s ovL/ovR | 250 mm/s ovL/ovR | worst | verdict |
|---|------|--------|-----------------|-------------------|-------------------|-------|---------|
| 1 (baseline, interim-shipped) | 0.0014 | 0.00135 | 8.6/0.0% | 25.0/15.7% | 5.6/4.4% | 25.0% | FAIL — reproduces the historical resonance pattern |
| 2 (raise kff) | 0.0014 | 0.0016 | 17.1/7.1% | 38.6/25.7% | 13.6/20.4% | 38.6% | FAIL, WORSE — kff is an open-loop `kff*target` kick added on top of `kp*error`; over-large kff over-drives the plant right at the step |
| 3 (lower kff) | 0.0014 | 0.0008 | 0.0/0.0% | 2.1/0.0% | 0.0/0.0% | 2.1% | PASS overshoot, but rise time slowed ~30-50% (0.9-1.5s vs baseline's 0.9-1.28s) — fails "rise time preserved" |
| 4 (restore rise via kp) | 0.0016 | 0.0010 | 0.0/0.0% | 18.6/10.0%\* | 0.0/0.0%\* | 18.6% | FAIL — kaw/settle artifact on 250 mm/s corrupted by leftover motion from the prior step (fixed by robustness fix #2 below) |
| 5 (candidate) | 0.0016 | 0.0008 | 0.0/0.0% | 9.3/2.1% | 0.0/0.0%† | 9.3% | PASS (250 mm/s re-verified alone: 0.0/0.0%, rise 0.76/0.98s) |
| 6 (confirmation run 2, same gains) | 0.0016 | 0.0008 | 4.3/0.0% | 3.6/2.1% | 0.0/0.0% | 4.3% | PASS |
| 7 (confirmation run 3, same gains) | 0.0016 | 0.0008 | 1.4/0.0% | 5.0/2.1% | 0.0/0.0% | 5.0% | PASS |
| 8 (clean-boot re-verify, no live `config()`) | 0.0016 (boot default) | 0.0008 (boot default) | 2.9/0.0% | 6.4/0.7% | 0.0/0.0% | 6.4% | PASS |

\* Trial 4's 140/250 mm/s rows were corrupted by a standstill-detection gap
(robustness fix #2, applied before trial 5) — not trusted, shown for the
trial log's own completeness only. † Trial 5's own 250 mm/s row was
similarly affected; re-run alone immediately after showed 0.0/0.0%,
rise 0.76 s/0.98 s.

**Winning gains** (`data/robots/tovez.json`, boot defaults via
`source/config/boot_config.cpp`, regenerated by `scripts/gen_boot_config.py`
as part of `just build-clean`): `vel_kp: 0.0014 -> 0.0016`,
`vel_kff: 0.00135 -> 0.0008`. `vel_ki`/`vel_imax`/`vel_kaw` unchanged
(0.005/0.3/20.0).

**Before/after summary** (worst-case overshoot per speed, across the trial
log above):

| speed | before (interim, kp=0.0014/kff=0.00135) | after (kp=0.0016/kff=0.0008, clean-boot trial 8) |
|-------|------------------------------------------|---------------------------------------------------|
| 70 mm/s | 8.6% | 2.9% |
| 140 mm/s | 25.0% (historical: +11% in the 2026-07-12 session, +24-33% at the original kp=0.0018) | 6.4% |
| 250 mm/s | 5.6% | 0.0% |
| rise time | 0.9-1.5 s | 0.38-0.77 s (clean boot) — FASTER, not just preserved |

**Root cause**: the resonance was not primarily a `kp`/`ki`/`kaw` problem —
`kff` (feedforward) was over-estimated. `kff*target` is an immediate,
error-independent duty kick added on top of `kp*error` at the very start of
a step; an over-large `kff` over-drives the plant right when the step
begins, which is exactly the ~140 mm/s peak the original issue described.
This confirms the 2026-07-11 `kff` derivation's own flagged uncertainty
("620-740 mm/s across windows, variance cause not established") was real
drift, not noise — the true value sits closer to the low end of that range.
Lowering `kff` alone cost rise time; raising `kp` to compensate restored it
— both overshoot AND rise time ended up BETTER than the interim-shipped
gains, not a trade-off.

**Decision 4 outcome**: constants-only tuning (`kp`/`kff`) hit the `<~10%`
bar — `velFiltAlpha` promotion and the notch-filter fallback were not
needed.

**Not re-run this session**: endpoint-accuracy verification (a
`turn_sweep.py`-style angle/speed grid) — the 2026-07-12 interim detune's
own precedent traded a little endpoint accuracy for trajectory smoothness;
this session's gains differ enough (`kp` raised back toward the original
0.0018, `kff` lowered) that endpoint accuracy should be re-checked as a
fast-follow, not assumed.

### Files created

- `tests/bench/velocity_step_response.py` — P4 binary-wire drive-arm
  step-response characterization tool (twist-from-stop, live `config()`
  gain application, peak/overshoot/rise-time/settled-mean summary, CSV
  trace log, standstill-confirmation + send-retry robustness). Used for
  this ticket's own sweep; reusable for any future resonance/gain work.

### Ticket status

`done`. All Acceptance Criteria met (checked above), full suite green
(569 passed), fully bench-verified including a clean-reflash re-check of
the persisted boot-default gains. Moved to `tickets/done/`.
