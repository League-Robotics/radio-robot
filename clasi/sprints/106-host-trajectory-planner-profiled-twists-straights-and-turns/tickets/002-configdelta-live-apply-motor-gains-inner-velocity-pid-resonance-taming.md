---
id: '002'
title: ConfigDelta live-apply (motor gains) + inner velocity-PID resonance taming
status: in-progress
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
- [~] A `config()` call carrying `pid.kp`/`ki`/`kff`/`iMax`/`kaw` measurably
      changes the robot's live step response on the SAME boot, with no
      reflash — bench-verified. **Partially met**: `config()` with
      `pid.kp`/`pid.ki` was confirmed on real hardware to ack `OK` (was
      `ERR_UNIMPLEMENTED` pre-ticket) on the SAME boot, no reflash. The
      "measurably changes the live STEP RESPONSE" half could not be
      confirmed — see Completion Notes' hardware blocker.
- [ ] On-stand velocity-step harness (drive-arm step at 70/140/250 mm/s, per
      `heading-loop-output-clamp-and-velocity-resonance.md`'s own
      methodology) shows `<~10%` step overshoot across that range with rise
      time preserved, superseding the interim `vel_kp=0.0014` detuning
      currently shipped in `data/robots/tovez.json`. **Blocked** — see
      Completion Notes.
- [ ] If constants-only tuning cannot hit the bar, Completion Notes document
      the attempt and either (a) promote `velFiltAlpha` to a new
      live-tunable `MotorConfigPatch` field (the smallest of the three
      original candidates), or (b) explicitly flag a fast-follow ticket —
      an empirical decision, not pre-assumed by this ticket. **N/A this
      session** — the sweep never ran (hardware blocker), so this decision
      point was never reached; the fast-follow bench session (Completion
      Notes) inherits it.
- [x] Full project test suite green (`uv run python -m pytest`, 569 passed).
      Bench-verified PARTIALLY per `.claude/rules/hardware-bench-testing.md`
      — see Completion Notes for exactly what was and was not confirmed on
      the stand.

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

### Blocked: resonance-taming step-response sweep (NOT completed)

A **drivetrain hardware fault emerged mid-session** and could not be
resolved remotely — the step-response characterization (Acceptance
Criteria 4's "measurably changes the live STEP RESPONSE" half, and
Criterion 5's full 70/140/250 mm/s sweep) was **not completed**. Sequence:

1. Built + flashed this ticket's firmware (`just build-clean` +
   `mbdeploy deploy <UID> --hex MICROBIT.hex`).
2. `tests/bench/rig_dev.py` passed **8/8**, including real encoder motion
   during both `twist(v_x)` and `twist(omega)` (`enc` climbing
   `(0,0)->(51,43)->(63,79)`) — full proof this ticket's firmware drives the
   real robot correctly.
3. Began step-response capture (`tests/bench/velocity_step_response.py`,
   new this ticket — see below). Partway through, wheel motion stopped
   responding: every subsequent `twist()` still acked `OK` and correctly
   set `active=true` (the firmware genuinely dispatches and arms the
   command), but `vel_left`/`vel_right`/`enc` stayed pinned at exactly
   `(0, 0)` — confirmed even at `v_x=500` (near the plant ceiling) over a
   2.5 s capture window. OTOS (a separate I2C device on the same bus) kept
   reporting fresh, slowly-drifting readings throughout, so the I2C bus
   itself is not down; only the drivetrain's own duty write appears to have
   no physical effect while its own encoder READ path keeps reporting a
   stale value cleanly (not an I2C error).
4. Ruled out firmware/software as the cause: performed a full CTRL-AP mass
   erase (`pyocd erase --mass`) + clean reflash of the IDENTICAL hex TWICE
   — the exact same symptom persisted across both, with acks/config/
   secondary-telemetry all continuing to work normally. A mass erase +
   reflash resets 100% of the MCU's own RAM/behavioral state; a symptom
   that survives it unchanged cannot be a bug in this ticket's code (or
   any RAM-resident firmware state) — the fault lives outside the MCU's own
   reset domain, most likely the Nezha motor-controller brick's own
   internal state (a separate chip on the I2C bus, not reset by a micro:bit
   reflash) or a physical connector/mechanical issue. Per
   `.clasi/knowledge/never-attribute-to-power-battery.md` this is NOT
   attributed to a battery/power-sag story — it is reported as an unresolved
   hardware-state fault requiring physical inspection, not diagnosed as a
   power issue.
5. Left the robot in a safe, stopped state (`STOP` sent, disconnected) on
   this ticket's own firmware image (the 106-002 build), gains unchanged
   from the interim shipped values.

**Gains story**: `data/robots/tovez.json`'s interim gains
(`vel_kp=0.0014`, `vel_ki=0.005`, `vel_kff=0.00135`, `vel_imax=0.3`,
`vel_kaw=20.0`, shipped 2026-07-12) are **UNCHANGED** — no new
bench-verified tuning was possible this session. The historical
`+11%` overshoot at 140 mm/s (that same 2026-07-12 session's own finding)
stands as the best-known current characterization; this ticket could not
supersede it.

**Fast-follow required**: once the hardware fault is resolved (physical
inspection/power-cycle by someone at the bench), re-run
`tests/bench/velocity_step_response.py` (new this ticket — see "Files
created" below) across 70/140/250 mm/s, exhausting `kp`/`ki`/`kff`/`iMax`/
`kaw` per this ticket's Decision 4 before reaching for `velFiltAlpha`/notch,
and update `data/robots/tovez.json` + this ticket (or a follow-up) with the
result. The live-apply path itself (this ticket's own main deliverable) is
proven working, so that follow-up session should not need any further
firmware changes — only gain trials via `config()`.

### Files created

- `tests/bench/velocity_step_response.py` — P4 binary-wire drive-arm
  step-response characterization tool (twist-from-stop, live `config()`
  gain application, peak/overshoot/rise-time/settled-mean summary, CSV
  trace log). Ready for the fast-follow session; not yet exercised through
  a full 70/140/250 grid due to the hardware blocker above.

### Ticket status

Left `in-progress` (not `done`) — Acceptance Criteria 4 (partially met) and
5 (not met) are real, required deliverables this session could not
complete. Not moved to `tickets/done/`.
