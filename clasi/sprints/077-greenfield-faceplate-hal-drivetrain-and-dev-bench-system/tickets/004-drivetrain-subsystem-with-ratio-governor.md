---
id: '004'
title: Drivetrain subsystem with ratio governor
status: open
use-cases:
- SUC-004
depends-on:
- '003'
github-issue: ''
issue: greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Drivetrain subsystem with ratio governor

## Description

Write `source/subsystems/drivetrain.{h,cpp}` — the minimal two-wheel
Drivetrain that turns a body twist or per-wheel targets into ratio-governed
velocity commands for two `Hal::Motor`s, returned (never pushed) as a
`DrivetrainToMotorCommand`. No PID lives here — that stays entirely inside
`NezhaMotor` (ticket 3). This ticket depends on ticket 3 because it takes
`msg::MotorState` observations and produces `msg::MotorCommand`s against the
`Hal::Motor` faceplate's message types.

## Acceptance Criteria

- [ ] `namespace Subsystems`, class `Drivetrain`, matching the issue's locked
      shape: primitive setters `setTwist(float v_x, float v_y, float
      omega)` (`v_y` honored only when `capabilities().holonomic` — always
      false this sprint, differential-only; see the note below),
      `setWheelTargets(float left, float right)`, `setNeutral(msg::Neutral
      mode)`; faceplate verbs `configure(const msg::DrivetrainConfig&)`,
      `apply(const msg::DrivetrainCommand&)` (unpacks the oneof onto the
      setters above), `tick(uint32_t now, const msg::MotorState& leftObs,
      const msg::MotorState& rightObs)` returning a
      `DrivetrainToMotorCommand`, `state() const` returning
      `msg::DrivetrainState`, `capabilities() const`.
- [ ] `DrivetrainToMotorCommand` (edge type, `<Producer>To<Consumer>Command`
      naming per `.claude/rules/naming-and-style.md`) has exactly `left` and
      `right` fields of type `msg::MotorCommand`.
- [ ] `tick()` takes observations as **arguments only** — no clock reads
      beyond the `now` parameter, no motor handles or references stored
      inside `Drivetrain`. This keeps `Drivetrain` free of any dependency on
      `Hal::Motor`'s concrete leaf (`NezhaMotor`) — it only knows the
      faceplate's message types.
- [ ] `tick()` computes: kinematics (twist → wheel velocity targets, via
      `BodyKinematics::inverse(v_x, omega, trackwidth, vL_out, vR_out)` — use
      the **scalar** differential overload, not the `BodyTwist3`
      array-form overload, since the latter's parameter type
      (`kinematics/Pose2D.h`'s `BodyTwist3` with `vx_mmps`/`vy_mmps` field
      names) is a different, old-style-named type from `msg::BodyTwist3`
      (`v_x`/`v_y`/`omega`, no unit suffix) — do not introduce a naming
      collision or an unnecessary conversion between the two BodyTwist3
      types when the scalar overload sidesteps the issue entirely) → ratio
      governor → two `msg::MotorCommand{velocity}` (or duty/neutral
      pass-throughs when the WHEELS/NEUTRAL arms were the last-applied
      command, not the twist arm).
- [ ] Ratio governor (`governRatio`-equivalent private method): if one
      wheel's observed velocity underachieves its target (bogged down), the
      shared speed ceiling for BOTH wheels is lowered so the commanded
      left/right ratio (curvature) is held, rather than letting the healthy
      wheel run away. Operates on velocity **targets** passed to the motors'
      embedded PIDs — never on duty cycle. This is the ported concept from
      `source_old/control/VelocityController.*`'s `syncGain`, re-targeted at
      velocity targets instead of duties; `DrivetrainConfig.sync_gain` is
      its tuning knob (kept, per ticket 2's proto pass).
- [ ] `v_y` is explicitly ignored for this sprint's differential-only
      Drivetrain, with an inline comment at the ignore site stating that a
      future mecanum ticket wires it in once `capabilities().holonomic` can
      be true (per architecture-update.md Open Question 6) — do not
      silently drop it with no trace.
- [ ] `capabilities()` reports `holonomic = false`, `wheel_count = 2`,
      `onboard_position` matching whether the bound motors' capabilities
      both report `position = true`.
- [ ] Naming/style: `namespace Subsystems`, class `Drivetrain`
      (UpperCamelCase), methods lowerCamelCase, no unit suffixes in any
      identifier — e.g. `setTwist(float v_x, float v_y, float omega); //
      [mm/s] [mm/s] [rad/s]`.
- [ ] `python build.py --clean` succeeds with `Drivetrain` compiled in (it
      does not need to be wired into `main.cpp` yet — that's ticket 5 — but
      it must compile standalone against `Hal::Motor`'s faceplate types and
      the regenerated `msg::` headers).

## Testing

- **Existing tests to run**: None in `tests/` yet at this ticket's position
  in the dependency order (ticket 6 creates the new tree after this ticket).
- **New tests to write**: None required at this ticket for the same reason
  as ticket 3 — defer to ticket 6/7 once `tests/unit/` exists. If a
  lightweight host-buildable unit test for `BodyKinematics`'s scalar inverse/
  saturate functions or the ratio-governor math is cheap to add once
  `tests/unit/` exists, note it as a candidate for ticket 6, but do not block
  this ticket on it.
- **Verification command**: `python build.py --clean`. Bench validation of
  the ratio governor's actual behavior under load (the coupled rig) is
  ticket 7's job.
