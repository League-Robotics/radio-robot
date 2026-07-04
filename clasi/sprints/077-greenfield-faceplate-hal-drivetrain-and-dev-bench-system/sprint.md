---
id: '077'
title: Greenfield Faceplate HAL, Drivetrain, and DEV Bench System
status: ticketing
branch: sprint/077-greenfield-faceplate-hal-drivetrain-and-dev-bench-system
use-cases: []
issues:
- greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 077: Greenfield Faceplate HAL, Drivetrain, and DEV Bench System

## Goals

Stand up a working debug system on a **greenfield** `source/` tree: DEV
commands over the standard wire protocol that drive individual Nezha motors
and, together, a minimal differential Drivetrain — nothing else. Prove the
message-model/faceplate discipline the 056-061 refactor established one tier
up (`Drive`/`Sensors`/`Planner`) also holds at the hardware tier, by building
it from the protos up against real I2C hardware, rather than incrementally
shimming the old `IVelocityMotor`/`MotorController` interfaces.

## Problem

The 056-061 message-model refactor never reached the hardware tier:
`messages/motor.h`/`gripper.h`/`ports.h` have zero includers, `IVelocityMotor`
exposes ten-plus encoder-plumbing virtuals, and `Motor` exposes six raw Nezha
register verbs directly. Incrementally refactoring the live tree to close
this gap means shimming the old interfaces one virtual at a time while a full
planner keeps running against them — slow, and all scaffolding. The
stakeholder chose a greenfield rebuild instead (see the linked issue, whose
decisions are locked): park `source/`/`tests/` as `source_old/`/`tests_old/`
(still buildable via a `codal.json` flip), and build the new hardware tier
directly against message types, with old code as porting reference only —
not a patient to operate on.

## Solution

Seven tickets, strictly ordered by dependency:

1. Park `source/`→`source_old/`, `tests/`→`tests_old/`; scaffold a minimal,
   buildable new `source/`; copy in dependency-clean comms/command/kinematics
   infra; condition `build.py` for the new tree's shape.
2. Proto accuracy pass on `motor.proto` (port identity, per-motor PID/slew
   config, per-mode capability booleans, `reset_position`); field-check
   `drivetrain`/`gripper`/`ports`/`sensors` protos; regenerate
   `source/messages/`.
3. `source/hal/capability/*.h` faceplate headers for all proto components;
   `NezhaMotor`/`NezhaHal` implemented — the only concrete leaf this sprint —
   porting the register map and the wedge-latch-sensitive split-phase 0x46
   encoder request/collect sequencing byte-for-byte from
   `source_old/hal/real/Motor.cpp`.
4. `Subsystems::Drivetrain`: body-twist/wheel-target/neutral commands,
   kinematics via the copied `BodyKinematics`, and a ratio governor (ported
   concept from `source_old/control/VelocityController`'s `syncGain`) that
   holds the commanded wheel-speed ratio under load by adjusting velocity
   targets — never duty, never a PID of its own.
5. `DEV` command family + minimal dev loop in `main.cpp` + non-negotiable
   serial-silence watchdog + a new "Development commands" section in
   `docs/protocol-v2.md`.
6. New `tests/` tree: three independent domains (`sim/`, `bench/`,
   `playfield/`) replacing the old `simulation/`/`sim/`/`field/`/`testgui/`/
   `calibrate/` mix; `velocity_chart.py` reinvigorated against `DEV`; new
   `dev_exercise.py`/`pid_hold_speed.py`/`ratio_governor_curve.py`.
7. HITL validation on the stand and on a coupled two-motor bench rig (ports
   3+4, mechanically linked) — the sprint's exit gate.

Full detail, locked decisions, and exact command/message shapes:
`clasi/issues/greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md`.
Architecture rationale, diagrams, and open questions:
`architecture-update.md` in this sprint directory.

## Success Criteria

- `python build.py --clean` produces a hex from the new `source/` tree at
  every ticket boundary from ticket 1 onward; the `codal.json`
  `application: source_old` rollback path works at least once.
- `source_old/`/`tests_old/` are untouched beyond their initial rename
  commits.
- The split-phase 0x46 encoder request/collect sequencing in the ported
  `NezhaMotor` is byte-for-byte identical to `source_old/hal/real/Motor.cpp`.
- Every bullet in the linked issue's "Verification" section passes on the
  stand, including the coupled-rig `pid_hold_speed.py` and
  `ratio_governor_curve.py` PASS conditions.
- The serial-silence watchdog neutralizes all motors within its window when
  comms silence is observed, regardless of which command family was active.
- `uv run python -m pytest` collects only the new `tests/` tree — zero
  collection from `tests_old/`/`source_old/`.

## Scope

### In Scope

- Parking `source/`/`tests/` as `source_old/`/`tests_old/` (pure rename).
- A new, minimal `source/` tree: copied dependency-clean infra, regenerated
  `source/messages/`, `hal/capability/` faceplate headers (all proto
  components), `NezhaMotor`/`NezhaHal` (the only implemented leaf),
  `Subsystems::Drivetrain` with the ratio governor, the `DEV` command
  family, the dev loop, the serial-silence watchdog.
- A new, minimal `tests/` tree: `sim/`/`bench/`/`playfield/`/`unit/`/`tools/`,
  with `bench/` fully populated (dev_exercise, pid_hold_speed,
  ratio_governor_curve, reinvigorated velocity_chart) and `playfield/`
  carrying over two parked scripts verbatim.
- `docs/protocol-v2.md`'s new "Development commands" section.
- HITL validation on the stand and the coupled two-motor rig.
- Differential (Tovez) drivetrain only.

### Out of Scope

- Any edit to `source_old/`/`tests_old/` beyond the initial rename commits.
- Sensor/gripper/ports leaf implementations (headers only this sprint).
- A fresh `tests/sim/` simulation harness (skeleton only this sprint).
- Subsystem/planner tiers, production motion commands (S/T/D/G/VW), the
  planner-driven authority stack, telemetry.
- Mecanum drivetrains (`v_y` stays ignored; `capabilities().holonomic` stays
  false).
- Reactivating `tests/playfield/plot_square.py`/`world_goto_chart.py` (parked
  until motion/odometry return in a later sprint).
- Retiring `source_old/`/`tests_old/` (that is a later sprint's decision).
- Any new wedge-latch diagnostic (`DBG WEDGE`-equivalent) or the newer,
  bench-validated zero-dwell reversal fix (see architecture-update.md Open
  Question 1) — this sprint ports the existing slew-cap mitigation as-is.

## Test Strategy

Three independent, never-combined test domains, per the linked issue:

- **`tests/sim/`** — skeleton only this sprint (no fresh sim harness yet);
  `uv run python -m pytest` must still collect cleanly against it.
- **`tests/bench/`** — the sprint's primary verification surface: scripted
  HITL tools (`dev_exercise.py`, `pid_hold_speed.py`,
  `ratio_governor_curve.py`) and an interactive dashboard
  (`velocity_chart.py`), all driven over the `DEV` protocol via
  `NezhaProtocol.send()`, exercised over both direct serial and the relay's
  `!GO` data plane.
- **`tests/playfield/`** — inert this sprint (two scripts carried over,
  explicitly parked).

No production-motion regression testing applies (no production motion
command family exists in this firmware). The bench domain's coupled two-
motor rig (ports 3+4) is this sprint's only load-bearing verification of the
embedded PID and the ratio governor, since the differential-drive robot's
own two wheels aren't mechanically coupled the way the rig's are.

## Architecture Notes

See `architecture-update.md` in this sprint directory for the full 7-step
architecture (module responsibilities, component/dependency/message-schema
diagrams, design rationale, and open questions). Key constraints carried
into every ticket:

- The 3-message/4-verb subsystem contract (`apply`/`tick`/`state`/
  `configure`/`capabilities`), already established one tier up by the
  056-061 refactor, is continued at the hardware tier via the "faceplate"
  terminology — not a new pattern.
- `hal/capability` is the dependency-direction seam: `subsystems/drivetrain`
  and `commands/` depend on the faceplate interfaces, never on `NezhaMotor`
  directly; `NezhaMotor` is a swappable infrastructure leaf behind it.
- The split-phase encoder sequencing and the register map are a hard
  preservation constraint (wedge-latch history), not a redesign
  opportunity — see Open Question 1 for what is deliberately deferred.
- `NezhaHal` owns motors by **port**, never by a baked-in L/R role — that
  semantic lives one tier up, in `Drivetrain`'s `DEV DT PORTS` binding.

## GitHub Issues

None. This sprint is driven entirely by the linked CLASI issue
(`greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md`),
which is not mirrored to a GitHub issue.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Scaffold new source/, park old tree, copy dependency-clean infra, condition build tooling | — |
| 002 | Proto accuracy pass (motor/drivetrain/gripper/ports/sensors) and message regen | 001 |
| 003 | Capability faceplate headers (all protos) and NezhaMotor/NezhaHal implementation | 002 |
| 004 | Drivetrain subsystem with ratio governor | 003 |
| 005 | DEV command family, dev loop, serial-silence watchdog, protocol doc | 004 |
| 006 | New tests/ structure: park old tests, three-domain skeleton, velocity_chart reinvigorated | 005 |
| 007 | HITL validation: bench scripts and stand/coupled-rig verification | 006 |

Tickets execute serially in the order listed.
