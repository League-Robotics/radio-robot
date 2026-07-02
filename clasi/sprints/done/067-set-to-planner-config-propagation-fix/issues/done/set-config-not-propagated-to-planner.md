---
status: done
severity: medium
sprint: '067'
tickets:
- 067-001
- 067-002
- 067-003
- 067-004
---

# SET silently fails to propagate unannotated config keys to the Planner — rotSlip (and others) frozen at boot-time defaults

## Problem

`SET rotSlip=1.0` replies `OK`, `GET rotSlip` reads back 1.0 — but turn
behavior is bit-identical to the default. Verified empirically in sim
(2026-07-02): RT 4500/9000 produce exactly the same true rotation with
rotSlip 0.92 (default) and 1.0.

Mechanism: the config registry only pushes a `configure()` delta into a
subsystem for entries carrying a subsystem annotation; plain `CFG_F` entries
have `subsystem = nullptr`
([source/robot/ConfigRegistry.cpp:24](../../source/robot/ConfigRegistry.cpp),
apply loop ~line 630). `rotSlip` is a plain `CFG_F`
(ConfigRegistry.cpp:100), and the Planner holds a **boot-time private copy**
of RobotConfig (`RobotConfig _cfg` in
[source/superstructure/Planner.h](../../source/superstructure/Planner.h),
"configure() updates it") — which is never re-invoked for these keys. So the
struct is updated, the reply says OK, and the consumer never sees the value.

Consequence: the per-robot calibration `rotational_slip: 0.92` in
[data/robots/tovez.json](../../data/robots/tovez.json) only "works" because
it coincidentally equals the compiled-in default
([source/robot/DefaultConfig.cpp:79](../../source/robot/DefaultConfig.cpp)).
Recalibrating it — which the camera evidence says we must, real playfield
scrub is ≈0, not 8% — would silently change nothing. This directly blocks
the sim-to-hardware fitting workflow
([sim-error-model-runtime-settable-hardware-fit.md](sim-error-model-runtime-settable-hardware-fit.md)):
fitted parameters must actually take effect when pushed to the robot.

## Scope

- Audit ALL plain `CFG_F`/`CFG_I`/`CFG_FI` registry entries: for each,
  determine whether any consumer caches a config copy (Planner is the known
  offender; check trackwidth `tw`, rotation gains/offsets, odom offsets,
  EKF noise keys, etc.).
- Fix: annotate the keys with their owning subsystem(s) so the existing
  post-commit `configure()` push fires, or have the Planner read the live
  RobotConfig by reference like Superstructure does
  (`const RobotConfig& _cfg`).
- Consider a guard against recurrence: a test that SETs every registered
  key and asserts the owning consumer observes the new value.

## Acceptance

- `SET rotSlip=<x>` measurably changes RT arc targets on the next turn
  (sim test: RT 9000 true rotation differs between rotSlip 0.92 and 1.0).
- Audit results recorded; every stale-copy key either annotated or its
  consumer converted to live-reference reads.
- Regression test covering SET→consumer propagation for motion-critical
  keys.
