---
status: in-progress
sprint: '114'
tickets:
- 114-001
- 114-002
- 114-003
- 114-004
---

# Config-as-truth completion: no behavioral defaults in source, fail-closed when unconfigured, version-erased persisted config

Sprint 113 delivered only the first slice — the sim now *configures-on-open* from
the robot config file. This issue finishes the ethic the stakeholder set on
2026-07-20:

> "There should be nothing that's configurable or tunable in the source code that
> doesn't come from the configuration... there are no defaults in the code. If you
> try to run it with those defaults, it throws an error... We have a persistent
> robot configuration. Every parameter that you can control in the drive system or
> in the sim must come from a configuration when you open it."

## Why now (concrete evidence the gap is live)

`src/sim/sim_harness.h` still hardcodes `velGains.kp = 0.003f` while
`data/robots/tovez_nocal.json` says `vel_kp = 0.002`. Since sprint 113 made the
sim read the file, a *configured* sim now runs 0.002 and an *unconfigured* one
runs 0.003 — sprint 113's own ticket-005 test asserts exactly this divergence.
That 0.003 was a sim-only period-2 damping patch. It is precisely the class of
hidden, code-side tunable this ethic exists to abolish, and it means the motion
traces tuned against 0.003 must be re-validated against the real config.

## Requirements

1. **No behavioral defaults in source.** Delete hardcoded behavioral values from
   at least: `src/sim/sim_harness.h` (`makeExecutorConfig()`, `makeMotorConfig()`
   — SimHarness must not hardcode anything that is loadable from configuration),
   `src/scripts/gen_boot_config.py`'s `*_DEFAULT` constants,
   `src/firm/devices/nezha_motor.h`'s `kDefaultOutputDeadband` /
   `kDefaultReversalDwell`, and any remaining in-class initializers for tunables
   in `App::Pilot` and friends.
   **Boundary:** only *behavioral* parameters (gains, deadband, speeds/accels,
   trackwidth, model taus, dwell tolerances) come from config. Structural
   invariants — array sizes, `kWheelCount`, protocol/message version, buffer
   lengths, math constants — stay compile-time; they are the code's identity, not
   tunables. Write the in/out list down.

2. **Fail closed when unconfigured.** An unconfigured device refuses *motion* and
   answers "not configured" rather than running on guesses. Keep a minimal
   always-live rump: accept the configuration command, accept STOP, answer
   PING/ID. Config-accept must validate **completeness** — reject a partial
   config and only flip to "ready" when every required parameter is present, so
   there is never a "configured but silently missing distance_kp" state.

3. **Version-erased persistence.** The device may persist its configuration, but
   it is stamped with the firmware version and **wiped on a version mismatch**, so
   a reflash forces reconfiguration and stale config whose field *meanings*
   changed between versions can never silently survive an update.

4. **Sim parity.** The sim obeys the same rules: opening the sim sends it the
   configuration from the robot config file (already true after 113) and an
   unconfigured sim fails closed the same way the robot does. No sim-only
   hardcoded fallback may remain as a silent second source of truth.

## Acceptance

- Grepping the source for a behavioral tunable's literal value finds it only in
  config files and in the (documented) structural-invariant list.
- Booting/opening unconfigured and issuing a motion command yields a clear
  "not configured" refusal, while config/STOP/PING still work.
- A version bump invalidates persisted config and forces reconfiguration.
- The `vel_kp` 0.003-vs-0.002 divergence above no longer exists in any form.
