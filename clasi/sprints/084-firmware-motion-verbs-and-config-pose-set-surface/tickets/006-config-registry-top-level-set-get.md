---
id: '006'
title: 'Config registry: top-level SET/GET'
status: open
use-cases: [SUC-005]
depends-on: ['001']
github-issue: ''
issue: firmware-config-and-pose-set-surface.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Config registry: top-level SET/GET

## Description

Register top-level `SET`/`GET` (already fully specified in
`docs/protocol-v2.md` §7 — grammar, atomicity, error codes are all
already documented and unchanged) mapping a **deliberately-scoped subset**
of the old 22-key table onto real fields already present in
`msg::DrivetrainConfig`/`msg::MotorConfig`/`msg::PlannerConfig`
(architecture-update.md Grounding fact 3), re-propagating validated
changes via `Drivetrain::configure()`/`PoseEstimator::configure()`/
`Planner::configure()`/the bound pair's `Motor::configure()`.

**Approved key table (architecture-update.md Decision 2, team-lead/
stakeholder-approved as-is):**

| Key(s) | New-tree target |
|---|---|
| `tw` | `DrivetrainConfig.trackwidth` |
| `ml` / `mr` | bound-pair motors' `MotorConfig.travel_calib` (via `hardware.motor(port)` — the SAME field `DEV M <n> CFG travel_calib=` already writes; `SET` is a convenience alias for the currently-bound pair via `drivetrain.ports()`, not new storage) |
| `pid.kp` / `pid.ki` / `pid.kff` / `pid.iMax` / `pid.kaw` | applied identically to both bound motors' `MotorConfig.vel_gains` (`Gains{kp,ki,kff,i_max,kaw}`) |
| `rotSlip` | `DrivetrainConfig.rotational_slip` |
| `ekfQxy` / `ekfQtheta` / `ekfROtosXy` / `ekfROtosTheta` | `DrivetrainConfig`'s matching `ekf_*` fields (closes 082 Decision 4's deferred item) |
| `minSpeed` | `PlannerConfig.min_speed` |
| `sTimeout` | the production streaming-drive watchdog window introduced in ticket 002 (plain field, no message — mirrors `DEV WD` reading `SerialSilenceWatchdog` directly) |

**Explicitly dropped, not registered** (so each correctly surfaces as
`ERR badkey`, identical wire behavior to any never-existed key — see
architecture-update.md Decision 2 for the one-line rationale per key):
`kff` (standalone; folded into `pid.kff`), `klf`/`klb`/`krf`/`krb`
(superseded by `travel_calib_wheel_[]`/`fwd_sign_wheel_[]`), `adjThr`/
`adjGain` (superseded by `sync_gain`, already reachable via
`DEV DT CFG`), `distScale`/`turnScale` (no fudge factor needed against
correct `BodyKinematics`), `tick` (loop cadence is now structural, sprint
079), `tlmPeriod` (already implemented as the `STREAM <ms>` verb itself,
082).

**Wire keys stay stable.** Every key this ticket registers keeps its
exact existing `docs/protocol-v2.md` §7 spelling and wire format (`%.3f`/
`%d` as documented); no key is renamed under the units-in-identifiers
rule (that rule governs C++/Python identifiers, not wire keys — see
`.claude/rules/coding-standards.md`'s "Wire/serialized identifiers are
excluded" section).

## Acceptance Criteria

- [ ] New `source/commands/config_commands.{h,cpp}` registers top-level
      `SET`/`GET`, matching `docs/protocol-v2.md` §7's existing grammar
      exactly: atomic all-or-nothing `SET` (`ERR badkey`/`ERR badval` on
      first failure, no partial application), `GET` with no args dumps
      all registered keys, `GET <key>...` returns only those, unknown
      keys each emit a separate `ERR badkey <key>`.
- [ ] Own config shadow state (a new struct, **not** `DevLoopState`'s
      `motorConfigShadow[]`/`drivetrainConfigShadow`) per
      architecture-update.md Decision 7 — `SET`/`GET` must exist
      independent of the `DEV` family's build-time scoping story.
- [ ] All keys in the approved table above implemented, reading/writing
      the real-tree targets listed; every dropped key returns
      `ERR badkey` (verified by an explicit test asserting this, not just
      absence of a handler).
- [ ] `SET tw=130` then `GET tw` round-trips to `130` and visibly changes
      arc/turn geometry (sim) — the sprint's own headline acceptance
      example.
- [ ] `SET` validates atomically: a `SET pid.kp=1.5 tw=0` (where `tw=0`
      is invalid) applies **neither** key and returns `ERR badval tw=0`.
- [ ] `ml`/`mr`/`pid.*` writes reach the CURRENTLY bound pair (read via
      `drivetrain.ports()` at `SET`-time, not a hardcoded port) —
      verified by a test that rebinds `DEV DT PORTS` then confirms `SET
      ml=...` affects the newly-bound motor, not the original one.

## Implementation Plan

**Approach:** Config-plane, not command-plane (architecture-update.md's
"Config-plane vs. command-plane" precedent, sprint 079) — `SET`/`GET`
handlers call `configure()` directly and synchronously; no outbox, no
`Planner`/`Drivetrain` staging involved.

**Files to create:**
- `source/commands/config_commands.h`, `source/commands/
  config_commands.cpp`

**Files to modify:**
- `source/main.cpp` (construct the new config-command state, seed its
  shadow from boot config, concatenate `configCommands()`'s table)
- `docs/protocol-v2.md` §7 (annotate the Named Key Table: mark this
  sprint's implemented subset as current, the dropped keys as
  superseded/removed with a one-line pointer to architecture-update.md
  Decision 2, so a future reader does not wonder whether they were simply
  forgotten)

**Testing plan:**
- Sim-level tests: round-trip for every implemented key; atomic-failure
  behavior; `ERR badkey` for every dropped key; `ml`/`mr`/`pid.*`
  following a `DEV DT PORTS` rebind; `ekf*`/`rotSlip` visibly changing
  `PoseEstimator`/`Drivetrain` behavior (not just accepted silently).
- Existing suites stay green.

**Documentation updates:** `docs/protocol-v2.md` §7's Named Key Table, per
Acceptance Criteria.
