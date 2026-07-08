---
id: '002'
title: 'Configured poll-set: replace NezhaHardware''s portInUse_ with polled_ + a
  DEV M CFG polled= escape hatch'
status: done
use-cases:
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: replace-portinuse-with-configured-poll-set.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Configured poll-set: replace NezhaHardware's portInUse_ with polled_ + a DEV M CFG polled= escape hatch

## Description

`Subsystems::NezhaHardware::portInUse_` is presented as an ownership/
"in-use" flag but its only real job is I2C poll-schedule membership — which
ports the brick flip-flop sequencer bothers to sample each `tick()`. It is
mutated as a side effect of ordinary command flow across three write sites
(`tick()`'s `motorIn[]` drain, both `apply()` overloads), never released
("a single `DEV M 3` during a bench session permanently adds port 3 to the
round-robin... with no way back short of reboot"), and invisible to
`SimHardware`/sim tests (no schedule concept there at all).

**This ticket does NOT implement a pure boot-time-fixed poll-set.** A
literal reading of the issue ("established once at construction... never
mutated by command flow") would silently break the coupled PID/governor
bench rig's standalone-port workflow: `tests/bench/ratio_governor_curve.py`'s
primary protocol binds the Drivetrain to ports 2/3 and drives port 4
STANDALONE (`DEV M 4 DUTY ...`, never `DEV DT`); `tests/bench/
pid_hold_speed.py` drives ports 3/4 standalone with no Drivetrain binding
at all. `test_dev_command_outbox.py`'s `scenarioUnboundPortLeavesDrivetrain
Untouched` proves this is accepted TODAY. See `architecture-update.md`
Decision 1 for the full reasoning — the resolution is a config-plane
opt-in door (`DEV M <n> CFG polled=<bool>`) alongside the boot-baked
default, not a purely static mask.

## Acceptance Criteria

### Poll-set mechanism

- [x] `NezhaHardware::portInUse_` and its three side-effect write sites
      (in `tick()`'s `motorIn[]` drain and both `apply()` overloads) are
      gone.
- [x] `NezhaHardware` owns `polled_[kPortCount]`, established once at
      construction from `configs[i].polled` (a new `msg::MotorConfig`
      field), and mutated ONLY through a new `setPolled(uint32_t port,
      bool polled)` method.
- [x] `anyPortInUse()`/`nextPortInUse()` are renamed `anyPolled()`/
      `nextPolled()`, reading `polled_[]` (no other behavior change to the
      flip-flop sequencing itself).
- [x] The broadcast-exemption branch/comment in
      `apply(const Hal::CommandProcessorToHardwareCommand&)` ("broadcast
      never marks a port in-use") is deleted — `apply()` no longer writes
      to poll state in any branch, so there is nothing to exempt.
      `apply()`'s broadcast forwarding to every port's setter is
      otherwise unchanged.

### Boot defaults

- [x] `msg::MotorConfig` gains `polled` (bool, default `false`) —
      `protos/motor.proto` + regenerate via `scripts/gen_messages.py`.
- [x] `scripts/gen_boot_config.py` bakes `polled=true` for
      `LEFT_PORT`/`RIGHT_PORT` (mirroring the existing
      `travel_calib_for_ports()`/`fwd_sign_for_ports()` per-port
      specialization pattern already in that file), `false` for every
      other port. `source/config/boot_config.cpp` is regenerated, not
      hand-edited.
- [x] `tests/_infra/sim/sim_api.cpp`'s own `defaultMotorConfigSet()` bakes
      `polled=true` for ports 1/2 (matching `defaultSimDrivetrainConfig()`'s
      `left_port=1`/`right_port=2`), `false` for ports 3/4 — this is the
      config every pytest-collected sim test actually runs against, so
      getting this right is what keeps the baseline green.

### Config-plane escape hatch

- [x] `DEV M <n> CFG polled=<bool>` is a new accepted key on the existing
      `DEV M <n> CFG` verb (routed through the existing
      `applyMotorCfgKey()`/`Rt::ConfigDelta(kMotor)` mechanism — no new
      command verb). Applying it calls `NezhaHardware::setPolled()` via
      the Configurator's existing `kMotor`-target apply path.
- [x] `docs/protocol-v2.md` §16 documents the new key.

### Unpolled-port rejection (the deliberate behavior change)

- [x] `DEV M <n> DUTY|VEL|POS` addressed at a port with
      `bb.motorConfig[port-1].polled == false` is rejected `ERR nodev
      <mode>` (mirroring the existing `ERR nodev` convention used by
      `OI`/`OZ`/`OR`/`OV` with no odometer, and the existing `ERR
      unsupported <mode>` capability-rejection shape) — posts nothing to
      `bb.motorIn[]`, steals no Drivetrain authority.
- [x] `NEUTRAL`/`RESET`/`STATE`/`CAPS`/`CFG` on the same port are
      unaffected by poll state — never gated.
- [x] `test_dev_command_outbox.py`'s `scenarioUnboundPortLeavesDrivetrain
      Untouched` is updated to prove the `ERR nodev` rejection (its new,
      intentional meaning) instead of acceptance.
- [x] A new scenario in the same harness proves `DEV M <n> CFG
      polled=true` on that same port, followed by the identical `DUTY`
      command, now succeeds and posts to `bb.motorIn[]`.

### Bench script + test updates

- [x] `tests/bench/pid_hold_speed.py` and `tests/bench/
      ratio_governor_curve.py` gain one `DEV M <n> CFG polled=true` setup
      line per non-default port they drive standalone (beside their
      existing `DEV WD 3000` preamble line).
- [x] `tests/sim/unit/test_nezha_flipflop.py`'s harness
      (`nezha_flipflop_harness.cpp`) scenarios are updated: the local
      config-builder gains a per-scenario `polled` parameter;
      `scenarioIdleScheduleNoBusActions` constructs with `polled=false`
      for all ports; `scenarioInUseTrackingAndRotation` constructs with
      the ports under test pre-`polled=true` (no longer relying on a
      command to bring them in); `scenarioBroadcastNeverMarksInUse` is
      simplified to confirm broadcast forwards to every setter and leaves
      `polled_[]` unaffected (there is no more "marks in-use" to NOT
      happen); `scenarioDrivetrainToHardwareCommandForwarding` similarly
      drops its "marks in-use" assertion, keeping only the forwarding
      assertion.
- [x] `uv run python -m pytest tests/sim` green, including the new/updated
      scenarios above.

## Implementation Plan

### Approach

1. Proto + generated-message change first (`protos/motor.proto` adds
   `polled`; regenerate).
2. `NezhaHardware`: add `polled_[kPortCount]`, populate at construction
   from `configs[].polled`, add `setPolled()`, rename
   `anyPortInUse()`/`nextPortInUse()` → `anyPolled()`/`nextPolled()`,
   delete the three write sites and the broadcast-exemption branch.
3. Boot config: `gen_boot_config.py` + regenerate `boot_config.cpp`;
   `sim_api.cpp`'s `defaultMotorConfigSet()`.
4. `dev_commands.cpp`: add the `polled` CFG key (mirrors any existing
   bool-valued `MotorConfig` CFG key's parsing shape — check
   `applyMotorCfgKey()` for the nearest precedent, e.g. how a bool field
   is parsed/applied today, if one exists, otherwise follow the float-field
   pattern with `true`/`false`/`1`/`0` token parsing) and the `ERR nodev`
   pre-validation gate in `handleDevM()`, checked before the existing
   capability gate (order doesn't matter functionally since both must
   pass, but checking poll-membership first gives a clearer error when
   both would fail).
5. Wire the Configurator's `kMotor` apply path to call
   `NezhaHardware::setPolled()` when a `ConfigDelta`'s mask includes the
   `polled` field (mirrors how every other per-motor CFG key already
   reaches its target through the Configurator).
6. Update the two harness test files + two bench scripts + protocol-v2.md.

### Files to Create/Modify

- `protos/motor.proto` (new `polled` field on `MotorConfig`)
- `source/messages/motor.h` (regenerated)
- `source/subsystems/nezha_hardware.h` / `.cpp`
- `source/runtime/configurator.h` / `.cpp` (wherever the `kMotor`
  `ConfigDelta` apply path lives — confirm exact filename during
  implementation)
- `scripts/gen_boot_config.py`
- `source/config/boot_config.cpp` (regenerated)
- `tests/_infra/sim/sim_api.cpp`
- `source/commands/dev_commands.h` / `.cpp`
- `tests/sim/unit/nezha_flipflop_harness.cpp`
- `tests/sim/unit/dev_command_outbox_harness.cpp`
- `tests/bench/pid_hold_speed.py`
- `tests/bench/ratio_governor_curve.py`
- `docs/protocol-v2.md`

### Testing Plan

- `uv run python -m pytest tests/sim` must stay green (309 baseline +
  this ticket's new/updated scenarios).
- Manually trace (code read, not a bench run — no hardware change in this
  ticket) that `defaultMotorConfigs()` (real firmware) and
  `defaultMotorConfigSet()` (sim) bake the SAME two ports (1/2) as
  `polled=true`, since a mismatch here would make sim tests pass while the
  real firmware behaves differently.
- No bench/HITL verification required for this ticket (no hardware
  behavior change beyond what the sim/harness tests already cover; the
  bench-script updates are config-line additions, not new behavior to
  verify on the stand).

### Documentation Updates

- `docs/protocol-v2.md` §16: document `DEV M <n> CFG polled=<bool>` and
  the `ERR nodev` reply for an unpolled port's motion verb.

## Completion Notes

- **Grep-zero confirmation**: `grep -rn "portInUse_" source/` returns
  **zero** hits (the only two prior hits, both prose mentions inside
  `nezha_hardware.h`'s own doc comments explaining the deleted flag's
  history, were reworded to avoid the literal token).
- **Test summary**: `uv run python -m pytest tests/sim` → **309 passed, 2
  xfailed** (matches the stated 309/2 baseline exactly — the new/updated
  scenarios below all live INSIDE existing harness `.cpp` binaries each
  already counted as one pytest test, so the pytest-level count is
  unchanged; the underlying C++ scenario count went up, independently
  verified by compiling and running each harness binary directly).
- **New/updated test names**:
  - `tests/sim/unit/dev_command_outbox_harness.cpp`:
    `scenarioUnpolledPortRejectedNodev` (renamed from/replaces
    `scenarioUnboundPortLeavesDrivetrainUntouched` — now proves `DEV M 3
    DUTY 40` → `ERR nodev duty`, posts nothing) and the new
    `scenarioCfgPolledTrueUnlocksMotionVerbs` (proves `DEV M 3 CFG
    polled=true` posts the `kMotor`/`kPolled` `ConfigDelta`, and that the
    identical `DUTY` command then succeeds and posts to `bb.motorIn[2]`).
  - `tests/sim/unit/nezha_flipflop_harness.cpp`: `resetDefaultConfigs()`
    gained a `polledMask` parameter; every scenario updated per the
    ticket's own scenario-by-scenario spec (idle=all-unpolled,
    in-use-tracking/rotation pre-polled at construction,
    broadcast/DrivetrainToHardwareCommand scenarios re-proved without any
    "marks in-use" assertion — the latter now proves forwarding via a
    direct `motor(port).tick()` bypass, plus a follow-up assertion that the
    HAL's OWN flip-flop still performs zero bus actions for the
    apply()'d-but-unpolled wheels).
- **Deviation 1 (mechanically required, not in the ticket's file list)**:
  `tests/sim/unit/hardware_seam_harness.cpp` (backing
  `test_hardware_seam.py`, ticket 081-002's `Subsystems::Hardware*`
  abstract-seam proof) also drove its scenarios through the OLD
  command-derived in-use marking (`apply()`/`motorIn[]` bringing a port
  into schedule) and broke identically to the two ticket-named harnesses.
  Updated the same way (`resetDefaultConfigs(polledMask)`), verified
  standalone and via the full suite.
- **Deviation 2 (scope judgment call on the bench scripts)**: the ticket's
  own acceptance text says "per non-default port they drive **standalone**".
  For `ratio_governor_curve.py`'s primary protocol, the Drivetrain-bound
  pair (`--dt-left`/`--dt-right`, default `2 3`) is driven via `DEV DT
  WHEELS`, not standalone `DEV M`. However, since `DEV DT PORTS` does NOT
  auto-follow the poll-set (architecture-update.md Open Question 2) and
  port 3 is not boot-polled, leaving it out would silently strand that
  wheel unsampled/undispatched by the REAL flip-flop on hardware (SimHardware
  is unaffected — it ticks all four ports unconditionally, which is why
  this does not show up as a sim-test failure). Opted to also send `DEV M
  <n> CFG polled=true` for `--dt-left`/`--dt-right` (not just
  `--disturb-port`), for both scripts, so the coupled bench rig keeps
  working exactly as documented on real hardware, not just at the wire.
  Flagging this judgment call explicitly since it goes slightly beyond the
  acceptance text's literal wording, in the direction the architecture's
  own stated goal (Decision 1) requires.
- No deviations from the architecture's Decision 1/Decision 2 shape itself
  (config-plane escape hatch + `ERR nodev` rejection) or from any other
  acceptance criterion.
