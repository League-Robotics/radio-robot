---
id: '009'
title: On-stand bench functional verification via encoders
status: open
use-cases: [SUC-009]
depends-on: ['001', '002', '003', '004', '005']
github-issue: ''
issue: rebuild-test-suite-and-verify-commands-functional.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# On-stand bench functional verification via encoders

## Description

The stakeholder's hard bar: "all motion and configuration commands should
function, test by looking at the encoders" — proven on real hardware, not
just in sim. The robot is mounted on a stand with wheels off the ground
(safe to drive freely). This ticket is sequenced last because it depends
on the wheel-direction fix (002) landing — a straight-drive check
attempted before that fix would read as a failure for the wrong reason —
and on the VER/HELP/HELLO fixes (001/003/005) and the rename (004) so the
firmware under test is the sprint's final, correct build. Per
`.claude/rules/hardware-bench-testing.md`, this sprint (touching the
command protocol and config generation) requires exactly this gate.

## Implementation Plan

**Approach**:
1. **Identify the robot, not the relay.** Two micro:bits are connected.
   Run `mbdeploy list` and confirm the ROLE column before touching
   anything — never blind-flash (`.clasi/knowledge/verify-microbit-before-flashing.md`).
2. **Deploy.** Build and flash the sprint's final firmware to the
   confirmed robot device (check `.clasi/knowledge/` for the current
   known-good deploy recipe — `mbdeploy deploy --build` has a known
   venv gap on this checkout per prior session knowledge; the fallback is
   `just build-clean` then `mbdeploy deploy <full-UID> --hex MICROBIT.hex`).
3. **Safety first.** Before driving anything, widen the DEV
   serial-silence watchdog (`DEV WD 3000`) for the session. Wrap the
   entire exercise in a try/finally (or equivalent script structure) that
   always sends `DEV STOP` and restores `DEV WD 1000` on exit or
   exception — motors must never be left running.
4. **Motion verbs.** For each of `S T D R TURN RT G STOP`: issue over
   serial, read encoders before/after, confirm direction and rough
   proportionality to the commanded value. Exercise the straight-drive
   case (`S`/`T`/`D`) and at least one additional motion verb over the
   radio relay as well, confirming round-trip.
5. **Config verbs.** Exercise `SET`/`GET` and the `DEV` config
   subcommands; confirm each takes effect (readback match, or an
   observable behavior change).
6. **Liveness/identity spot-checks.** Confirm `VER`, `HELP`, and the
   `HELLO`/boot `DEVICE:` banner (tickets 001/003/005) all work over the
   real link as part of this same pass, rather than re-verifying them in
   isolation.
7. **Document stand limits.** For commands that cannot be fully validated
   on the stand (OTOS absolute position without real translation, camera
   `SI` pose-inject, playfield-frame `G`/goto), note smoke/dispatch-only
   status and what would fully validate them (bench-with-motion or
   playfield).
8. **Write the log.** Capture pass/fail per verb/command in a bench
   checklist/log file in the sprint directory.

**Files to create/modify**: a new bench checklist/log file in the sprint
directory (e.g. `bench-verification-log.md`); a `tests/bench/` CLI helper
only if an existing one doesn't cleanly cover a needed verb (per
`tests/CLAUDE.md`, these are HITL Python tools, not pytest-collected).

**Testing plan**: this ticket IS the test (HITL, not pytest-automated).
Run the full sim gate immediately before the bench pass as a sanity
check, but it does not substitute for the bench pass itself.

**Documentation updates**: the bench log itself. If the bench run
surfaces a new defect, raise a new issue rather than silently patching
mid-bench (follow the exception protocol if it blocks this ticket's own
acceptance).

## Acceptance Criteria

- [ ] Robot vs. relay identified via `mbdeploy list`'s ROLE column before
      any flash; firmware deployed to the confirmed robot device.
- [ ] Every motion verb (`S T D R TURN RT G STOP`) is issued over serial
      and drives the wheels with encoders incrementing in the expected
      direction, roughly proportional to the commanded value.
- [ ] The straight-drive case (`S`/`T`/`D`) and at least one additional
      motion verb are also exercised over the radio relay, round-tripping
      successfully.
- [ ] `SET`/`GET` and the `DEV` config subcommands are exercised and
      their effect is confirmed (readback match or observable behavior
      change).
- [ ] The DEV serial-silence watchdog is widened (`DEV WD 3000`) at
      session start and restored (`DEV WD 1000`), with `DEV STOP` sent in
      a `finally` block — motors are never left running on an exception
      or Ctrl-C.
- [ ] `VER`, `HELP`, and the boot/`HELLO` `DEVICE:` banner are confirmed
      working over the real link as part of this pass.
- [ ] Commands that cannot be fully validated on the stand (OTOS absolute
      position, camera `SI` pose-inject, playfield-frame `G`/goto) are
      explicitly noted as smoke/dispatch-only, with why and what would
      fully validate them.
- [ ] A written bench checklist/log is committed to the sprint directory
      distinguishing fully-verified from smoke-only commands.
- [ ] `uv run python -m pytest` (the sim gate) is confirmed green
      immediately before the bench pass.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full sim gate, as
  a pre-bench sanity check).
- **New tests to write**: none (HITL bench pass, not pytest content);
  optionally a `tests/bench/` CLI helper if needed for the exercise
  itself.
- **Verification command**: N/A (HITL) — the deliverable is the bench log,
  cross-checked against a green `uv run python -m pytest`.
