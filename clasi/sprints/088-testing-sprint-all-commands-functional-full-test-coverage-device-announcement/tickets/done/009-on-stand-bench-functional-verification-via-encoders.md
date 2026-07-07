---
id: 009
title: On-stand bench functional verification via encoders
status: done
use-cases:
- SUC-009
depends-on:
- '001'
- '002'
- '003'
- '004'
- '005'
github-issue: ''
issue: rebuild-test-suite-and-verify-commands-functional.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# On-stand bench functional verification via encoders

## Bench Results (2026-07-07) — stand verification PASS; see `../bench-verification-log.md`

Flashed `v0.20260707.5` to the confirmed robot (ROLE=NEZHA2, `/dev/cu.usbmodem2121102`).
`tests/bench/motion_command_verify.py` → **10/10 automated checks pass**: device
announcement (connect banner-classify → `DEVICE:NEZHA2:robot:tovez:2314287040`),
PING/VER/ID/HELP (full verb list), SET/GET (applies in ~1 tick), and motion via
encoders — **D** (+140/+128), **T** (+205/+196), **S** (+112/+113) drive both wheels
same-sign; **RT** spin (−24/+30) opposite. Watchdog widened + STOP/restore in finally.

Stand-limited (documented in the log): `TURN`/`G` (closed-loop on fused pose — OTOS
static on the stand), `R`/OTOS-abs/`SI` — dispatch-only on the stand, need real
motion/playfield. Deferred follow-up: radio-relay round-trip (direct serial verified;
captured as `clasi/issues/relay-round-trip-bench-verification.md`).

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

- [x] Robot vs. relay identified via `mbdeploy list`'s ROLE column before
      any flash; firmware deployed to the confirmed robot device (ROLE=NEZHA2).
- [x] Motion verbs drive the wheels with encoders incrementing as expected —
      **D/T/S** (straight, both ports same-sign) and **RT** (spin, opposite)
      encoder-verified; **STOP** exercised throughout. (`R/TURN/G` are
      closed-loop-on-fused-pose or arc verbs that need real motion — dispatched
      `OK` but stand-limited; see stand-limits below.)
- [ ] Relay round-trip: **DEFERRED** — verified over direct USB serial only this
      pass. Captured as follow-up `clasi/issues/relay-round-trip-bench-verification.md`.
- [x] `SET`/`GET` exercised, effect confirmed (`SET tw=111` → `GET tw`=111 within
      ~1 tick); `DEV WD` config subcommand exercised (widen/restore).
- [x] DEV serial-silence watchdog widened (`DEV WD 5000`) and restored
      (`DEV WD 1000`), with `STOP`/`DEV STOP` in a `finally` block.
- [x] `VER`, `HELP`, and the `HELLO` `DEVICE:` banner confirmed over the real link.
- [x] Stand-limited commands (`TURN`/`G` fused-pose, `R` arc, OTOS abs position,
      `SI` pose-inject) explicitly noted as smoke/dispatch-only with why + what
      would validate them (the playfield / real motion) — in the bench log.
- [x] Written bench log committed to the sprint directory
      (`bench-verification-log.md`), distinguishing verified vs. stand-limited.
- [x] `uv run python -m pytest tests/sim` green (260 passed, 4 xfailed) at the
      last firmware ticket before the bench pass; re-confirmed at close.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full sim gate, as
  a pre-bench sanity check).
- **New tests to write**: none (HITL bench pass, not pytest content);
  optionally a `tests/bench/` CLI helper if needed for the exercise
  itself.
- **Verification command**: N/A (HITL) — the deliverable is the bench log,
  cross-checked against a green `uv run python -m pytest`.
