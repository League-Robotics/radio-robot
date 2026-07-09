---
id: '004'
title: Bench-gate verification on the stand
status: open
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004]
depends-on: ['003']
github-issue: ''
issue:
- simplify-the-main-loop-strip-it-to-bare-wheel-driving.md
- get-wire-output-events-telemetry-out-of-the-main-loop.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench-gate verification on the stand

## Description

Per `.claude/rules/hardware-bench-testing.md`, any sprint touching the HAL,
motor control, or command protocol must be seen working on the real robot,
not just in sim — this sprint touches all three. The robot is mounted on a
stand with wheels off the ground for this entire verification (this is also
the condition that makes SUC-004's safety-watchdog removal acceptable at
all — do not perform this verification with the robot able to drive off any
surface).

Host tooling fallout is ACCEPTED for this sprint (team-lead disposition):
`robot_radio`/TestGUI send verbs this firmware no longer understands, so
this ticket verifies over a **bare serial script** (e.g. direct `pyserial`
or the project's own minimal serial-connection helper), not through
TestGUI or the full `robot_radio` client stack. Writing that bare script
(or reusing an existing minimal one under `tests/bench/`) is part of this
ticket's scope if one doesn't already fit.

## Acceptance Criteria

- [ ] Build + flash the current tree: `just build-clean` then
      `mbdeploy deploy <full-UID> --hex MICROBIT.hex` (per
      `hardware-bench-testing.md`'s known-good deploy path;
      `mbdeploy deploy --build` is broken per prior bench sessions — do not
      rely on it).
- [ ] Confirm exactly one micro:bit V2 is connected (`mbdeploy probe`
      and/or `pyocd list`) and that it is the ROBOT, not the radio relay
      dongle (`mbdeploy list`'s ROLE column) before flashing.
- [ ] `PING` → `OK` over the real serial link.
- [ ] `HELLO` → `DEVICE:...` banner, matching the boot-time banner text.
- [ ] `S 200 200` → both wheels spin forward; encoders climb together
      (read back via whatever encoder-observing path is still live — note
      `GET`/`ENC`/`TLM` are unregistered this sprint, so encoder
      confirmation may need to fall back to a `DEV M ... CFG`-free
      observation method or, if none remains reachable, an eyeball/manual
      wheel-rotation-count confirmation — document which method was used
      and why in the ticket's closing notes, since this is a real
      diagnostic-surface gap the gut created).
- [ ] `S 200 -200` → wheels spin in opposite directions; magnitude tracks
      the commanded value comparably to the `200 200` case.
- [ ] `STOP` → both wheels neutralize; no residual spin.
- [ ] Round-trip works over serial (required); round-trip over the radio
      relay is a stretch goal for this ticket (the once-per-slack yield
      from the prior sprint's fix must still keep radio alive) — if radio
      is not verified, note it explicitly as a follow-up rather than
      silently skipping it.
- [ ] No `EVT`/`TLM` lines appear unsolicited on the wire during this
      session (confirms Decision 1's "removal, not queued" — there is
      genuinely no loop-originated wire output left).
- [ ] Findings — pass/fail per step above, plus the encoder-observation
      method actually used — are written into this ticket's closing notes
      (not just reported verbally), since this is the sprint's sole
      hardware evidence.

## Implementation Plan

**Approach**: Follow `.claude/rules/hardware-bench-testing.md`'s standing
verification gate and `.claude/rules/debugging.md`'s preconditions
(`pyocd list` first) exactly. This ticket is verification, not new
production code — any script written to drive it is test/bench tooling
under `tests/bench/`, not `source/`.

**Files to create/modify**:
- Possibly a new or extended minimal script under `tests/bench/` (bare
  serial send/receive for `PING`/`HELLO`/`S`/`STOP`), if no existing bench
  script already fits without TestGUI/`robot_radio` dependencies.
- No `source/` changes expected — this ticket verifies tickets 001-003's
  work; if it finds a defect, fix it in the ticket where the defect was
  introduced (reopen, don't silently patch here) unless the team-lead
  directs otherwise.

**Testing plan**:
- Existing: none (this IS the test).
- New: the bare-serial verification script above, plus its console
  transcript/log kept as the ticket's evidence.
- Verification command: manual bench session per the acceptance criteria
  above; no `pytest` gate applies to this ticket.

**Documentation updates**: none required (protocol-v2.md currency is
deferred per architecture-update.md Step 7 item 2; the un-mounting-risk
banner note is deferred per Step 7 item 1 — both are team-lead-confirmed
DEFERs, not this ticket's scope).
