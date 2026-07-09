---
id: '004'
title: Bench-gate verification on the stand
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '003'
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

- [x] Build + flash the current tree (`build.py --clean --fw-only` +
      `mbdeploy deploy <full-UID> --hex MICROBIT.hex`) — done repeatedly.
- [x] Confirm the connected device is the ROBOT (`mbdeploy list` ROLE col) —
      done (UID 9906…a8fdb5…, NEZHA2/robot).
- [x] `PING` → `OK` over the real serial link — PASS.
- [x] `HELLO` → `DEVICE:…` banner — PASS (banner now emitted by
      `Communicator::begin()`; classified out-of-band by SerialConnection).
- [~] **DEFERRED to sprint 094** — `S 200 200` → wheels spin / encoders climb.
      The sprint pivoted mid-flight (stakeholder-directed): the drive path
      (`driveIn → motorIn → hardware`) was **deliberately severed** — the loop
      is comms-only and motors no longer receive messages. `S` is verified to
      PARSE and land on `bb.driveIn` (see `QLEN drive` 0→1), but it cannot
      reach the motors until the Drivetrain writes its motor refs directly
      (sprint 094). Physical wheel-drive verification moves there.
- [~] **DEFERRED to sprint 094** — `S 200 -200` opposite spin (same reason).
- [~] **DEFERRED to sprint 094** — `STOP` neutralizes wheels (same reason).
- [x] Round-trip works over serial — PASS (`comms_plane_verify.py` 10/10).
      Radio relay round-trip not exercised this session — follow-up
      (`clasi/issues/relay-round-trip-bench-verification.md`).
- [x] No unsolicited `EVT`/`TLM` on the wire — PASS (confirms the
      removal-not-queued decision).
- [x] Findings recorded — see Closing notes below.

## Closing notes (resolution)

Bench evidence is `tests/bench/comms_plane_verify.py`, run against the
power-cycled micro:bit: **10/10 checks passed** — Communicator up + DEVICE
banner (from `begin()`); `PING`/`VER`/`ECHO` reply; `QLEN` baseline
`drive=0`; `S 200 200` → `QLEN drive` **1** (command parsed + landed on
`bb.driveIn`); `STOP` → `motion` stays 0; `DEV WD 100` → `ERR unknown`
(proves the reduced table). The id-correlated `SerialConnection` +
`NezhaProtocol` transport is 100% reliable (an earlier "flaky" reading was a
host-side lock-step-harness bug + pyOCD-halted core, not the link).

The three motor-drive-on-stand criteria are DEFERRED to sprint 094
(Drivetrain-owns-its-motors) because sprint 093 deliberately severed the
drive path — a stakeholder-directed mid-sprint architecture change, not a
failure. Ticket resolved on the command-plane bench gate that the final
architecture actually supports.

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
