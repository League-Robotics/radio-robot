---
id: '010'
title: "Bench gate — the robot drives on the new firmware"
status: open
use-cases: [SUC-010]
depends-on: ['008', '009']
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench gate — the robot drives on the new firmware

## Description

The sprint's Definition of Done, per the 2026-07-14 stakeholder hard
scoping rule ("every sprint ends bench-runnable") and
`.claude/rules/hardware-bench-testing.md`. Deploy ticket 008's firmware to
the bench rig and, using ticket 009's minimal host slice, prove: telemetry
from power-on, twist-driven wheels under PID in both directions with
encoders tracking, an ack observed in the telemetry ring over BOTH direct
USB and the radio relay, a deadman kill-test, and the `runAndWait`/
`sleepUntil` schedule grep. No sprint in this arc closes on tests alone —
this ticket is real hardware, on the stand, wheels off the ground.

Depends on tickets 008 (firmware) and 009 (host slice) — strictly last in
this sprint.

## Acceptance Criteria

- [ ] `mbdeploy probe` confirms exactly one micro:bit connected (per
      `.claude/rules/debugging.md`'s precondition discipline); `mbdeploy
      deploy --build` flashes ticket 008's firmware.
- [ ] Boot banner + telemetry frames observed from power-on, BEFORE any
      command is sent (confirms the boot loop's telemetry-from-power-on
      property).
- [ ] A `twist` sent via ticket 009's script drives both wheels under
      velocity PID; encoders increment in the commanded direction, roughly
      proportional to commanded speed, confirmed in BOTH directions
      (forward/backward at minimum; a left/right turn twist as well if
      time permits within this ticket's own session).
- [ ] The twist's `corr_id` is observed in the telemetry ack ring — once
      over direct USB serial, and again (reconnect, repeat) over the radio
      relay's `!GO` data plane. Both transports confirmed independently,
      not just one.
- [ ] Deadman kill-test: arm a twist, then stop sending from the host
      (kill the script/process); confirm the wheels stop within one stale
      window (`Deadman`'s configured timeout, ticket 004) with no further
      host input.
- [ ] `grep 'runAndWait\|sleepUntil' source/main.cpp` output is captured
      and confirmed to match the archived plan's schedule one-for-one (the
      same check as ticket 008's own acceptance criterion, re-verified
      here as part of the final gate).
- [ ] No motor is left energized at the end of the verification session
      (explicit `stop()` sent and confirmed via telemetry before
      disconnecting).
- [ ] Session is conducted per `.claude/rules/hardware-bench-testing.md`
      (robot on the stand, wheels off the ground) — confirmed explicitly
      in completion notes, not assumed.

## Implementation Plan

**Approach**: This ticket is verification, not new code — if it finds a
defect in tickets 001-009's work, the fix belongs in whichever ticket
owns the broken module (this ticket does not silently patch around a
found defect; it reports and, if the defect is small and clearly scoped
to one already-closed ticket, may reopen that ticket per the project's
normal ticket-reopen mechanism — a call for whoever executes this ticket
to make in the moment, not pre-decided here).

**Files to create/modify**: none expected (verification only); if a defect
requires a real code fix, it lands in the ticket that owns the broken
module, not here.

**Testing plan**:
- Existing tests to run: none (this IS the test — real hardware, not
  pytest).
- New tests to write: none (ticket 009's bench script is the tooling this
  ticket exercises; no NEW test artifacts expected from this ticket
  itself).
- Verification command: manual bench session per the Acceptance Criteria
  above, using `mbdeploy` + ticket 009's `tests/bench/` script + a second
  connection through the radio relay for the transport-parity check.

**Documentation updates**: record the session's results (encoder
directions/magnitudes observed, ack-ring confirmation on both transports,
deadman kill-test timing, the `runAndWait`/`sleepUntil` grep output) in
this ticket's own completion notes — this IS the sprint's evidence of
"bench-runnable," and should be detailed enough that a future reader does
not have to re-run the session to trust the sprint closed correctly.
