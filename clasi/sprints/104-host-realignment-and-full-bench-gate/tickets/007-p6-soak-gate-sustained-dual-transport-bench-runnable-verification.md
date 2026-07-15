---
id: '007'
title: "P6 soak gate — sustained dual-transport bench-runnable verification"
status: open
use-cases:
- SUC-017
depends-on:
- '001'
- '002'
- '003'
- '004'
- '005'
- '006'
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# P6 soak gate — sustained dual-transport bench-runnable verification

## Description

This sprint's Definition of Done, per the 2026-07-14 stakeholder hard
scoping rule ("every sprint ends bench-runnable") and P6 of the
continuation issue. Deploy this sprint's firmware (ticket 004's fault-bit
additions) and, using ticket 006's rewritten `rig_soak.py`, run a
sustained (soak-duration, not smoke-duration — materially longer than
103-010's short bench-gate captures) verification pass over BOTH direct
USB and the radio relay. Strictly last in this sprint — depends on every
other ticket.

Like 103 ticket 010, this ticket is verification, not new code: if it
finds a defect in tickets 001-006's work, the fix belongs in whichever
ticket owns the broken module (reopen per the project's normal mechanism),
not silently patched here.

## Acceptance Criteria

- [ ] `mbdeploy probe` confirms exactly one micro:bit connected;
      `mbdeploy deploy --build` flashes this sprint's firmware (ticket
      004's `kFaultCommsMalformed` bit + `kFaultI2CSafetyNet` doc fix —
      note: doc-only changes don't affect the binary, so this really
      verifies ticket 004's bit-wiring compiled and flashed correctly).
- [ ] Sustained soak run (duration a ticket-time/stakeholder judgment
      call — architecture-update.md Step 7 Open Question 4 — informed by
      103-010's own 120s/1875-frame USB continuity capture as a floor, not
      a ceiling) over direct USB: zero I2C NAK/timeout errors
      (`kFaultI2CNak` stays clear throughout), TLM drop rate measured and
      reported (not assumed), wedge latch clears promptly once motion
      resumes after any transient idle-state assertion (103-010 §6's
      documented contract — the acceptance bar is "clears promptly," not
      "never asserts").
  - [ ] Same soak run repeated over the radio relay's `!GO` data plane
        (reconnect, repeat) — both transports measured independently.
- [ ] Deadman kill-test repeated under soak conditions (mid-soak host
      process kill, not just at idle) on both transports; wheels stop
      within one stale window with no further host input.
- [ ] `kFaultI2CSafetyNet` observed to NOT re-trip after its initial
      boot-time latch during either soak window — corroborates ticket
      004's characterization under sustained load, not just 103-010's
      short session.
- [ ] `kFaultCommsMalformed` (new this sprint) stays clear throughout both
      soak windows — the host's own well-formed traffic should never trip
      it; if it DOES trip, that is a real finding (either a host-side
      encoding bug this ticket surfaces, or evidence the bit's wiring in
      ticket 004 is miscalibrated) to be investigated, not waved through.
- [ ] `uv run python -m pytest tests/unit -q` reports 0 failed, 0 errors
      immediately before this bench session (confirms ticket 002's sweep
      held through the rest of the sprint's changes).
- [ ] No motor left energized at the end of the verification session.
- [ ] Session conducted per `.claude/rules/hardware-bench-testing.md`
      (robot on the stand, wheels off the ground) — confirmed explicitly
      in completion notes.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/unit -q`
  (must be green — see Acceptance Criteria).
- **New tests to write**: none (this IS the test — real hardware, soak
  duration, not pytest); ticket 006's `rig_soak.py` is the tooling this
  ticket exercises.
- **Verification command**: a real bench soak session per Acceptance
  Criteria, using `mbdeploy` + ticket 006's `rig_soak.py` + a second
  connection through the radio relay.

## Implementation Plan

**Approach**: Verification only, mirroring 103 ticket 010's own
Implementation Plan posture. If a defect is found, report it and, if
small and clearly scoped to one already-closed ticket in this sprint,
reopen that ticket — a call for whoever executes this ticket to make in
the moment.

**Files to create/modify**: none expected; if a defect requires a real
code fix, it lands in the ticket that owns the broken module.

**Testing plan**: covered above.

**Documentation updates**: record the full soak session's results (drop
rates both transports, fault/event bit timeline, deadman kill-test
timing, encoder motion sanity, the confirmed soak duration used) in this
ticket's own completion notes, matching 103-010's level of detail — this
IS the sprint's evidence of "bench-runnable," and future readers should
not have to re-run the session to trust the sprint closed correctly.

## SUC-017: P6 soak gate — sustained, dual-transport, bench-runnable

Parent: `single-loop-firmware-p3-p7-continuation.md` (P6); the hard
scoping rule's "every sprint ends bench-runnable" requirement.

- **Actor**: Bench operator; the physical rig.
- **Preconditions**: Tickets 001-006 complete; firmware flashed.
- **Main Flow**: Run the soak; observe fault/event bits; repeat the
  deadman kill-test under load; measure drop rate both transports.
- **Postconditions**: Full host tooling drives the robot over the binary
  plane on both transports, sustained and clean. This IS this sprint's
  Definition of Done.
- **Acceptance Criteria**: see above.
