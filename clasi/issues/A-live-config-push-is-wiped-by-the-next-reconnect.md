---
status: pending
priority: high
---

# A live config push is silently wiped by the next reconnect

## Description

Sprint 132 delivered live per-wheel calibration over the wire — and it works,
but **only within a single serial connection.** Any reconnect silently discards
it, with no error and no indication anything was lost.

Found on hardware during 132-019's bench acceptance. The first push attempt
appeared to do nothing: push, then re-measure via a *separate* script
invocation, and the robot behaved exactly as before.

## Cause

Two independent facts compose into a silent data loss:

1. **`SerialConnection.connect()` pulses DTR on every direct-USB open**, which
   resets the MCU. Every fresh script invocation therefore reboots the robot.
2. **`DRIVE` is not in `Configurator::persistIfEligible()`'s flash-persistence
   table**, so a live `DRIVE` push exists only in RAM.

Together: a live push is erased by the very next connection, and nothing
reports it. This is precisely the "config that acks OK and does nothing" class
of failure sprint 132 set out to eliminate — reintroduced one level up, at the
connection boundary rather than inside the firmware.

Confirmed by measuring in the SAME connection as the push, where the gap did
close (trapezoid 11.0 → 1.6 points, square 12.4 → 0.6 points) and
`get_config(DRIVE)` read back the exact pushed values.

## Proposed fix

Options, roughly in order of preference:

1. **Make the loss loud.** If a live-pushed group is not persistable, say so at
   push time (a distinct ack code, or a warning in the reply) so a caller knows
   the value dies at reset. Smallest change, and it matches the sprint's own
   "no silent no-ops" principle.
2. **Stop pulsing DTR on connect** for direct USB, or make it opt-in. A host
   opening a port should not reboot the robot as a side effect. Note the
   existing project knowledge that the board does *not* reset on port open in
   general — this DTR pulse is `SerialConnection`'s own doing.
3. **Add `DRIVE` to the persistence table.** Careful: the persisted blob has a
   hard 128 B ceiling (4 CODAL keys × 32 B) whose `static_assert` lives inside
   `#ifndef HOST_BUILD`, so an overflow is invisible to host tests and only
   breaks at ARM build time. Sprint 132 shrank the blob to 51 B / 2 chunks, so
   there is room — but measure, do not assume.

Also worth deciding: whether bench tooling should read back after every push as
a matter of course. 132-019 only caught this because it read back.

## Verification

- Push a `DRIVE` value, disconnect, reconnect, `get_config(DRIVE)` — the value
  must either still be there, or the push must have warned it would not be.
- The 132-019 workaround (measure in the same connection as the push) must
  remain valid.

## Related

- Sprint 132 ticket 019's completion notes carry the full 8-run bench record.
- [[the-configuration-object]] — the design this completes.
- `.claude/rules/configuration-discipline.md` — the rule permits ad-hoc dev
  pushes *because* read-back makes them safe. This gap undercuts that: a
  read-back in a later session shows the baked value, not what you pushed.
