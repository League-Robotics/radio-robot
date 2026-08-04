---
status: pending
priority: high
---

# Closing the serial port resets the robot — live config is still wiped

## Description

Sprint 133 ticket 006 stopped `SerialConnection` asserting DTR on **open**,
which was measured and correct. It did not fix the reported defect, because
the reset does not happen at open.

**Measured on `tovez`, 133-004:** opening the port does **not** reset the MCU.
**Closing it does.** The robot clock tracks `gap + connect_cost` across a
close/reopen — 18960 vs 18996 ms at an 8 s gap, 25962 vs 26005 ms at a 15 s
gap — i.e. the boot clock restarts when the previous session's port closes.

Root cause: **`_disable_hupcl()` does not work on macOS.** HUPCL (hang up on
last close) is still in force, so the final close drops DTR and resets the
board.

Net effect: [[A-live-config-push-is-wiped-by-the-next-reconnect]] is **not
resolved**, only relocated. A live push still dies before the next session
sees it.

Ticket 006's provenance work is unaffected and still correct — a group that
survives reports `LIVE`, one that does not reports `BAKED`. The *reporting*
is honest; the *survival* is not.

## Why the earlier measurement looked conclusive

The 2026-08-04 morning measurement (`dtr=False`, clock held at 40416) was
real but did not discriminate. The reset from the *previous* close had
already happened; the clock simply read the uptime accumulated since.
Opening with DTR deasserted correctly did not add a *second* reset — which
is exactly what "opening does not reset" looks like, and also exactly what a
surviving close-reset looks like from inside one session.

**Two probes separated by a close cannot tell the two apart.** Only a
same-session close/reopen with a controlled gap can, which is what 133-004
finally ran, at two different gap lengths so a coincidence could not pass.

## Proposed fix

Make HUPCL actually off on macOS. `_disable_hupcl()` exists and is called —
establish *why* it has no effect (termios flags applied to the wrong fd,
applied before open, or overridden by pyserial's own configuration on this
platform) and fix it there, rather than adding a second mechanism beside a
function that already looks like it works.

Verify by measurement, not by inspection. The function already reads as
correct; that is precisely how it survived this long.

## Verification

- Same-session close/reopen with a controlled gap: the robot clock must
  advance by the gap, not restart. Run at two gap lengths, as 133-004 did.
- Push a `DRIVE` value, close the port, reopen, `get_config(DRIVE)` — the
  value is still present and its source reads `LIVE`.
- The relay path still handshakes (133-006 proved by measurement on `getez`
  that it needs no reset).

## Related

- [[A-live-config-push-is-wiped-by-the-next-reconnect]] — the original issue,
  closed by 133-006 on the strength of the open-side fix. This is the
  unresolved remainder.
- `.claude/rules/configuration-discipline.md` — the rule permits ad-hoc dev
  pushes *because* read-back makes them safe. That relaxation stays unsound
  across sessions until this is fixed.
- Sprint 133 ticket 004's completion notes carry the two-gap measurement.
