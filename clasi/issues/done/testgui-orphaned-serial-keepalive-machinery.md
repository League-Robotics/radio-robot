---
status: done
---

# Orphaned "+" keepalive machinery: `Transport.arm_keepalive()`/`disarm_keepalive()`, `SerialConnection.start_keepalive()`/`stop_keepalive()`

**Source:** 128-009 (doc-rot-and-minor-sweep), team-lead-accumulated sweep
item #2 from ticket 005's `KeyboardDriver` deletion. Verified, not deleted
this ticket — deletion turned out nontrivial (see below), so per that
item's own instruction this is recorded as a follow-up issue instead.

## What is confirmed

Zero live callers, in-tree, of any of:
- `src/host/robot_radio/testgui/transport.py`: `Transport.arm_keepalive()`/
  `Transport.disarm_keepalive()` (base no-op methods) and
  `_HardwareTransport.arm_keepalive()`/`_HardwareTransport.disarm_keepalive()`
  (the concrete override that delegates to `SerialConnection`).
- `src/host/robot_radio/io/serial_conn.py`:
  `SerialConnection.start_keepalive()`/`SerialConnection.stop_keepalive()`.

(grep across `src/host` and `src/tests` for `arm_keepalive`,
`disarm_keepalive`, `start_keepalive`, `stop_keepalive` turns up only their
own definitions plus `disconnect()`'s unconditional
`self.stop_keepalive()` call — no external caller anywhere.)

Their own module docstrings already say who used to call them: the
TestGUI's `KeyboardDriver` (cursor-key `DEV`-family teleop), which
128-005 deleted outright as fully dead in both Sim and hardware. This is
the last piece of that machinery left standing.

## Why this was NOT deleted in 128-009

`SerialConnection._keepalive_loop()` (the actual background-thread body of
`start_keepalive()`) reads `self._last_write_s` to decide whether enough
idle time has passed to justify sending a `"+"` keepalive line (the
idle-gate added in sprint 040, avoiding colliding a redundant `"+"` with
an in-flight command on the relay's RAW250 framing). But `_last_write_s`
is WRITTEN by roughly six other call sites throughout
`serial_conn.py` (`send()`, `send_fast()`, and other write paths — see
`grep -n "_last_write_s" src/host/robot_radio/io/serial_conn.py`), each
updating it as a side effect of every wire write, specifically so a future
keepalive check can see "a command already fed the watchdog recently."

If `_keepalive_loop`/`start_keepalive`/`stop_keepalive` are deleted,
`_last_write_s` becomes a write-only variable with no remaining reader —
itself newly-dead bookkeeping scattered across every write call site in
the file. Removing THAT cleanly means either (a) leaving `_last_write_s`
in place as now-purposeless bookkeeping (a fresh doc-rot landmine, not a
fix), or (b) auditing and un-threading it from every one of those ~6+ call
sites, which is a materially bigger, more invasive change than "delete two
small orphaned wrapper methods" — and risks touching write-path code this
sweep ticket has no business touching (128-009's own scope note: mechanical
sweep only, no scope-creep).

## What to do

A future ticket should:
1. Delete `Transport.arm_keepalive()`/`disarm_keepalive()` (base + hardware
   override) and their module-docstring sections in `transport.py`.
2. Delete `SerialConnection.start_keepalive()`/`stop_keepalive()`/
   `_keepalive_loop()`, the `_ka_thread`/`_ka_stop` instance attributes, and
   the `_KEEPALIVE_PERIOD_S` constant in `serial_conn.py`.
3. Remove `disconnect()`'s now-pointless `self.stop_keepalive()` call.
4. Audit every `_last_write_s = time.monotonic()` call site (`send()`,
   `send_fast()`, and others) and remove the now-dead bookkeeping, OR
   confirm it serves a second purpose independent of the keepalive gate
   (re-check before assuming it's single-purpose) and keep it with an
   updated comment.
5. Update `serial_conn.py`'s module docstring (`_ser` access-point list
   currently names `_keepalive_loop`) and the "Arm-on-demand contract"
   comment block accordingly.
6. Confirm on the bench that a long-running open-ended session (whatever
   currently exercises `S`/`VW`/`R`-family motion, if anything still does
   under protocol v5's bounded-`Move`-only wire shape) doesn't depend on
   the ambient `"+"` watchdog feed being available, even latent/unarmed —
   protocol v5's "no deadman" contract (every `Move` carries its own bounded
   `timeout`, per `.claude/rules/hardware-bench-testing.md`) suggests this
   whole mechanism may be legacy-protocol-v2-only and dead at the wire
   level too, not just at the Python caller level, but that should be
   confirmed rather than assumed before deleting.

## Acceptance

- Zero references to `arm_keepalive`/`disarm_keepalive`/`start_keepalive`/
  `stop_keepalive`/`_keepalive_loop`/`_ka_thread`/`_ka_stop`/
  `_KEEPALIVE_PERIOD_S` remain anywhere in `src/host`.
- `_last_write_s` either has a real remaining reader (documented) or is
  itself removed along with every write site that set it.
- `uv run python -m pytest` green; no behavior change to any live command
  path (`send()`/`send_fast()`/etc. still work identically minus the dead
  keepalive side effect).
