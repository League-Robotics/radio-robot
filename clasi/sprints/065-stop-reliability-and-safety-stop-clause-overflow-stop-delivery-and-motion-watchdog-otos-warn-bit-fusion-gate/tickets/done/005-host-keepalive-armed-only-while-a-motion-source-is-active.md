---
id: '005'
title: Host keepalive armed only while a motion source is active
status: done
use-cases:
- SUC-003
depends-on:
- '004'
github-issue: ''
issue: stop-delivery-and-keepalive-watchdog-architecture.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host keepalive armed only while a motion source is active

## Description

CR-05b, host side (part of CR-04/CR-05, high). `SerialConnection.connect()`
calls `self.start_keepalive()` unconditionally (both the fast cache-hit path
at `serial_conn.py:314` and the normal handshake path at `:363`);
`disconnect()` calls `stop_keepalive()`. The daemon thread streams `+` for
the entire lifetime of the connection, independent of whether anything is
actually driving — this is the host-side half of the "ambient keepalive
defeats the watchdog" mechanism (the other half, the firmware resetting on
any line, is fixed in ticket 002). A repo-wide grep of maintained code
confirms `start_keepalive`/`stop_keepalive` are called *only* from
`connect()`/`disconnect()` today; no maintained bench script or `rogo` path
depends on the ambient daemon (`T`/`D`/`G`/`TURN`/`RT` are already
watchdog-exempt via their own `TIME` net; `smoke_ritual` already sends `+`
explicitly per project knowledge).

Fix: arming moves to the layer that owns motion — `KeyboardDriver` (ticket
004) — via a small `Transport`-level passthrough. See
`architecture-update.md` Step 4-5 item 5 for the full design and the Impact
table for the two existing tests this ticket must update.

Depends on ticket 004 (adds the `arm_keepalive()`/`disarm_keepalive()` call
sites inside `KeyboardDriver`'s press/release/focus-loss handling that this
ticket's `Transport` methods are called from).

## Acceptance Criteria

- [x] `host/robot_radio/io/serial_conn.py`'s `connect()` no longer calls
      `self.start_keepalive()` on either the fast cache-hit path or the
      normal handshake path. `disconnect()` still calls
      `self.stop_keepalive()` (idempotent cleanup, harmless whether or not
      it was armed).
- [x] `SerialConnection.start_keepalive()`/`stop_keepalive()` public API
      shape is unchanged — only the caller moves.
- [x] `host/robot_radio/testgui/transport.py`'s `Transport` ABC gains two
      new methods, `arm_keepalive()` and `disarm_keepalive()`, defaulting to
      no-ops (not abstract — existing subclasses must not break).
- [x] `_HardwareTransport` (the shared `SerialTransport`/`RelayTransport`
      base) overrides both to delegate to `self._conn.start_keepalive()`/
      `stop_keepalive()`.
- [x] `SimTransport` uses the inherited no-op default (no real serial link;
      the sim's parallel watchdog-classification fix is ticket 002/003,
      exercised directly by `sim_command()`).
- [x] `KeyboardDriver` (from ticket 004) calls
      `self._transport.arm_keepalive()` when a driving session starts (first
      key press while not already armed) and
      `self._transport.disarm_keepalive()` once the deadman `STOP` sequence
      completes (or on `detach()`).
- [x] `tests/simulation/unit/test_serial_relay_handshake.py`'s
      `test_keepalive_is_plain` and `test_keepalive_plain_plus` are updated
      to call `conn.start_keepalive()` explicitly after `conn.connect()`,
      preserving their original intent (verify `+` is sent plain, never
      relay-prefixed) under the new arm-on-demand contract.
- [x] New test: after `connect()` alone (no `arm_keepalive()` call), no `+`
      is observed on the wire for the test's observation window.
- [x] New test (TestGUI-level, in `tests/testgui/test_drive.py` alongside
      ticket 004's new tests): a fake `Transport` records
      `arm_keepalive()`/`disarm_keepalive()` calls; assert they bracket a
      key-press-then-release-then-deadman-complete sequence correctly.
- [x] Full default sim suite green, including the two updated
      `test_serial_relay_handshake.py` tests.

## Implementation Plan

**Approach**: Minimal, additive `Transport`-level passthrough; the real
behavior change is removing two call sites from `SerialConnection.connect()`
and adding two call sites inside `KeyboardDriver` (already modified by
ticket 004 for the deadman logic — this ticket adds the arm/disarm calls
alongside it).

**Files to modify**:
- `host/robot_radio/io/serial_conn.py` — remove the two `start_keepalive()`
  calls from `connect()`; leave `disconnect()`'s `stop_keepalive()` call.
- `host/robot_radio/testgui/transport.py` — `Transport` ABC: new
  `arm_keepalive()`/`disarm_keepalive()` no-op methods (with a docstring
  explaining the default is intentional for `SimTransport`).
  `_HardwareTransport`: override both to delegate to `self._conn`.
- `host/robot_radio/testgui/drive.py` — `KeyboardDriver`: call
  `self._transport.arm_keepalive()`/`disarm_keepalive()` at the appropriate
  points in the (ticket-004-modified) press/release/focus-loss handlers.
- `tests/simulation/unit/test_serial_relay_handshake.py` — add
  `conn.start_keepalive()` after `connect()` in the two named tests.

**Testing plan**:
- Update the two named `test_serial_relay_handshake.py` tests (they will
  fail without the explicit `start_keepalive()` call once `connect()` no
  longer arms automatically — confirm this is understood as an intentional,
  documented behavior change, not a regression, per
  `architecture-update.md`'s Migration Concerns).
- New `SerialConnection`-level test: `connect()` alone produces no `+`
  traffic within an observation window comfortably longer than the
  keepalive period.
- New `KeyboardDriver`-level test with a fake `Transport` double recording
  arm/disarm calls, asserting correct bracketing around a drive session.
- Run the full default sim suite.

**Documentation updates**: `architecture-update.md` already documents this
change (Step 4-5 item 5, Impact table, Migration Concerns, Open Question 2).
Update `SerialConnection`'s module docstring / the `_KEEPALIVE_PERIOD_S`
comment block (`serial_conn.py:82-87`) to describe the new arm-on-demand
contract instead of "while connected."
