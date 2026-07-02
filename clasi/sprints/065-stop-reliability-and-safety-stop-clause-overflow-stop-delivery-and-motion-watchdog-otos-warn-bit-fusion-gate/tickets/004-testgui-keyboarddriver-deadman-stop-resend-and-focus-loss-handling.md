---
id: '004'
title: 'TestGUI KeyboardDriver: deadman STOP resend and focus-loss handling'
status: open
use-cases: [SUC-002]
depends-on: []
github-issue: ''
issue: stop-delivery-and-keepalive-watchdog-architecture.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI KeyboardDriver: deadman STOP resend and focus-loss handling

## Description

CR-04 (part of CR-04/CR-05, high). `KeyboardDriver._on_key_release`
(`host/robot_radio/testgui/drive.py:263-288`) stops the `VW` resend timer and
sends exactly one `STOP` via `transport.send()` — fire-and-forget, no ack,
no retry. Direct USB intermittently drops 15-50% of lines; a dropped `STOP`
means the robot coasts at the last commanded velocity with nothing left to
stop it (compounded by the pre-fix ambient keepalive addressed in ticket
005). Separately, if the window loses focus while an arrow key is logically
held, Qt never delivers `keyReleaseEvent` at all — `_on_key_release` never
runs, so the robot keeps being resent the last held-key `VW` indefinitely.

Fix: turn key-release into a bounded deadman resend of `STOP` (reusing the
existing non-blocking timer/`_send_cmd` machinery, not a new blocking
acked-retry loop — see `architecture-update.md` Design Rationale Decision 4
for why a blocking `command()` retry was rejected), and treat window
focus-loss as an implicit release.

This ticket is independent of tickets 001-003 (firmware) and can be executed
in any order relative to them; sequenced after the firmware cluster in
`sprint.md` for scope-severity ordering only (CR-01 first).

## Acceptance Criteria

- [ ] `KeyboardDriver._on_key_release` no longer stops the timer and sends a
      single `STOP`. Instead it sets `self._cmd = "STOP"` and lets the
      existing timer keep firing for a bounded count (`STOP_RESEND_COUNT`,
      a new named constant — e.g. 5 ticks at the existing 100 ms interval)
      before actually stopping the timer.
- [ ] `_on_timer_tick` resends `self._cmd` unconditionally as it already
      does today — no special-casing needed there beyond `_cmd` now
      sometimes being `"STOP"` instead of a `VW` line during the deadman
      window.
- [ ] `KeyboardDriver.attach()`/`detach()` additionally save/restore
      `window.focusOutEvent`, monkeypatched the same way
      `keyPressEvent`/`keyReleaseEvent` already are.
- [ ] A new `_on_focus_out` handler: if a key is currently tracked as held
      (`self._cmd` is a `VW` line, not `None`/`"STOP"`), triggers the same
      deadman-resend sequence a real key-release would. Forwards to the
      original `focusOutEvent` handler afterward (or before — whichever
      preserves existing Qt semantics; verify no double-handling).
- [ ] `vw_line_for_key`/`vw_line_for_key_set` (pure, Qt-free helpers) are
      unchanged — this ticket only touches `KeyboardDriver`'s stateful
      event handlers.
- [ ] New unit test in `tests/testgui/test_drive.py`: simulate a key press
      followed by a release, assert the driver sends `STOP` on the release
      tick and continues resending it for `STOP_RESEND_COUNT - 1` further
      timer ticks, then stops.
- [ ] New unit test: simulate a "dropped STOP" by having the fake transport
      raise/no-op on the first `STOP` send — assert a subsequent deadman
      resend still gets through and the timer still stops on schedule
      (mirrors the sprint's "simulate a dropped STOP" acceptance test,
      exercised at the driver level since `drive.py` has no direct sim/bench
      hook of its own).
- [ ] New unit test: simulate `_on_focus_out` while `self._cmd` is a `VW`
      line — assert the deadman `STOP` sequence fires exactly as a real
      key-release would.
- [ ] Existing `tests/testgui/test_drive.py` coverage (key press/release,
      timer start/stop, `vw_line_for_key*` pure functions) stays green.
- [ ] Full default sim suite green (no firmware files touched by this
      ticket).

## Implementation Plan

**Approach**: Reuse the existing `QTimer`/`_send_cmd` resend machinery;
change only what `self._cmd` holds and when the timer stops. No new thread,
no blocking `command()` call on the Qt main thread (rejected alternative —
see Design Rationale Decision 4 in `architecture-update.md`).

**Files to modify**:
- `host/robot_radio/testgui/drive.py` — `KeyboardDriver`: new
  `STOP_RESEND_COUNT` constant; `_on_key_release` becomes the deadman
  trigger; `_on_timer_tick` needs no logic change (already resends
  `self._cmd` unconditionally) but its docstring should be updated to
  reflect the new dual purpose (VW keepalive resend vs. STOP deadman
  resend); `attach()`/`detach()` gain the `focusOutEvent` save/restore;
  new `_on_focus_out` handler.

**Testing plan**:
- Extend `tests/testgui/test_drive.py` with the three new cases above
  (deadman resend count, dropped-STOP recovery, focus-loss-as-release).
  These are pure-Python/mock-transport tests per the existing file's
  pattern (no `QApplication` required for the pure helpers; a minimal fake
  `window`/`transport` double is already used for the stateful
  `KeyboardDriver` tests per the existing test file's structure — follow
  that pattern).
- Run `uv run --with pytest python -m pytest tests/testgui/ -q` plus the
  full default sim suite (host-only change, but full suite confirms no
  cross-tier breakage).

**Documentation updates**: `architecture-update.md` already documents this
change (Step 4-5 item 4, Design Rationale Decision 4). Update the module
docstring at the top of `drive.py` (the "Guard" section currently says
"STOP is never suppressed — it is always sent on key release") to describe
the deadman-resend behavior and the focus-loss handling.
