---
id: '006'
title: 'SimTransport/TestGUI integration: connect()/robot-change call configure_from_robot()'
status: open
use-cases: [SUC-001, SUC-003]
depends-on: ['005']
github-issue: ''
issue: config-as-truth-sim-configure-on-open.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# SimTransport/TestGUI integration: connect()/robot-change call configure_from_robot()

## Description

Today, Tier-1 config only reaches the sim as a side effect of the GUI's
manual "robot select" action (`_push_robot_calibration()` in
`testgui/__main__.py`) — a plain `Connect` click, with no robot-select click,
pushes nothing. This ticket makes `SimTransport.connect()` itself call
`SimLoop.configure_from_robot()` (ticket 005) automatically, using whatever
robot config is currently active — closing that gap for SUC-001 ("No manual
'robot select' GUI click is required for the push to occur — it happens as
part of `connect()`").

`SimTransport.connect()` (`testgui/transport.py`) already resolves a
trackwidth from `sim_prefs.load_sim_error_profile()` before constructing the
`SimLoop`. It does **not** currently resolve "the active robot config" at
all — that resolution (`get_robot_config()`) lives in `__main__.py`'s GUI
code, called by `_push_robot_calibration()`. Determine how `SimTransport`
should get the active `RobotConfig` without creating a new `testgui/`→GUI
coupling in the wrong direction: `get_robot_config()` is itself GUI-adjacent
(it depends on whatever the GUI's currently-selected-robot state is), so
either (a) `SimTransport.connect()` takes an optional `RobotConfig` parameter
that `__main__.py` passes in (caller-supplied, no new coupling), or (b)
`SimTransport` gains its own accessor property the caller sets before
`connect()` (mirroring how `_speed_factor` is already set before `connect()`
and re-applied on connect). Prefer (a) if it doesn't disturb `SimTransport`'s
existing `Transport` base-class `connect()` signature (check `Transport`'s
own ABC for a `connect(*, robot_config: RobotConfig | None = None)`-shaped
extension point, or add one) — otherwise (b). Document the choice actually
made in this ticket's completion notes.

Also update the existing manual "robot select" action
(`_push_robot_calibration()` in `__main__.py`) to ALSO trigger the new
Tier-2 push on robot-change (SUC-003) — today it only pushes Tier 1. Simplest
correct approach: have it call `transport.configure_from_robot(config)` (a
thin `SimTransport` method delegating to `self._loop.configure_from_robot()`)
when connected to Sim, in addition to (or instead of, if it becomes fully
redundant — see below) its own existing per-command push loop.

**Idempotency note**: after this ticket, `connect()` pushes full config once
automatically, and `_push_robot_calibration()` re-pushes on every manual
robot-select — both landing on the same `configure_from_robot()` path is
fine (repushing the same values is a harmless no-op), but avoid pushing
*twice in immediate succession* on a fresh Connect if the GUI's own connect
flow also fires a robot-select-equivalent afterward; check `__main__.py`'s
actual Connect handler wiring and avoid a redundant double-push if one would
otherwise occur (not a correctness bug, just wasted wire traffic worth a
one-line guard if trivial).

## Acceptance Criteria

- [ ] `SimTransport.connect()` calls (directly or via a small private
      helper) `self._loop.configure_from_robot(config)` for the active robot
      config as part of connecting — before `connect()` returns, i.e. before
      any twist/move could reach the sim through a normal GUI action.
- [ ] `SimTransport` gains a public `configure_from_robot(config)` method
      (thin delegation to `self._loop.configure_from_robot(config)`, guarded
      by `if self._loop is not None`) so `__main__.py`'s robot-select handler
      can call it directly instead of only pushing text `SET` commands
      through `transport.command()`.
- [ ] `__main__.py`'s `_push_robot_calibration()` (or a sibling call added
      alongside it) triggers the Tier-2 push on every robot-change while
      connected to Sim (SUC-003) — verified by the new ticket-007 test that
      switches robots mid-session and checks both tiers landed.
- [ ] `testgui/transport.py` imports the relocated `SimConfigConn` (ticket
      005) rather than defining its own — confirmed by grep (no duplicate
      class definition remains).
- [ ] No behavior change for hardware transports (`_HardwareTransport`) —
      this ticket only touches `SimTransport`/`__main__.py`'s Sim-specific
      wiring.

## Testing

- **Existing tests to run**: `test_calibration_push_on_connect.py` (must
  still pass — its existing assertions on manual robot-select behavior are
  unaffected, only augmented) and the four `_SimConfigConn`-consumer tests
  named in ticket 005's Testing section (transitively affected by the
  relocation, but that's ticket 005's regression surface — re-run here only
  if this ticket's `__main__.py` changes touch anything they cover).
- **New tests to write**: none required directly by this ticket — SUC-001's
  "no manual click needed" and SUC-003's "profile switch re-pushes both
  tiers" acceptance criteria are exercised by ticket 007's parity/integration
  tests, which depend on this ticket's wiring existing.
- **Verification command**:
  `QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_calibration_push_on_connect.py -v`,
  then the full suite.

## Files to touch

- `src/host/robot_radio/testgui/transport.py` (`SimTransport.connect()`,
  new `configure_from_robot()` method, import-site update for the relocated
  config-sender)
- `src/host/robot_radio/testgui/__main__.py` (`_push_robot_calibration()` or
  a sibling call, wired to also trigger the Tier-2 push)

## Depends On

- Ticket 005 (needs `SimLoop.configure_from_robot()` and the relocated
  config-sender to exist before `SimTransport` can call/import them).
