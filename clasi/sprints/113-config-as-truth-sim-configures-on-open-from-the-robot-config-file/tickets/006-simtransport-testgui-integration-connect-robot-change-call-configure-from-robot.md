---
id: '006'
title: 'SimTransport/TestGUI integration: connect()/robot-change call configure_from_robot()'
status: done
use-cases:
- SUC-001
- SUC-003
depends-on:
- '005'
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

- [x] `SimTransport.connect()` calls (directly or via a small private
      helper) `self._loop.configure_from_robot(config)` for the active robot
      config as part of connecting — before `connect()` returns, i.e. before
      any twist/move could reach the sim through a normal GUI action.
- [x] `SimTransport` gains a public `configure_from_robot(config)` method
      (thin delegation to `self._loop.configure_from_robot(config)`, guarded
      by `if self._loop is not None`) so `__main__.py`'s robot-select handler
      can call it directly instead of only pushing text `SET` commands
      through `transport.command()`.
- [x] `__main__.py`'s `_push_robot_calibration()` (or a sibling call added
      alongside it) triggers the Tier-2 push on every robot-change while
      connected to Sim (SUC-003) — verified by the new ticket-007 test that
      switches robots mid-session and checks both tiers landed.
- [x] `testgui/transport.py` imports the relocated `SimConfigConn` (ticket
      005) rather than defining its own — confirmed by grep (no duplicate
      class definition remains).
- [x] No behavior change for hardware transports (`_HardwareTransport`) —
      this ticket only touches `SimTransport`/`__main__.py`'s Sim-specific
      wiring.

## Completion Notes

**Design choice**: option (a) — `Transport.connect()`'s ABC signature was
widened to `connect(self, *, robot_config: RobotConfig | None = None) ->
None` (keyword-only, so no existing positional call site anywhere in the
tree could break). `_HardwareTransport.connect()` accepts and ignores it
(documented no-op — real hardware boots from its own reflash).
`SimTransport.connect()` uses the passed value, or resolves
`robot_radio.config.robot_config.get_robot_config()` itself when omitted —
so EVERY existing caller across the test suite that constructs a bare
`SimTransport()` and calls `.connect()` with no arguments still gets the
automatic push (SUC-001), with zero call-site changes required at any of
those sites.

`__main__.py`'s own `_on_connect()` deliberately calls `transport.connect()`
with **no** `robot_config` keyword, even though it has one in scope
(`get_robot_config()`) — several existing test fixtures in
`src/tests/testgui/` define lightweight `Transport` doubles with a narrower
`connect(self) -> None` override (no `robot_config` parameter at all); a
keyword argument would raise `TypeError` on those and silently abort
`_on_connect()` before `_state["transport"]` is set (regression caught by
`test_sim_errors_panel.py`/`test_sim_errors_from_cal_button.py` during this
ticket's own verification pass). Since `SimTransport.connect()` already
falls back to `get_robot_config()` internally, the bare no-arg call is
sufficient and safest.

`_push_robot_calibration()` keeps its existing per-command `SET`/OTOS push
loop UNCHANGED for both transport kinds (it is what pushes `OI`/`OL`/`OA` —
neither Tier 1 nor Tier 2 of `configure_from_robot()` covers OTOS, see
sprint.md's Out of Scope — and what populates `SimTransport._config_echo`,
the host-side cache `GET` reads from). For a connected `SimTransport`, it
additionally calls `transport.configure_from_robot(cfg)` after that loop, to
cover Tier 2. This means Tier 1 is pushed twice for Sim (once per-command,
once again inside `configure_from_robot()`) — accepted per this ticket's own
Idempotency note as harmless, since eliminating it cleanly would require
either giving `SimLoop` a Tier-2-only entry point (out of this ticket's file
scope: only `transport.py`/`__main__.py`) or duplicating the host-side echo
bookkeeping outside `_handle_config_set()`.

**Real bug found and fixed** (`src/host/robot_radio/io/sim_config.py`,
technically outside this ticket's stated file scope, but required for
correctness): `SimConfigConn` assigned corr_ids from a private
per-instance counter starting at 1, on the documented assumption ("this
adapter is the only sender on this path") that exactly one instance ever
talks to a given `SimLoop`. This ticket's wiring is the first to violate
that: `SimTransport` keeps its own long-lived `SimConfigConn`
(`self._config_conn`, backing `SET`/`GET`) while `SimLoop.
configure_from_robot()` constructs a second, throwaway one on every call.
Two independent counters both starting at 1 assigned the same corr_id to
different wire commands; since neither sender drains its own ack, a later
`poll_ack()` call (e.g. `_handle_config_set()`, invoked moments after a
fresh Sim Connect) could match a stale ack left by the OTHER sender's
corr_id=1 — observed as a spurious `ERR nak ml err_code=6` on a `SET ml=...`
that was never actually rejected, breaking
`test_calibration_push_on_connect.py`'s GET-based assertions. Fixed by
routing `SimConfigConn.send_envelope_fast()` through `SimLoop`'s own
existing thread-safe `_next_corr_id()` counter (already shared by
`twist()`/`stop()`/`move()`) instead of a private one — corr_ids are now
globally unique per `SimLoop` regardless of how many `SimConfigConn`
instances exist over its lifetime.

**Testing performed**: `src/tests/testgui/test_calibration_push_on_connect.py`
(14/14 passed), `src/tests/testgui/test_sim_errors_panel.py` +
`test_sim_errors_from_cal_button.py` (17/17 passed, after the corr_id fix),
the full targeted sweep `uv run python -m pytest src/tests/testgui -q -k
"transport or connect or config or sim"` (133 passed, 4 xfailed as
pre-existing), a full `src/tests/testgui` run (484 passed, 10 xfailed, 3
xpassed — all pre-existing markers, no regressions), and (out of caution,
beyond this ticket's stated test scope, since `sim_config.py` is shared
foundational code) a full `src/tests/sim` run: 382 passed, 1 pre-existing
unrelated failure (`test_wire_differential.py::
test_field_numbers_match_pb2_descriptors_telemetry` — a stale
`PlannerConfigPatch` field-number regression pin that has not tracked
`distance_kp` since it was added in sprint 112 ticket 003, commit
`21c0a066`, well before this sprint; confirmed via `git log` that this
predates ticket 001 of sprint 113 and is untouched by any file this ticket
modifies — flagged for team-lead, not fixed here as out of scope).

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
