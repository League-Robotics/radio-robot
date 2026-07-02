---
id: '003'
title: 'CR-15 maintenance batch: eight small independent cleanups'
status: done
use-cases:
- SUC-007
depends-on:
- '001'
github-issue: ''
issue: small-cleanups-from-2026-07-01-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# CR-15 maintenance batch: eight small independent cleanups

## Description

Eight independent, already-diagnosed small findings from the 2026-07-01
review (`clasi/issues/small-cleanups-from-2026-07-01-review.md`, CR-15).
Two are verify-only (already resolved by earlier work); the remaining six
are implemented here. See `architecture-update.md` §"CR-15 maintenance
batch" for the full per-item design.

**Depends on ticket 001** because item 1 (`PhysicsWorld._truePoseH` wrap) is
resolved *by* ticket 001 (it becomes load-bearing there) — this ticket only
verifies it, so it must run after ticket 001 lands.

**Important — read current code before touching `drive.py` (item 8):**
`KeyboardDriver` was substantially reworked by sprints 064/065 (deadman STOP
resend, keepalive arm/disarm). Read the current file in full before adding
held-key tracking; do not reintroduce the single-shot fire-and-forget STOP
the 065 work removed.

## Acceptance Criteria

- [x] **Item 1 — VERIFY ONLY.** Confirm `PhysicsWorld::_truePoseH` wraps to
      `(-π, π]` (resolved by ticket 001). No code change here; cite the
      ticket-001 commit in this ticket's notes.
- [x] **Item 2 — FIX.** `serial_conn.py`'s `probe_devices()` sends plain
      `HELLO` (not `>PING`) and matches a `DEVICE:` banner line, per
      `_banner_classify`'s protocol
      (`.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`). Return
      shape (`{port, lines, responsive}`) unchanged.
- [x] **Item 3 — FIX.** `SerialConnection.connect()`'s result dict gains
      `relay_info` (from the already-computed, currently-discarded
      `_relay_handshake()` return value at `serial_conn.py:355`/`:373`),
      matching the existing `announcement` field's pattern.
- [x] **Item 4 — FIX.** `SimTransport.connect()` sets `_connected = True`
      only after `_tick_loop` confirms `Sim()` construction succeeded (not
      before the tick-thread starts).
- [x] **Item 5 — FIX.** `traces.py`'s `_feed_encoder()` uses midpoint heading
      integration (`hMid = self._enc_h + dT * 0.5`) instead of
      post-increment, matching the convention used by `PhysicsWorld::update`,
      `SimOdometer::tick`, and `Odometry::predict`.
- [x] **Item 6 — VERIFY ONLY.** Run the `D ... stop=... sensor=...` scenario
      and confirm stop-slot count matches sprint 065-001's architecture
      update (2 internal + N wire, no wasted duplicate). No code change
      expected.
- [x] **Item 7 — FIX.** `rgbToHSV` moved out of `StopCondition.cpp`
      (verbatim) into new `source/control/ColorUtil.h`/`ColorUtil.cpp`;
      `Kind::COLOR` branch calls the moved function. Existing `Kind::COLOR`
      coverage (`test_stop_condition_coverage.py`) passes unmodified.
- [x] **Item 8 — FIX.** `KeyboardDriver` tracks a `_held_keys: set[int]`.
      Releasing one arrow key while another is held switches to driving the
      remaining held key instead of starting the STOP deadman sequence; the
      STOP deadman sequence (sprint 065) still fires when the last held key
      is released.
- [x] Full default test suite green, including `tests/testgui -q`.

## Completion Notes

- **Item 1 (verify).** `PhysicsWorld::update()` wraps `_truePoseH` to
  `(-π, π]` at `source/hal/sim/PhysicsWorld.cpp:105-106`, landed in ticket
  001 commit `495338c` (`feat(066-001): sim OTOS ground-truth sampling +
  shared lever-arm compensation`). Confirmed present on this branch; no
  additional code change.
- **Item 2.** `probe_devices()` rewritten to the plain-`HELLO` classify
  protocol (matches `_banner_classify`); `responsive` now means "a `DEVICE:`
  banner line was seen". Return shape (`{port, lines, responsive}` /
  `{port, error}`) unchanged. New tests:
  `tests/simulation/unit/test_probe_devices.py` (5 tests).
- **Item 3.** `connect()`'s result dict now includes `relay_info` (from
  `_relay_handshake()`'s already-computed return value) on both the
  auto-detect and explicit `mode="relay"` code paths; absent for a direct
  NEZHA2 connection. New tests added to
  `tests/simulation/unit/test_serial_relay_handshake.py` (3 tests).
- **Item 4.** Added `SimTransport._sim_ready_event` (a `threading.Event`)
  signaled by `_tick_loop` right after `Sim()` construction succeeds, or on
  the import-failure / construction-failure paths. `connect()` waits on it
  (bounded by a new `_SIM_READY_TIMEOUT_S = 5.0`) before setting
  `_connected`. New tests:
  `tests/testgui/test_transport.py::TestSimTransportConnectedFlagRace`
  (3 tests).
- **Item 5.** `_feed_encoder()` now computes `hMid = self._enc_h + dT * 0.5`
  and integrates `(dC*cos(hMid), dC*sin(hMid))` before advancing
  `self._enc_h += dT` — a minimal, surgical change to only the three
  integration lines (reset-detection logic in the same function left
  untouched per the ticket's note that ticket 004 rewrites it next).
- **Item 6 (verify).** Ran
  `tests/simulation/unit/test_065_001_stop_clause_overflow.py` and
  `tests/simulation/unit/test_motion_command.py` (44 tests, all pass) —
  confirms sprint 065-001's recount (`D` = 2 internal stops, no duplicate;
  `D ... stop=... sensor=...` = 2 internal + 2 wire = 4, exactly
  `kMaxStopConds`, no overflow). No code change.
- **Item 7.** `rgbToHSV` moved verbatim to new
  `source/control/ColorUtil.h`/`ColorUtil.cpp`; `StopCondition.cpp`'s
  `Kind::COLOR` branch calls it unchanged. `source/control/*.cpp` is
  glob-picked up by both the firmware and sim CMake builds
  (`utils/cmake/util.cmake`, `tests/_infra/sim/CMakeLists.txt`) — no build
  file edits needed. Verified with a clean sim rebuild
  (`cmake --build tests/_infra/sim/build --clean-first`) and
  `tests/simulation/system/test_stop_condition_coverage.py` (6/6 pass,
  unmodified).
- **Item 8.** `KeyboardDriver` now tracks `_held_keys: set[int]`. On
  release, if `vw_line_for_key_set(frozenset(self._held_keys))` (after
  discarding the released key) still returns a command, driving continues
  with it instead of starting the STOP deadman; the deadman still fires
  when the last held key is released. Focus-loss clears `_held_keys`
  entirely (all keys are implicitly released). New tests:
  `tests/testgui/test_drive.py::TestKeyboardDriverMultiKeyRelease`
  (5 tests).
- **Full suite:** `uv run --with pytest python -m pytest -q` → 2506 passed
  (baseline 2498 + 8 new). `uv run --with pytest python -m pytest
  tests/testgui -q` → 535 passed (baseline 527 + 8 new).

## Implementation Plan

**Approach:** Eight independent point-fixes across eight files; no shared
state between items. Implement in the order listed (matches file-touch
order, not a dependency order — items are independent of each other).

**Files to modify:**
- `source/control/StopCondition.cpp` (item 7)
- `host/robot_radio/io/serial_conn.py` (items 2, 3)
- `host/robot_radio/testgui/transport.py` (item 4)
- `host/robot_radio/testgui/traces.py` (item 5)
- `host/robot_radio/testgui/drive.py` (item 8)

**Files to create:**
- `source/control/ColorUtil.h`, `source/control/ColorUtil.cpp` (item 7)

**Verify-only (no file change expected):**
- `source/hal/sim/PhysicsWorld.{h,cpp}` (item 1 — confirm ticket 001's fix)
- Stop-condition slot accounting (item 6 — confirm sprint 065-001)

**Testing plan:**
- Existing tests to run: `tests/simulation/system/test_stop_condition_coverage.py`
  (item 7 regression guard), `tests/testgui/test_traces.py` (item 5),
  `tests/testgui` full tier (item 8, plus any existing `KeyboardDriver`
  tests), any `test_serial_relay_handshake.py`-adjacent tests (items 2-3),
  full default suite.
- New tests: a `probe_devices()` test against the new `HELLO` protocol
  (item 2); a multi-key release test for `KeyboardDriver` (item 8,
  exercising `vw_line_for_key_set` with a held-key set); item 1 and item 6
  verification can be existing-test re-runs rather than new tests.
- Verification command: `uv run --with pytest python -m pytest -q` and
  `uv run --with pytest python -m pytest tests/testgui -q`.

**Documentation updates:** None beyond this ticket and
`architecture-update.md`.
