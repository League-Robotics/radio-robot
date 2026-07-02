---
id: '003'
title: 'CR-15 maintenance batch: eight small independent cleanups'
status: open
use-cases: [SUC-007]
depends-on: ['001']
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

- [ ] **Item 1 — VERIFY ONLY.** Confirm `PhysicsWorld::_truePoseH` wraps to
      `(-π, π]` (resolved by ticket 001). No code change here; cite the
      ticket-001 commit in this ticket's notes.
- [ ] **Item 2 — FIX.** `serial_conn.py`'s `probe_devices()` sends plain
      `HELLO` (not `>PING`) and matches a `DEVICE:` banner line, per
      `_banner_classify`'s protocol
      (`.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`). Return
      shape (`{port, lines, responsive}`) unchanged.
- [ ] **Item 3 — FIX.** `SerialConnection.connect()`'s result dict gains
      `relay_info` (from the already-computed, currently-discarded
      `_relay_handshake()` return value at `serial_conn.py:355`/`:373`),
      matching the existing `announcement` field's pattern.
- [ ] **Item 4 — FIX.** `SimTransport.connect()` sets `_connected = True`
      only after `_tick_loop` confirms `Sim()` construction succeeded (not
      before the tick-thread starts).
- [ ] **Item 5 — FIX.** `traces.py`'s `_feed_encoder()` uses midpoint heading
      integration (`hMid = self._enc_h + dT * 0.5`) instead of
      post-increment, matching the convention used by `PhysicsWorld::update`,
      `SimOdometer::tick`, and `Odometry::predict`.
- [ ] **Item 6 — VERIFY ONLY.** Run the `D ... stop=... sensor=...` scenario
      and confirm stop-slot count matches sprint 065-001's architecture
      update (2 internal + N wire, no wasted duplicate). No code change
      expected.
- [ ] **Item 7 — FIX.** `rgbToHSV` moved out of `StopCondition.cpp`
      (verbatim) into new `source/control/ColorUtil.h`/`ColorUtil.cpp`;
      `Kind::COLOR` branch calls the moved function. Existing `Kind::COLOR`
      coverage (`test_stop_condition_coverage.py`) passes unmodified.
- [ ] **Item 8 — FIX.** `KeyboardDriver` tracks a `_held_keys: set[int]`.
      Releasing one arrow key while another is held switches to driving the
      remaining held key instead of starting the STOP deadman sequence; the
      STOP deadman sequence (sprint 065) still fires when the last held key
      is released.
- [ ] Full default test suite green, including `tests/testgui -q`.

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
