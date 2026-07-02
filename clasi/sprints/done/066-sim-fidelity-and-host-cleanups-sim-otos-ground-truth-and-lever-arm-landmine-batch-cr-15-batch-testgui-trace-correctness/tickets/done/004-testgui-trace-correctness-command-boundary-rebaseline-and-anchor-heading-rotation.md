---
id: '004'
title: 'TestGUI trace correctness: command-boundary rebaseline and anchor-heading
  rotation'
status: done
use-cases:
- SUC-008
- SUC-009
depends-on:
- '003'
github-issue: ''
issue: testgui-trace-correctness-slow-tlm-and-anchor-rotation.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI trace correctness: command-boundary rebaseline and anchor-heading rotation

## Description

Two TestGUI trace defects
(`clasi/issues/testgui-trace-correctness-slow-tlm-and-anchor-rotation.md`,
CR-09/CR-10):

- (a) `TraceModel._feed_encoder`'s magnitude-based reset heuristic
  (`_ENC_RESET_EPS_MM`/`_ENC_RESET_BASE_MM`) misses resets on slow relay TLM
  (1-2 Hz, 100-200 mm robot travel between frames), re-breaking the original
  "encoder track ignores turns" fix on exactly the transport where it
  matters. Fixed by an explicit host-side rebaseline signal fired at
  command-send time instead of inferred from data.
- (b) `_feed_otos`/`_feed_fused` rotate world-frame firmware deltas by the
  anchor heading alone, correct only when the firmware pose was freshly
  zeroed at anchor time. Fixed by rotating by
  `(anchor_yaw − firmware_heading_at_baseline)`, using the already-captured
  (but currently unused) `hdg_cdeg` baseline field.

See `architecture-update.md` §"TestGUI trace correctness (CR-09/CR-10)" and
Design Rationale Decision 3 (command-boundary rebaseline chosen over a
firmware TLM reset-epoch field, evaluated explicitly and rejected for this
sprint — see that section for the full tradeoff) for the full design.

**Depends on ticket 003** because both tickets edit
`TraceModel._feed_encoder()` in `traces.py` (003 fixes midpoint-heading
integration; 004 replaces the reset-detection heuristic) — landing 003 first
avoids rewriting the reset-detection logic against a since-changed
integration formula.

## Acceptance Criteria

- [x] `TraceModel` gains `notify_reset_pending()`, which forces
      `_enc_baseline = None` so the next `_feed_encoder()` call establishes a
      fresh baseline from whatever value that frame carries (no magnitude
      check).
- [x] The `_ENC_RESET_EPS_MM`/`_ENC_RESET_BASE_MM` heuristic and its
      magnitude-based branch in `_feed_encoder()` are removed.
- [x] `Transport` (the ABC in `testgui/transport.py`) classifies
      reset-inducing outbound commands (`D`, `ZERO enc`, `ZERO`) at its
      `command()`/`send()` choke point and invokes a new `on_reset_pending`
      callback (default no-op) before the command is sent.
- [x] `__main__.py` wires `transport.on_reset_pending =
      trace_model.notify_reset_pending` (the one wiring line outside
      `sprint.md`'s originally-listed TestGUI file set — see
      architecture-update.md Migration Concerns).
- [x] `_feed_otos`/`_feed_fused` rotate the world-frame delta by
      `(self._anchor_h - math.radians(baseline[2] / 100.0))` instead of
      calling `self._tw()` (which remains unchanged, used only by the
      body-frame encoder trace).
- [x] New test: delayed-TLM reset scenario — first post-reset frame at
      ~150 mm (well past the old 20 mm epsilon), reset signalled via
      `notify_reset_pending()` before the frame arrives — preserves
      accumulated heading (no spurious reverse motion, no cancelled turn).
- [x] New test: anchor with non-zero firmware heading — otos/fused traces
      align with the camera trace (not rotated by the stale firmware
      heading).
- [x] Full default test suite green, including `tests/testgui -q`.

## Implementation Plan

**Approach:**
1. Add `notify_reset_pending()` to `TraceModel`; remove the magnitude
   heuristic from `_feed_encoder()`.
2. Add the command classifier and `on_reset_pending` callback to `Transport`.
3. Fix `_feed_otos`/`_feed_fused`'s rotation (small new helper alongside
   `_tw`, or inline — programmer's call given it's a two-line change per
   method).
4. Wire `__main__.py`'s one line.
5. Write both new tests.

**Files to modify:**
- `host/robot_radio/testgui/traces.py`
- `host/robot_radio/testgui/transport.py`
- `host/robot_radio/testgui/__main__.py` (one wiring line only)

**Testing plan:**
- Existing tests to run: `tests/testgui/test_traces.py` (extensively —
  confirm no existing reset/rotation test regresses), `tests/testgui -q`
  full tier, full default suite.
- New tests: delayed-TLM reset scenario; mid-session anchor rotation
  scenario (both described in Acceptance Criteria).
- Verification command: `uv run --with pytest python -m pytest tests/testgui -q`
  and the full default suite.

**Documentation updates:** None beyond this ticket and
`architecture-update.md`.
