---
id: '001'
title: Kill the silent-off shaping/anticipation config boundary
status: done
use-cases:
- SUC-067
depends-on: []
github-issue: ''
issue: kill-the-silent-off-shaping-config-boundary.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Kill the silent-off shaping/anticipation config boundary

## Description

`SimHarness` constructs `MoveQueue` with shaping/anticipation OFF and the
sim build deliberately excludes `config/boot_config.cpp`, so a
correctness feature that changes turn accuracy ~20× is silently off in
any session that doesn't push `EstimatorConfigPatch` over the wire. The
TestGUI's own connect-time push covers the GUI path — every OTHER entry
point (`SimLoop.configure_from_robot()`, bench scripts, `repl.py`/
`cli.py`/`robot_mcp.py`) still runs silent-off.

**Verified against the tree (2026-07-23, post-118):**
- `src/host/robot_radio/calibration/push.py`'s `estimator_kwargs()`
  already exists. Its own docstring already documents that the former
  anticipation field (`stop_lead_ms`) was deleted (118 ticket 004). The
  live field set it selects is exactly `config.estimator.*`
  (`weight_heading_otos`/`weight_omega_otos`/`staleness_ms`) +
  `config.control.*` (`a_max`/`a_decel`/`alpha_max`/`alpha_decel`/
  `j_max`/`yaw_jerk_max`).
- `src/host/robot_radio/io/sim_loop.py:487`'s `configure_from_robot()`
  still only calls Tier 1 (`calibration_kwargs()`) and Tier 2
  (`motor_boot_config_for()`) — it does NOT call `estimator_kwargs()`.
  The silent-off defect is fully live post-118, unchanged in shape from
  the original issue.
- `docs/protocol-v4.md` §8.2's `flags` bit table has bits 8
  (`kFlagFaultI2CNak`), 10 (`kFlagEventDeadmanExpired`, explicitly
  "orphaned... left declared, not repurposed" per its own row), and 12
  (`kFlagEventConfigApplied`) already declared-but-unwired for OTHER,
  unrelated future meanings — do not repurpose any of them. Bits 16-31
  are genuinely free. Use bit **16**, the first free slot, per the same
  document's own "Reserved, not reused" append-only convention already
  established for message field numbers (§3, §6) and already followed by
  118 ticket 004 when it touched this same document's §5.2.

## Proposed fix (both halves, per the issue)

1. **Default-on at the composition seam every caller already goes
   through:** extend `SimLoop.configure_from_robot()` to also call
   `estimator_kwargs()` and push the result (the same proven
   `EstimatorConfigPatch` wire mechanism the TestGUI's own push already
   uses) — one change covers every `configure_from_robot` caller (GUI,
   bench scripts, tests, future scripts). The TestGUI's own existing push
   becomes redundant-but-harmless (idempotent acks) — no dedup is
   mandated by the issue, and none should be added speculatively.
2. **Loud off-state:** telemetry `flags` bit 16, set on every frame while
   a MOVE is active with BOTH angular and linear `ShaperLimits` disabled
   (mirror `shapeAndStage()`'s own early-return condition —
   `move_queue.cpp:143` — so the bit tracks exactly the regime where the
   land-at-zero gate can never fire and the threshold/timeout backstop is
   the only completion path). Host: TestGUI status-bar banner + log line
   when the bit is seen; bench-script tooling
   (`turn_prediction_capture.py`, `estimator_capture.py`) prints it too.
   `docs/protocol-v4.md` §8.2 gets a new table row (append-only — do not
   renumber or repurpose 8/10/12); pick a `kFlagFault*`/`kFlagEvent*` name
   following the existing prefix convention (this is accuracy-degrading,
   so `kFlagFaultShapingDisabled` likely reads better than an `Event`
   prefix — not a hard requirement, use judgment).

## Design overlay coordination

`src/firm/app/DESIGN.md` (the app subsystem's own DESIGN.md) needs a new
row in its own §4 "flags bit-string" enumeration (the same list
`docs/protocol-v4.md` §8.2 mirrors) for bit 16 — this sprint's overlay
slot went to `src/firm/motion/DESIGN.md` (ticket 002's more substantive
contract), so edit `src/firm/app/DESIGN.md` DIRECTLY on its canonical
path, not through the overlay. Also verify (do not assume) whether
`src/host/robot_radio/DESIGN.md`'s `io`/`config` directory rows need a
one-line update describing `configure_from_robot()`'s now-three-tier
push — if ticket 004 (docs relocation, sequenced after this ticket) also
touches this same file, coordinate so ticket 004's own diff includes this
ticket's already-landed change rather than reverting it.

## Acceptance Criteria

- [x] `SimLoop.configure_from_robot()` calls `estimator_kwargs()` and
      pushes it, alongside the existing Tier 1/2 calls.
- [x] A bare `SimLoop` + `configure_from_robot()` session (no GUI, no
      manual push) runs Tour 1 with shaping/anticipation active —
      per-leg accuracy matches the GUI-path bands; read-back/ack counts
      confirm the push landed.
- [x] `turn_prediction_capture.py` and `estimator_capture.py` inherit the
      push with zero script changes (verify by running them, not just
      inspecting).
- [x] New `flags` bit 16 (name per ticket owner's choice, following the
      `kFlagFault*`/`kFlagEvent*` convention) set on every frame while a
      MOVE is active with both `ShaperLimits` axes disabled; clear
      otherwise. Verified in sim: strip the push → bit asserts, TestGUI
      banner shows, bench tooling prints it; push present → bit clear.
- [x] `docs/protocol-v4.md` §8.2 bit table gets the new row (append-only;
      bits 8/10/12 untouched, not reassigned).
- [x] `src/firm/app/DESIGN.md` §4 flags-bit-string enumeration updated
      with the new bit, edited directly on its canonical path.
- [x] Button-acceptance suite unaffected — still green at its tightened
      bands.
- [x] Full `uv run python -m pytest` suite green.
- [x] Bench verification is DEFERRED to the phase-B bench session — not
      required to close this ticket.

## Completion Notes

**Bit used: 16, `kFlagFaultShapingDisabled`** — as planned, first free slot
in the 16-31 range, `docs/protocol-v4.md` §8.2 and `src/firm/app/telemetry.h`
both updated (append-only; bits 8/10/12 untouched).

**Firmware:** `App::MoveQueue::shapingDisabled()` (new public query,
`move_queue.h`/`.cpp`) mirrors `shapeAndStage()`'s own
`!linearShaping && !angularShaping` early-return gate exactly.
`RobotLoop::cycle()` ANDs it with `moveQueue_.active()` and sets the bit
right after the existing `kFlagFaultMoveTimeout` `setFlag()` call
(`robot_loop.cpp`). Two new C++ scenarios in
`app_robot_loop_harness.cpp`/`test_app_robot_loop.py` cover both states
(default-off ShaperLimits asserts the bit while a Move is active and clears
on completion; both-axes-enabled ShaperLimits stays clear throughout).

**Host push (Tier 3):** `SimLoop.configure_from_robot()` gains a third push
after Tier 1/2, reusing the SAME `SimConfigConn` Tier 1 builds (one config
connection). Applied/rejected/timed-out logging via the stdlib `logging`
module (`robot_radio.io.sim_loop` logger) — a real-time session (tick
thread running) polls the ack and logs the TestGUI's own
applied/rejected/timed-out shapes; a manual-step session (no tick thread —
`turn_prediction_capture.py`'s own established pattern) skips the blocking
ack poll (nothing would ever produce the ack without an explicit `step()`
call) and logs "sent, ack pending caller's own stepping" instead — see
completion note below for why this branch exists.

**Regression found and fixed mid-implementation (not in the original
plan):** `test_tour_closure_gate.py`'s own `_make_loop()` helper was ALREADY
manually pushing the same six shaper fields (matching
`data/robots/tovez_nocal.json`'s own committed values) as a workaround for
Tier 3 not existing yet. Once Tier 3 landed, this became a genuinely
redundant SECOND `EstimatorConfigPatch` push — harmless in VALUE (identical
numbers) but not in TIMING: it consumes one extra `Comms::pump()` cycle
before the first turn's Move is issued, which shifts `WheelPlant`'s
rest-dither phase (108-011) enough to tip an already-narrow-pocket
measurement (`TOUR_2/ideal` turn 12) from 2.491° to 2.509° against its
2.5° tolerance — a real, reproducible regression (confirmed via
`git stash`/`git stash pop` against the pre-change tree), not flakiness.
Fixed by changing `_make_loop()`'s six shaper parameters to default `None`
(skip the now-redundant manual push; rely on Tier 3 alone) while preserving
the override capability for any future caller wanting a genuinely different
sweep value — full rationale is inline in `_make_loop()`'s own updated
comment. This also surfaced a second, independent defect in Tier 3's own
ack-poll: a manual-step `SimLoop` session (no tick thread) blocked 500ms of
real wall-clock time per `configure_from_robot()` call and then logged a
misleading "TIMED OUT... 0/9 confirmed applied" even though the push had, in
fact, landed synchronously (`_run_or_enqueue()`'s own "run now if no tick
thread" contract) — observed running `turn_prediction_capture.py` directly,
per this ticket's own acceptance criterion. Fixed by having Tier 3 detect
the no-tick-thread case and skip the blocking poll, logging an honest "sent,
ack pending caller's own stepping" instead — regression-tested by
`test_configure_from_robot_deterministic_session_never_logs_a_false_timeout`/
`test_configure_from_robot_deterministic_session_still_applies_shaper_limits`
(`test_sim_loop.py`).

**Host loud off-state:** `TLMFrame.fault_shaping_disabled` (bit 16,
`protocol.py`) drives a new `shaping_disabled_banner` QLabel
(`testgui/__main__.py`, hidden by default, next to `mode_label`) plus an
edge-triggered `_append_log()` line (never per-frame — would flood the log
for the duration of any legitimately-unshaped Move) in
`_TelemetryBridge.on_frame_ready()`. `tlm_log.py` gained a
`flag_fault_shaping_disabled` CSV column (shared by both bench scripts) and
an edge-triggered stdout print in `stream_to_csv()` (covers
`estimator_capture.py --sim` and real-hardware `tlm_log.py` captures with
zero change to either caller); `turn_prediction_capture.py`'s own
`_CsvSink.write_frame()` gained the same edge-triggered print directly
(it does not route through `stream_to_csv()`).

**Verification:** full `uv run python -m pytest` — 1386 passed, 2 skipped,
9 xfailed, 2 xpassed (both xpasses pre-existing and unrelated — confirmed
against the pre-ticket tree). `test_tour_closure_gate.py` and
`test_gui_button_acceptance.py` both green. Both bench scripts run directly
(`estimator_capture.py --sim`, `turn_prediction_capture.py`) — confirmed via
their own CSV output that `flag_fault_shaping_disabled` stays `False`
throughout (shaping active, inherited with zero script changes to either
file's own `configure_from_robot()` call site).

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite);
  sim tour-closure gate; button-acceptance suite;
  `turn_prediction_capture.py`/`estimator_capture.py` (bench scripts, run
  directly to confirm zero-script-change inheritance).
- **New tests to write**: a sim-level test asserting flags bit 16's
  assert/clear behavior under push-absent vs. push-present conditions; a
  headless (no-GUI) `configure_from_robot()` + Tour 1 accuracy test if
  one doesn't already exist in this shape.
- **Verification command**: `uv run python -m pytest`, plus a manual or
  scripted run of both bench capture scripts to confirm the inherited
  push.
