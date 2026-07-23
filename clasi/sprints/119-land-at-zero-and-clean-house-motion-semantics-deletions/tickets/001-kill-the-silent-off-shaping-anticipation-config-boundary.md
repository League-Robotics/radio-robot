---
id: '001'
title: Kill the silent-off shaping/anticipation config boundary
status: open
use-cases: [SUC-067]
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

- [ ] `SimLoop.configure_from_robot()` calls `estimator_kwargs()` and
      pushes it, alongside the existing Tier 1/2 calls.
- [ ] A bare `SimLoop` + `configure_from_robot()` session (no GUI, no
      manual push) runs Tour 1 with shaping/anticipation active —
      per-leg accuracy matches the GUI-path bands; read-back/ack counts
      confirm the push landed.
- [ ] `turn_prediction_capture.py` and `estimator_capture.py` inherit the
      push with zero script changes (verify by running them, not just
      inspecting).
- [ ] New `flags` bit 16 (name per ticket owner's choice, following the
      `kFlagFault*`/`kFlagEvent*` convention) set on every frame while a
      MOVE is active with both `ShaperLimits` axes disabled; clear
      otherwise. Verified in sim: strip the push → bit asserts, TestGUI
      banner shows, bench tooling prints it; push present → bit clear.
- [ ] `docs/protocol-v4.md` §8.2 bit table gets the new row (append-only;
      bits 8/10/12 untouched, not reassigned).
- [ ] `src/firm/app/DESIGN.md` §4 flags-bit-string enumeration updated
      with the new bit, edited directly on its canonical path.
- [ ] Button-acceptance suite unaffected — still green at its tightened
      bands.
- [ ] Full `uv run python -m pytest` suite green.
- [ ] Bench verification is DEFERRED to the phase-B bench session — not
      required to close this ticket.

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
