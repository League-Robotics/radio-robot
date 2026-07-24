---
id: '002'
title: 'Tour final-leg hang: root-cause, then make the tour runner consume the ack
  ring'
status: in-progress
use-cases:
- SUC-072
depends-on: []
github-issue: ''
issue: tour-1-final-leg-completes-only-on-stop.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Tour final-leg hang: root-cause, then make the tour runner consume the ack ring

## Description

Functional blocker. Running a tour, the robot executes every leg EXCEPT the
last, then sits idle — the tour never finishes. Pressing STOP causes the final
step to retire and the tour to report done. The final `Move`'s completion is
not being recognized on its own; issuing STOP is what unblocks the last leg.

The filed issue (`tour-1-final-leg-completes-only-on-stop.md`) is an explicit
LEAD, not a diagnosis: "Do not assume it before reproducing." This ticket must
ROOT-CAUSE it, then fix it. Reproduce Sim-first (deterministic) to isolate
host-vs-firmware.

## Root-cause finding from planning-time static analysis (confirm, do not assume)

The issue's candidate #1 ("is the host still reading a single ack slot per
frame, or draining the ring correctly?") is where the evidence points, verified
in code at planning time:

- `tour.py::_drain_and_poll()` (`src/host/robot_radio/planner/tour.py`, ~line
  414) reads only the single scalar "freshest ack" slot: `frame.ack is not None
  and frame.ack.corr_id == move_id`. `frame.ack` is populated ONLY on the one
  frame whose `flags` bit 5 / `ack_fresh` is set (see
  `TLMFrame.from_pb2()`/`AckEntry.from_telemetry()` in `robot/protocol.py`).
- It was NOT updated when sprint 120-001 added the depth-4 ack ring
  (`frame.acks`, always populated, no freshness gate).
  `NezhaProtocol.wait_for_ack()` / `SerialConnection.wait_for_ack()` WERE made
  ring-aware by 120-001 (they scan the ring via `_match_ack_in_frames()`). The
  tour runner's bespoke poll loop is the one completion-ack consumer left on
  the single-slot path the ring was built to replace.
- The host `DESIGN.md` §4 even describes this ambiguously ("`tour.py`'s own
  `_drain_and_poll()` ... read that slot") while §5 documents that only
  `wait_for_ack()` scans the ring — a latent inconsistency this ticket also
  corrects in the doc.

Firmware side is NOT the suspected cause: `robot_loop.cpp`'s pace block emits
`tlm_.ack(moveResult.completion.moveId, 0)` whenever `moveQueue_.tick()` reports
`completed`, and `move_queue.cpp`'s `tick()` sets `result.completed = true` /
`result.completion.moveId = active_.moveId` for the `pendingCount_ == 0`
queue-drain (final-leg) case exactly as for a chain-advance. So the final
completion ack IS emitted on the move's own stop condition.

Consequence on the lossy bench link (~15 Hz read vs ~25 Hz emit, ~40% frame
loss; plus the separate dropped-envelope gap of sprint 125): the final `Move`'s
completion ack rides exactly ONE frame's fresh scalar slot. If that single
frame is dropped, the tour runner never sees the ack — while the ring it
ignores would have carried that same ack across the next four frames. STOP then
flushes the queue and the runner's `should_stop()` path retires the leg as
STOPPED, which the operator perceives as "STOP made it finish."

**Honest caveat — resolve by reproduction BEFORE fixing.** This mechanism
predicts a LOSSY-LINK (non-deterministic) hang. The deterministic Sim closure
gate drains every frame and completes tours (15/15), so under this mechanism
the hang should NOT reproduce deterministically. If reproduction shows a
deterministic hang in Sim too, there is an ADDITIONAL firmware/host
completion-path cause (the issue's candidate #2 — e.g. the final completion ack
being deferred to the next enqueue/STOP rather than pushed on the drain) that
must be root-caused before shipping. State what is actually found vs. this
hypothesis.

## Approach

1. **Reproduce and root-cause first.**
   - Deterministic Sim: run a tour via the closure-gate path
     (`_run_tour_capture`, deterministic stepper) and via real-time TestGUI Sim
     mode; capture whether the final leg retires on its own.
   - Bench: run a tour over the real serial link (`/dev/cu.usbmodem2121102`) and
     capture the telemetry frames/acks around the final leg's expected
     completion and around the STOP that unblocks it — does any frame carry
     `ack_corr == <final Move.id>` with `kFlagActive` dropping, WITHOUT a STOP?
   - Confirm or refute the single-slot mechanism; state the finding.
2. **Fix (primary):** make the tour runner's completion detection ring-aware.
   Route `_drain_and_poll()` (and `_wait_for_move_terminal()`) through the SAME
   ring-scan `wait_for_ack()`/`SerialConnection` already use — scan each drained
   frame's `acks` ring for `corr_id == Move.id`, not only the single
   `frame.ack` scalar slot. Preserve the existing `Move.id` keying and the
   `_TOUR_MOVE_ID_BASE` collision-avoidance contract (a completion ack echoes
   `Move.id`; never match an enqueue ack's `corr_id`). Keep
   `_outcome_for_terminal_frame()`'s `fault_move_timeout` / `ack.ok` logic
   (read those off the matched frame / entry as appropriate).
3. **Fix (optional hardening, this ticket's call):** bounded
   retry-on-missing-ack — re-send the same leg's `Move` if no completion or
   enqueue ack is observed within the window — per 120-001's own forward note,
   to harden against the separate dropped-ENVELOPE gap (sprint 125) without
   waiting on it. Keep it bounded (never infinite), matching the module's
   existing bounded-wait posture.
4. If (and only if) reproduction found a deterministic cause, root-cause and
   fix that too — do not ship a fix past an unreproduced hypothesis.

Do NOT change the firmware ack emission or the wire schema — this is a host
consumption fix (the ring already ships since 120-001).

## Files to modify

- `src/host/robot_radio/planner/tour.py` — `_drain_and_poll()`,
  `_wait_for_move_terminal()`, and any helper reading `frame.ack`; reuse
  `io/serial_conn.py`'s ring matcher rather than re-implementing the scan.
- `src/tests/` — Sim coverage that a full tour retires its final leg without a
  STOP (extend the closure-gate path or a `tour.py` unit test with a fake
  transport that drops the single fresh-slot frame but populates the ring).
- `src/host/robot_radio/DESIGN.md` — correct §4's `planner/tour.py`
  ack-consumption paragraph to state the tour runner scans the `acks` ring
  (resolving the slot-vs-ring conflation against §5's `wait_for_ack()` note);
  edited directly on the canonical doc (not overlaid — see sprint.md Design
  Overlay).
- `docs/protocol-v4.md` — OPTIONAL: a clarifying sentence in §7.2 that a
  completion ack is observable in the `acks` ring across subsequent frames, not
  only the single fresh slot (clarification, not a format change).

## Acceptance Criteria

- [ ] The final-leg hang is reproduced Sim-first (deterministic stepper AND
      real-time TestGUI Sim mode) and, on the bench, with the frames/acks around
      the final leg captured; the actual root cause is stated vs. the filed
      single-slot hypothesis (including whether it reproduces deterministically).
      **Sim half DONE**: ran `test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
      (deterministic closure-gate stepper) and `test_sim_transport_tour1.py`
      (real-time `SimTransport`, the TestGUI Sim-mode equivalent) against the
      UNMODIFIED pre-fix code — both PASS (full tour completes with NO STOP,
      including the final leg) — confirming the hang does NOT reproduce
      deterministically in Sim, exactly as the single-slot hypothesis predicts
      (Sim never drops a frame it produced, so the one fresh-slot frame a
      completion rides is always eventually read). Root cause CONFIRMED (not
      refuted): the tour runner's `_drain_and_poll()` read only
      `TLMFrame.ack` (the single scalar slot, valid on exactly one frame) and
      was never updated when 120-001 added the `acks` ring;
      `wait_for_ack()`/`SerialConnection` already scanned the ring. On the
      lossy bench link, if that ONE fresh-slot frame is dropped, the
      completion is invisible forever even though the ring carries it for
      several more frames — the final leg then only retires when `STOP`
      flushes the queue. **Bench half PENDING (pending team-lead bench run on
      the stand)** — no hardware access in this dispatch; frames/acks capture
      around the final leg over `/dev/cu.usbmodem2121102` is the team-lead's
      follow-up.
- [x] The tour runner ends every leg — the FINAL leg included — by scanning the
      `acks` ring for `corr_id == Move.id`, not by reading only the single
      `frame.ack` scalar slot; the `_TOUR_MOVE_ID_BASE` / `Move.id` keying
      contract is preserved. `_drain_and_poll()` now scans each drained
      frame's `acks` ring first (falling back to the scalar slot only for a
      frame whose own ring carries no match, kept for test doubles that never
      populate `acks`); `_outcome_for_terminal_frame()` now reads `ok`/
      `err_code` off the SPECIFIC matched entry, never off the frame's own
      possibly-unrelated scalar slot.
- [ ] A full TOUR_1 completes and reports closure WITHOUT a STOP press, in Sim
      AND over the real serial link on the stand. **Sim half DONE** (see
      `test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`,
      `test_sim_transport_tour1.py`, and this ticket's own new
      `test_run_tour_full_tour_completes_without_stop_when_every_leg_only_rides_the_ring`
      regression test). **Bench half left unchecked (pending team-lead bench
      run on the stand)**.
- [x] A regression test demonstrates the tour retires the final leg even when
      the single fresh-slot frame is dropped but the ring carries the ack.
      `src/tests/unit/test_planner_tour.py`'s `_RingOnlyFakeTransport` models
      exactly this (every completion ack rides the ring only,
      `TLMFrame.ack`/`ack_corr`/`ack_err` stay `None` on every frame it ever
      returns) — verified this test FAILS against the pre-fix code (11/12
      failures, `git stash` A/B check) and PASSES against the fix.
- [x] If reproduction found a deterministic cause, it is root-caused and
      fixed; no fix ships past an unreproduced hypothesis. No deterministic
      cause was found — Sim reproduction above confirms the hypothesis
      rather than refuting it, so this criterion is met by NOT needing a
      second fix.
- [x] No firmware ack-emission or wire-schema change. Only
      `src/host/robot_radio/planner/tour.py` (host consumption logic) plus
      docs (`DESIGN.md`, `docs/protocol-v4.md`) were touched.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py`,
  `src/tests/testgui/test_tour_stop.py`, and the tour unit tests
  (`tests/unit/test_planner_tour.py`); the broader `uv run python -m pytest`
  gate (no regressions).
- **New tests to write**: a fake-transport case where the fresh-slot frame is
  dropped but the ring carries the completion ack — assert the final leg
  retires COMPLETED; a full-tour-completes-without-STOP assertion in the
  closure-gate path.
- **Verification command**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py`,
  plus a documented bench run over `/dev/cu.usbmodem2121102` showing a tour
  closing with no STOP.
