---
status: pending
review: docs/code_review/2026-07-01-full-codebase-review.md
findings: CR-09, CR-10
severity: medium
sprint: '066'
---

# TestGUI traces: encoder-reset heuristic misses resets on slow TLM; otos/fused traces rotated when anchored mid-session

## Problem

**(a) Reset heuristic misses late TLM frames (CR-09).** The (uncommitted)
encoder-reset detector in `traces.py` recognizes a firmware encoder zeroing
only when the incoming frame reads within 20 mm of zero on both wheels
([traces.py:88-95](../../host/robot_radio/testgui/traces.py),
[traces.py:334-350](../../host/robot_radio/testgui/traces.py)). Over the
relay, TLM arrives at ~1–2 Hz while the robot moves 100–200 mm between
frames, so the first post-reset frame often exceeds 20 mm — the reset is
missed and integrated as spurious reverse motion whose `(dR−dL)` cancels the
just-turned heading. I.e. the original "encoder track ignores turns / drifts
into a corner" bug survives on exactly the transport (relay/playfield mode)
where it matters. Genuine return-through-zero can also false-positive.
[tests/testgui/test_traces.py:176-231](../../tests/testgui/test_traces.py)
covers only the prompt-reset case.

**(b) otos/fused traces assume the firmware frame was zeroed at anchor
(CR-10).** `_feed_otos`/`_feed_fused` rotate firmware world-frame deltas by
the **anchor yaw**
([traces.py:371-403](../../host/robot_radio/testgui/traces.py)) — correct
only when the firmware pose was freshly re-referenced (heading 0) at anchor
time. Anchoring mid-session leaves those traces rotated by the firmware
heading at baseline relative to the camera trace.

## Fix direction

- Stop inferring resets from data: rebaseline on command boundaries (the GUI
  knows when it sends `D`/`ZERO enc`), or better, add a firmware reset
  counter/epoch to the TLM `enc=` field so any consumer rebaselines exactly.
- Rotate otos/fused deltas by `(anchor_yaw − firmware_heading_at_baseline)`;
  the baseline tuples already carry `hdg_cdeg` (currently unused).

## Acceptance / tests

- test_traces: delayed-TLM reset scenario (first post-reset frame at
  e.g. 150 mm) preserves accumulated heading.
- test_traces: anchor with non-zero firmware heading — otos/fused traces
  align with the camera trace.
