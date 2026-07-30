---
id: '002'
title: Move.id dedup verification under ack loss
status: open
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: duplicate-move-enqueue-on-ack-loss-retry.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Move.id dedup verification under ack loss

## Description

`duplicate-move-enqueue-on-ack-loss-retry.md` reported the firmware
double-executing a move when a host retry (fresh `corr_id`, same
`Move.id`) followed a lost enqueue ack. The fix already shipped in
`src/firm/app/robot_loop.{h,cpp}` (sprint 126 side effect, confirmed
present at `robot_loop.cpp:224-248`: `alreadyAccepted()` / `recordAccepted()`
/ `acceptedMoveIds_[16]`, checked before `drive_.estop()` and before
`planner_.move()`, id 0 exempt, `ERR_FULL` not recorded) but has **never
been verified** — no sim test, no hardware run with a logged retry. This
ticket closes that gap. It reuses ticket 001's sim-unit-harness and
`move_protocol_bench.py` infrastructure rather than building a second copy.

**Hard constraint**: this ticket verifies existing, already-shipped
firmware behavior. It must not modify any file under `src/firm`, any wire
message, or any `.proto` definition — not even the dedup code itself,
however tempting a "small tweak" looks once you're reading it. If a test
here reveals the shipped behavior is wrong, stop and flag it (a new
issue), do not fix it in this ticket.

**The four dedup rules to verify** (per the issue's own Verification
section), each as an independent sim assertion:

1. **Ordinary duplicate suppressed**: enqueue the same non-zero `Move.id`
   twice; planner queue depth rises by exactly one; both acks `err==0`; a
   *different* id still enqueues normally.
2. **`Move.id == 0` exemption**: two moves both carrying `Move.id == 0`
   both enqueue (dedup must never fire on the "unset" sentinel).
3. **Window outlives completion**: a duplicate id arriving *after* the
   first move has already completed is still suppressed (the accepted-id
   ring is not evicted on queue-exit).
4. **`ERR_FULL` non-recording**: a move rejected with `ERR_FULL` is not
   recorded as accepted; re-sending the same id after the queue drains
   enqueues normally (the host is entitled to retry it for real).

**Hardware acceptance**: a run that reproduces or waits for a real lost
enqueue ack, capturing at least one logged `(retry N for move …)` line —
per the issue's own words, "a run without a retry proves nothing." The
robot is currently reachable **only** over the RADIOBRIDGE relay
(`/dev/cu.usbmodem2121302`, dongle `zavaz`) — the relay's own known
sporadic ack loss is exactly the condition that provokes a real retry, so
this is a favorable environment for this ticket, not an obstacle; leave
it in the loop rather than trying to work around it. Confirm exactly-once
execution from **encoder-derived geometry**, not just the ack count — the
issue's own repro signature for a failed dedup is a runaway leg (wheel
speed held far longer than commanded) or an extra turn (`dR-dL` matching
N+1 turns' worth of differential instead of N).

**Files**:
- Modify: `src/tests/sim/unit/test_app_robot_loop_replace.py` (or wherever
  ticket 001 lands its harness) — add the four dedup assertions as
  additional test cases in the same harness, or a sibling
  `test_app_robot_loop_dedup.py` if that reads more cleanly; either is
  fine, do not fork a second harness.
- New/modify: a bench script run for the hardware leg — reuse
  `src/tests/bench/move_protocol_bench.py` or
  `src/tests/bench/radio_bench_gate.py` (both already carry narrative
  comments about `alreadyAccepted()` and move-id collision hazards; this
  ticket is what finally backs those comments with a real assertion).

**Coding standards**: lowerCamelCase functions/variables, UpperCamelCase
types; no units in identifiers (a retry count is `retryCount`, not
`retry_n`).

**Safety obligations** (`.claude/rules/playfield-testing.md`,
`.claude/rules/hardware-bench-testing.md`): this ticket's hardware leg
runs on the **stand**, not the camera-covered playfield, so the
lights/geofence/per-boundary-camera-fix obligations do not apply (no
camera in this ticket's loop). What does carry over unconditionally:
every connection path calls `estop()` (never `stop()`) in a `finally`
block. Confirm the RADIOBRIDGE role via `mbdeploy list` before running —
the robot is on battery and not on its own USB port right now.

## Acceptance Criteria

- [ ] Rule 1 (ordinary duplicate suppressed): sim assertion passes —
      queue depth rises by exactly one for two sends of the same id, both
      ack `err==0`, and a different id enqueues separately.
- [ ] Rule 2 (id-0 exemption): sim assertion passes — two `Move.id==0`
      sends both enqueue as distinct moves.
- [ ] Rule 3 (window outlives completion): sim assertion passes — a
      duplicate id sent after the first instance's completion is still
      suppressed.
- [ ] Rule 4 (`ERR_FULL` non-recording): sim assertion passes — an
      `ERR_FULL`-rejected id is not recorded, and re-sending it after the
      queue drains enqueues normally.
- [ ] Hardware run: at least one run's log contains a real
      `(retry N for move …)` line (not synthetically forced) — a run
      without a retry does not satisfy this criterion.
- [ ] That run's encoder-derived move count/geometry confirms exactly-once
      execution (no runaway leg, no extra turn) — raw encoder data
      included in the ticket's Completion Notes, not just a summary
      verdict.
- [ ] No file under `src/firm`, no `.proto` file, and no wire message
      changed anywhere in this ticket's diff.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (confirm no regression); ticket 001's replace-preemption sim tests still
  pass unmodified.
- **New tests to write**: the four dedup-rule assertions in the sim unit
  harness (see Files above).
- **Verification command**:
  `uv run python -m pytest src/tests/sim/unit/test_app_robot_loop_replace.py -q -k dedup`
  and, on the stand (confirm ROLE with `mbdeploy list` first):
  `uv run python src/tests/bench/move_protocol_bench.py --port /dev/cu.usbmodem2121302`
  (repeat as needed until a run logs at least one real retry).
