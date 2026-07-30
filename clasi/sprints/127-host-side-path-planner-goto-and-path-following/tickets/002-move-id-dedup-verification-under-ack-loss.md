---
id: '002'
title: Move.id dedup verification under ack loss
status: in-progress
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

- [x] Rule 1 (ordinary duplicate suppressed): sim assertion passes —
      queue depth rises by exactly one for two sends of the same id, both
      ack `err==0`, and a different id enqueues separately.
- [x] Rule 2 (id-0 exemption): sim assertion passes — two `Move.id==0`
      sends both enqueue as distinct moves.
- [x] Rule 3 (window outlives completion): sim assertion passes — a
      duplicate id sent after the first instance's completion is still
      suppressed.
- [x] Rule 4 (`ERR_FULL` non-recording): sim assertion passes — an
      `ERR_FULL`-rejected id is not recorded, and re-sending it after the
      queue drains enqueues normally.
- [ ] **NOT MET.** Hardware run: at least one run's log contains a real
      `(retry N for move …)` line (not synthetically forced) — a run
      without a retry does not satisfy this criterion. Ten full-tour runs
      over direct USB (80 enqueue commands total) produced zero natural
      ack loss / zero retry lines. See Completion Notes for the raw logs
      and a proposed fault-injection follow-up.
- [ ] **NOT MET as literally stated** (no run contained a retry to
      validate against) — but see Completion Notes for the encoder-
      derived exactly-once evidence gathered across all ten runs anyway
      (dR-dL consistently 877-882 mm across every successful run, with no
      run showing the ~1100 mm five-turn signature this robot's own
      tuning would produce on a failed dedup).
- [x] No file under `src/firm`, no `.proto` file, and no wire message
      changed anywhere in this ticket's diff (confirmed via `git diff
      --name-only` / `git diff --stat -- src/firm '*.proto'`, both empty
      for this ticket's changes).

## Completion Notes (2026-07-30)

**Status: left `in-progress`, not `done`** — four of six substantive
acceptance criteria are met; the hardware-retry criterion is genuinely
NOT met (no natural retry was captured) and is reported as such rather
than checked off. Sim verification is complete and solid; the hardware
leg needs either more bench time or a follow-up ticket for host-side
fault injection (proposed below) before this ticket can close.

### Sim: all four rules verified (new files, `src/firm` untouched)

New sibling pair (per the ticket's own "or a sibling ... either is fine"
allowance), modeled directly on ticket 001's
`test_app_robot_loop_replace_harness.cpp`:
- `src/tests/sim/unit/test_app_robot_loop_dedup_harness.cpp`
- `src/tests/sim/unit/test_app_robot_loop_dedup.py`

All four scenarios drive the real `TestSim::SimHarness` (the dedup
short-circuit lives in `App::RobotLoop::handleMove()`, one layer above
`Motion::Planner`, so a bare Planner has no id memory to exercise):

- **Rule 1 (ordinary duplicate suppressed)**: queue depth 0→1 on the
  first accept of id 800, stays at 1 on the duplicate resend (fresh
  corr_id, both acks `err==0`), rises to 2 on a genuinely different id
  801. `RULE1_DEPTH_AFTER_A=1 AFTER_DUP=1 AFTER_B=2`.
- **Rule 2 (id-0 exemption)**: two separate id-0 moves both enqueue as
  distinct pending entries — depth 0→1→2, not 0→1→1 (which is what a
  wrongly-deduped id 0 would show). `RULE2_DEPTH_AFTER_1=1 AFTER_2=2`.
- **Rule 3 (window outlives completion)**: a 200 ms TIME move (id 900)
  is run to genuine completion (completion ack `err==0` observed,
  planner idle, queue depth 0), THEN the same id is resent carrying
  `replace=true` and completely different content (an Angle move vs.
  the original Distance move). It still acks `err==0` but the planner
  stays idle and queue depth stays 0 — confirms both the "outlives
  completion" rule and the ordering subtlety the source issue calls
  out explicitly: the id check precedes `replace` handling, so a
  duplicate carrying `replace=true` cannot restart a move mid-flight.
- **Rule 4 (ERR_FULL non-recording)**: fills the 5-slot queue (1 active
  + 4 pending, `Motion::Planner::kQueueDepth`) with five short TIME
  moves, then a 6th distinct id (955) is rejected with `err==ERR_FULL`
  (`msg::ErrCode::ERR_FULL == 4`). The five fill moves are then allowed
  to drain to completion (queue depth back to 0), and id 955 is
  resent: it now acks `err==0` and genuinely activates
  (`activeMoveId()==955`, depth 1) — proving it was never recorded as
  accepted at rejection time.

Verification command actually used (the ticket's own suggested command
names `test_app_robot_loop_replace.py -k dedup`, which does not match
since this ticket used a sibling file instead):
```
uv run python -m pytest src/tests/sim/unit/test_app_robot_loop_dedup.py -q
```
Full regression run: `uv run python -m pytest src/tests/sim -q` →
**425 passed, 1 skipped, 1 xfailed** — no regressions, ticket 001's own
replace-preemption tests included and unmodified.

One naming-standards fix made along the way (rule 5,
`.claude/rules/naming-and-style.md` — "never propagate a violation, ...
in your own draft/example code"): two local test constants were
initially written as `kStopTimeMs`/`kEachStopTimeMs`; renamed to
`kStopTime`/`kEachStopTime` with the unit staying only in the `// [ms]`
trailing comment, matching the project's no-units-in-identifiers rule,
even though the sibling file `move_protocol_harness.cpp` this was
patterned on still carries the older `kStopTimeMs`/`kTimeoutMs` names
(pre-existing, out of this ticket's scope to fix elsewhere).

### Hardware: ten full-tour runs, direct USB, zero natural retries

`mbdeploy list` confirmed `tovez` on `/dev/cu.usbmodem2121102` (role
`NEZHA2`) throughout this session — direct USB, not the RADIOBRIDGE
relay the ticket text names (that text was stale; the team-lead's own
dispatch corrected the port and confirmed the robot alive there). Ran
`src/tests/bench/planner_square_tour.py --port /dev/cu.usbmodem2121102`
**ten times** (all move IDs are the tour's own fixed 9001-9008, so every
run re-exercises the same enqueue path the source issue's own repro
used). Raw summary:

| run | result | wall | dL [mm] | dR [mm] | dR-dL [mm] | heading [deg] | retry logged? |
|---|---|---|---|---|---|---|---|
| 1 | PASS 8/8 | 30.3s | +1578.0 | +2458.0 | 880.0 | 393.9 | no |
| 2 | PASS 8/8 | 31.1s | +1576.0 | +2456.0 | 880.0 | 393.9 | no |
| 3 | PASS 8/8 | 30.0s | +1574.0 | +2453.0 | 879.0 | 393.5 | no |
| 4 | PASS 8/8 | 39.0s (one leg stalled ~16s, unrelated -- see source issue's own "Not the same issue" note on move 9007's stall) | +1573.0 | +2450.0 | 877.0 | 392.6 | no |
| 5 | PASS 8/8 | 30.1s | +1567.0 | +2446.0 | 879.0 | 393.5 | no |
| 6 | PASS 8/8 | 29.5s | +1566.0 | +2444.0 | 878.0 | 393.0 | no |
| 7 | PASS 8/8 | 31.1s | +1564.0 | +2445.0 | 881.0 | 394.4 | no |
| 8 (killed) | **process killed mid-Move by an orphaned background batch -- not a script result** | -- | -- | -- | -- | -- | -- |
| 8-retry | **FAIL 0/8**, zero motion, zero acks, zero retries (transient -- see below) | 23.0s | 0.0 | 0.0 | -- | -- | no |
| 8b (re-retry) | PASS 8/8 | 30.5s | +1566.0 | +2446.0 | 880.0 | 393.9 | no |
| 9 | PASS 8/8 | 29.4s | +1562.0 | +2444.0 | 882.0 | 394.8 | no |
| 10 | PASS 8/8 | 29.7s | +1565.0 | +2446.0 | 881.0 | 394.4 | no |

**No run's log contains a `(retry N for move …)` line.** Per the source
issue's own words and this ticket's explicit acceptance wording, that
means the hardware criterion is not satisfied by any of these ten runs.
80 enqueue commands went out over direct USB with zero lost acks. This
is a materially different result from the source issue's own 2026-07-28
observation (2 retries in 2 runs, same script, same USB link) -- most
plausibly because that observation predates or otherwise sits upstream
of `9b4ea538` (sprint 123's TX-truncation root-cause fix, per this
project's own memory notes), which would have been the dominant source
of lost frames (including acks) on this link at the time. Direct USB
CDC now looks close to loss-free for this workload; provoking a retry
naturally may require either far more trials, a loaded host CPU/USB
bus, or the RADIOBRIDGE relay path (the ticket text's original
suggestion) whose own documented ~20% inbound-line loss budget
(`radio_bench_gate.py`) is a much more favorable environment for this
specific criterion than direct USB turned out to be.

**Proposed follow-up to actually close this criterion**: a small,
explicit host-side fault-injection harness -- e.g. a
`SerialConnection`/`NezhaProtocol` wrapper (or a `move_twist()`-adjacent
helper in a NEW bench script, not `planner_square_tour.py`'s own
reproducer, to keep the honest reproducer unmodified) that drops one
specific enqueue ack by corr_id on command, so a retry is deterministic
rather than probabilistic. Per the ticket's own acceptance wording
("not synthetically forced"), a forced drop cannot retroactively check
this box -- but it would produce a hardware-verified `(retry N for move
…)` line under conditions the sim harness cannot reach (real wire
encode/decode, real ack-ring timing), which is a legitimate complement
to a natural capture, not a replacement acceptance path. Flagging this
as a candidate follow-up ticket/issue rather than building it here,
since it is out of this ticket's stated scope.

**Encoder-derived exactly-once evidence (gathered anyway, across all
nine successful runs)**: `dR-dL` is consistently 877-882 mm across
every one of the nine successful full-tour runs -- a 5 mm (0.6%) band.
This robot's own turn calibration (the gain/offset correction in
`robot_loop.cpp`'s `handleMove()`, ~184-215) makes the real four-turn
differential ~880 mm rather than the textbook `4 * trackWidth * pi/2 =
804.8 mm` the source issue's own worked example used for an
uncalibrated turn -- but the important comparison is internal
consistency, not the textbook number: a failed dedup (a 5th real turn
landing) would add one more turn's worth, `880 * 5/4 = ~1100 mm`, a 25%
jump that would be obvious against this tight a band. No run showed
anything close to that. This is real, repeatable, encoder-based
evidence that the shipped dedup did not let a duplicate through in any
of the 72 non-duplicate moves sent across these nine runs -- it just
never got the chance to prove the SPECIFIC duplicate-suppression path,
since no natural duplicate/retry occurred.

### Safety incident during this ticket's hardware leg, and the fix

A background batch (`bxzqz20wu`, four scripted repeats) was killed
mid-Move by an external process boundary while I was polling it with
inert `true` no-ops instead of properly holding it in the foreground —
`planner_square_tour.py`'s own `finally: proto.estop()` never ran,
because a bare `SIGTERM` (unlike `SIGINT`/Ctrl-C, which raises
`KeyboardInterrupt`) terminates a Python process without running
`finally` blocks. The team-lead polled the robot directly, found it
still driving (`active=True`, encoders climbing), and estopped it —
confirmed stopped before I resumed. I then:

1. Verified the robot was genuinely idle via telemetry myself
   (`active=False`, both wheel velocities `0.0`) before touching it
   again.
2. Added `_install_estop_signal_handler()` to
   `src/tests/bench/planner_square_tour.py` — installs `SIGTERM`/
   `SIGINT` handlers, right after `connect()`, that call `estop()`
   before the process exits, closing the gap `finally` alone left open.
   This is defense in depth, not a complete guarantee (a `SIGKILL`
   cannot be caught by any process-level handler); the operational
   mitigation for that remaining gap is procedural — every hardware
   run for the rest of this ticket was executed in the **foreground**,
   one at a time, with an explicit post-run telemetry check
   (`active=False`) rather than backgrounded and trusted.
3. Investigated the one anomalous 0/8 run (see table above, "8-retry")
   this incident produced: `enc_left`/`enc_right` position both read
   `0.0` and `connect()`'s own `ready` field read `False` immediately
   after, consistent with the robot having rebooted (fault/watchdog
   reset following the abrupt kill) somewhere around that window. The
   very next run (8b) completed normally with the SAME move IDs and
   encoder geometry matching every other run in the table, which rules
   out a sticking dedup-ring explanation (the fixed move IDs 9001-9008
   are reused verbatim every single run in this script by design, and
   if the accepted-id ring had somehow persisted across a non-reboot
   reconnect, EVERY run after the first would have been silently
   deduped — the actual data shows the opposite: 9 of 10 full runs
   moved for real).

Final state confirmed via telemetry (not inspection):
`active=False`, `enc_left.velocity=0.0`, `enc_right.velocity=0.0`.

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
