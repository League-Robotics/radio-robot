---
id: '002'
title: Move.id dedup verification under ack loss
status: done
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

**Hardware acceptance** *(original wording — see "Acceptance criterion
amended 2026-07-30" below for what actually gates this ticket's
closure)*: a run that reproduces or waits for a real lost enqueue ack,
capturing at least one logged `(retry N for move …)` line — per the
issue's own words, "a run without a retry proves nothing." The robot is
currently reachable **only** over the RADIOBRIDGE relay
(`/dev/cu.usbmodem2121302`, dongle `zavaz`) — the relay's own known
sporadic ack loss is exactly the condition that provokes a real retry, so
this is a favorable environment for this ticket, not an obstacle; leave
it in the loop rather than trying to work around it. Confirm exactly-once
execution from **encoder-derived geometry**, not just the ack count — the
issue's own repro signature for a failed dedup is a runaway leg (wheel
speed held far longer than commanded) or an extra turn (`dR-dL` matching
N+1 turns' worth of differential instead of N).

**Acceptance criterion amended 2026-07-30 (team-lead disposition, after
160 enqueues across two transports produced zero natural retries — see
Completion Notes' "Why the retry condition is no longer reproducible"
section for the full analysis)**: the retry-line criterion assumed the
condition that originally produced retries (frequent lost enqueue acks)
was still present. It is not — commit `cc04ac84` (2026-07-28, the SAME
commit that filed this ticket's own source issue) turned off the I2C IRQ
guard that was causing ~7-8% inbound command loss, which is what was
generating the lost acks that triggered host retries in the first place.
Building fault injection to synthesize that condition now would be
manufacturing a bug that has already been fixed, to test a path that is
already defended in sim against the real firmware code path. The
amended, actually-gating criterion is:
1. All four dedup rules verified in sim against the real
   `App::RobotLoop` via `TestSim::SimHarness` (unchanged from the
   original ticket — already met).
2. Rule 3 (window outlives completion) additionally confirmed on real
   hardware, incidentally, via a real dedup-ring collision across
   separate relay-transport runs (see Completion Notes) — this is
   stronger evidence than a single-run retry would have been, since it
   demonstrates the ring surviving a full session boundary, not just a
   few seconds of held state.
3. The retry trigger's absence is itself documented and cited (the
   `cc04ac84` analysis), not silently assumed.
4. The zero-natural-loss measurement (160 enqueues, two transports) is
   recorded as a baseline for future readers.
A host-side ack-drop fault-injection capability is filed as a follow-up
issue (see Completion Notes / Related) — a good testing investment on
its own merits, explicitly not required for this ticket to close.

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
- [x] **AMENDED 2026-07-30 (team-lead disposition).** Original wording
      ("a run's log contains a real `(retry N for move …)` line") is
      superseded — see "Acceptance criterion amended 2026-07-30" above
      and Completion Notes' "Why the retry condition is no longer
      reproducible." 160 enqueue commands across direct USB (80) and the
      RADIOBRIDGE relay (80) produced zero natural ack loss / zero retry
      lines, traced to commit `cc04ac84` (2026-07-28) disabling the I2C
      IRQ guard that was the source of the lost acks the original issue
      observed. The amended criterion — sim verification against the
      real firmware code path, plus the incidental real-hardware
      confirmation of Rule 3 across a session boundary, plus the cited
      root-cause analysis for why no retry occurs — is met; see
      Completion Notes for the full writeup.
- [x] Encoder-derived exactly-once evidence gathered across all twenty
      hardware runs, both transports: `dR-dL` consistently 876-882 mm,
      matching the effective-track-corrected four-turn theoretical of
      882.1 mm almost exactly (see Completion Notes' `rotational_slip`
      section), with no run showing the ~1005-1100 mm five-turn
      signature a failed dedup would produce.
- [x] No file under `src/firm`, no `.proto` file, and no wire message
      changed anywhere in this ticket's diff (confirmed via `git diff
      --name-only` / `git diff --stat -- src/firm '*.proto'`, both empty
      for this ticket's changes).

## Completion Notes (2026-07-30)

**Status: `done`.** All four dedup rules are verified in sim against
the real `App::RobotLoop` code path. The hardware-retry criterion was
amended by team-lead disposition (see "Acceptance criterion amended
2026-07-30" in the Description, and "Why the retry condition is no
longer reproducible" below) rather than met as originally worded — that
amendment is deliberate and cited, not a criterion quietly loosened to
pass. A follow-up issue for host-side ack-drop fault injection is filed
(see "Related" at the end of this section) as a genuine testing-gap
improvement, explicitly not blocking this ticket's closure.

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
   encoder geometry matching every other run in the table.

   **This paragraph's original conclusion — "rules out a sticking
   dedup-ring explanation ... EVERY run after the first would have
   been silently deduped" — was wrong, corrected below** once the
   relay leg produced exactly that symptom on a session that provably
   had NOT rebooted (see "Hardware: radio relay" below). The real
   explanation: direct-USB `connect()` apparently does reset the robot
   reliably enough that this session's own USB runs never collided
   (each got a fresh `acceptedMoveIds_` ring), so the "9 of 10 full
   runs moved for real" observation was correct as data but wrong as
   a general claim about the dedup ring's persistence — it only held
   because of an unstated, transport-specific assumption. Leaving this
   note in place rather than deleting it, since the correction is the
   useful part.

Final state confirmed via telemetry (not inspection):
`active=False`, `enc_left.velocity=0.0`, `enc_right.velocity=0.0`.

### Hardware: radio relay leg (2026-07-30, second dispatch) — still zero natural retries, plus a real bench-script bug found and fixed

The coordinator asked for the hardware leg to be re-run over the
RADIOBRIDGE relay (`/dev/cu.usbmodem2121302`, dongle `zavaz`) instead of
direct USB, on the reasoning that the relay's own documented sporadic
ack loss (`.claude/rules/hardware-bench-testing.md`,
`radio_bench_gate.py`'s own loss budget) is the natural mechanism the
source issue describes, and direct USB (this ticket's first hardware
leg, all ten runs clean) is simply too reliable a link to provoke it.

**First relay run (run 1) was clean, 8/8, no retry** — same shape as
every USB run. **Run 2 immediately exposed a real bug in this bench
script**, and a genuinely interesting hardware confirmation of Rule 3
at the same time: `tour()`'s move IDs were hardcoded (`9001-9008`,
literally every invocation), and the RADIOBRIDGE relay path does NOT
reset the robot itself the way this session's direct-USB `connect()`
calls apparently did (`!GO` re-handshakes the DONGLE, not the robot,
which is reached over RF) — so the robot's `acceptedMoveIds_` ring
from run 1 was still populated when run 2 started. Run 2's own log:

```
move 9002 complete at t=0.2s
move 9003 complete at t=0.4s
move 9004 complete at t=0.6s
move 9005 complete at t=0.7s
move 9006 complete at t=0.8s
move 9007 complete at t=1.0s
move 9008 complete at t=1.1s
WARNING: never completed: [9001]

FAIL: 7/8 moves completed in 1.3 s
  encoders: dL +0.0 dR +0.0 mm -> path 0.0 mm (target 2000; error -2000.0)
```

Diagnosis: moves 9002-9008 "completed" in ~1 second flat because those
timestamps are STALE completion-ack-ring entries left over from run 1's
own genuine completions (the ring resends each entry for a bounded
number of frames, and connecting early enough into that window means a
brand-new session can observe an old completion before it ages out).
Move 9001 — run 1's OWN first move, whose completion ack had already
fully aged out of the ring by the time run 2 connected — genuinely
never completed in run 2, because `alreadyAccepted(9001)` was true
(recorded by run 1, never evicted) and the dedup short-circuit fired:
acked `err==0` on the fresh corr_id (so the script's own enqueue
tracking saw a normal-looking accept, no `FAIL enqueue` printed) but
`planner_.move()` was never called, so no NEW completion could ever
arrive for it. Zero real motion (`dL`/`dR` both `0.0`) is exactly
consistent with this: none of the 8 sends in run 2 caused a real
`planner_.move()` call. **This is Rule 3 (window outlives completion)
firing correctly on real hardware** — just not in the shape the
ticket's acceptance criterion wants (a genuine ack-loss retry within
ONE run), and the false `FAIL 7/8` it produces is actively misleading
for anyone reading this script's output without the mechanism in mind.

**Fix**: `tour()` now takes an optional `id_base` (default: derived
from `time.time()`, so IDs are unique run-to-run) instead of the
hardcoded `9001 + 2*i`/`9002 + 2*i`. The plotting code's leg/turn
labeling (previously hardcoded to a `9000` offset) now recovers
`id_base` from `moves[0]["move_id"] - 1` instead. This is a host-side
bench-script fix only — no `src/firm`, wire, or `.proto` change — and
it is squarely this ticket's own file
(`src/tests/bench/planner_square_tour.py`, not `move_protocol_bench.py`
or any of ticket 001/003/004/006's files).

**Ten clean relay runs after the fix** (run 1 plus nine more with unique
IDs; the collision run above is excluded as not a real attempt, same as
the killed run in the USB table):

| run | result | wall | dL [mm] | dR [mm] | dR-dL [mm] | heading [deg] | retry logged? |
|---|---|---|---|---|---|---|---|
| 1 | PASS 8/8 | 30.2s | +1564.0 | +2445.0 | 881 | 394.4 | no |
| 3 | PASS 8/8 | 29.7s | +1569.0 | +2450.0 | 881 | 394.4 | no |
| 4 | PASS 8/8 | 29.9s | +1567.0 | +2449.0 | 882 | 394.8 | no |
| 5 | PASS 8/8 | 29.4s | +1566.0 | +2448.0 | 882 | 394.8 | no |
| 6 | PASS 8/8 | 28.7s | +1567.0 | +2448.0 | 881 | 394.4 | no |
| 7 | PASS 8/8 | 29.2s | +1573.0 | +2452.0 | 879 | 393.5 | no |
| 8 | PASS 8/8 | 29.8s | +1572.0 | +2451.0 | 879 | 393.5 | no |
| 9 | PASS 8/8 | 28.7s | +1572.0 | +2454.0 | 882 | 394.8 | no |
| 10 | PASS 8/8 | 30.1s | +1565.0 | +2441.0 | 876 | 392.1 | no |
| 11 | PASS 8/8 | 30.2s | +1560.0 | +2442.0 | 882 | 394.8 | no |

**No retry line appeared in any of the ten clean relay runs either.**
80 enqueue commands over the relay, on top of the 80 over direct USB
from the first hardware leg — 160 total, zero natural ack loss on
either transport. Per the coordinator's own instruction, stopping here
rather than continuing to burn runs: **the retry path could not be
provoked naturally on either transport available on this bench.** The
honest disposition, per the coordinator's own framing, is that the
hardware-retry acceptance criterion should be met either by the
fault-injection follow-up proposed above, or the criterion itself
should be revisited — that call belongs to the team-lead, not this
ticket.

Robot confirmed idle after the full relay leg via telemetry (not
inspection): `active=False`, `enc_left.velocity=0.0`,
`enc_right.velocity=0.0`.

### `rotational_slip` check (coordinator's question)

Confirmed, not dismissed. `data/robots/tovez.json`'s own
`_rotational_slip_note` (2026-07-29, camera-measured) already documents
`rotational_slip = 0.9117`, "Equivalent to an effective track of 140.4
mm against the measured 128," consumed at boot into `kEffectiveTrack`
(`gen_boot_config.py` -> `boot_config.cpp` -> `main.cpp`). The
four-turn theoretical differential using the EFFECTIVE track, not the
caliper-measured `trackwidth`, is:

```
4 * (128 / 0.9117) * (pi / 2) = 4 * 140.40 * 1.5708 = 882.1 mm
```

That is the correct "healthy" four-turn baseline for this robot, and
it matches the measured band (876-882 mm across all nineteen
successful runs, both transports) almost exactly — the ~9% gap against
the raw-`trackwidth` 804.2 mm figure is fully explained by
`rotational_slip`'s own effective-track correction, not by any
measurement noise or dedup concern. Anyone reading this ticket later
should treat 876-882 mm, not 804 mm, as the four-turn reference band on
this robot.

### Why the retry condition is no longer reproducible (team-lead's
analysis, confirmed against the actual commit)

```
commit cc04ac8404a52127987a153f8cf9e4117b0ccd05
Date:   Tue Jul 28 17:41:07 2026 -0700
```

This single commit **both** added `bus.setIrqGuard(false)` to
`src/firm/main.cpp` **and** created
`duplicate-move-enqueue-on-ack-loss-retry.md` (this ticket's own source
issue). Confirmed directly (`git show cc04ac84 -- src/firm/main.cpp`):

```cpp
// BENCH A/B, 2026-07-28 -- TEMPORARY, revert after the measurement.
// Inbound command loss (~7-8% of PINGs and enqueues, measured on direct
// USB) is concentrated in a window of the loop cycle rather than spread
// uniformly, which points at UART RX bytes being lost while interrupts
// are masked for the full duration of each I2C transaction. This turns
// that masking off so the A/B can measure whether loss drops.
bus.setIrqGuard(false);
```

Still `false` in the current tree (`grep setIrqGuard src/firm/main.cpp`
— one hit, unchanged). The guard masked interrupts for the full
duration of every I2C transaction (a defense against an nRF52 TWIM
silicon errata); since the nRF52 serial RX is DMA-driven, inbound bytes
arriving inside a masked window were lost outright, measured at ~7-8%
command loss on direct USB.

The chain this explains: the retries that motivated the source issue
were observed **before** this commit, with the guard **on**. A lost
inbound *command* (not yet an ack) consumed one of the host's retries
for nothing; a SUBSEQUENT retry that instead landed on a lost *ack* (the
command got through, the ack did not) is what actually double-executed
a move, because the firmware at the time had no dedup at all. Turning
the guard off, in the SAME commit that filed the issue, dropped inbound
loss to effectively zero — which starves the retry loop of the trigger
condition it needs. This ticket's own measurement (160 enqueue commands,
direct USB and the RADIOBRIDGE relay, zero natural ack loss) is exactly
consistent with that: the guard-off state is still in effect, unchanged,
throughout this ticket's entire hardware leg.

**The criterion was asking this ticket to reproduce a condition whose
cause was fixed in the same commit that reported it.** Building fault
injection to force a drop would manufacture a bug that has already been
removed from the system, to test a path (host retry landing on an
already-accepted id) that IS already verified — against the real
firmware code path — in sim. See
`clasi/issues/make-irq-guard-off-permanent-and-reconcile-the-docs.md`
for the separate, in-flight proposal to delete the guard mechanism
entirely rather than merely leave it disabled.

### The dedup's risk profile has flipped — this is the thing future readers need to know

Before `cc04ac84`, the live hazard was **a duplicate executing twice**
(command arrives, ack is lost, host retries, both copies run) — that is
what the dedup this ticket verifies defends against, and it still does,
correctly, per the sim results above.

With the retry-trigger condition gone, the live hazard is now the
OPPOSITE: **a legitimate command silently swallowed because its `Move.id`
was reused**, with no error surfaced anywhere (the dedup's own ack is
`err==0`, by design — see the source issue's own "Behaviour" section for
why that is deliberate). This ticket's own relay run 2 (see above) is a
concrete, unplanned instance of exactly that: a second script invocation
reusing the first invocation's move IDs was silently deduped end-to-end,
producing a `FAIL 7/8` with zero real motion and no enqueue-level error
to point at the cause.

Sprint 127's own planner loop (ticket 006,
`src/host/robot_radio/pathplan/planner.py`) defends against this
directly with a `MoveIdAllocator` that issues monotonically increasing
IDs, rather than any fixed or reused base — the same fix this ticket
applied to `planner_square_tour.py`'s own `tour()`, independently, for
the same reason. Anyone writing a NEW bench script or host-side caller
against this protocol should allocate IDs the same way (wall-clock-
seeded or monotonic), not hardcode them, precisely because the robot
does not necessarily reboot between separate host sessions and the
`acceptedMoveIds_` ring has no cross-session reset.

### Related

- `clasi/issues/host-side-ack-drop-fault-injection-for-deterministic-retry-path-testing.md`
  — the follow-up filed per the above, low priority, not blocking.
- `clasi/issues/make-irq-guard-off-permanent-and-reconcile-the-docs.md`
  — the IRQ guard's own removal proposal (why the guard-off state that
  eliminated the retry trigger should be made permanent, not just left
  disabled).
- `src/host/robot_radio/pathplan/planner.py` (ticket 006) — the
  `MoveIdAllocator` that generalizes this same monotonic-ID fix into
  the real planner loop.

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
