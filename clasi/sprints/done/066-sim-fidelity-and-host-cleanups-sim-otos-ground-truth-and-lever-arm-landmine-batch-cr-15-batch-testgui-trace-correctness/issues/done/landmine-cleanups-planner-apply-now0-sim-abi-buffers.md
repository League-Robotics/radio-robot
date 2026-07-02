---
status: done
review: docs/code_review/2026-07-01-full-codebase-review.md
findings: CR-11, CR-12, CR-13, CR-14
severity: medium
sprint: '066'
tickets:
- 066-002
---

# Landmine cleanups: Planner::apply now=0, OdomTracker conventions, sim-ABI global clock, SimConnection buffer

## Problem

Four medium-severity latent defects that will bite future work if left.

**(a) `Planner::apply()` passes `now = 0` into `begin*()` (CR-11).**
[Planner.cpp:368-380](../../source/superstructure/Planner.cpp) hard-codes
`now = 0`, which becomes `MotionBaseline.t0Ms = 0`
([MotionCommand.cpp:98-116](../../source/commands/MotionCommand.cpp)); every
TIME stop then computes elapsed = full uptime and fires instantly once uptime
exceeds the timeout. Currently unreachable (BusDrain's PLANNER verb is a
no-op placeholder, [BusDrain.cpp:73-83](../../source/robot/BusDrain.cpp)),
but whoever finishes the PlannerCommand encoding trips this immediately and
mysteriously. Fix now (thread a real timestamp, or capture the baseline on
first `tick()`), or leave a loud comment + failing-by-construction test.

**(b) `OdomTracker` world transform is an untested convention stack (CR-12).**
[odom_tracker.py:277-310](../../host/robot_radio/sensors/odom_tracker.py)
treats TLM pose as "x=right, y=forward" with a CW-positive world yaw;
firmware pose is world-frame, heading 0=+X CCW. The composition happens to be
a proper rotation, but nothing anchors it to the aprilcam world convention
(A1-centred, +x east, +y north) — the exact "guessed geometry" class behind
past incidents. Add a convention test (anchor at a known camera pose, feed a
straight-ahead TLM track, assert world track matches) or retire the class if
nothing consumes it.

**(c) Sim C-ABI global clock / thread-safety (CR-13).** `g_sim_now_ms` is
process-global and `sim_create()` resets it to 0
([sim_api.cpp:39-44](../../tests/_infra/sim/sim_api.cpp),
[187-195](../../tests/_infra/sim/sim_api.cpp)); a second SimHandle (e.g. GUI
reconnect racing a slow-exiting tick thread — `SimTransport.disconnect` gives
up joining after 3 s) yanks the clock backwards for the still-live instance,
corrupting watchdog/TIME-stop deltas. The shared `replyStore` is also
unsynchronized. Move the clock into SimHandle; document/assert single-thread
usage.

**(d) `SimConnection._raw_command` truncates replies at 512 bytes (CR-14).**
[sim_conn.py:333-337](../../host/robot_radio/io/sim_conn.py) — reply store is
2048 (matched by firmware.py); long replies (`GET CFG`) silently cut. Use a
2048-byte buffer.

## Acceptance

- (a) A PlannerCommand-path timed motion runs its full duration in a test,
  or a guard test documents the landmine.
- (b) Convention test exists or OdomTracker is removed from the public API.
- (c) Two sequential Sim instances in one process no longer interact; clock
  is per-handle.
- (d) `GET CFG` via SimConnection returns complete output.
