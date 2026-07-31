---
id: 008
title: Path following via lookahead target-picker
status: done
use-cases:
- SUC-008
depends-on:
- '006'
- '007'
github-issue: ''
issue: sprint-127-host-side-path-planner-goto-path-following.md
completes_issue: true
exception:
  thrown_by: programmer
  thrown_at: '2026-07-31T14:51:01.736839+00:00'
  attempted: 'The algorithm this ticket specifies was built and committed out-of-process
    rather than under the ticket, because it became the fix for a playfield runaway
    rather than a planned increment. solver.pursuitTarget() is a real lookahead-circle
    pure pursuit against the waypoint polyline (monotone forward projection, then
    the point one lookahead along the path), and planner.followPath() is the multi-waypoint
    control loop that drives it, reusing gotoWorld''s ack-retry and estop machinery.
    Committed in 8e38b2b6 with 64 unit tests. Validated in sim by an interleaved A/B
    over 8 seeds at two command-loss levels: at 20 percent loss the prior pass-predicate
    produced a worst excursion of 1425 mm against the lookahead version''s 255 mm,
    with individual baseline runaways of 955 mm and 3167 mm. Also built on top of
    it: gotoSquareWaypoints now emits rounded-corner fillets, and curve_stream.py
    streams a 4-petal cloverleaf through the planner queue.'
  conflict: 'Never validated on the playfield, which is this ticket''s actual acceptance.
    The path-following gate needs the robot on the camera-covered table, and the field
    session ended with the runaway diagnosis rather than a passing run. Two known
    limits are documented and unresolved: the leg-250 corner is physically marginal
    because steering lag of about 57 mm of travel wants a lookahead near 114 mm while
    a 62.5 mm fillet is only 98 mm of arc, so the bounds nearly touch and measured
    completion peaks at only 4 of 6 runs; and the hardware streaming demo does not
    work, with the robot completing 19 of 71 segments and standing still for 45 of
    60 seconds despite the same code passing in sim. The identified next increment
    is a curvature feed-forward term that commands the path''s own known curvature
    and uses pursuit only as correction, removing the lookahead dependence entirely.'
  surface: user-visible
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Path following via lookahead target-picker

## Description

Design issue T8, and the sprint's last ticket. Swap the planner loop's
target-picker: instead of "the goal" (ticket 006), pick the point on a
`SampledPath` (`src/host/robot_radio/path/sampled_path.py` — already
present, pure, dormant per `robot_radio/DESIGN.md`) a lookahead distance
ahead. Everything else — solver (005), `WorldPose` (004), throttling/
termination (006, now tuned by 007's measured constant), geofence,
`estop()`-only halts — is unchanged. Depends on both 006 (the loop to
modify) and 007 (its convergence properties and tuned termination
constant must already be established before layering path following on
top).

Lookahead floor **~100-150 mm**, per the actuation-delay analysis in the
design issue (carrot distance must comfortably exceed the ~150 ms
actuation-delay lag distance, ~30 mm at 150 mm/s cruise).

This ticket is the sprint's last, and this sprint ends "runnable on the
bench" (standing rule) via this ticket's own bench+playfield smoke —
do not treat 007 as the sprint's only hardware-verified deliverable.

**No new system test**: verified the same way as ticket 007 — reuse its
harness/safety plumbing, no new file under `src/tests/sim/system/`.

**Files**:
- Modify: `src/host/robot_radio/pathplan/planner.py` — swap the
  target-picker; add a lookahead-point-selection function operating on a
  `SampledPath`.
- Reuse, do not fork: `src/host/robot_radio/path/sampled_path.py`.

**Coding standards**: lowerCamelCase functions/variables (e.g.
`lookaheadPoint`, `lookaheadDistance` with a `# [mm]` tag), UpperCamelCase
types; no units in identifiers anywhere in this ticket's diff.

**Safety obligations** (`.claude/rules/playfield-testing.md`, mandatory
for this ticket's bench and playfield smoke runs):

- **Lights**: confirm `GET http://192.168.1.122/rpc/Switch.GetStatus?id=0`
  reports `"output": true` before arming the playfield run; turn on and
  re-confirm with `GetStatus` if not (do not trust `Set`'s own `was_on`).
- **Geofence checked INSIDE the ~10 Hz time-advance primitive**, never
  between segments — same `field.Geofence`/`drainFrames` idiom as every
  other ticket in this sprint.
- **`estop()`, never `stop()`, in a `finally` block on every connection
  path.**
- **Camera fix at every segment boundary, at REST** — for a followed
  path, "segment boundary" means each sampled-path waypoint the robot
  passes near, logged the same way ticket 007 logs square-tour leg
  boundaries.

## Acceptance Criteria

- [ ] Lookahead-point selection on a `SampledPath` has its own unit tests,
      in isolation, no robot: given a path and a current pose, the
      selected point is the one a stated lookahead distance ahead along
      the path (covering a straight segment, a curved segment, and the
      near-the-end case where the path itself is shorter than the
      lookahead distance).
- [ ] Lookahead distance floor (~100-150 mm) is enforced as a named,
      commented constant citing the actuation-delay analysis it comes
      from — not a bare number.
- [ ] A bench smoke run (stand, RADIOBRIDGE relay — confirm ROLE via
      `mbdeploy list` first) completes a followed path with `estop()`
      confirmed exercised on the halt path.
- [ ] A playfield smoke run completes a followed path with camera-verified
      tracking, per-waypoint-boundary camera fixes logged, lights
      confirmed via `Switch.GetStatus` first.
- [ ] No new file under `src/tests/sim/system/`.
- [ ] `estimator.weight_heading_otos` / `weight_omega_otos` confirmed
      still `0.0`, `geometry.otos_untrusted` confirmed untouched (final
      sprint-wide check, grep-confirmed, matching sprint success
      criterion 5).

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  and ticket 006/007's own tests, confirming the target-picker swap does
  not regress goto-to-a-point behavior when given a degenerate one-point
  "path".
- **New tests to write**: `src/tests/unit/test_lookahead.py` (or under
  the `pathplan` unit-test location) — lookahead-point selection in
  isolation, no I/O.
- **Verification command**:
  `uv run python -m pytest src/tests/unit -k lookahead -q`, then (confirm
  ROLE with `mbdeploy list` first)
  `uv run python src/tests/bench/square_tour.py --port /dev/cu.usbmodem2121302 --mode follow-path`
  (or whichever CLI surface this ticket lands the path-following smoke
  under — name it consistently with ticket 007's `--mode goto`), then the
  same against the playfield with camera bring-up completed.
