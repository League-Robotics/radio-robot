---
id: '001'
title: "Behavior-lock acceptance harness (D700 straight + 360\xB0 pivot)"
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: motion-control-terminal-blips-reconciled-fix-plan.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Behavior-lock acceptance harness (D700 straight + 360° pivot)

## Description

Build a numeric acceptance harness that drives a D700 straight and a 360°
pivot through the REAL `App::RobotLoop` / `App::Pilot` / `Motion::Executor`
stack, captures the per-cycle wheel-velocity trace, numerically
differentiates it for acceleration/jerk, and asserts limits and
single-lobe shape. This is Step 0 of the driving issue
(`clasi/issues/motion-control-terminal-blips-reconciled-fix-plan.md`) —
"land a numeric jerk / single-lobe acceptance test first so every
subsequent deletion is guarded." Today's patch stack (straight-lead
padding, terminal top-up, pivot overshoot lead, the min-speed floor —
none of which this ticket touches) cannot satisfy most of these
assertions; mark those `xfail(strict=False)` citing the driving issue so
sprint 2 has a byte-for-byte regression fence to flip green. This ticket
also adds a same-boot repeated-move scenario targeting the driving
issue's §1.8/F7 stale-executor-state reliability finding.

`completes_issue: false` — this ticket is one of four implementing the
same sprint issue; only the last ticket to land should trigger archival
(set on ticket 004, the final ticket in dependency order — see that
ticket's own frontmatter).

## Acceptance Criteria

- [x] A new harness (`src/tests/sim/system/behavior_lock_harness.cpp`)
      compiles and links against the real `App::RobotLoop`/`App::Pilot`/
      `Motion::Executor` graph via `TestSim::SimHarness`, mirroring
      `move_queue_harness.cpp`'s own source list and compile flags
      (`test_move_queue.py`'s `_all_sources()`/`_compile_harness()` shape).
- [x] The harness commands a D700 `kArc` straight (`injectMove(distance=700,
      deltaHeading=0, ...)`) and a 360° `kPivot`
      (`injectMove(distance=0, deltaHeading=2*pi, ...)`), each to
      completion, capturing `Telemetry::Frame.velLeft`/`velRight` every
      cycle via `drainTelemetry()`.
- [x] Each trace is numerically differentiated (finite difference) into
      per-cycle acceleration and jerk, for both wheels.
- [x] Assertions exist for: velocity/accel/jerk within `PlannerConfig`'s
      own configured limits at both the ramp-up and the terminal decel;
      exactly one velocity lobe per wheel for the straight; one +lobe and
      one -lobe per wheel for the pivot.
- [x] A separately-named, separately-marked assertion exists: "no nonzero
      command survives past the terminal zero" (once both wheels first
      reach 0 after the command's own completion event, no cycle before a
      NEW command sees a nonzero `velLeft`/`velRight`). This is the one
      ticket 003 flips from xfail to passing — keep it distinguishable
      from the hump/tail-shape assertions in the harness's own output.
- [x] Every assertion the CURRENT code cannot satisfy is marked
      `xfail(strict=False)` with a comment/reason string citing
      `motion-control-terminal-blips-reconciled-fix-plan.md`. Do not use
      `strict=True` — the fence tolerates today's known-bad numbers
      without pinning exact values; a marker flipping to green is the
      signal sprint 2 needs, not a byte-exact match.
- [x] A same-boot scenario reuses ONE `SimHarness` instance across dozens
      (30-50) of consecutive alternating straight/pivot Move commands
      (no reboot between them, unlike `turn_windage_sweep.py`'s
      deliberate per-run isolation) and asserts every one reaches
      `ACK_STATUS_DONE` — its outcome (pass, or an honest, cited finding
      if it reproduces the §1.8/F7 stale-executor-state bug) is recorded
      in this ticket's own completion notes.
- [x] `uv run pytest` collects and runs the new harness with no collection
      errors; the overall suite stays green (pass or intentional,
      cited xfail).

## Implementation Plan

**Approach**: new harness module, following the exact established shape
of `move_queue_harness.cpp`/`test_move_queue.py` and
`heading_source_harness.cpp`/`test_heading_source.py` (both under
`src/tests/sim/system/`) — a `TestSim::SimHarness`-driven C++ binary
compiled per-test via `subprocess` from a Python pytest file, printing a
human-readable trace and using the same `beginScenario()`/`fail()`/
`checkTrue()` helper idiom those files already use.

1. `TestSim::SimHarness::injectMove(distance, deltaHeading, vMax, omega,
   timeMs, replace, id, corrId)` — use `timeMs=0`/`replace=false` for a
   fresh DISTANCE-mode command in each case. Read `PlannerConfig`'s
   actual configured limits (`a_max`/`a_decel`/`v_body_max`/`j_max` for
   the straight; `yaw_acc_max`/`yaw_rate_max`/`yaw_jerk_max` for the
   pivot) from the SAME default config the harness boots with — do not
   hand-duplicate numeric limits in the test; read them from the
   `msg::PlannerConfig` the harness constructs (avoids drift if a
   default ever changes).
2. Step the sim (`sim.step(1)` per cycle) until the injected Move's own
   completion event drains via `sim.drainTelemetry()` (mirror
   `move_queue_harness.cpp`'s `findAck()` helper), recording
   `frame.velLeft`/`velRight` from each decoded `Telemetry::Frame` along
   the way.
3. Differentiate: `accel[i] = (v[i]-v[i-1]) / dt`, `jerk[i] =
   (accel[i]-accel[i-1]) / dt`, `dt` = the real elapsed cycle time (the
   harness's own virtual clock step, matching `robot_loop.cpp`'s current
   `kCycle=20ms` — read it from the same constant/harness accessor other
   sibling harnesses use for timing assertions, e.g. `test_sim_api.py`'s
   own timing scenario, rather than hardcoding 20).
4. Assert bounds at both ends of the trace (first few cycles after
   activation; last few cycles before/at the completion event) and
   lobe-count over the whole trace (a lobe = a maximal run of
   same-signed nonzero velocity bounded by near-zero samples).
5. Implement "no command survives past the terminal zero" as its own,
   independently reportable check — structure the harness so ticket 003
   can flip exactly this one assertion without touching any other.
6. Give each named check independent pass/xfail visibility — either by
   having the Python driver expose one `pytest` function per named
   scenario (each with its own `@pytest.mark.xfail(strict=False,
   reason=...)` where needed), or by having the C++ harness itself print
   a machine-parseable PASS/FAIL/XFAIL line per check that the Python
   side asserts against selectively. Either mechanism is acceptable; the
   acceptance criterion is independent visibility per named check, not a
   specific implementation.
7. Same-boot scenario: a separate `main()` scenario block (or a second
   harness binary/argument mode) that constructs ONE `SimHarness`,
   `boot()`s once, then loops N times alternating a D700 straight and a
   360° pivot back-to-back on the SAME instance, asserting
   `ACK_STATUS_DONE` each time.

**Files to create**:
- `src/tests/sim/system/behavior_lock_harness.cpp`
- `src/tests/sim/system/test_behavior_lock.py`

**Files to modify**: none required — this is a pure addition. If
`src/tests/sim/system/README.md` catalogs sibling harnesses (it exists
and does describe the tier's own testing pattern), add one entry for
this harness to keep that catalog current.

## Testing

- **Existing tests to run**: `uv run pytest` (confirm no collection
  errors introduced and no pre-existing test's own behavior changes —
  this ticket is additive only).
- **New tests to write**: `src/tests/sim/system/test_behavior_lock.py`
  (see Acceptance Criteria above for required scenarios).
- **Verification command**: `uv run python -m pytest
  src/tests/sim/system/test_behavior_lock.py -v -s`, then `uv run pytest`
  for the full suite.

## Completion Notes

**Files added**: `src/tests/sim/system/behavior_lock_harness.cpp`,
`src/tests/sim/system/test_behavior_lock.py`. **Files modified**:
`src/sim/sim_harness.h` gained ONE new test-only accessor,
`SimHarness::plannerConfig()` (returns `pilot_.plannerConfig()`), needed to
satisfy "read limits from the SAME `msg::PlannerConfig` the harness boots
with, never hand-duplicate" — `App::Pilot::plannerConfig()` already existed
as a public read-only accessor (109-008) but nothing on `SimHarness` itself
exposed it. This is test infrastructure only (`src/sim/`, HOST_BUILD-only),
not production motion code — `pilot.cpp`/`executor.cpp` are untouched.
`src/tests/sim/system/README.md` gained one catalog entry for the new
harness pair.

**Harness design**: two assertion tiers, deliberately separate (see the
harness's own file header) — harness-plumbing sanity
(`checkTrue()`/`fail()`, the sibling idiom; a failure here is a real
test-infrastructure bug, hard-fails the compile-and-run pytest step) vs.
12 named behavior-lock checks (`report()`, printing machine-parseable
`RESULT: <name> :: PASS/FAIL :: <detail>` lines) that
`test_behavior_lock.py` parses once (a module-scoped fixture compiles and
runs the harness ONE time, shared across all 13 test functions — avoids
recompiling the ~20-file HOST_BUILD graph per named check).

**Non-obvious finding, worth flagging for future harness authors**: the
firmware's async command-completion ack (`RobotLoop::processEvents()`,
`robot_loop.cpp:350`, `tlm_.ack(event.id, toWireAckStatus(event.status),
0)`) reuses `AckEntry.corr_id` to carry the command's own `id` field
(`Motion::CompletionEvent::id`), NOT the wire envelope `corr_id` the
immediate accept-time OK/TRIVIAL/ERR ack uses
(`tlm_.ack(env.corr_id, ACK_STATUS_OK, 0)`). A test that wants to know
when a Move's own motion is DONE must watch its `id`, not its
injection-time `corrId` — this cost real debugging time while building
this harness (every command appeared to never complete, `pilotState()`
was correctly `kIdle` the whole time, and only instrumenting raw
`exec_state`/`acks_[]` fields revealed the DONE ack was arriving under a
completely different key than expected).

**Empirical results** (`uv run python -m pytest
src/tests/sim/system/test_behavior_lock.py -v -s`): **7 passed, 6 xfailed
in ~11s**. Currently-failing (xfailed, citing the driving issue):
`straight_ramp_bounds` (wheel accel ~4000mm/s² within the first cycle of
Move activation, vs. an `a_max`/`a_decel`-derived bound — the observed
jump from ~11mm/s to ~213mm/s in one 50ms cycle is inconsistent with a
legitimate `a_max=800`/`j_max=8000` jerk-limited ramp from rest, which
would only reach ~35-40mm/s by that point; consistent with a patch-stack
component (plausibly the straight-lead padding) injecting extra velocity
at activation, not just at the terminal), `straight_terminal_bounds`
(jerk spike right at completion), `pivot_ramp_bounds` (same
activation-region signature, rotational domain), `pivot_single_lobe_left`
/`pivot_single_lobe_right` (the pivot's own documented sign-changing
terminal tail splits each wheel's trace into 3 lobes instead of 1 —
`[- idx 6..37] [+ idx 38..42] [- idx 45..48]`), and
`pivot_lobes_opposite_sign` (depends on the two lobe-count checks above).
Currently PASSING (kept as plain, non-xfailed assertions — an xfail on an
already-passing check would XPASS, technically tolerated but misleading):
`straight_single_lobe_left`/`_right`, `straight_no_command_after_terminal_zero`,
`pivot_terminal_bounds`, `pivot_no_command_after_terminal_zero`. Honest
note on the two "no_command_after_terminal_zero" passes: the documented
post-completion "shelf" (creeping at the last staged twist until the
300ms deadman lease expires) does NOT measurably reproduce for a D700
straight (`deltaHeading=0`, so the heading-lead channel most padding
operates on is inert) or this 360° pivot in the IDEAL sim — both traces
settle to <5mm/s within one cycle of the DONE ack and never rise back
above the 15mm/s near-zero bar afterward. This matches this project's own
prior finding that the ideal sim sometimes cannot reproduce
hardware-only terminal artifacts (see `.clasi/knowledge/d-drive-terminal-
instability.md`'s "ideal sim can't repro" note) — ticket 003's own fix is
still independently justified by SUC-002's Main Flow (a Pilot::tick()
correctness argument, not solely a harness-driven one), and this harness
keeps both checks live as a currently-green regression fence rather than
xfailing them dishonestly.

**Same-boot scenario outcome (acceptance criterion, recorded here as
required)**: **PASSES.** 40 consecutive alternating D700-straight/360°-pivot
Move commands on ONE booted `SimHarness` instance (no reboot between them)
all reached `ACK_STATUS_DONE` within budget — `same_boot_all_moves_completed`
reports PASS, `40/40 moves completed`. The driving issue's §1.8/F7
stale-executor-state bug does **not** reproduce in this ideal-sim
same-boot scenario. This is a genuine, useful finding (not a gap in the
harness): §1.8/F7 may require a condition this scenario doesn't create
(e.g. a mid-flight preemption/replace stream, real hardware I2C timing,
or a specific interleaving this straight/pivot-only alternation doesn't
hit) — worth noting for whoever next investigates that finding, rather
than assuming this harness's clean pass means the bug doesn't exist.

**Full-suite verification**: `uv run python -m pytest` (module-invocation
form — see below) collects and runs cleanly: **1218 passed, 13 xfailed, 2
xpassed, 9 failed in 356.77s**. Of the 9 failures: **4 exactly match**
sprint 111's own documented baseline (`test_plant.py`,
`test_profiled_motion_sim.py::test_profiled_turn_leg_sim_ramp_shape_and_heading_target`,
`test_sim_api.py`, `test_app_robot_loop.py` — all under `src/tests/sim/`,
ticket 002's scope). **5 additional pre-existing failures were found
under `src/tests/testgui/`** that sprint 111's own baseline count did not
mention: `test_sim_errors_from_cal_button.py::…`,
`test_sim_errors_panel.py::…`, `test_tour_closure_gate.py::…`, and
`test_turn_error_characterization.py::test_postcompensation_ideal_matches_shipped_defaults[30.0/170.0]`.
These are **confirmed unrelated to this ticket** — this ticket's only
non-additive change is one new read-only accessor on `SimHarness`
(`src/sim/sim_harness.h`), which cannot affect Qt/characterization-sweep
tests that never construct a `SimHarness` at all — and are **not fixed
here** per this ticket's explicit out-of-scope instruction. A follow-up
scoped confirmation run, `uv run python -m pytest src/tests/sim
src/tests/unit -q` (exactly sprint 111's own documented baseline scope,
excluding `src/tests/testgui/`), came back **4 failed, 735 passed, 6
xfailed in 202.57s** — the 6 xfailed are entirely this ticket's own new
checks (zero pre-existing xfails in that narrower scope) and the 4
failures are an EXACT match for sprint 111's documented baseline, with
zero unexpected additions. Flagging for team-lead / ticket 002: the
sprint's own documented "4 pre-existing failures" baseline undercounts
the actual pre-existing failure surface by 5 (only visible once
`src/tests/testgui/` is included); ticket 002's scope (which names only
the 4 `src/tests/sim/` tests) may
need widening, or a decision that the `src/tests/testgui/` failures are
out of this sprint's scope entirely and belong to a separate issue.

**Environment note (pre-existing, not this ticket's to fix)**: bare `uv
run pytest` with no path argument hits a hard collection error
(`ModuleNotFoundError: No module named 'src'` importing
`src.scripts.gen_boot_config` in `test_turn_error_characterization.py`)
that **aborts the entire session before any test runs** — this is the
documented `.clasi/knowledge/pytest-env-uv-run-gotcha.md` issue (`uv run
python -m pytest`, not bare `uv run pytest`, is required; single root
package since 2026-07-02). `uv run pytest <specific-path>` (scoped to
this ticket's own new file) works fine either way, confirmed above.

**This ticket's own new test file, scoped run**: `uv run pytest
src/tests/sim/system/test_behavior_lock.py -v` → `7 passed, 6 xfailed in
11.01s` — matches the `-m` invocation exactly, no discrepancy.
