---
id: '005'
title: TestGUI square-tour button (OPTIONAL, droppable)
status: done
use-cases:
- SUC-001
depends-on:
- '004'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI square-tour button (OPTIONAL, droppable)

# THIS TICKET IS OPTIONAL AND MAY BE DROPPED

**Dropping this ticket does not affect sprint acceptance.** The sprint succeeds
or fails on ticket 004. Stakeholder direction, verbatim: "It would be great if
the TestGUI works too, and then I can actually run the tour from the TestGUI"
— a nice-to-have, explicitly secondary to "getting this to do the square tour."

**Drop it if**: ticket 004 is not comfortably green, time is short, or the GUI
work starts pulling in changes to firmware or the transport layer. Under no
circumstances should this ticket cause a change to `src/motion` or
`src/firm`. If it seems to need one, stop and drop the ticket instead.

## Description

Give the stakeholder a button in TestGUI that runs the square tour, so he can
run it himself tomorrow morning without a terminal.

The tour logic already exists and was validated by ticket 004
(`src/tests/bench/planner_square_tour.py`, sequential, no host trim). This
ticket is a **UI entry point onto behaviour that already works** — not a new
tour implementation. Reuse; do not reimplement.

### Standing rule for GUI work

`.claude/rules` and the project's standing practice: **GUI work needs
`src/tests/testgui/test_gui_button_acceptance.py` to pass headlessly before it
counts as done.** Note this file is **already in the master failing baseline
(×2)** — so the bar is *by identity*: your button's test must pass, and you
must not add new failures to that file. Do not claim the pre-existing failures
as yours to fix.

### Transport note

Select **"Relay"** for the radio path; a plain serial selection reaches the
robot's own USB port, not the relay dongle. On battery the robot is reached
through the relay's `RADIOBRIDGE` port (`mbdeploy list`'s ROLE column).

### Same-message principle

A button's command must be the **same message** the bench path sends — a
button's behaviour never depends on the transport, and config comes from the
robot, not from a GUI-side file.

## Acceptance Criteria

- [x] TestGUI exposes a control that runs the square tour, sequential, with
      **no host-side trim** (the firmware does the correction)
- [x] It reuses the existing tour path rather than reimplementing the sequence
- [x] `src/tests/testgui/test_gui_button_acceptance.py` passes headlessly for
      the new control, and adds **no new failures** to that file
- [x] Zero changes to `src/motion` or `src/firm`
- [x] Run once against the real robot if hardware time permits; if not, say so
      plainly rather than implying it was exercised

## As-built

**A "Square" button next to Tour 1 / Tour 2** (`tour_btn_square`). Geometry
is `planner.tour.TOUR_SQUARE` — 4 × 500 mm legs, 4 × 90° left turns, the same
`LEG`/`CRUISE`/`TURN` values `src/tests/bench/planner_square_tour.py` uses,
expressed as `D`/`RT` steps so the existing `parse_tour()`/`run_tour()` path
drives it. No host-side trim anywhere in the path.

**The sequencing is the load-bearing part, and it is now first-class.** Each
named tour carries a `planner.tour.TourExecution` (`commands.TOUR_EXECUTION`,
read via `commands.tour_execution(name)`) saying *how* it is driven, separate
from *where* it goes:

| tour | execution |
|---|---|
| Tour 1, Tour 2 | `PIPELINED_EXECUTION` — one-leg lookahead, unchanged |
| Square | `SQUARE_EXECUTION` — sequential, 1.2 s passive settle, 2.4 rad/s |

`run_tour()` gained `sequential=` and `settle=`. In sequential mode it keeps
exactly one Move in flight and dwells passively at each boundary — **sending
nothing at all**, which is why the firmware's `carryValid_` heading ledger
survives and the tour closes. `MoveTransport` has no `wheels()` member, so the
lease that would clear the ledger is structurally unreachable from this path;
a unit test pins that alongside the sequencing.

The GUI tour path never held a zero-velocity lease to begin with (`_TourRunner`
drives `run_tour()`, which only ever calls `move()`/`stop()`), so there was no
existing bug to fix there — the gap was that it had no rest-to-rest mode at all.

### Not verified on hardware

**This was not run against `tovez`.** The robot was left estopped and idle for
the stakeholder. The 6.3 mm closure number belongs to ticket 134-004's bench
run of the equivalent bench script, not to this button. What is verified here
is that the button exists, dispatches the rest-to-rest execution, retires real
legs against the sim, and stops on demand.

The new acceptance test deliberately does **not** use `_run_tour()`, whose bar
is whole-tour closure + per-leg turn accuracy against the sim plant — the two
bars the pre-existing `test_tour_1`/`test_tour_2` failures in that file are
already failing (sim 90° turns missing by ~13°). Reusing it would have added a
third failure of an unrelated defect.

## Implementation Plan

1. Find the existing button/tour wiring in TestGUI and follow its pattern.
2. Add the control; point it at the existing tour path with `--sequential`
   semantics and trim off.
3. Add/extend the headless button-acceptance test.
4. Run `uv sync --group gui` if the GUI deps are not present; `just testgui` to
   launch interactively.

**Timebox this.** If it exceeds a modest, focused session, drop it and report
that it was dropped — that is a correct outcome, not a failure.

## Testing

- **Existing tests to run**: `src/tests/testgui/test_gui_button_acceptance.py`
  (headless), compared **by identity** against its 2 pre-existing failures.
- **New tests to write**: headless acceptance for the new control.
- **Verification command**:
  `uv run python -m pytest src/tests/testgui/test_gui_button_acceptance.py -q`
