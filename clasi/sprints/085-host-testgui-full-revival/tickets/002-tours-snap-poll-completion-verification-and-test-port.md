---
id: "002"
title: "Tours: SNAP-poll completion verification and test port"
status: open
use-cases: [SUC-001]
depends-on: ["001"]
github-issue: ""
issue: host-testgui-full-revival.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Tours: SNAP-poll completion verification and test port

## Description

`_TourRunner` (`host/robot_radio/testgui/__main__.py` lines ~1254–1372) and
`commands.TOURS` (`TOUR_1`/`TOUR_2`, hardcoded `D`/`RT` wire-string
sequences) already implement the full tour feature: it sends each step via
`transport.command()`, then polls completion with a fire-and-forget `SNAP`
and reads `mode=I` off the cached `TLMFrame` in `_state["last_tlm"]` (not
`transport.command("SNAP")`, whose corr-id-less reply never reaches the
reply queue — see the class's own docstring). This code predates the
greenfield rebuild and has never run against a real sprint-084 firmware/sim
(per `architecture-update.md` Grounding fact 1) — sprint 084's `D`/`RT`
verbs and `mode=` machine are what it was always waiting on.

This ticket ports the three tours-related historical test files from
`tests_old/testgui/` to `tests/testgui/`, updates them for the current API
if anything drifted, and — critically — actually runs Tour 1 and Tour 2
against the sim for the first time since 084 closed, fixing whatever a real
run surfaces. Completing with zero production-code changes (beyond a real
bug fix, if one is found) is an acceptable, expected outcome — the
acceptance bar is that the ported tests actually pass against the sim, not
merely that they exist (see `architecture-update.md` Migration Concerns,
"Risk of verification finds nothing to fix").

## Acceptance Criteria

- [ ] `tests_old/testgui/test_tour_idle_detection.py`,
      `test_tour_stop.py`, and `test_tour1_geometry.py` are ported to
      `tests/testgui/`, updated for any API drift since they were written,
      and pass under `QT_QPA_PLATFORM=offscreen`.
- [ ] Tour 1 (`commands.TOUR_1`) runs to completion against the sim
      (`SimTransport`) with no step timing out, and the robot's fused pose
      ends near world origin (the tour is a closed geometric loop).
- [ ] Tour 2 (`commands.TOUR_2`) likewise runs to completion.
- [ ] `_wait_for_idle`'s stale-frame rejection (a cached `TLMFrame`
      timestamped before the current step began must not end the wait
      early) is exercised and holds against the real `mode=` machine.
- [ ] Stopping a running tour re-enables the tour buttons synchronously
      (not dependent on the `finished` signal being delivered during the
      blocking `thread.wait()` — see the existing
      `testgui-tour-stop-reactivation.md` root-cause doc).
- [ ] Any bug a real run surfaces (e.g. a `SNAP`-poll timing constant that
      needs retuning against actual 084 mode-machine latency — flagged as
      Open Question 1 in `architecture-update.md`) is fixed in this ticket
      and the fix is documented in this ticket's file, not silently folded
      in.

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression); the
  three newly-ported files specifically.
- **New tests to write**: the three ported files above, adapted as needed;
  no net-new test file (unlike ticket 003).
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
