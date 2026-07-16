---
status: in-progress
sprint: '108'
tickets:
- 108-005
- 108-006
---

# A new ctypes C ABI over `SimApi` is needed before TestGUI tours can run in Sim mode

## Problem

Sprint 107 (TestGUI revival: tours execute and close) scoped tour
execution to real-hardware transports only (`SerialTransport`/
`RelayTransport`) — `SimTransport` tour support was explicitly deferred,
not silently skipped. This issue records why and what a future sprint
needs to build to close the gap.

`host/robot_radio/io/sim_conn.py`'s `SimConnection` and `tests/testgui/
test_tour1_geometry.py` both resolve to `tests/_infra/sim/build/
libfirmware_host.{dylib,so}`. That entire directory
(`tests/_infra/sim/CMakeLists.txt`, `sim_api.cpp`, `firmware.py`,
`smoke_check.py` — 226 + 895 + 590 + 81 lines) was **deleted wholesale**
by commit `72d8be7e` ("feat(102-005): delete Elite plumbing + banner-only
stub main"), as part of the single-loop rebuild. The directory does not
exist in the working tree today. `SimConnection.connect()` therefore
always fails ("Sim library not found"), and `test_tour1_geometry.py`'s own
`_LIB_PRESENT` guard means that file has been silently SKIPPING (not
failing, not running) every `uv run python -m pytest` invocation since
before the rebuild — confirmed by direct investigation during sprint 107's
own architecture pass, not merely assumed stale.

Sprint 105 built a DIFFERENT sim harness (`tests/sim/support/sim_api.h`, a
C++ class linked directly into pytest-driven host-build test binaries) to
replace the deleted infrastructure for its own purposes — but that harness
has **no ctypes-callable C ABI**, unlike the deleted `tests/_infra/sim/
sim_api.cpp`. There is no drop-in replacement library for `io/sim_conn.py`
to point at.

105's own `architecture-update.md` (Decision 4) said "sprint 107 builds
`io/sim_conn.py`" — but that file already exists (reconciled against the
now also-deleted 081-004 ABI); the real remaining gap, confirmed by 107's
own investigation, is a NEW ctypes C ABI over 105's `SimApi` C++ class,
which has never existed in any form.

## Why this was not fixed in sprint 107

Building a new ctypes C ABI over a C++ harness that has never had one is a
genuinely new engineering task (design the export surface, implement it,
validate it against a real consumer), not "point existing wiring at a
rebuilt artifact." Sprint 107's own architecture-update.md (Decision 1)
explicitly declined to build it speculatively, citing the same
"don't build ahead of a validated consumer" reasoning sprints 104 and 105
already applied to this exact seam. The stakeholder's own literal
acceptance for sprint 107 ("demonstrate that the tours... actually
execute") is fully satisfied by the real-hardware path; sim-mode tours are
a CI/headless convenience, not part of that stated acceptance bar.

## What sprint 107 leaves behind for a future sprint

- `host/robot_radio/planner/tour.py` (sprint 107) is a concrete,
  already-proven consumer interface — a real `TwistTransport`-shaped
  protocol (`twist()`/`stop()`/`read_pending_binary_tlm_frames()`) that any
  new sim ctypes ABI needs to satisfy, unlike the situation when 105's
  Decision 4 was written (no consumer existed yet to validate a
  ctypes-ABI shape against).
- `tests/testgui/test_tour1_geometry.py`/`test_tour_stop.py` (sprint 107's
  own ticket 004) were rewritten against a `FakeTransport` double instead
  of the deleted sim — CI coverage of the TOUR CONTROL FLOW does not
  depend on this gap, so there is no urgency pressure from the test suite
  itself; this is purely about restoring a headless/CI-runnable, PHYSICS-
  backed sim path for tours (and for `SimTransport` generally, which is
  also broken for every other purpose today).

## Recommended direction

A future sprint should:
1. Design a ctypes-callable C ABI over `tests/sim/support/sim_api.h`'s
   `SimApi` class (a `.cpp` shim exporting `extern "C"` functions, mirroring
   the shape the deleted `tests/_infra/sim/sim_api.cpp` used — same design
   pattern, fresh implementation against the current `SimApi`).
2. Reconcile `io/sim_conn.py` against the NEW ABI's actual exported symbol
   set (the same kind of reconciliation that file's own header already
   documents doing once, for the now also-obsolete 081-004 ABI) —
   expect real differences (105's `SimApi` has a different capability set
   than the deleted one: no independent OTOS drift model by design, per
   105's own Decision 3, for example).
3. Re-enable `SimTransport` tour support in the TestGUI (sprint 107's own
   `tests/testgui/test_tour1_geometry.py` rewrite gives a concrete target
   to restore full-physics coverage against once the ABI exists), and
   re-enable the sim-mode tour buttons sprint 107 left deliberately
   disabled.

## Evidence

- `git show 72d8be7e --stat` — confirms `tests/_infra/sim/` deletion.
- `host/robot_radio/io/sim_conn.py`'s own module docstring — documents its
  prior reconciliation against the now-also-deleted 081-004 ABI.
- `docs/architecture/architecture-update-105.md` Decision 4 — the prior
  "defer to 107" pointer this issue supersedes with a sharper, verified
  finding.
- `clasi/sprints/107-testgui-revival-tours-execute-and-close/
  architecture-update.md` Step 1 finding 2, Decision 1.
