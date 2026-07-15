---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Use Cases — Sprint 107: TestGUI revival: tours execute and close

Continues SUC numbering from the single-loop-firmware arc (103: SUC-001..010,
104: SUC-011..017, 105: SUC-018..023, 106: SUC-024..030). This sprint is the
ARC FINALE — it delivers the stakeholder's literal, stated end goal directly
against the planner sprint 106 just built.

## SUC-031: Streaming executor's fault check excludes the boot-time baseline

- **Actor**: Any real caller of `planner/executor.py`'s `StreamingExecutor`
  (this sprint's tour driver, sprint 106's bench script, any future caller).
- **Preconditions**: `executor-fault-check-needs-baseline-exclusion.md` —
  `StreamingExecutor.tick()` stops the run the instant ANY drained frame's
  `fault_bits` is nonzero, with no baseline-relative exclusion for the
  boot-time one-shot `kFaultI2CSafetyNet` bit. On real hardware that bit is
  latched from boot and essentially always present, so `tick()` as written
  never completes a real run — confirmed 100% reproducible during 106-006's
  own bench session (worked around there only by a bench-script-local
  wrapper, `BaselineFaultMaskingTransport`, explicitly scoped to that one
  script). A tour chains 7+ legs through the executor; without this fix
  fixed IN `executor.py` itself, EVERY caller (this sprint's tour driver
  included) would need to reimplement the same wrapper.
- **Main Flow**: `StreamingExecutor` captures whichever `fault_bits` the
  first telemetry frame drained at `begin()` carries as that run's own
  baseline (mirroring `rig_soak.py`'s established "only a bit that turns on
  DURING the run counts as new" convention, and `profiled_motion_verify.py`'s
  own `BaselineFaultMaskingTransport.rebaseline()` per-run — not merely
  per-process — re-baselining rationale), and only raises
  `RunOutcome.FAULT` for a bit that turns on NEW relative to that baseline.
- **Postconditions**: Every real caller of `StreamingExecutor` — this
  sprint's tour driver, 106's own bench script (which can now drop its
  bench-script-local wrapper), and any future one — gets correct
  baseline-relative fault handling for free, with no adapter needed.
- **Acceptance Criteria**:
  - [ ] `StreamingExecutor.begin()` captures the first drained frame's
        `fault_bits` as the run's baseline; `tick()`'s fault check only
        trips on a bit set NEW relative to that baseline.
  - [ ] A real, latched-since-boot fault bit present at `begin()` time no
        longer stops the run on tick 2 (unit-tested with a fake transport
        whose first frame carries a nonzero `fault_bits`).
  - [ ] A bit that turns on freshly DURING a run (not present in the
        baseline frame) still stops the run with `RunOutcome.FAULT`
        (regression-protects the existing behavior — unit-tested).
  - [ ] `tests/unit/test_planner_executor.py` covers both cases; the full
        suite stays green.

## SUC-032: Heading-loop gains retuned for the bench rig's high-inertia turn

- **Actor**: Host planner's heading corrector, driving a profiled turn leg
  on the bench rig.
- **Preconditions**: `heading-loop-default-gains-overshoot-on-bench-rig.md`
  — `PlannerParams`' shipped field defaults (`heading_kp=2.0`,
  `heading_omega_clamp=0.5`) saturate the correction trim on the bench
  rig's high-inertia proxy load, adding substantial extra rotation on top
  of the profile's own already-complete open-loop trajectory: a 60° turn
  landed at ~79° (+19°, ~+32% overshoot) with the shipped defaults.
  `profiled_motion_verify.py`'s own bench session found `heading_kp=0.4`,
  `heading_omega_clamp=0.2` performs much better (landing errors across 4
  runs: -4.09°, -1.18°, +2.10°, +15.75° — one outlier), but that finding was
  exposed only as bench-script CLI overrides, never promoted to
  `PlannerParams`' own field defaults.
- **Main Flow**: `PlannerParams`' own `heading_kp`/`heading_omega_clamp`
  field defaults are updated to the bench-proven values
  (`heading_kp=0.4`, `heading_omega_clamp=0.2`), so every caller
  (including this sprint's tour driver) gets the gentler, bench-verified
  starting point without needing its own CLI override. A full gain sweep to
  reliably hold turn-landing error inside a tight (`±3°`) tolerance is
  NOT this sprint's job (the issue's own "Recommended follow-up") — this
  sprint promotes the ALREADY-bench-proven values only, and documents the
  known `+15.75°` outlier risk so tour closure tolerances are set with
  that risk in view, not assumed away.
- **Postconditions**: `PlannerParams()` (no override) matches what
  `profiled_motion_verify.py` had to pass explicitly before; tours chain
  turn legs against the gentler default without every caller needing to
  know the bench-specific override values.
- **Acceptance Criteria**:
  - [ ] `PlannerParams.heading_kp` defaults to `0.4`, `heading_omega_clamp`
        to `0.2`; `tests/unit/test_planner_model.py` (or equivalent)
        asserts the new defaults.
  - [ ] `profiled_motion_verify.py`'s own `--heading-kp`/
        `--heading-omega-clamp` CLI overrides remain functional (still
        override-able) but are no longer NEEDED to reach the bench-proven
        behavior — its own default `None` (falls through to
        `PlannerParams`' field default) now already IS the tuned value.
  - [ ] The known outlier risk (`+15.75°` observed in one of four bench
        runs) is documented in this sprint's own architecture/ticket notes
        and factored into the tour closure tolerance chosen in SUC-036 —
        not silently assumed to have been solved.

## SUC-033: Tour geometry chains through the host planner with closure bookkeeping

- **Actor**: Any caller (TestGUI, bench script) that wants to run a named,
  multi-leg tour end to end.
- **Preconditions**: `TOUR_1`/`TOUR_2` (`host/robot_radio/testgui/
  commands.py`) are ordered lists of legacy `D`/`RT` text-verb wire strings
  — the tour GEOMETRY (leg distances/turn angles) is a reusable asset, but
  both verbs are retired: sprint 102/103's single-loop rebuild deleted the
  entire on-robot trajectory planner (`Motion::SegmentExecutor`) and the
  `segment`/`replace` envelope arms that `testgui/binary_bridge.py`'s
  existing D/RT translation still targets (confirmed by this sprint's own
  reading of `protos/envelope.proto`: the current `CommandEnvelope.cmd`
  oneof carries exactly `twist`/`config`/`stop` — no `segment`/`replace`
  arm exists any more). There is no substitute for "drive this far" /
  "turn this many degrees" on the current wire other than sprint 106's
  host-side profiled-twist planner (`planner/profile.py` +
  `planner/executor.py`).
- **Main Flow**: A new module parses `TOUR_1`/`TOUR_2`'s existing
  wire-string lists into an ordered sequence of typed legs (signed straight
  distance, or signed turn angle — `RT`'s own sign convention preserved),
  keeping `commands.py`'s `TOUR_1`/`TOUR_2` constants as the SINGLE source
  of truth for tour geometry (no hand-transcribed duplicate). For each leg
  in order, it builds a `profile.py` setpoint sequence (distance or turn)
  and runs it through a `StreamingExecutor` (SUC-031/032's fixes/gains
  applied), recording that leg's `RunOutcome` plus the measured pose
  (`TLMFrame.pose`) before leg 1 begins (the tour's own closure baseline —
  `App::Odometry` never resets across a boot session, so this baseline is
  always relative, never an absolute zero) and after the final leg settles.
  Any leg outcome other than `COMPLETED` stops the tour immediately (no
  further legs attempted) and is reported, not silently swallowed.
- **Postconditions**: A tour runs to completion (or stops cleanly and
  reports exactly which leg failed and why), and the tour's own closure —
  final measured pose vs. the pre-leg-1 baseline pose, in both position and
  heading — is computed and available to the caller, independent of any
  GUI or bench-script-specific trace-capture concern layered on top.
- **Acceptance Criteria**:
  - [ ] A pure parser converts `TOUR_1`/`TOUR_2`'s wire-string lists into
        typed leg specs; unit-tested directly against `commands.TOUR_1`/
        `TOUR_2` (regression-protects the geometry against silent drift).
  - [ ] Running a tour executes each leg's profile through a real
        `StreamingExecutor` in order, stopping immediately (and reporting
        which leg and why) on any non-`COMPLETED` leg outcome.
  - [ ] Tour closure (position delta + heading delta, final measured pose
        relative to the pose captured immediately before leg 1) is computed
        and returned to the caller — unit-tested with a fake transport.
  - [ ] Reusable by BOTH the TestGUI (SUC-034) and a bench script (SUC-036)
        with no duplicated per-leg execution/telemetry-capture logic
        between them (a single shared per-leg run loop, matching
        `profiled_motion_verify.py`'s own `run_leg()` shape, promoted to
        production and parameterized over an ordered leg list).
  - [ ] 100% unit-tested under `tests/unit/`, no hardware/sim dependency for
        the parsing/chaining logic itself.

## SUC-034: TestGUI tour buttons execute against real hardware on the live wire surface

- **Actor**: Operator using the TestGUI on the bench rig.
- **Preconditions**: The TestGUI's tour buttons (`_TourRunner`,
  `__main__.py`) currently send each `D`/`RT` step as a literal wire string
  through `testgui/binary_bridge.py`'s `translate_command()`, which builds
  a `segment`/`replace` envelope for it — an arm that no longer exists on
  the wire (SUC-033's own preconditions). `_wait_for_idle()`'s own
  completion detection (a `SNAP`-poll for `frame.active`) is a mechanism
  the new `StreamingExecutor`-based run loop (SUC-033) does not need at all
  (`run()`/the per-leg loop already knows synchronously when a leg
  finishes). Both `SerialTransport`/`RelayTransport` (real hardware, via
  `_HardwareTransport` wrapping `NezhaProtocol`/`SerialConnection`) already
  satisfy `StreamingExecutor`'s `TwistTransport` protocol as-is — the SAME
  live surface sprint 106's bench script already runs against for real,
  proven in that sprint's own bench session.
- **Main Flow**: `_TourRunner`'s worker-thread `run()` is rewired to call
  SUC-033's tour driver against the connected transport's underlying
  `NezhaProtocol` (a new accessor on `_HardwareTransport` exposes it) in
  place of the old wire-string-per-step + SNAP-poll loop, narrating
  progress (`[TOUR] ... leg i/N: ...`) from the tour driver's own per-leg
  outcomes rather than from raw wire traffic. The GUI's existing canvas/
  avatar telemetry-update path keeps working from the SAME telemetry the
  executor is already draining (ticket-time investigation resolves the
  exact wiring — see this document's Step 7). Tour buttons are scoped to
  real-hardware transports only this sprint (`SerialTransport`/
  `RelayTransport`) — `SimTransport`'s backing library
  (`tests/_infra/sim/build/libfirmware_host.*`) was deleted wholesale at
  sprint 102 ticket 005 (`git show 72d8be7e --stat`) and was never rebuilt
  against the current single-loop firmware, so a sim-mode tour path has no
  working foundation to rewire onto this sprint (see this document's Step 1
  finding 3) — this is a deliberate, documented scope boundary, not a
  silent gap: tour buttons stay disabled (with a clear tooltip) when
  connected via Sim.
- **Postconditions**: Clicking a tour button on a real, connected robot
  actually drives the tour via the live twist wire surface, narrates
  progress, and can be stopped mid-flight (Stop Tour re-enables buttons
  synchronously, matching the existing, already-correct control-flow
  contract `testgui-tour-stop-reactivation.md` established).
- **Acceptance Criteria**:
  - [ ] Tour buttons, when connected via `SerialTransport`/`RelayTransport`,
        drive the tour through `StreamingExecutor`/SUC-033's tour driver —
        no `D`/`RT` wire string, no `binary_bridge.translate_command()`
        call, no `SNAP`-poll, anywhere in the tour code path.
  - [ ] Tour buttons are disabled (clear tooltip explaining why) when
        connected via `SimTransport` — no crash, no silent no-op.
  - [ ] Stop Tour still re-enables the tour buttons synchronously (existing
        contract, regression-tested).
  - [ ] Demonstrated end to end on the bench rig against real hardware
        (`.claude/rules/hardware-bench-testing.md`) — this is the
        stakeholder's own literal acceptance wording ("demonstrate that the
        tours... actually execute").

## SUC-035: Tour test suite rewritten and passing against the current architecture

- **Actor**: CI / any engineer running `uv run python -m pytest`.
- **Preconditions**: `tests/testgui/test_tour1_geometry.py`/
  `test_tour_stop.py`/`test_tour_idle_detection.py` target the OLD
  `tests/_infra/sim` ctypes firmware sim, deleted wholesale at sprint 102
  ticket 005 — `test_tour1_geometry.py`'s own `_LIB_PRESENT` guard means it
  silently SKIPS today (never actually runs), and `tests/testgui/` is not
  even in `pyproject.toml`'s `testpaths` (dropped at 102). None of these
  tests currently exercise anything against the current, post-102
  single-loop architecture.
- **Main Flow**: The tour-behavior tests are rewritten against a
  `FakeTransport`-backed harness (mirroring `tests/unit/
  test_planner_executor.py`'s own established double convention) instead
  of the deleted ctypes sim — proving the GUI's tour buttons correctly
  drive SUC-033's tour driver and correctly handle Stop, without requiring
  a rebuilt sim library this sprint (explicitly out of scope — see Step 1
  finding 3). `test_tour_idle_detection.py`, which tests the now-removed
  `_wait_for_idle()`/SNAP-poll mechanism specifically, is deleted or
  rewritten to test whatever (if anything) replaces it.
  `tests/testgui/` is re-added to `pyproject.toml`'s `testpaths` once these
  tests pass reliably headless.
- **Postconditions**: `uv run python -m pytest` actually exercises the
  tour buttons' control flow again, for the first time since sprint 102.
- **Acceptance Criteria**:
  - [ ] `test_tour1_geometry.py`/`test_tour_stop.py` (or their rewritten
        equivalents) pass under `uv run python -m pytest`, using a fake/
        double transport — no dependency on the deleted `tests/_infra/sim`
        library.
  - [ ] `test_tour_idle_detection.py` is deleted or rewritten; no test in
        the suite asserts behavior of the removed SNAP-poll mechanism.
  - [ ] `tests/testgui/` (or the rewritten subset of it this ticket
        touches) is added back to `pyproject.toml`'s `testpaths`.
  - [ ] The full suite (`uv run python -m pytest`) stays green.

## SUC-036: Bench tour runs execute end-to-end with captured traces and closure numbers

- **Actor**: Stakeholder / bench engineer verifying this sprint's own
  acceptance bar.
- **Preconditions**: SUC-031..034 exist and are individually verified. No
  tour has ever executed end-to-end against the post-102 architecture.
- **Main Flow**: Tour 1 and Tour 2 are each run for real on the bench rig
  (`.claude/rules/hardware-bench-testing.md`, wheels off the ground),
  through SUC-033's tour driver, capturing the FULL per-leg commanded-vs-
  measured telemetry trace (mirroring `profiled_motion_verify.py`'s own
  `LegResult`/CSV+JSON-sidecar convention, promoted to cover a whole
  multi-leg tour rather than one isolated leg) to
  `tests/bench/out/tour_<name>_<timestamp>.{csv,json}`, plus the tour's own
  closure numbers (final pose vs. pre-leg-1 baseline, position and heading
  delta).
- **Postconditions**: A captured, reviewable bench trace exists for both
  Tour 1 and Tour 2, with an explicit closure tolerance stated and checked
  (chosen empirically from the captured runs, following the project's own
  "measure then set tolerance with headroom" precedent — 106-006, 086-004
  — not assumed from the pre-098 tours' now-inapplicable 100mm figure) —
  this is the raw material sprint 107's own notebook (SUC-037) charts.
- **Acceptance Criteria**:
  - [ ] Both Tour 1 and Tour 2 run to completion on the bench rig with no
        leg timing out, through the real (not simulated) live wire surface.
  - [ ] A captured trace (CSV/JSON) exists per tour run under
        `tests/bench/out/`, recording every leg's commanded-vs-measured
        velocity/heading over time.
  - [ ] Tour closure (final pose vs. pre-leg-1 baseline) is measured,
        recorded, and checked against an explicitly stated tolerance
        (chosen from the captured runs' own numbers, with documented
        headroom) — a real pass/fail judgment, not "it looked right."
  - [ ] The standing bench verification gate
        (`.claude/rules/hardware-bench-testing.md`) is satisfied before any
        tour run: sensors alive, wheels drive both directions with encoders
        incrementing, round-trip confirmed over the real link.

## SUC-037: Notebook — clean accel/decel and tour-closure charts

- **Actor**: Stakeholder reviewing this sprint's (and the whole single-loop
  rebuild arc's) own literal, stated deliverable.
- **Preconditions**: SUC-036's captured tour traces exist. The stakeholder's
  own verbatim acceptance (2026-07-14): "I want to see charts in Jupyter
  Notebooks that show nice acceleration and deceleration on straights and
  turns."
- **Main Flow**: A new notebook under `tests/notebooks/` (alongside the
  established `motion_control.ipynb`/`wheel_motion_trace.ipynb` precedent)
  loads SUC-036's captured trace files and renders: commanded-vs-measured
  velocity per leg (the accel/decel envelope evidence, straight legs and
  turn legs both shown), heading over time during straights (hold) and
  turns (tracking), the tour's own (x, y) path with start/end proximity
  visualized, and a per-leg summary table (outcome, target, measured,
  error). The implementer loads the project's `dataviz` skill before
  writing any chart code, per this sprint's own explicit instruction —
  chart quality (clean, legible, consistent) IS the acceptance bar here,
  not merely "a chart exists."
- **Postconditions**: The notebook runs end-to-end and renders cleanly,
  committed to the repo — the literal artifact the stakeholder asked to
  see.
- **Acceptance Criteria**:
  - [ ] The notebook loads SUC-036's captured CSV/JSON trace files (not
        synthetic/hand-built data).
  - [ ] Charts render: commanded-vs-measured velocity with visible
        accel/cruise/decel phases for at least one straight leg and one
        turn leg; heading over time for a straight (hold) and a turn
        (tracking); the tour's (x, y) path with the closure gap visualized;
        a per-leg summary table.
  - [ ] The `dataviz` skill's guidance (palette, form heuristics,
        light/dark consistency) is applied — reviewed for chart quality,
        not just presence.
  - [ ] The notebook runs top-to-bottom without error and is committed with
        its rendered output.
