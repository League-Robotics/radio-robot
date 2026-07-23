---
id: '119'
title: 'Land at zero and clean house: motion semantics + deletions'
status: planning-docs
branch: sprint/119-land-at-zero-and-clean-house-motion-semantics-deletions
worktree: false
use-cases: ['SUC-067', 'SUC-068']
issues:
- kill-the-silent-off-shaping-config-boundary.md
- specify-and-assert-the-leg-handoff-contract.md
- delete-the-config-attic-and-dead-tour-kwargs.md
- relocate-narrative-comments-and-refresh-stale-docs.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 119: Land at zero and clean house: motion semantics + deletions

## Goals

**Amended (2026-07-23, mid-execution) — scope reduced from six issues to
four.** Land-at-zero completion (`land-at-zero-completion-delete-stop-lead.md`)
and its companion test-disposition issue
(`turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md`)
were PULLED FORWARD into sprint 118 as its ticket 004: ticket 118-002's
own closure-gate run went red at the unchanged `stop_lead_ms=45` once
odometry freshness landed (fresh data overcompensation, not a bug in
002), a 0-120ms sweep found no safe re-baseline value, and per the
turn-execution review's own R6 rule and this project's
sprint-end-must-be-testable convention, the fix was deleted forward into
118 rather than left for this not-yet-detailed sprint to inherit as a
known-red gate. Full rationale: sprint 118's own sprint.md, Decision
Record and Design Rationale Decision 4.

Follow-on to sprint 118 ("loop schedule truth" — now shipping land-at-zero
completion too). With the loop's timing, odometry-freshness, AND
completion-predicate defects fixed in 118, this sprint closes the
*remaining* open items from the 2026-07-22 turn-execution review: close
the silent-off shaping/anticipation config boundary that cost weeks of
confusion, specify the chain-advance leg hand-off contract that today is
only tuned around, and sweep the accumulated dead config surface
(`control.*` attic keys, dead `run_tour()` kwargs) and stale doc
references the review's bloat inventory (§6) flagged. Sequenced strictly
after 118: every remaining issue here depends on 118's completed fixes
(the config-boundary/attic/doc cleanup tickets reference
`stop_lead_ms`'s deletion, already done in 118).

## Problem

Per `docs/code_review/2026-07-22-turn-execution-review.md` D1/D5 and its
bloat inventory (§6) — the `stop_lead_ms`/completion-predicate problem
(R2/D3) is now RESOLVED by sprint 118's ticket 004, not this sprint's
problem to solve: the correctness feature that fixes turn accuracy
(velocity-shaper taper + land-at-zero completion) has a silent-off config
boundary that left it disabled in every live GUI session until recently.
The chain-advance leg hand-off (carried shaper state across tour legs,
asymmetric reversal dwell) is tuned around, never specified. And "config
as source of truth" has drifted into "config as attic": (118's ticket 004
already removed `stop_lead_ms` from this list) 20 dead `control.*` keys
and 4 dead `run_tour()` kwargs survive with zero living consumers, each
one an invitation for a future agent to "wire it back up."

## Solution

**Amended: four issues** (was six; land-at-zero + its test-disposition
companion delivered by sprint 118's ticket 004 — see Goals). Sequenced so
tickets 1-3 (independent of each other, each depends only on 118, already
merged) can run in any order, doc relocation runs last:

1. **`kill-the-silent-off-shaping-config-boundary.md`** — default-on
   shaping/anticipation at `SimLoop.configure_from_robot()` (the seam
   every caller already goes through) instead of only the TestGUI's
   connect-time push; add a loud telemetry-flag + GUI-banner off-state
   indicator so a 20×-accuracy-delta feature never again has an invisible
   off state. **Verified against the tree (2026-07-23):** `push.py`'s
   `estimator_kwargs()` already exists and its own docstring already
   documents that the former `stop_lead_ms` field was deleted (118 ticket
   004) — the live field set is exactly `config.estimator.*`
   (`weight_heading_otos`/`weight_omega_otos`/`staleness_ms`) +
   `config.control.*` (`a_max`/`a_decel`/`alpha_max`/`alpha_decel`/
   `j_max`/`yaw_jerk_max`). `SimLoop.configure_from_robot()`
   (`io/sim_loop.py:487`) still only calls `calibration_kwargs()` (Tier 1)
   + `motor_boot_config_for()` (Tier 2) — it does NOT call
   `estimator_kwargs()` yet, so the silent-off defect this issue describes
   is still live post-118 exactly as scoped; this ticket adds that one
   call. The telemetry flag bit: `docs/protocol-v4.md` §8.2's `flags` bit
   table has bits 8/10/12 already declared-but-unwired for OTHER meanings
   (do not repurpose) and bits 16-31 genuinely free — the new bit is
   **16**, the first free slot, appended per the table's own existing
   append-only convention (118 ticket 004 already used this exact pattern
   when it touched `docs/protocol-v4.md` §5.2 for the land-at-zero
   paragraph — same doc, same discipline, this ticket's own precedent to
   follow).
2. **`specify-and-assert-the-leg-handoff-contract.md`** — one contract
   paragraph in `motion/DESIGN.md` (carried-axis ramp, decay-axis
   behavior, reversal-dwell asymmetry budget), asserted in the tour
   boundary test instead of tuned around. **Verified against the tree
   (2026-07-23):** a NEW pool issue,
   `clasi/issues/chain-advance-completion-margin-narrow-pocket.md` (filed
   today from 118 ticket 003's resolution, commit `b736a4ab`), documents
   that the chain-advance completion margin
   (`kStoppingMarginFactorChain=0.60` + a per-cycle
   `kDiscretizationCyclesChain=0.53` discretization term, both in
   `move_queue.cpp`, re-swept at 40ms parity) sits in a narrow accuracy
   pocket rooted in exactly this contract's own subject: "every
   chain-advance turn hands its axis to a Move that doesn't command it;
   completion is scored at the ack instant while the post-handoff coast
   is only partially visible." The contract paragraph THIS ticket writes
   must define that axis-drop coast precisely (not merely gesture at it),
   because the pool issue's own candidate fixes ("have the handoff
   contract define the axis-drop coast explicitly so the predicate can
   subtract it") depend on this contract existing first. This ticket
   SPECIFIES the behavior (and its heading/margin cost), it does not
   re-engineer `kStoppingMarginFactorChain` — closing the narrow pocket
   itself stays future work per the pool issue's own "not urgent... Phase
   B bench data should say whether it's even observable" framing.
   **Also verified:** the tour-level boundary test this issue names
   (`test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`,
   `test_tour_closure_gate.py:694`) is currently `xfail(strict=False)`
   with a reason entirely about the 112-005 cycle-order-hoist experiment
   ("frame.twist oscillates... a direct, confirmed consequence of the
   live reorder experiment") — that experiment was RETIRED by 118 ticket
   001. The xfail's own stated cause no longer exists; the un-xfail
   decision this issue calls for is not merely a citation re-point, it is
   "does this test pass now that its own stated cause is gone" — see
   ticket 2's acceptance criteria.
3. **`delete-the-config-attic-and-dead-tour-kwargs.md`** — remove the 20
   dead `control.*` keys (schema + all three robot JSONs + allowlist) and
   `run_tour()`'s 4 dead kwargs + `DEFAULT_INTER_LEG_SETTLE`. **Verified
   against the tree (2026-07-23):** all 20 keys (`heading_kp`,
   `heading_kd`, `heading_source`, `heading_dwell_tol_deg`,
   `heading_dwell_rate_dps`, `heading_lead_bias`, `plan_lead`,
   `terminal_lead`, `distance_kp`, `distance_tol`, `actuation_lag`,
   `model_tau_lin`, `model_tau_ang`, `turn_gate`, `arrive_dwell`,
   `arrive_tol_mm`, `sync`, `min_speed`, `yaw_rate_max`,
   `max_rot_accel_dps2`) are still present in the `control` object of all
   three `data/robots/*.json` (`turn_gate` absent from `togov.json` only,
   as the issue itself notes). `stop_lead_ms` is confirmed absent from
   every JSON and from `config_sync_allowlist.json` (grep-verified) — the
   issue's own "coordinate if sequenced with the land-at-zero deletion"
   scope guard is moot; 118 already did that deletion, cleanly, with
   nothing left to coordinate.
4. **`relocate-narrative-comments-and-refresh-stale-docs.md`** — last,
   so it trims only what survives the deletions above. **Verified against
   the tree (2026-07-23) — scope is what SURVIVES 118, not the issue's
   original list verbatim:**
   - `move_queue.h` is STILL 73% comment (281/385 lines) — 118 ticket 004
     added substantial new derivation prose for the land-at-zero formula;
     needs re-measuring and shrinking post-118, not just re-confirming
     the pre-118 number.
   - `sim_harness.h` is STILL 63% comment (383/608 lines), unchanged by
     118 (118 only touched `kCycleDtUs` and one DESIGN-note there).
   - `tour.py`'s 104-line module docstring is unchanged, still needs the
     relocation treatment.
   - `test_tour_closure_gate.py`'s xfail reasons already shrank somewhat
     as a SIDE EFFECT of 118 ticket 004 rewriting their content
     (`_XFAIL_REASON_IDEAL` now ~30 lines, was 116; `_XFAIL_REASON_REALISTIC`
     now ~21 lines, was 65) but are still far from the issue's own
     "one sentence + live issue link" target — real work remains.
     `_STOP_LEAD_MS`'s ~51-line block is confirmed FULLY GONE (118 deleted
     it with the field) — nothing left to do there.
   - **The dangling xfail citations of the deleted
     `cycle-order-reorder-experiment-ab-before-hardware.md` are CONFIRMED
     ALREADY FIXED** — 118 ticket 001 re-pointed both (in
     `test_tour_closure_gate.py` and
     `src/tests/sim/unit/test_app_robot_loop.py`) as part of its own
     scope; grep for the deleted filename returns zero hits in either
     file. **Remove this item from this ticket's scope** — it is done,
     not this sprint's work, and ticket 2 above separately handles the
     boundary test's OWN xfail (a different test, a different reason,
     citing the now-archived-but-not-deleted `restore-the-interleaved-...`
     issue and a stale reorder-experiment theory 118 also invalidated).
   - `.claude/rules/hardware-bench-testing.md` STILL points at
     `docs/protocol-v2.md` as current (2 live links, verified) — unfixed,
     full scope stands.
   - `.claude/rules/coding-standards.md`'s `SystemCommands.cpp` reference
     — unfixed, unchanged.
   - TestGUI "Managed — Ruckig" label (`testgui/__main__.py:925`, stale
     comments `:735,:758`) — unfixed, unchanged. (A broader repo grep for
     "Ruckig" turns up many more hits, but they are all in
     `docs/architecture/architecture-update-*.md` historical sprint
     records, `docs/code_review/` point-in-time reviews, and
     `docs/design/simple-velocity-control-guide.md`/
     `wpilib-motion-stack-comparison.md`'s own legitimate
     why-we-didn't-use-it discussion — none of those are in scope; only
     the GUI label and its two stale comments are.)
   - `docs/specification.md`/`docs/architecture.md` — verified: BOTH
     documents describe the entire pre-077 `source/` tree (ASCII
     `CommandProcessor`, `DriveController`, `PathFollower`/
     `PurePursuit`/`Stanley` nav layer) wholesale — not a few stale
     lines, the documents' complete subject is the deleted architecture.
     Per the issue's own "refresh or mark superseded-by pointers to
     docs/design/" instruction, the fix is a clear superseded-by banner
     at each document's top pointing to `docs/design/design.md`, not a
     line-by-line rewrite of two documents describing dead code — full
     rewrite is out of scope for a mechanical-relocation ticket.
   - No rule/doc points at `docs/protocol-v4.md` as current — confirmed,
     same root cause as the hardware-bench-testing.md item above (fixing
     that pointer fixes this).
   - `src/host/robot_radio/DESIGN.md`'s `planner/` row claims `from
     robot_radio.planner.tour import TOUR_1, TOUR_2` "raises
     `AttributeError` at import time" — **empirically verified FALSE**:
     `uv run python -c "from robot_radio.planner.tour import TOUR_1,
     TOUR_2"` succeeds cleanly today (both symbols non-`None`). The
     `planner/` package is live enough to power `test_tour_closure_gate.py`
     and the button-acceptance suite's managed-motion tests, which import
     and call `run_tour()` directly. Correct the row to the actual mixed
     status — this ticket's own acceptance criterion gives the exact
     verification command.
   - **NEW finding, not in the original issue (discovered verifying the
     tree, per this dispatch's own instruction not to trust the roadmap
     blindly):** `docs/protocol-v4.md` §8's own header line ("rides
     `ReplyEnvelope`... emitted every loop cycle — primary period == cycle
     period, ~50 Hz / 20 ms... unchanged by sprint 116") is now WRONG —
     118 changed `kCycle`/`kPrimaryPeriod` to 40ms/~25Hz and this line was
     never updated (118's own doc-update scope named
     `src/firm/app/DESIGN.md`/`src/sim/DESIGN.md`/`docs/design/design.md`,
     not this file). Folded into this ticket's "protocol-v4 pointers"
     item since it is the same document and the same class of staleness.

## Success Criteria

Full `uv run` pytest suite green, sim tour-closure gate + button-acceptance
suite green, no dead `control.*` key survives, every stale doc reference
in the review's bloat inventory fixed. (`stop_lead_ms` deletion + isolated
90° accuracy are now sprint 118's success criteria, already delivered by
its ticket 004 — verify at this sprint's own detail-promotion that they
still hold, but they are not this sprint's own deliverable.)

## Scope

### In Scope

- Default-on shaping/anticipation push at `configure_from_robot()`; loud
  telemetry/GUI off-state indicator (append-only `docs/protocol-v4.md`
  change).
- Chain-advance leg hand-off contract (`motion/DESIGN.md`) + boundary-test
  assertion.
- Deletion of the 20 dead `control.*` config keys and 4 dead `run_tour()`
  kwargs + `DEFAULT_INTER_LEG_SETTLE`.
- Narrative-comment relocation and stale-doc-reference sweep (sequenced
  last).

### Out of Scope

- **(Amended)** Land-at-zero MOVE completion predicate, `stop_lead_ms`
  deletion, and `test_turn_error_characterization.py`'s postcompensation
  test disposition — delivered by sprint 118's ticket 004, not this
  sprint's scope.
- Hardware bench verification — deferred to the phase-B bench session
  that follows both 118 and 119 (see 118's own deferral note; the same
  applies here — this sprint's acceptance bar is the sim suite + closure
  gate + button acceptance, not the stand).
- Any new StateEstimator consumer (fake-OTOS/fusion bench work) —
  `bodyAt()` is quarantined by 118, not wired to a new consumer, this
  sprint either.
- **(New, 2026-07-23)** Actually closing the chain-advance completion
  margin's narrow pocket (`clasi/issues/chain-advance-completion-margin-narrow-pocket.md`)
  — ticket 2 specifies and asserts the axis-drop-coast contract the
  narrow pocket is rooted in, it does not re-derive
  `kStoppingMarginFactorChain`/`kDiscretizationCyclesChain` or otherwise
  change `MoveQueue::landAtZero()`'s behavior. That pool issue's own
  "not urgent... Phase-B bench data should say whether it's even
  observable" framing stands; revisit in a future sprint if it does flake.
- Anything from the review's §6 bloat inventory not named in the four
  remaining issues above (e.g. `Header archaeology` items outside the
  three named files) — out of scope unless discovered to be entangled
  during execution.

## Test Strategy

Sim-only this sprint (bench deferred, per the overnight mandate). Each
issue's own acceptance criteria (isolated-turn accuracy bands, closure
gate bands, grep gates for dead strings/keys) is the ticket-level test
plan; full detail lands when this sprint is detail-promoted. The full
`uv run python -m pytest` suite and the sim tour-closure/button-acceptance
gates must stay green after every ticket, not just at sprint end.

## Architecture

**Sizing: Substantial** — this sprint touches 4+ modules with
independent ownership (`src/host/robot_radio` — `io/sim_loop.py`,
`calibration/push.py`-adjacent field wiring, `testgui/`;
`src/firm/app` — `App::Telemetry`'s flags bit-string; `src/firm/motion`
— the leg hand-off contract; the config/schema layer — pydantic model +
all three `data/robots/*.json` + `config_sync_allowlist.json`; the
test/harness tree). Substantial by module count per the sizing rubric —
see Step 4 for why no component diagram is included: like 118, this
sprint completes existing plumbing and deletes/relocates existing
surface; it does not compose anything new.

### Step 1 — Understand the problem

Covered above (Problem/Solution, each with 2026-07-23 tree-verification
notes). Four independent defects survive 118's landing: one config seam
that still silently skips a live-but-unpushed field set (ticket 1); one
specified-nowhere behavior a fresh pool issue just tied a real accuracy
finding to (ticket 2); one config schema that still carries 20 dead
fields and 4 dead kwargs (ticket 3); and a documentation surface whose
staleness has to be re-measured post-118, not assumed from the pre-118
issue text (ticket 4).

### Step 2 — Identify responsibilities

Four responsibility groups, independent of each other except ticket 4's
dependency on the other three landing first:

- **Config-push completeness** — `SimLoop.configure_from_robot()` must
  push every field a real serial boot bakes in, not a subset; the
  off-state must be observable on the wire, not silent. Independent of
  the other three groups.
- **Leg hand-off specification** — the chain-advance carried-shaper-state
  contract needs to exist in `motion/DESIGN.md` (not just tuned around)
  and the one integration test that already probes it needs to reflect
  118's retirement of the reorder experiment its own xfail reason cites.
  Independent of the other three groups; touches firmware documentation
  and a Python test, not runtime code (specify-then-assert, minimal-to-no
  behavior change per the issue's own framing).
- **Config-schema hygiene** — 20 dead `control.*` keys and 4 dead
  `run_tour()` kwargs have zero living consumers and should not survive
  as "config as attic." Independent of the other three groups (verified:
  no field-name overlap with ticket 1's live estimator/shaper fields).
- **Documentation truth** — narrative comment relocation and stale
  doc-reference correction. Depends on the other three groups (it "trims
  only what survives" per the issue's own framing — relocating comments
  out of `move_queue.h` before ticket 1/2/3 land risks relocating
  soon-to-be-stale content).

### Step 3 — Subsystems and modules

- **`Sim::SimLoop`** (`src/host/robot_radio/io/sim_loop.py`) — purpose:
  configure a running sim from a robot's JSON config. Boundary:
  `configure_from_robot()`'s own two-tier push; gains a third push
  (estimator/shaper via `estimator_kwargs()`) but does not change Tier
  1/2's own boundary. Use case: SUC-067.
- **`App::Telemetry`** (`src/firm/app/telemetry.h`/`robot_loop.cpp`) —
  purpose unchanged (per-cycle outbound frame). Boundary: gains one new
  `flags` bit (16) set while a MOVE is active with angular+linear shaping
  disabled; no other behavior change. Use case: SUC-067.
- **TestGUI** (`src/host/robot_radio/testgui/`) — purpose unchanged
  (operator-facing session UI). Boundary: gains a status-bar banner + log
  line reacting to the new flags bit; the existing connect-time
  estimator-config push becomes redundant-but-harmless (idempotent acks)
  once `configure_from_robot()` also pushes it, per the issue's own
  "dedup or leave harmless" framing — no dedup is mandated. Use case:
  SUC-067.
- **`Motion` (documentation)** (`src/firm/motion/DESIGN.md`) — purpose:
  the persistent architecture record for `src/firm/motion/`. Boundary:
  gains one new Design-section contract paragraph (carried-axis ramp,
  decay-axis behavior, reversal-dwell asymmetry budget, axis-drop coast
  at chain boundaries — the last one net-new relative to the original
  issue text, tying directly to the fresh pool issue); loses the
  corresponding Open Questions entry. No runtime module changes. Use
  case: SUC-068.
- **`test_tour_closure_gate.py`'s boundary test** — purpose: the sole
  surviving tour-level coverage of SUC-003's "no dip to zero at a
  compatible same-`v_max` boundary" property (its own file-header
  comment, `:645-652`). Boundary: its `xfail` marker is re-evaluated (not
  merely re-pointed) against 118's retirement of the reorder experiment
  its current reason cites. Use case: SUC-068.
- **Config/schema layer** (`data/robots/*.json`, the pydantic
  `ControlConfig` model, `robot_config.schema.json`,
  `config_sync_allowlist.json`) — purpose: persisted/generated robot
  configuration. Boundary: 20 dead `control.*` keys removed; every live
  key (vel_gains, output_deadband, reversal_dwell, trackwidth, estimator,
  shaper — all confirmed live by 118's own delete-list discipline)
  untouched. This sprint's one data-model change. No use case (internal
  cleanup, no behavior change per the issue's own acceptance criteria).
- **`tour.py`** (`src/host/robot_radio/planner/tour.py`) — purpose
  unchanged (tour geometry + chained execution). Boundary: 4 dead kwargs
  (`a_max`, `alpha_max`, `cadence`, `inter_leg_settle`) +
  `DEFAULT_INTER_LEG_SETTLE` removed from `run_tour()`'s signature and
  the constants block; `omega_max` (live) untouched. No use case.
- **Documentation/test-narrative surface** (`move_queue.h`,
  `sim_harness.h`, `tour.py`'s docstring, `test_tour_closure_gate.py`'s
  xfail-reason strings, `.claude/rules/hardware-bench-testing.md`,
  `.claude/rules/coding-standards.md`, `testgui/__main__.py`'s GUI label,
  `docs/specification.md`, `docs/architecture.md`,
  `src/host/robot_radio/DESIGN.md`, `docs/protocol-v4.md`'s own §8
  cadence line) — not a module for cohesion purposes; relocation/
  correction only, zero behavior risk per the issue's own framing. No use
  case.

### Step 4 — Diagrams

**No component/module diagram.** As with 118, this sprint composes
nothing new: ticket 1 completes an existing two-tier push mechanism with
a third call to an already-existing function
(`estimator_kwargs()`, which already exists and is already called
elsewhere in this codebase — e.g. `testgui/__main__.py`'s connect-time
push); ticket 2 documents an existing, already-shipped behavior (carried
shaper state, decay, dwell asymmetry) and re-evaluates one test's `xfail`
status against a defect 118 already fixed; ticket 3 deletes fields;
ticket 4 relocates/corrects prose. No new module, no new cross-module
edge, no dependency-direction change anywhere in the four tickets — the
sprint-020/118 "nothing new is being composed" escape applies for the
same reason it did in 118.

**One real data-model change, no ERD needed.** Ticket 3 deletes 20
scalar fields from an existing, flat `control` config object — no new
entity, no new relationship, nothing an ERD would clarify beyond the
delete list itself (ticket 3's own acceptance criteria enumerate all 20
by name, same discipline 118 ticket 004 used for its own one-field
schema deletion).

**One wire-visible addition, documented in place, no diagram needed.**
Ticket 1 adds one bit (16) to an already-append-only-documented 32-bit
`flags` word (`docs/protocol-v4.md` §8.2). A bit-table row addition is
its own complete documentation; a diagram would not add anything the
table doesn't already say.

### Step 5 — What Changed / Why / Impact / Migration Concerns

**What Changed:**
- `io/sim_loop.py`: `configure_from_robot()` gains a call to
  `estimator_kwargs()` (Tier 1.5, alongside the existing Tier 1/2 calls).
- `telemetry.h`/`robot_loop.cpp`: new `flags` bit 16
  (`kFlagEventShapingDisabled` or equivalent name — ticket 1's own
  naming call, following the existing `kFlagEvent*`/`kFlagFault*` prefix
  convention), set every cycle a MOVE is active with angular+linear
  `ShaperLimits` both disabled.
- `testgui/`: status-bar banner + log line on the new bit; bench-script
  tooling (`turn_prediction_capture.py`, `estimator_capture.py`) prints
  it too, per the issue's own acceptance criterion.
- `docs/protocol-v4.md`: §8.2 bit-table row for bit 16; §8's own stale
  cadence line (~50Hz/20ms) corrected to 40ms/~25Hz (new finding, folded
  into ticket 4 per Solution above — a documentation-only fix, not
  ticket 1's, since it is unrelated to the new bit).
- `motion/DESIGN.md`: new §4 (Design) contract paragraph replacing the
  current §6 (Open Questions) tuned-around-limitation entry.
- `test_tour_closure_gate.py`: the boundary test's `xfail` marker either
  removed (if it now passes, expected given 118 retired the reorder
  experiment the current reason blames) or its reason re-pointed at a
  live issue with a concrete unblocking condition (if some other cause
  remains) — ticket 2's own acceptance criterion covers both outcomes.
- `data/robots/*.json` (×3), the pydantic `ControlConfig` model,
  `robot_config.schema.json`, `config_sync_allowlist.json`: 20 dead keys
  removed, in one commit, per 118's own established schema-deletion
  discipline.
- `tour.py`: 4 dead kwargs + `DEFAULT_INTER_LEG_SETTLE` removed.
- Documentation surface named in Step 3 above: relocated/corrected,
  sequenced last.

**Why:** Per Problem/Solution above — closes the one remaining silent-off
entry point, specifies a behavior every campaign has re-tuned around
instead of documenting, removes config-as-attic surface, and brings the
documentation tree back into agreement with the post-118 codebase.

**Impact on Existing Components:** Every bare `SimLoop` session (bench
scripts, headless tests, future scripts) that calls
`configure_from_robot()` starts running WITH shaping/anticipation active
by default after ticket 1 — any test or script that was implicitly
relying on the unshaped baseline (none identified, but the closure gate
and button-acceptance suite are the check) needs to still pass. The new
flags bit is purely additive (existing hosts reading `flags` as an
opaque bitmask are unaffected; only a host that specifically decodes bit
16 sees anything new). The boundary test's possible un-xfail (ticket 2)
restores real coverage of SUC-003's carried-velocity property that has
been running unconditionally-quarantined since 111-002. `run_tour()`
callers passing any of the 4 dead kwargs (none found in ticket 3's own
call-site inventory, matching the issue's own claim) would break — the
issue's acceptance criteria requires re-verifying this at execution time,
not just trusting the pre-118 inventory.

**Migration Concerns:** Real config-schema migration in ticket 3 (20
fields removed from JSON + pydantic + JSON-schema + allowlist, same
same-repo/same-deploy-cycle reasoning 118's own Migration Concerns
established for its one-field deletion — no robot runs an old JSON
against a new binary or vice versa in this project's workflow). No
migration for ticket 1's new telemetry bit (additive, append-only,
existing readers unaffected) or ticket 2/4's documentation-only changes.
Deployment sequencing: bench verification for all four tickets is
deferred to phase-B, per the same stakeholder mandate 118 recorded (see
Scope/Success Criteria above).

### Step 6 — Design Rationale

**Decision 1: `estimator_kwargs()` joins `configure_from_robot()`
directly, rather than being pushed by a new, separate call.** *Context:*
the issue itself frames this as "extend `configure_from_robot()`... one
change covers every caller." *Alternatives considered:* (a) a new
`SimLoop.configure_estimator_from_robot()` method callers must
separately remember to call — rejected, this is the EXACT shape of the
defect being fixed (an opt-in call site a caller can forget); (b) fold
the call directly into `configure_from_robot()`'s existing body, after
Tier 1/2 — chosen, matches the issue's own "one change covers every
configure_from_robot caller" reasoning and the function's own existing
docstring convention (numbered tiers, each independent). *Consequence:*
every existing and future `configure_from_robot()` caller (GUI, bench
scripts, tests, future scripts) inherits the push with zero caller-side
change — the issue's own acceptance criterion #2 ("both bench capture
scripts inherit the push with zero script changes") is a direct test of
this choice.

**Decision 2: new flags bit is 16, not a repurposed 8/10/12.** *Context:*
`docs/protocol-v4.md` §8.2's bit table already has three
declared-but-special bits in the low range: 8 (`kFlagFaultI2CNak`,
declared-not-wired but reserved for a FUTURE I2C NAK aggregate, a
different concern), 10 (`kFlagEventDeadmanExpired`, explicitly
"orphaned... left declared, not repurposed" per 116's own stated
policy), 12 (`kFlagEventConfigApplied`, declared-not-wired but reserved
for a FUTURE config-applied event, a different concern again).
*Alternatives considered:* (a) repurpose bit 10 (orphaned, "free" in the
sense nothing sets it) — rejected, 116 explicitly decided not to
repurpose it ("reassigning a bit number to a new meaning without a
version signal would be a silent protocol break for any reader still
checking it" — src/firm/app/DESIGN.md §6), and this sprint has no reason
to reverse that decision; (b) take the next genuinely free bit (16, the
first of the reserved 16-31 range) — chosen, matches the append-only
convention `docs/protocol-v4.md`'s own "Reserved, not reused" pattern
(§3, §6) already establishes for message field numbers, extended by
analogy to flag bits. *Consequence:* zero collision risk with any
existing or future use of bits 8/10/12; a host reading an old firmware's
frame simply never sees bit 16 set (safe default).

**Decision 3: the boundary test's disposition is "re-evaluate," not
"assume it now passes" or "assume it's still broken."** *Context:* the
issue's own text says "un-xfail once the loop-schedule work lands" — 118
IS that work, landed. But this sprint-planner has not run the test
(sprint-planners do not execute code); the xfail reason's stated cause
(the 112-005 reorder experiment) being retired is strong evidence, not
proof, the test now passes — the reason also names a SECOND effect
(114-006's fail-closed configuration requirement making the test fault
rather than dip) that may or may not still apply independently.
*Alternatives considered:* (a) mandate un-xfail unconditionally in the
ticket's acceptance criteria — rejected, this sprint-planner cannot
verify the test's actual current result and should not assert a fact it
hasn't checked; (b) require the ticket to RUN the test first, then
un-xfail if it passes or re-point the reason to a live issue if it still
fails for a documented, current cause — chosen, matches the issue's own
"un-xfail... or its xfail cites a live issue with a concrete unblocking
condition" acceptance criterion exactly. *Consequence:* ticket 2's
acceptance criteria are conditional on the ticket's own verification
step, not a pre-asserted outcome.

**Decision 4 (carried from 118's own precedent): overlay slot goes to
`src/firm/motion/DESIGN.md`.** *Context:* four subsystem `DESIGN.md`
files are candidates this sprint (`app`, `motion`, `robot_radio`,
`tests`), but the flat overlay directory holds only one `DESIGN.md` at a
time (116/117/118 precedent). *Alternatives considered:* each of the
other three — `app/DESIGN.md`'s own edit (one new flags-bit-table row) is
a single-line-class addition to an already-large enumeration, not a new
architectural narrative; `robot_radio/DESIGN.md`'s edits (the
`configure_from_robot()` push, the `planner/` dormancy correction, the
`tour.py` docstring relocation) are corrections/completions to existing
rows, not a new contract; `tests/DESIGN.md` may not need an edit at all
this sprint (the boundary test's disposition is a test-file change, not
necessarily a DESIGN.md-level architectural statement) — verify at
ticket 2's own execution time whether `tests/DESIGN.md` needs a touch;
if it does, it is a direct edit, not a slot contender, per this
decision. `motion/DESIGN.md` — chosen: it is the only one of the four
gaining genuinely NEW architectural content (a contract that did not
exist before, moved out of Open Questions into Design, directly
motivated by a freshly-filed pool issue with real accuracy data behind
it) rather than a correction or a table-row addition. *Consequence:*
`app/DESIGN.md` and `robot_radio/DESIGN.md` are edited directly on their
canonical path by tickets 1 and 1/4 respectively (own acceptance
criteria each); `tests/DESIGN.md` is touched directly by ticket 2 only
if execution reveals it needs to be.

### Step 7 — Open Questions

- **Exact name for the new flags bit.** `kFlagEventShapingDisabled` is
  this document's placeholder; ticket 1's implementer should follow the
  existing `kFlagFault*`/`kFlagEvent*` prefix convention (this is a
  fault-like condition — accuracy-degrading, not a normal event — so
  `kFlagFaultShapingDisabled` may read better; not blocking, pick either
  and update `docs/protocol-v4.md` §8.2 to match).
- **Whether `tests/DESIGN.md` needs a direct edit this sprint.** Per
  Decision 4 above — deferred to ticket 2's own execution; not a
  planning-time blocker either way.
- **Whether the boundary test now passes.** Per Decision 3 — deferred to
  ticket 2's own execution; both outcomes (un-xfail, or re-pointed xfail
  citing a new live issue) are acceptable per the ticket's own acceptance
  criteria.

## Design Overlay

Design-docs opt-in is enabled. Per the flat-overlay-slot precedent
established in sprints 116/117/118, this sprint touches (or may touch)
four subsystem `DESIGN.md` files but can only overlay one — see Design
Rationale Decision 4 above for the full reasoning.

**Overlaid** (seeded pristine via `seed_sprint_design_overlay(sprint_id="119",
doc_names=["../../src/firm/motion/DESIGN.md"])`, edited in place to add
the leg hand-off contract, diffed, and committed on `master` before
`acquire_execution_lock` branches the sprint off it):
- `src/firm/motion/DESIGN.md` — new §4 (Design) contract paragraph
  (carried-axis ramp, decay-axis behavior, reversal-dwell asymmetry
  budget, axis-drop coast at chain boundaries); the corresponding §6
  (Open Questions) entry removed. Owner: ticket 2.

Note: `docs/design/design.md` (the system doc 118 also overlaid
alongside `app/DESIGN.md`) is NOT overlaid this sprint — verified during
Architecture planning that it contains no existing mention of
`configure_from_robot()`, the sim/production config boundary, or
anything else this sprint's tickets change; there is nothing on it to
correct or extend.

**Not overlaid — edited directly on the canonical doc during execution,
by the ticket that owns the change** (same convention 118 used for
`src/sim/DESIGN.md`):
- `src/firm/app/DESIGN.md` — one new row in the §4 `flags` bit-string
  enumeration (bit 16). Owner: ticket 1 (own acceptance criterion).
- `src/host/robot_radio/DESIGN.md` — the `config`/`io` directory rows'
  description of `configure_from_robot()`'s now-three-tier push (owner:
  ticket 1); the `planner/` row's dormancy correction and `tour.py`
  docstring-relocation notes (owner: ticket 4). Two different tickets
  touch this one file at two different points in the sprint — both add
  acceptance criteria naming this file explicitly so neither silently
  clobbers the other's edit (ticket 4 runs last, after ticket 1, so
  ticket 4's own diff should include ticket 1's already-landed change).
- `src/tests/DESIGN.md` — touched only if ticket 2's own execution
  determines it needs the boundary-test disposition recorded there (Open
  Question above); if untouched, no acceptance criterion fires.

At sprint close, `overlay.apply()` copies `src/firm/motion/DESIGN.md`
onto its canonical target; the directly-edited files above are already
at their canonical location by then and need no apply step.

## Use Cases

Sized to the change: two sprint-level use cases (config-boundary
default-on, leg hand-off contract), continuing SUC numbering from sprint
118's allocation (SUC-063 through SUC-066). The config-attic/dead-kwarg
deletion and doc-relocation tickets do not get their own use case — pure
internal cleanup with no behavior change, per their own issues'
acceptance criteria (zero behavior diffs except the one GUI label
string).

### SUC-067: Default-on shaping/anticipation at the sim composition seam
Parent: none (closes an invisible-off-state defect the turn-execution
review's F1/D1 first identified; SUC-066's own land-at-zero completion
predicate is the feature this use case makes default-observable-and-on,
not itself a parent/child relationship)

- **Actor**: A test, bench-script, or GUI session author calling
  `SimLoop.configure_from_robot()` (internal developer-facing use case).
- **Preconditions**: A `SimLoop` connected to a running sim; a
  `RobotConfig` with non-`None` `estimator`/`control` sections (the
  common case — every shipped robot JSON has them).
- **Main Flow**:
  1. Caller calls `configure_from_robot(config)`.
  2. Tier 1 (calibration) and Tier 2 (motor boot config) push as before.
  3. NEW: a third push calls `estimator_kwargs(config)` and sends the
     result through the same `EstimatorConfigPatch` wire mechanism the
     TestGUI's own connect-time push already proved out.
  4. The robot's `ShaperLimits`/estimator weights are now live-tuned to
     match the JSON, matching what a real serial boot would bake in.
  5. If shaping ends up disabled anyway (e.g. the JSON's own shaper
     fields are absent/zero), `flags` bit 16 goes high on every frame
     while a MOVE is active — visible, not silent.
- **Postconditions**: A bare `SimLoop` session with no GUI and no manual
  push runs shaped/anticipated motion by default; the off state, when it
  occurs, is observable on the wire and in the TestGUI/bench-script log.
- **Acceptance Criteria**:
  - [ ] A bare `SimLoop` + `configure_from_robot()` session (no GUI, no
        manual push) runs Tour 1 with shaping active — per-leg accuracy
        matches the GUI-path bands; read-back/ack counts confirm the
        push landed.
  - [ ] `turn_prediction_capture.py`/`estimator_capture.py` inherit the
        push with zero script changes.
  - [ ] Flags bit 16 verified in sim: strip the push → bit asserts, GUI
        banner shows, bench tooling prints it; push present → bit clear.
  - [ ] Button-acceptance suite unaffected (still green at its tightened
        bands).
  - [ ] `docs/protocol-v4.md` §8.2 bit-table row added (append-only); no
        existing bit renumbered or repurposed.

### SUC-068: Chain-advance leg hand-off contract specified and asserted
Parent: SUC-051 (chain-advance seamless hand-off, sprint 116) — this use
case specifies what SUC-051's carried state SHOULD do, closing an Open
Question SUC-051's own implementation left unresolved

- **Actor**: `App::MoveQueue` (internal — no host-visible actor; observed
  via the boundary test's own assertions and the DESIGN.md contract a
  future maintainer reads).
- **Preconditions**: A chain-advance boundary between two queued `Move`s
  (`pendingCount() > 0` at the moment the active `Move` completes).
- **Main Flow**:
  1. The completing `Move`'s shaped axis (if the next `Move` also
     commands it) ramps from the carried `commandedSpeed()` — SUC-051's
     existing behavior, now stated as a contract rather than left
     implicit.
  2. An axis the next `Move` does NOT command decays per the contract's
     stated decay behavior, with an explicit heading-cost bound.
  3. A sign reversal on an axis at the boundary does not carry speed
     through — the contract states the accepted per-wheel dwell
     asymmetry (D→RT) and its heading budget, tying directly to why the
     chain-advance completion margin (`kStoppingMarginFactorChain`/
     `kDiscretizationCyclesChain`) differs from the final-move case: the
     axis-drop coast this step describes is the mechanism the pool issue
     (`chain-advance-completion-margin-narrow-pocket.md`) traces the
     narrow-pocket sensitivity to.
  4. The tour-level boundary test asserts 1-3 directly (not xfailed
     around them), or its xfail cites a live issue with a concrete
     unblocking condition if some cause independent of the retired
     112-005 experiment still applies.
- **Postconditions**: `motion/DESIGN.md` states the hand-off contract as
  a Design decision, not an Open Question; the isolated-vs-tour turn gap
  is measured and within the budget the contract states.
- **Acceptance Criteria**:
  - [ ] `motion/DESIGN.md` contract paragraph exists (carried-axis ramp,
        decay-axis behavior + heading-cost bound, reversal/dwell
        behavior + heading budget, axis-drop coast at chain boundaries);
        the corresponding Open Questions entry removed.
  - [ ] The `simple-velocity-control-acceleration-limited-shaper.md`
        issue's vExit design (exit velocity = next move's cruise on the
        axis; 0 on reversal or empty queue) is adopted or explicitly
        rejected in the paragraph.
  - [ ] `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`
        is re-run against the current tree; if it passes, `xfail` is
        removed entirely (not just re-pointed); if it still fails, the
        reason cites a live issue with a concrete unblocking condition,
        not the retired reorder experiment.
  - [ ] Tour vs. isolated turn gap measured and within the budget the
        contract states.
  - [ ] No behavior change beyond what the contract specifies as already
        true (specify-then-assert, per the issue's own framing) —
        anything the ticket discovers needs an actual behavior change
        stays within the land-at-zero acceptance bands already shipped
        by 118, per the issue's own "rides the land-at-zero ticket's
        acceptance bands" note.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [ ] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

Detail-promoted 2026-07-23. Four tickets, one per remaining issue,
dependency-ordered:

| # | Title | Depends On | Issue(s) |
|---|-------|------------|----------|
| 001 | Kill the silent-off shaping/anticipation config boundary | — | kill-the-silent-off-shaping-config-boundary.md |
| 002 | Specify and assert the chain-advance leg hand-off contract | — | specify-and-assert-the-leg-handoff-contract.md, chain-advance-completion-margin-narrow-pocket.md |
| 003 | Delete the config attic (20 dead `control.*` keys) and dead `run_tour` kwargs | — | delete-the-config-attic-and-dead-tour-kwargs.md |
| 004 | Relocate narrative comments to DESIGN.md/git; refresh stale doc references | 001, 002, 003 | relocate-narrative-comments-and-refresh-stale-docs.md |

Tickets execute serially in the order listed (`worktree: false`). 001-003
are independent of each other (each depends only on sprint 118, already
merged to master) and could in principle parallelize; 004 must run last
since it trims only what survives 001-003's own changes, and both 001 and
004 touch `src/host/robot_radio/DESIGN.md` — 004's own diff must include
001's already-landed edit there, not revert it (see each ticket's own
"design overlay coordination" note).
