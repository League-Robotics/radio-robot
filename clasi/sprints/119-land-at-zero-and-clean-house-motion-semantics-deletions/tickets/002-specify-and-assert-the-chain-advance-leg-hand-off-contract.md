---
id: '002'
title: Specify and assert the chain-advance leg hand-off contract
status: open
use-cases:
- SUC-068
depends-on: []
github-issue: ''
issue:
- specify-and-assert-the-leg-handoff-contract.md
- chain-advance-completion-margin-narrow-pocket.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Specify and assert the chain-advance leg hand-off contract

## Description

Chain-advance deliberately carries shaper state across legs (SUC-051),
and a D→RT boundary crosses `NezhaMotor`'s per-wheel 100ms reversal dwell
on the reversing wheel only (asymmetric by construction). None of this is
specified anywhere — `motion/DESIGN.md`'s own Open Questions section
lists it as a tuned-around limitation. Write the contract; assert it in
the one integration test that already probes it.

**A design-overlay draft already exists** —
`clasi/sprints/119-land-at-zero-and-clean-house-motion-semantics-deletions/design/DESIGN.md`
(the `src/firm/motion/DESIGN.md` overlay, this sprint's chosen overlay
slot) already has a full draft "Chain-advance leg hand-off contract"
paragraph in its §4 Design section, written during planning from a direct
read of `move_queue.cpp`. **This ticket's job is to verify/refine that
draft against the actual shipped code** (same convention 118 ticket 004
used when it refined the app/DESIGN.md overlay note written ahead of its
own execution), not write it from scratch. The draft's headline finding,
already verified once during planning but worth re-confirming: the
original issue's proposed-fix text speculated "carried axis the next
Move does NOT command: decay behavior (current: shaped decay from
carry-over)" — **this is stale**. The actual shipped mechanism
(`move_queue.cpp`, the unconditional reset ahead of the chain-advance/
drain branch) is an UNCONDITIONAL hard reset of the completing axis's
shaper to `(0, 0)` at every completion boundary, tested against a
conditional-decay variant in 118 ticket 003's resolution and kept because
the decay variant bought nothing (best worst-case 2.932° vs. no
improvement). Verify this is still accurate, then finalize the paragraph.

**New pool issue ties directly to this contract**:
`clasi/issues/chain-advance-completion-margin-narrow-pocket.md` (filed
2026-07-23 from 118 ticket 003's resolution) found the chain-advance
completion margin (`kStoppingMarginFactorChain=0.60` +
`kDiscretizationCyclesChain=0.53`, `move_queue.cpp`) sits in a narrow
accuracy pocket, root-caused to exactly this contract's subject: a
chain-advance turn hands its axis to a Move that doesn't command it, and
completion is scored at the ack instant while the post-handoff coast is
only partially visible. **This ticket SPECIFIES that axis-drop-coast
mechanism (why the chain constants differ from the final-move case) — it
does NOT re-derive `kStoppingMarginFactorChain`/`kDiscretizationCyclesChain`
or otherwise change `MoveQueue::landAtZero()`'s behavior.** Closing the
narrow pocket itself is out of scope, per the pool issue's own "not
urgent... future sprint" framing.

## Proposed fix (per the issue, refined by the planning-time draft)

1. **One contract paragraph in `src/firm/motion/DESIGN.md`'s §4 Design
   section** (already drafted in this sprint's overlay — verify/refine,
   don't write from scratch), stating at minimum:
   - Carried axis the next Move commands: ramps from carried
     `commandedSpeed()` — SUC-051, unchanged, keep.
   - The axis matching the ENDING Move's own stop-condition kind: hard
     reset to `(0, 0)`, unconditionally, chain-advance or drain (verify
     against `move_queue.cpp`; correct the original issue's stale "decay"
     premise if the draft's finding holds).
   - Sign reversal: subsumed by the unconditional reset above (no
     separate case) — BUT the `NezhaMotor` `reversal_dwell_ms` hardware
     asymmetry is a genuinely separate, still-open decision: accept the
     asymmetric per-wheel dwell (state its measured heading-cost budget)
     OR specify symmetric dwell. **This ticket must pick one explicitly.**
   - vExit design reference
     (`simple-velocity-control-acceleration-limited-shaper.md`): the
     draft's analysis is that the shipped unconditional reset already
     realizes vExit's "0 on reversal or empty queue" half unconditionally
     (a conservative superset), and the surviving axis's continuity
     already realizes the "ramp from next move's cruise" half — verify
     this reading and either adopt it explicitly or reject it with
     reasoning in the paragraph.
   - Axis-drop coast at chain boundaries: the mechanism the narrow-pocket
     pool issue traces the margin's sensitivity to (draft already covers
     this — verify and refine).
   - Move this content out of §6 Open Questions once §4 states it (the
     overlay draft already struck the old bullet with a pointer — verify
     the pointer is accurate once the §4 paragraph is finalized).
2. **Assert it in the boundary test**:
   `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`
   (`src/tests/testgui/test_tour_closure_gate.py:694`). **Verified during
   planning**: this test's current `xfail(strict=False)` reason is
   entirely about the 112-005 cycle-order-hoist experiment ("frame.twist
   oscillates... a direct, confirmed consequence of the live reorder
   experiment") — that experiment was RETIRED by 118 ticket 001. **Run
   this test first.** If it now passes, remove the `xfail` marker
   entirely (not merely re-point its reason — the stated cause is gone).
   If it still fails for some cause independent of the retired
   experiment, re-point the reason to a live issue with a concrete
   unblocking condition (open a new issue if needed) — do not leave it
   citing the reorder-experiment theory once that theory is known-stale.
   Either way, once the test is either passing-and-unmarked or citing a
   live cause, extend/adjust its own assertions so it actually checks the
   contract paragraph's stated carried-velocity/reset/reversal-dwell
   behavior, not just the pre-existing "no dip to zero" property.

Per the original issue's own framing: "Behavior changes here should be
minimal-to-none; this is specify-then-assert." Any behavior change this
ticket discovers it actually needs (e.g. the reversal-dwell
accept-vs-symmetrize decision, if symmetric dwell is chosen) rides the
land-at-zero acceptance bands already shipped by 118 ticket 004 — do not
widen them.

## Acceptance Criteria

- [ ] `motion/DESIGN.md` contract paragraph finalized (verified/refined
      against `move_queue.cpp`, not assumed from the planning-time
      draft) — carried-axis ramp, completing-axis reset (or corrected
      description if the draft's finding doesn't hold), reversal/dwell
      decision made explicitly, vExit adoption/rejection stated, axis-drop
      coast defined. Corresponding §6 Open Questions entry removed
      (already struck in the draft — verify the final pointer).
- [ ] The `simple-velocity-control-acceleration-limited-shaper.md`
      issue's vExit design is explicitly adopted or explicitly rejected
      in the paragraph, with reasoning either way.
- [ ] `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`
      re-run against the current tree first. If it passes: `xfail`
      removed entirely. If it still fails: reason re-pointed to a live
      issue with a concrete unblocking condition (not the retired
      reorder experiment).
- [ ] That same test's assertions extended to check the contract's
      stated behavior (carried velocity through a compatible boundary;
      the reversal/dwell decision), not just the pre-existing
      no-dip-to-zero check.
- [ ] Tour vs. isolated turn gap measured and within the budget the
      contract paragraph states.
- [ ] No behavior change beyond what the contract specifies as already
      true, except the explicit reversal-dwell decision if it requires
      one — any such change stays within 118's already-shipped land-at-zero
      acceptance bands.
- [ ] Full `uv run python -m pytest` suite green; sim tour-closure gate
      and button-acceptance suite green.
- [ ] Bench verification is DEFERRED to the phase-B bench session — not
      required to close this ticket.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite); the
  named boundary test specifically, before and after the xfail
  disposition change; sim tour-closure gate; button-acceptance suite.
- **New tests to write**: extended assertions on the existing boundary
  test per the contract (see Proposed fix step 2); if the reversal-dwell
  decision changes behavior, a targeted test for the new dwell semantics.
- **Verification command**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py -v`
  to directly observe the boundary test's disposition, then the full
  suite.
