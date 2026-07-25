---
status: pending
filed: 2026-07-24
filed_by: team-lead (stakeholder-directed parallel-effort kickoff)
related:
- extract-motion-library-to-src-motion.md
- firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
tickets: []
---

# Motion-library development: kick off the parallel effort (worktree/repo)

## Why now (stakeholder, 2026-07-24)

Sprints 123-125 are all firmware-base hardening and will run for a while.
The motion library — where every remaining exactness problem lives — has NO
scheduled work anywhere: the entire S1/S2 campaign sits in `later/`. The
prerequisites for parallel work already exist (sprint 122): `src/motion`
with the `Motion::WheelSink` boundary, its own CMakeLists, and the
standalone Python-free `motion_tests` build (three ctest executables incl.
an end-to-end chained-move scenario against `TestSim::WheelPlant`). The
stakeholder develops this on its own branch/worktree (likely its own repo
later); planning stays visible in this repo's pool.

## The contract pin (the one coordination hazard — read first)

Sprint 124 CHANGES the boundary: base primitive velocity → DUTY; the
velocity PID and `MoveWheels` handling relocate INTO `src/motion`; the
base-side per-wheel observer becomes the upward input
(`WheelEstimate` + `appliedDuty` up, per-wheel duty down —
`docs/design/base-explicit-loop-sketch.md` is the contract of record).
The motion effort therefore builds against the POST-124 contract from day
one — mock the observer input in `motion_tests` (trivial: the model plant
plus a pass-through estimate) rather than building on today's
velocity-shaped `WheelSink` and rebasing later. Boundary-header changes
require sign-off from BOTH efforts once 124 lands. Division of the
relocation: 124 (base repo) deletes the PID/MoveWheels from `firm/`; the
motion effort RECEIVES and owns them (gains become motion config) — one
joint checkpoint, not two competing edits to the same files.

## Backlog (pull from later/, in this order — substance unchanged from the
exactness campaign)

- **M1 — Terminal-settle completion** (`later/land-at-zero-at-orthogonal-
  chain-boundaries.md` + the 122-era falsified-analytic finding): complete
  on MEASURED state — |remaining| ≤ ε_θ AND |speed| ≤ ε_v, bounded settle
  window, no prediction constants. Prove on the model plant at unit speed:
  per-motion ≤0.1°, boundary leak ≤0.1°. This is the goal-doc
  non-negotiable 5, finally built where iteration is cheap.
- **M2 — Same-axis carry + S1 ratchet** (`later/chain-advance-reset-…` +
  `later/s1-gate-ratchet-…`): conditional reset, no-dip floor, then the S1
  numbers become hard motion-test asserts.
- **M3 — Heading hold on Distance moves** (`later/heading-hold-…`): the P
  loop on the uncommanded axis; with the PID now motion-resident this is
  purely in-library.
- **M4 — PID ownership + duty-plane bring-up** (joint checkpoint with 124):
  velocity loop tuned against the model plant; MoveWheels tier restored on
  the new primitive.
- **M5 — Estimator v2 / OTOS fusion sim-first** (`later/estimator-v2-…`):
  heading weights on, position arm, common-epoch consumption.
- **M6 — Tours 3/4 + S1/S2 closure gates** (`later/tour-3-…`): requires
  re-linking into `libfirmware_host` and running this repo's sim gates —
  the integration checkpoint back into the base repo, after 124/125.

## Gates

The goal doc's bars govern unchanged (`docs/design/goal-exact-tours.md`):
M1-M3 measured on the model plant (motion_tests, milliseconds per run);
M6 re-measured through the full sim (deterministic gate). A motion change
is done when its motion-test numbers hold AND the sim gate reproduces them
— divergence between the two is a base/boundary bug by definition, which
is exactly the diagnostic power the split was built for.

## Stakeholder to-do to start

1. Cut the branch/worktree from master (post-122).
2. Decide worktree-in-this-repo vs separate repo now (`git subtree split`
   is ready either way; worktree defers the decision at zero cost).
3. Point the motion effort's first session at this issue; M1 is fully
   specified and collision-free with 123 (framing) — only M4 touches 124's
  territory, at the named checkpoint.
