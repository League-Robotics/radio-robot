
---

## SUPERSEDED IN PART (stakeholder restructuring, 2026-07-24)

The stakeholder has redirected the architecture: the codebase splits into a
hardened FIRMWARE BASE (this repo's focus) and a separate MOTION LIBRARY
(developed on its own branch/worktree, likely its own repo). Two new issues
carry the plan: `extract-motion-library-to-src-motion.md` and
`firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md`.

Effect on this directive's sequence:

- **122** closes now per the stakeholder's recorded decision: revert 001 to
  the margin baseline (keep tau_plant plumbing + the falsified-analytic
  finding), restore the 2.5deg gate, do NOT proceed to 002/003 here. The
  reopened 001 is satisfied by the revert; 002 (same-axis carry) and 003
  (ratchet) transfer to the motion library's plan.
- **Next sprints in THIS repo:** the extraction issue, then base hardening
  (observer + characterization + base gate), then 125 (transport) and 127's
  hygiene — all firmware-base work.
- **The exactness sequence (terminal-settle completion, heading hold 123,
  tours 124, estimator v2 126, S1/S2 bars)** transfers unchanged in
  substance to the motion library's plan, executed against the standalone
  `motion_tests` target first, sim gates second. S3/S4 (bench accuracy 128,
  playfield 129) remain joint milestones — they need both the frozen base
  and the motion library.
- Goal doc `docs/design/goal-exact-tours.md` remains the governing target
  for the combined system; stage bars unchanged.
