---
status: pending
filed: 2026-07-23
filed_by: team-lead (stakeholder-directed re-plan directive)
related:
- land-at-zero-at-orthogonal-chain-boundaries.md
- heading-hold-during-distance-moves.md
- tour-3-icosagon-and-tour-4-infinity-test-patterns.md
- bench-move-commands-intermittently-never-reach-firmware.md
- i2c-safety-net-bit-conflates-otos-settle-wait-with-loop-schedule-health.md
- stale-ruckig-cmake-comment-and-dead-dev-family-docs.md
- testgui-dbg-otos-bench-verb-dead-on-serial-connect.md
tickets: []
---

# RE-PLAN sprints 122+ to close docs/design/goal-exact-tours.md (stakeholder directive)

## Standing instruction

The stakeholder has stopped execution after sprint 121. Before any sprint past
121 executes, the sprint planner re-plans 122-forward per THIS document. The
governing targets are [`docs/design/goal-exact-tours.md`](../../docs/design/goal-exact-tours.md)
(stage bars S1-S4) and the gap analysis in
[`docs/code_review/2026-07-23-exactness-review.md`](../../docs/code_review/2026-07-23-exactness-review.md)
(§4 bench, §5 OTOS, §6-7 hygiene, §8 validation). Sprint 121 is NOT re-planned —
it executes as detailed (its SUC-072/073/074 already carry the right numbers).

Current plan's defect, in one line: **122-127 as written finish the sim (S1) and
bench reliability, and contain zero work toward S2 (OTOS fusion), S3 (bench
accuracy), or S4 (playfield)** — the second half of the goal. This directive
keeps 122-124, keeps 125, and rewrites/adds the rest.

## Step 0 — file the missing issues first

Create these pool issues (content by reference; each becomes its sprint's
spine). Names are prescriptive:

1. `s1-gate-ratchet-harden-ideal-chip-gates-at-goal-bars.md` — from review §8
   (G3): convert the ideal-chip xfail gates to permanent hard asserts at the S1
   bar (per-motion ≤0.1°/≤1 mm; tour net ≤0.5°, closure ≤5 mm, per-leg straight
   gain ≤0.1°) once 122 lands; a bar that cannot be met is answered with a
   named, measured physical floor — never a loosened tolerance.
2. `estimator-v2-otos-fusion-sim-first.md` — from review §5 (G1): (a) enable
   the existing heading/omega weights in sim and validate against truth;
   (b) ADD the position-fusion arm (schema `EstimatorConfigPatch` weight for
   x/y + staleness), StateEstimator blend, and the consumption decision —
   recommended: `MoveQueue` stop conditions consume ESTIMATOR pose; `Odometry`
   stays the raw encoder integrator (never corrupted). Wrapped-OTOS-heading vs
   unwrapped-odometry discipline is designed up front, not discovered.
   (c) promote the realistic-error-profile closure gate from xfail to a hard
   S2 gate (per-motion ≤0.5°/≤5 mm; tour ≤1°/≤25 mm).
3. `bench-accuracy-campaign-s3.md` — from review §4 (G2): calibration battery
   on the tour robot — velocity loop (including the `later/nocal-straight-
   terminal-wedge-needs-velocity-integrator.md` vel_ki work), per-wheel travel
   calib + trackwidth, OTOS OL/OA scales; **physical precondition, stakeholder
   task: remount the OTOS rigidly and clear `otos_untrusted`**; then real-OTOS
   fusion enabled on hardware and a numeric S3 bench gate (per-motion ≤1°/≤1%;
   tour ≤3°/≤50 mm) run on the stand, mirroring the sim gate's per-leg
   assertions.
4. `playfield-verification-s4.md` — from review §8 (G4): camera-truth gates
   for all four tours at the S4 bar (≤2°/≤2%; tour closure ≤100 mm; TOUR_3
   reads as a circle, TOUR_4 crossings within band on camera).

Re-stamp `sprint:` assignments on existing pool issues per the table below.

## The re-planned sequence

Numeric order = recommended execution order. 125 may run any time hardware is
available (it has no dependency on 122-124); nothing else reorders.

| Sprint | Disposition | Goal (one line) | Stage |
|---|---|---|---|
| 122 | **KEEP + one ticket** | Same-axis carry through chain boundaries; margin machinery deleted, not re-swept. **Add ticket: the S1 gate ratchet** (issue 1) — S1 is declared met (or its floor named) at this sprint's close | S1 |
| 123 | **KEEP as detailed** | Heading hold on Distance moves (re-adds `heading_kp` with a consumer) | S1→S3 payoff |
| 124 | **KEEP as detailed** | TOUR_3/TOUR_4 patterns; arc-axis shaping gap measured with go/no-go | S1 evidence |
| 125 | **KEEP + one ticket** | Bench link transport reliability (envelope loss). **Add ticket: fix `.claude/rules/hardware-bench-testing.md`** (wrong banner, "ack slot", stale deploy commands — it is the live bench checklist and it lies; review §7) | S3 prereq |
| 126 | **REWRITE** (was: dead-legacy cleanup) | **OTOS estimator v2, sim-first** (issue 2). Ends with the S2 hard gate green | **S2** |
| 127 | **REWRITE** (was: I2C bit only) | **Bench diagnostics trust + legacy hygiene**, merged: the I2C bit-6 semantics fix (existing issue, unchanged scope, stakeholder picks candidate b/c) PLUS the old-126 legacy scope (Ruckig/DEV/DBG) PLUS review §6-7 residue: the four comment/UI lies, the naming-residue list (`writePoseMm`, `_cm` cluster, 3 stray `Ms` locals), `src/sim/DESIGN.md` walkthrough rewrite, protocol-v4 bit-6 row, monolith growth-freeze rule recorded in `.claude/rules/` | hygiene, pre-S3 |
| 128 | **NEW** | **Bench accuracy campaign** (issue 3). Cannot start before: 125 done (transport trustworthy), 126 done (fusion exists), 127's bit-6 fix done (fault flags trustworthy on the stand), and the stakeholder's physical OTOS remount | **S3** |
| 129 | **NEW** | **Playfield verification** (issue 4). Goal doc flips to "met" only when this closes | **S4** |

Why this order and not OTOS-before-heading-hold: 123's hold uses odometry
heading and is correct without fusion; fusion then upgrades its reference for
free. Why bit-6 before the bench campaign: S3 debugging on a stand with a
lying fault bit burns bench time (120-003's own finding). Why hygiene merged
into one sprint: both halves are low-risk, no-behavior-change work; two
ceremony-sized sprints for it is overhead.

## Standing rules for every re-planned sprint (from the goal doc; enforce in review)

1. Success criteria are NUMBERS tied to a stage bar, verified against sim
   ground truth or bench/camera reference — never adjectives.
2. Gates ratchet: once a stage bar is met its gate is a hard assert forever;
   no tolerance is ever loosened to make a sprint close.
3. No tuned compensation constants. Any new constant states its physical
   derivation. A sprint whose fix requires re-sweeping an existing constant
   has found a defect, not a tuning opportunity.
4. Anything requiring stakeholder hands (OTOS remount) or a stakeholder
   decision (estimator consumption point; carry-dip acceptance in 122; bit-6
   candidate in 127) is surfaced at sprint START, not discovered mid-sprint.
5. Bench sprints obey `.claude/rules/hardware-bench-testing.md` — after 125
   fixes it.

## Acceptance for this re-plan itself

- The four Step-0 issues exist and are assigned per the table; existing pool
  issues re-stamped (old-126 scope issues → 127; i2c issue stays 127).
- `clasi/sprints/126-*` and `127-*` sprint.md files rewritten to the new
  scopes; `128-*` and `129-*` created with Goals/Problem/Success Criteria at
  the same detail standard as the current 122-125 plans; the 122 and 125
  added tickets appear in their sprint.md scopes.
- Every success-criteria block cites its goal-doc stage bar explicitly.
- The stakeholder has signed off on the sequence table above (or amended it)
  before 122 begins executing.

---

## EXECUTED (team-lead, 2026-07-23 late — stakeholder-directed)

The stakeholder had the team-lead perform this re-plan directly. State now in
the tree:

- Step-0 issues filed and stamped: `s1-gate-ratchet…` (→122),
  `estimator-v2-otos-fusion-sim-first` (→126), `bench-accuracy-campaign-s3`
  (→128), `playfield-verification-s4` (→129); old-126 legacy issues restamped
  →127.
- `sprints/122…/sprint.md` REWRITTEN: analytic completion
  (`remaining <= |speed_measured| * (kCycle/2 + tau_plant)`) is the plan of
  record, ALL margin constants deleted (incl. 121-003's interim 0.67),
  `tau_plant` as the one new named bench-derived constant, same-axis
  conditional reset, S1 gate ratchet. No sweeping — a miss means re-derive.
- `126-otos-estimator-v2-sim-first/` (renamed from dead-legacy-cleanup) and
  `127-bench-diagnostics-trust-and-legacy-hygiene/` (renamed from
  i2c-safety-net-bit-semantics; merges old-126 + old-127 + review §6–7
  residue) rewritten; `128-bench-accuracy-campaign-s3/` and
  `129-playfield-verification-s4/` created. 125 gained the bench-rules-doc
  ticket. 123/124 unchanged.
- Directory renames were done with git mv; the sprint-planner agent should
  RECONCILE its MCP/DB state with these paths on resume rather than
  re-creating parallel sprints, then detail-plan each sprint at execution
  time as usual. Close 121 first per the stakeholder's recorded
  Accept-and-defer amendment (analytic completion → 122; 0.67 merges only as
  a labeled interim defect marker).
- Remaining stakeholder gates unchanged: consumption-point decision at 126
  start; OTOS remount before 128 hardware work; bench/camera sessions at
  125/127(bit-6)/128/129.
