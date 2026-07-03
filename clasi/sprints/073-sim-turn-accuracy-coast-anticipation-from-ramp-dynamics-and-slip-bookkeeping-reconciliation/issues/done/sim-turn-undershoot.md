---
status: done
sprint: '073'
tickets:
- 073-001
- 073-002
- 073-003
- 073-004
---

# SIM turns miss the commanded angle — coast constant + slip bookkeeping (measured)

## Symptom

In SIM mode, commanded turns land off target. Stakeholder-observed (TestGUI,
2026-07-02): 90° → ~87°, 180° → ~3° short, 300° → 2.8°–6° short ("kind of
all over"). Setting bodyRotScrub to 0.97 appeared to do nothing.

## Measured root causes (sim experiment, 2026-07-02)

Batch runs against `tests/_infra/sim` (fresh dylib, `GET rotSlip` → 0.920,
scrub 1.0, trackwidth 128):

| setup                       | RT 4500 | RT 9000 | RT 18000 | RT 30000 |
|-----------------------------|---------|---------|----------|----------|
| clean sim (no profile)      | +0.72°  | +4.54°  | +13.55°  | +22.63°  |
| default profile (ste=0.26)  | −8.90°  | −15.66° | −28.57°  | −44.66°  |

(miss = true body rotation − commanded; `ste` = `slip_turn_extra`)

Every measurement fits:

    body ≈ commanded / (cfg.rotationalSlip × (1 + slip_turn_extra)) − 3.3°

Three independent defects compose:

1. **Constant ~3.3° coast shortfall — `kRtCoastArcMm` mistuned ~2×.**
   `Planner::beginRotation` (source/control/PlannerBegin.cpp) arms the
   ROTATION stop 8.0 mm of per-wheel arc early (`kRtCoastArcMm`, ~7.2° at
   tw=128) expecting the SOFT ramp-down to coast through the remainder. The
   BVC ramp actually covers only ~4.3 mm (~3.9°) from 100°/s, leaving every
   turn ~3.3° short of its natural endpoint regardless of angle. This is the
   whole story behind the stakeholder's constant ~3° miss. Fix direction:
   compute the anticipation from the actual ramp-down dynamics
   (ω²/(2·decel) · tw/2) instead of a hand-tuned constant.

2. **Proportional term — three slip factors with no shared bookkeeping.**
   The planner inflates the arc target by `1/effectiveSlip(cfg.rotationalSlip
   = 0.92)` (boot-time copy); the ROTATION stop consumes *reported* encoders,
   which over-report turns by `(1 + slip_turn_extra)` when a field profile is
   applied; the plant's body-truth rotation applies
   `effectiveSlip(_rotationalSlip) × bodyRotScrub` — and in the TestGUI sim
   the first factor is dead (see 3), so nothing physically scrubs. Net:
   - clean sim **over**-rotates +8.7% (the planner's ÷0.92 is never scrubbed
     back by the ideal plant) — the known "RT 9000 → ~95°" behavior;
   - TestGUI default profile (ste=0.26) **under**-rotates ~−14% net;
   - the two cancel exactly at ste ≈ 0.087, leaving only the constant 3.3°
     miss — evidently ≈ the stakeholder's current persisted profile.
   Fix direction: make the sim's default plant actually model the scrub that
   `rotationalSlip=0.92` compensates (body slip 0.92), or boot the sim config
   with rotSlip=1.0 — either way the factors must be co-owned, not three
   independently-set knobs.

3. **`setSlip` couples the encoder knob to a dead body-slip channel.**
   `PhysicsWorld::setSlip(straight, turnExtra)` also sets `_rotationalSlip =
   straight + turnExtra` (PhysicsWorld.h:128). The TestGUI path negates
   `slip_turn_extra` (encoder over-report convention), so `_rotationalSlip`
   goes negative and `effectiveSlip()` maps it to 1.0 — silently no
   body-truth slip, ever, in TestGUI sim mode. The body channel is
   unreachable from the GUI (SIMSET has no rotSlip key; registry is
   bodyRotScrub/bodyLinScrub/trackwidth/offsets/enc*/otos* only).

Also explained:

- **"bodyRotScrub 0.97 does nothing"** — it works (measured: −2.2° at 90°,
  0.80 gives −14.9°) but a 0.97 scrub only moves a 90° turn ~2°, invisible
  next to the run-to-run scatter.
- **Scatter at 300° (2.8°–6°)** — the stop condition is evaluated once per
  ~24 ms tick; at 100°/s that quantizes the stop by up to ~2.4°, plus any
  encoder noise in the persisted profile.

## Acceptance sketch

- Coast anticipation derived from ramp dynamics (or re-tuned + regression
  test): clean-sim RT lands within ~1° across 45°–300°.
- Slip bookkeeping reconciled so the ideal sim with a neutral profile turns
  the commanded angle (no baked-in +8.7%).
- Body-truth slip channel either properly reachable or removed from
  `setSlip`'s side effects.
- Regression tests over the RT angle sweep (45/90/180/300) asserting miss
  bounds.

## Repro

`clasi/issues/` sibling script of record: the measurement harness lives at
the session scratchpad `turn_experiment.py` pattern — `Sim()` +
`set_field_profile()` + `RT <cdeg>` + accumulate `get_true_pose()` heading
deltas; ROTSTOP EVT gives arc/tgt (per-wheel mm).
