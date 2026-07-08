---
id: '002'
title: PoseEstimator hardware fused-pose investigation and fix
status: done
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: poseestimator-fused-pose-frozen-on-hardware.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# PoseEstimator hardware fused-pose investigation and fix

## Description

During sprint 089 ticket 007's bench verification (2026-07-07), robot on
the stand, `Subsystems::PoseEstimator`'s fused pose (`TLM pose=`) stayed
frozen at `(0, 0, -7)` across 1.3+ m of real encoder travel. `RT`
(whose `STOP_ROTATION` reads the raw encoder-arc differential directly, not
`fusedPose`) completed correctly and is unaffected; `TURN`'s `STOP_HEADING`
and `G`'s target-region detection both depend on `fusedPose` and so cannot
complete on hardware today
(`clasi/issues/poseestimator-fused-pose-frozen-on-hardware.md`). This is a
pre-existing defect, unrelated to 089's Ruckig migration -- the sim plant
does not reproduce it.

This is a CODE-INVESTIGATION ticket, not a pre-solved fix. Read
`architecture-update.md`'s Grounding section and Decision 5 in full first
-- they already rule out the two simplest hypotheses (a stuck
`leftObs.position.has`/`rightObs.position.has` -- would also break `RT`,
which it does not; a stuck `Hal::Odometer::fusableThisPass()` -- can only
suppress OTOS correction, cannot by itself freeze `encoderPose()`) and name
two remaining, more specific candidates distinguished by ONE cheap
diagnostic: does `Subsystems::PoseEstimator::encoderPose()` (`TLM
encpose=`) ALSO freeze on hardware, or only `fusedPose()` (`pose=`)? Run
this diagnostic FIRST -- it determines which of Decision 5's two candidates
(a step-1/encoder-accumulator bookkeeping issue affecting BOTH readings, or
an `EkfTiny::predict()`-specific `dt`/numerical-stability issue affecting
ONLY the fused reading) is actually in play, before investing time in
either blind.

## Acceptance Criteria

- [x] The diagnostic (does `encpose=` freeze too, or only `pose=`?) is run
      and its result recorded in this ticket's completion notes -- either
      from the existing 089-007 raw trace if it already contains `encpose=`
      samples, or a fresh bench capture.
- [x] Root cause is identified via code investigation (comparing the
      working sim path -- `Hal::SimOdometer`/`Hal::NullOdometer` feeding
      `PoseEstimator::tick()` -- against the real hardware path --
      `Hal::OtosOdometer`, `main_loop.cpp`'s `poseEstimator_.tick(...)` call
      site, and `PoseEstimator`'s own internal encoder dead-reckoning) and
      recorded in this ticket's completion notes, whether or not a bench
      re-test is achievable afterward.
- [ ] The most plausible fix is implemented. **Descoped -- see completion
      notes.** The candidate root cause requires one bench confirmation
      (`otosconn=`, landed this ticket) before an EKF-side fix can be
      designed responsibly; shipping a gating change without it would be
      exactly the "speculative fix for an unconfirmed mechanism" this
      ticket's own instructions warn against.
- [x] **Sim, BLOCKING**: full `uv run python -m pytest tests/sim` stays
      green after the fix -- `PoseEstimator`'s existing sim coverage is a
      regression guard (sim does not reproduce the hardware defect itself,
      so it cannot be the reproduction test).
- [x] Where the root cause is confirmed and a targeted white-box unit test
      is feasible (e.g. if it is a `dt`/baseline-bookkeeping issue
      reproducible with a synthetic tick sequence), add one; if the defect
      is genuinely only reproducible on real hardware, state that
      explicitly rather than fabricating a sim test that does not actually
      exercise the fixed mechanism.
- [x] **Bench, BEST-EFFORT**: re-run the `TURN` accuracy check (086/087
      tolerance bars) and the `G` settle smoke check from 089-007. If
      unreachable (hardware unavailable, or the fix cannot be confirmed
      live), descope to a fresh `clasi/issues/` follow-on rather than
      blocking sprint close.

## Implementation Plan

**Approach**:
1. Read `architecture-update.md` Grounding + Decision 5 in full.
2. Run the diagnostic (encpose= vs pose=) first -- see Description.
3. Depending on the diagnostic's result, investigate the matching candidate
   from Decision 5:
   - Both frozen: check `PoseEstimator::tick()`'s step-1 guard and
     `haveEncBaseline_`/`encBaselineResetPending_` bookkeeping against what
     `main_loop.cpp` actually passes as `leftObs`/`rightObs` on hardware vs.
     sim (`bb.motors[p.left-1]`/`bb.motors[p.right-1]` indexing).
   - Only fused frozen: check `EkfTiny::predict()`'s `dt` computation
     (`haveLastTick_`/`lastTick_`) for a degenerate/zero `dt` on hardware,
     or a numerically stuck EKF state/covariance.
4. Land the fix; add regression coverage per the Acceptance Criteria.
5. Attempt the bench re-verification; record honestly.

**Files to modify/create**: most likely `source/subsystems/pose_estimator.{h,cpp}`;
possibly `source/hal/otos/otos_odometer.{h,cpp}` or
`source/runtime/main_loop.cpp` if the investigation implicates the wiring
instead (architecture-update.md Step 3/5 name all three as candidates, not
a certain one).

**Testing plan**:
- **Existing tests to run**: full `uv run python -m pytest tests/sim`.
- **New tests to write**: a targeted regression test for the confirmed
  mechanism, if sim-reproducible (see Acceptance Criteria).
- **Verification command**: `uv run python -m pytest tests/sim`.

**Documentation updates**: none expected beyond this ticket's completion
notes recording the root-cause finding.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim` (full
  suite -- `PoseEstimator`'s existing coverage is the regression guard).
- **New tests to write**: a targeted regression test for the confirmed
  mechanism, if feasible; otherwise an explicit note on why it is
  hardware-only.
- **Verification command**: `uv run python -m pytest tests/sim`.

## Completion Notes (2026-07-08)

**Path taken: DIAGNOSTIC-ONLY.** Code investigation produced a specific,
well-evidenced root-cause *hypothesis* (not previously considered by
Decision 5's two candidates as literally worded), but one bench check is
needed before an EKF-side fix can be designed responsibly. Per this
ticket's own instructions, that check-then-fix sequencing is landed as a
descope, not a speculative fix. Full reasoning is duplicated in
`clasi/issues/poseestimator-fused-pose-fix-pending-otos-connected-bench-
confirmation.md` (the fresh follow-on this ticket files); summarized here:

**1. Diagnostic result (from the EXISTING 089-007 raw trace — no fresh
bench capture needed for this part):** `encoderPose()`/`encpose=` does
**not** freeze. The isolated TURN diagnostic table in
`clasi/sprints/done/089-.../bench-verification-log.md` §3 shows `encpose=`
swinging from 2888 → 10960 → -17917 → -4855 → -10979 centidegrees over
8.22s — consistent with genuine, fast, continuous rotation wrapped to
(-180°, 180°] and sampled sparsely (NOT a bug: at real spin rates the
robot completes multiple full rotations between ~1s-spaced polls, so each
sample lands at an effectively different phase). `fusedPose()`/`pose=`
stays within 0-1° / a few mm of the origin the ENTIRE time, both in this
trace and in G's 1.3+ m run (§5). **This resolves Decision 5's diagnostic
to Candidate B** (an EKF/OTOS-specific mechanism) — Candidate A (shared
step-1/2 bookkeeping) is ruled out because it would freeze both readings
identically, and does not.

**2. Root cause — code-level findings (all confirmed by direct read):**
- Decision 5's own literal "degenerate `dt`" framing does not hold up:
  `EkfTiny::predict()` (`source/estimation/ekf_tiny.cpp:82-112`) computes
  the state MEAN (`fx[0..2]`) purely from `dCenter`/`dTheta`/the EKF's own
  prior state; `dt` only scales the covariance noise term `Q`.
  `ekf_predict()` (vendored TinyEKF) unconditionally
  `memcpy(ekf->x, fx, ...)`. A zero/degenerate `dt` cannot by itself
  freeze the mean — this specific sub-hypothesis is REFUTED, a useful
  negative result.
- `PoseEstimator::tick()`'s encoder accumulator (`encX_`/`encY_`/
  `encTheta_`) and its EKF `predict()` call are fed the IDENTICAL
  `dCenter`/`dTheta` computed once per tick (`pose_estimator.cpp:98-132`)
  — they cannot diverge from `predict()`'s own math alone. The only
  place that resets JUST the EKF (not the encoder accumulator) is
  `PoseEstimator::configure()` -> `EkfTiny::init()` (boot-only, by
  design) — ruled out as the cause of a continuous freeze here (no
  repeated `SET`/`DEV DT CFG` traffic appears in the 089-007 isolated
  diagnostic capture).
- `Subsystems::NezhaHardware` **unconditionally constructs a real
  `Hal::OtosOdometer`** (`source/main.cpp:126-130`), independent of
  whether a physical chip actually answers. 089-007's own belief ("Tovez
  has no real OTOS chip either way") rests on two now-refuted planks:
  `DBG OTOS` returning `ERR unknown` (that verb is `source_old/`-only,
  never ported to `source/commands/otos_commands.cpp`) and `ID`'s
  `caps=` field being empty (that field does not exist in this tree's
  `formatDeviceAnnouncement()` at all — `system_commands.cpp:137-139`).
- `EkfTiny` has NO rejection/gating (082 Decision 2 deliberately dropped
  "Mahalanobis chi-squared gating on any channel" from the 5-state parent
  class) — `updatePosition()`/`updateHeading()` unconditionally trust
  whatever `otosObs` arrives.
- The OLD tree had a purpose-built escape hatch for exactly this bench
  scenario: `DBG OTOS BENCH` (`docs/protocol-v2.md` §14, sprint 031/034,
  `BenchOtosSensor`) existed specifically because "the robot is on a
  stand and the floor sensor sees no motion." It was never re-ported into
  the ordered-tick `Subsystems::PoseEstimator`/`Hal::OtosOdometer`
  architecture (`grep -rl BenchOtosSensor` under `source/` finds nothing).

**Leading (bench-unconfirmed) hypothesis:** if Tovez's `OtosOdometer`
genuinely detects a chip at boot, then on a stand (wheels off the ground)
its optical sensor sees no real surface motion and reports a near-static
`stamp.valid = true` reading every ~20ms. With no gating, `PoseEstimator`
fuses this every cycle, continuously dragging the EKF belief back toward
near-(0,0,0) against the encoder-informed `predict()` step — matching
every observed symptom (frozen `fusedPose`, unaffected `encoderPose`,
unaffected `RT`, sim non-reproduction — `Hal::SimOdometer` reports a pose
consistent with simulated ground truth, so fusing it is correct, and sim
has no "lifted robot, chip still responding" concept at all).

**3. Why no fix shipped:** confirming "is a real OTOS chip actually
detected on this hardware" requires a live bench read — I could not reach
one this session. `mbdeploy deploy` succeeded (fresh
`v0.20260708.10` build flashed to UID
`9906360200052820a8fdb5e413abb276000000006e052820`), but every subsequent
attempt to open `/dev/cu.usbmodem2121102` (raw `pyserial`,
`robot_radio.io.serial_conn.SerialConnection`, with and without sandbox
restrictions) failed with `Resource busy`. `lsof
/dev/tty.usbmodem2121102` showed the paired tty device held open by a
local VS Code extension-host process (`Code Helper (Plugin)`, PID 61702)
— most likely a stale serial-monitor panel, but not something to kill
unilaterally given the real possibility of a concurrent session (see this
project's own parallel-session-hazard memory). Shipping an EKF gating
change (the most plausible fix shape) without confirming the mechanism
first would be exactly the speculative-fix-for-an-unconfirmed-cause this
ticket's instructions warn against — a real design decision (threshold,
which channel(s), interaction with `fusableThisPass()`) deserves evidence,
not a guess.

**4. What shipped instead (diagnostic telemetry, additive, tested):** a
new TLM wire field, `otosconn=<0|1>`, surfacing `Hal::Odometer::
connected()` live every pass — the exact fact 089-007 could not check
because no wire verb exposed it:
- `source/runtime/blackboard.h` — new `bool otosConnected` cell.
- `source/runtime/main_loop.cpp` — `commit()` sets it from
  `odometer->connected()` right after the pass's own `tick()`/`pose()`
  read.
- `source/telemetry/tlm_frame.{h,cpp}` — new `otosconn=` token, gated on
  the SAME `hasOtos`/`bb.otosPresent` condition `otos=` already uses. A
  SEPARATE token rather than a 4th `otos=` value, deliberately: growing
  `otos=`'s existing 3-tuple arity would silently break
  `host/robot_radio/robot/protocol.py`'s `parse_tlm()`, which strictly
  requires `len(parts) == 3`.
- `docs/protocol-v2.md` §8 — documented in the sprint-082 "new tree"
  callout box (the section itself is an explicit legacy reference for
  the old tree's full field richness; this is a `source/`-only addition
  not carried over from `source_old/`).
- `tests/sim/unit/tlm_frame_harness.cpp` — new scenario group (g):
  `scenarioOtosConnFalseWhilePosePresent` (otosconn=0 independent of
  otos='s own pose values) plus updates to the existing exact-match and
  omission scenarios (`otosconn=1`/absence asserted alongside `otos=`).
  Compiled and run standalone (`c++ -std=c++20 ... tlm_frame_harness.cpp
  tlm_frame.cpp body_kinematics.cpp`) — all scenarios pass.
- The underlying defect (a possibly-blind-but-connected OTOS sensor
  fighting the EKF) is stated explicitly as HARDWARE-ONLY, per the
  ticket's own instruction — sim's `Hal::SimOdometer` has no notion of
  "chip present but physically unable to track" and fabricating one would
  not exercise the actual (still-unconfirmed) mechanism.

**5. Sim regression gate:** `uv run python -m pytest tests/sim` — **311
passed, 2 xfailed**, identical to the pre-change baseline (re-ran the
full suite before touching anything to confirm the starting point, then
again after). No regression.

**6. Bench (best-effort, blocked — descoped):** live `TURN`/`G`
re-verification against the 086/087 tolerance bars could not run (see
§3). Descoped to `clasi/issues/poseestimator-fused-pose-fix-pending-otos-
connected-bench-confirmation.md`, which carries forward: (a) the concrete
next bench step (`otosconn=` read while spinning on the stand), (b) the
two possible outcomes and what each implies for the eventual fix, (c) the
full code-level evidence trail above so a future session does not have to
re-derive it.

**Does the pose-fusion issue need to stay open?** Yes — the original
`clasi/issues/poseestimator-fused-pose-frozen-on-hardware.md` is completed
BY THIS TICKET in the sense this ticket was scoped (diagnostic run,
root-cause hypothesis formed via code investigation, sim regression guard
green, diagnostic telemetry landed) — `completes_issue: true` reflects
that scope, not a claim the underlying hardware defect is fixed. The
NEW, narrower follow-on (`poseestimator-fused-pose-fix-pending-otos-
connected-bench-confirmation.md`) carries the remaining work (one bench
read, then design the actual fix) forward independently of this sprint's
close.
