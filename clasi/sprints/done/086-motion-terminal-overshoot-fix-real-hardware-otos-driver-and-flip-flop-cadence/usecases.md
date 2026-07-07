---
status: complete
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 086 Use Cases

Parent issues:
[`clasi/issues/motion-turn-drive-terminal-overshoot.md`](../../issues/motion-turn-drive-terminal-overshoot.md)
(the priority — root-caused, stakeholder-ordered two-phase fix),
[`clasi/issues/nezha-hardware-otos-driver-for-new-source-tree.md`](../../issues/nezha-hardware-otos-driver-for-new-source-tree.md),
[`clasi/issues/flip-flop-cadence-below-design-target.md`](../../issues/flip-flop-cadence-below-design-target.md)
(measure-first, not urgent).

Three independent pool issues, grouped below in the order the tickets execute:
motion terminal overshoot (SUC-001..004), the real-hardware OTOS driver
(SUC-005..007), and flip-flop cadence (SUC-008..009).

## SUC-001: A turn (`TURN`/`RT`) stops without a post-completion reverse-spin

- **Actor**: Developer/stakeholder driving the robot (sim first; stand HITL
  gates the ticket that lands the fix).
- **Preconditions**: A `TURN` or `RT` command is in flight, decelerating
  toward its commanded heading/rotation target under the default `SMOOTH`
  stop style.
- **Main Flow**:
  1. The commanded turn approaches its target; the motor velocity loop
     (`Hal::MotorVelocityPid` + `Hal::Motor::armoredWrite()`'s zero-crossing
     dwell) decelerates each wheel toward zero.
  2. The `EVT done rt`/`EVT done turn` fires at the correct heading (this
     already works today — the pose estimate is not at fault).
  3. Each wheel's physical velocity settles at (approximately) zero and
     stays there — it does **not** overshoot through zero into a sustained
     opposite-direction spin.
- **Postconditions**: Post-stop heading drift is small (tight tolerance, not
  the pre-fix ~4-10°/turn); no wheel reports a sustained reverse velocity
  after the `EVT done` tick.
- **Acceptance Criteria**:
  - [ ] A regression test reproduces the pre-fix failure quantitatively
        (per-wheel `vel(L,R)` sampled across an `RT 9000`, matching the
        issue's own measured signature) and is used to verify the fix
        flips it from failing to passing.
  - [ ] Post-`EVT done` residual wheel velocity is bounded by an explicit,
        tight tolerance (no reverse-sign residual velocity beyond the bound)
        for at least one `RT`/`TURN` case in each rotation direction.
  - [ ] `tests/sim/unit/test_motion_commands_arc_turn.py`'s existing loose
        (±10°) `RT 9000` over-rotation tolerance is tightened to reflect the
        fixed behavior, not left at its pre-fix margin.

## SUC-002: A distance drive (`D`) stops at the commanded distance without overshoot

- **Actor**: Developer/stakeholder.
- **Preconditions**: A `D` command is in flight, decelerating toward its
  `STOP_DISTANCE` target under `SMOOTH`.
- **Main Flow**: Same shape as SUC-001's steps 1-3, for linear (not
  rotational) motion — the issue's own measurement (`D 200 200 500` →
  true ~535 mm, ~7% over) is the reproduction target.
- **Postconditions**: Final traveled distance is within a tight tolerance of
  commanded; no post-stop reverse-direction wheel motion.
- **Acceptance Criteria**:
  - [ ] A regression test reproduces the pre-fix `D` overshoot quantitatively.
  - [ ] Post-fix, a `D` command's final encoder-measured distance is within
        an explicit tight tolerance of commanded (materially better than the
        pre-fix ~7%).
  - [ ] Post-stop residual wheel velocity is bounded the same way as SUC-001.

## SUC-003: A multi-leg tour traces the intended geometry, not a tangle

- **Actor**: Developer/stakeholder running TestGUI's Sim Tour 1/Tour 2.
- **Preconditions**: SUC-001 and SUC-002's fixes are both in place.
- **Main Flow**:
  1. The tour runs its full sequence of `D`/`RT` legs back to back
     (immediate next-leg dispatch once `mode=I` is observed, matching how
     `_TourRunner` actually drives it — no artificial inter-leg settle
     delay, since the issue found that does not help).
  2. Each leg's actual heading/position change (sim ground truth) is
     checked against its commanded value, not just the tour's final
     endpoint.
  3. A human-reviewable rendered trace of the tour is produced.
- **Postconditions**: Tour 1 and Tour 2 both return near world origin
  (closed loop) **and** every individual leg's geometry is within tolerance
  — not just the final endpoint distance (the exact gap that let this bug
  ship in sprints 084/085).
- **Acceptance Criteria**:
  - [ ] A new per-leg geometry test (sim ground truth vs. commanded, for
        every leg of at least Tour 1) replaces/extends the endpoint-only
        assertion `tests/testgui/test_tour1_geometry.py` currently carries;
        endpoint-distance-only tour tests are banned per the issue's own
        mandate.
  - [ ] A rendered Tour 1 and Tour 2 trace image is produced (matching the
        established `tests/bench/velocity_chart.py`/`tests/playfield/
        plot_square.py` charting precedent) and manually confirmed to look
        like the intended figure, not a tangle.
  - [ ] `test_tour1_geometry.py`'s current xfail path for the OLD (already
        diagnosed, out-of-scope-for-085) `rotSlip`/RT-coast source of drift
        is revisited: either it now passes at a tightened tolerance, or the
        remaining gap is re-documented against the new, smaller residual.

## SUC-004: The motor reversal/wedge safety armor still holds after the velocity-loop fix

- **Actor**: Developer (sim regression); stakeholder (stand HITL).
- **Preconditions**: SUC-001/002's `Hal::MotorVelocityPid`/`Hal::Motor::
  armoredWrite()` change has landed.
- **Main Flow**:
  1. Every existing sprint-078/079 armor test (`tests/sim/unit/
     test_motor_policy.py` and its harness) is re-run against the changed
     code, unmodified in intent.
  2. New test(s) specifically cover the zero-crossing/decel-to-zero
     scenario this sprint's fix touches: a legitimate braking-direction
     duty change near a velocity target of zero must not (a) get
     needlessly held off by the reversal dwell in a way that makes the
     overshoot worse, nor (b) defeat the dwell's protection against a
     genuine spurious/unrequested reversal.
  3. On the stand: exercise `DEV M`/`DEV DT` sequences known to have
     previously triggered wedge/reversal issues (078/079/081 lineage) and
     confirm `wedged()`/`wedgeSuspect()` behave exactly as before.
- **Postconditions**: No armor regression — every pre-existing armor test
  still passes; the new zero-crossing tests pass; the stand HITL pass shows
  no wedge/runaway behavior.
- **Acceptance Criteria**:
  - [ ] 100% of pre-existing `test_motor_policy.py` cases pass unmodified
        (or, if a case's fixture value must change, the change is justified
        in the ticket, not silently loosened).
  - [ ] New zero-crossing armor regression test(s) added and passing.
  - [ ] Stand HITL pass (`.claude/rules/hardware-bench-testing.md`) exercises
        wheel spin-up, turn, and stop in both directions with wheels
        observed settling cleanly — no runaway, no persistent wedge latch.

## SUC-005: Developer reads live, plausible OTOS pose/velocity on real hardware

- **Actor**: Developer on the stand.
- **Preconditions**: `NezhaHardware::odometer()` returns a live OTOS-backed
  `Hal::Odometer` leaf instead of the inherited `nullptr` default; the
  physical SparkFun OTOS sensor is connected via I2C (address `0x17`).
- **Main Flow**:
  1. Firmware boots; the OTOS leaf's `begin()` detects the chip and applies
     its boot-baked linear/angular scalar + lever-arm mounting offset
     (`data/robots/*.json`'s `geometry.odometry_offset_mm` /
     `calibration.otos_linear_scale`/`otos_angular_scale` — present in the
     schema and robot JSON today, but not yet consumed by this tree's
     `scripts/gen_boot_config.py`).
  2. `devLoopTick()`'s existing, already-wired `hardware.odometer()` seam
     (`source/dev_loop.cpp`) calls the leaf's `tick(now)`/`pose()` every
     pass and feeds the sample into `Subsystems::PoseEstimator` — no
     `dev_loop.cpp` change needed (confirmed: this call is unconditional and
     already handles a non-null odometer generically).
  3. Developer moves the robot (or spins a wheel) on the stand and observes
     the OTOS-derived pose/velocity change plausibly and in the expected
     direction.
- **Postconditions**: The fused `pose` telemetry field and the raw `otos=`
  field are both live on hardware — no longer permanently absent/stale as
  they are with the current `nullptr` odometer.
- **Acceptance Criteria**:
  - [ ] `NezhaHardware::odometer()` returns a non-null, OTOS-backed leaf.
  - [ ] On the stand: OTOS position and velocity reads change plausibly
        (correct sign/magnitude) as the robot or a wheel is moved.
  - [ ] `TLM`'s `pose=`/`otos=` fields are live (not the `ERR nodev`
        placeholder state) on real hardware.

## SUC-006: OTOS calibration and lever-arm mounting offset correct the reported pose to the chassis centre

- **Actor**: Developer/stakeholder calibrating a robot on the stand.
- **Preconditions**: SUC-005's leaf exists; the robot's `data/robots/*.json`
  carries a non-zero `geometry.odometry_offset_mm` (e.g. tovez.json's
  `x: -47.7, y: 3.5`, confirmed present today).
- **Main Flow**:
  1. At boot, the leaf applies the mounting-offset compensation **host-side
     within the leaf itself** — the known hardware quirk (OTOS `REG_OFFSET`
     is unwritable on this chip; verified in `source_old`, must not be
     re-derived) means this cannot be pushed to the chip's own offset
     register.
  2. `OI`/`OL`/`OA`/`OZ`/`OV` (the existing, already-implemented wire verbs)
     operate against the new leaf exactly as documented in
     `docs/protocol-v2.md` §11 — no wire/message-schema change.
  3. A pure spin in place produces a small residual translation (bounded),
     not the ~433 mm phantom translation a past regression (commit
     `db11b7c`, pre-rebuild tree) produced when the offset math used a
     lagged heading.
- **Postconditions**: The chassis-centre pose the EKF fuses is corrected for
  the sensor's own physical offset from the centre of rotation.
- **Acceptance Criteria**:
  - [ ] The lever-arm compensation math is ported from (or verified
        equivalent to) `source_old/hal/capability/OtosLeverArm.h`'s
        `sensorToCentre()`/`centreToSensor()` — same-instant heading, not a
        lagged one.
  - [ ] A pure spin-in-place on the stand produces bounded residual
        translation, not a large phantom offset.
  - [ ] `OL`/`OA` read back the values the leaf was configured with
        (matching `otos_commands.cpp`'s existing shadow-read contract).

## SUC-007: All seven OTOS wire verbs ack on real hardware

- **Actor**: Developer on the stand.
- **Preconditions**: SUC-005's leaf is wired.
- **Main Flow**: Each of `OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA` is sent against
  the real robot.
- **Postconditions**: Every verb replies `OK ...` (matching
  `docs/protocol-v2.md` §11's documented reply shapes) instead of
  `ERR nodev <verb>`.
- **Acceptance Criteria**:
  - [ ] All seven verbs ack `OK` on the stand.
  - [ ] The hardware-bench gate's "OTOS alive" check
        (`.claude/rules/hardware-bench-testing.md`) passes.

## SUC-008: Developer measures where the flip-flop's per-slice time actually goes

- **Actor**: Developer on the stand.
- **Preconditions**: A 2-port closed-loop motion session reproducing the
  079-006 measured ~19-22 ms (~44-52 Hz) per-motor cadence.
- **Main Flow**:
  1. Developer instruments per-phase timing — via the existing (already
     compiled-in, currently wire-unexposed) `I2CBus` per-device counters/
     transaction-log ring (`txnCount()`/`errCount()`/`dumpRecent()`), read
     live via a `pyOCD`/`gdb` session per `.claude/rules/debugging.md`
     (preferred — zero firmware change), or via a minimal new dev-only
     surface if a debugger read proves impractical for repeated sampling.
  2. Developer isolates how much of the ~19-22 ms period is the mandatory
     `preClear=4000`/`postClear=4000` clearances (079-006's TWIM-stall fix)
     vs. the `COLLECT_DUE` poll spin vs. genuine I2C transaction time, and
     specifically tests the "double-counted clearance" hypothesis (does a
     single in-use port pay both the request's `preClear` AND the
     preceding collect's `postClear` for the same real-world gap, or two
     independent 4 ms waits back to back).
- **Postconditions**: A documented, data-backed breakdown of where the
  per-slice time goes exists; the double-counting hypothesis is confirmed
  or refuted.
- **Acceptance Criteria**:
  - [ ] A per-phase timing breakdown (request write, settle wait, collect
        read, scheduling overhead) is captured and recorded for at least
        one 2-port session.
  - [ ] The double-counted-clearance hypothesis is explicitly confirmed or
        refuted with data, not left as a guess.
  - [ ] No firmware behavior change in this step (measurement only).

## SUC-009: Cadence is either safely improved or the design estimate is corrected — never at the expense of the TWIM-stall fix or the reversal-latch armor

- **Actor**: Developer/stakeholder.
- **Preconditions**: SUC-008's measurement is complete.
- **Main Flow** (one of two outcomes, decided from the data):
  1. **Targeted optimization**: if the measurement shows genuine
     double-counted or otherwise reclaimable clearance time, apply the
     narrowest fix that reclaims it, then re-measure to confirm improved
     Hz **and** re-run the full 079-006 TWIM-stall regression suite plus a
     stand soak to confirm no stall/wedge regression.
  2. **Doc correction**: if no safe win exists, update the design
     document's cadence estimate (the tick-model sketch this issue
     references) to state the measured ~44-52 Hz as the real budget,
     with the reasoning from SUC-008.
- **Postconditions**: Either a measured cadence improvement with proven
  non-regression, or an honestly corrected design estimate — no chase of Hz
  at the expense of safety/stability.
- **Acceptance Criteria**:
  - [ ] The chosen outcome (optimize vs. document) is explicitly recorded
        with its supporting measurement.
  - [ ] If optimizing: the 079-006 TWIM-stall fix's own tests/soak still
        pass unmodified, and the reversal-latch armor tests (SUC-004) still
        pass.
  - [ ] If documenting: the design doc's cadence estimate section is
        updated to match measured reality, cross-referenced to this issue.
