#include "LoopTickOnce.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "Inputs.h"
#include "HaltController.h"
#include "superstructure/MotionController.h"
#include "MotionCommand.h"
#include <cstdio>
#include <cmath>   // fmaxf, fabsf — control-collect outlier filter (039-002)

// ---------------------------------------------------------------------------
// loopTickOnce — one iteration of the firmware cooperative loop body.
//
// Called by:
//   - LoopScheduler::run_blocks()  (firmware, after controlCollectSplitPhase)
//   - sim_tick()                   (sim, after hal.tick() + controlCollectSplitPhase)
//
// See LoopTickOnce.h for the full contract.
//
// ---------------------------------------------------------------------------
// TWO COMPILE-TIME PATHS — controlled by USE_ORDERED_TICK
//
// Default (USE_ORDERED_TICK undefined): the LEGACY LOOP.
//   Unchanged from sprint 043 — all full-robot tests and the golden-TLM canary
//   are byte-exact against this path.  This is the PRODUCTION default.
//
// Optional (USE_ORDERED_TICK defined): the ORDERED-TICK PATH (059-005).
//   Rewired to the eight-step message-driven sequence:
//     1. COMMS DRAIN      — drive.periodic (outlier filter + control)
//     2. drive2.tickUpdate(now)   — SENSE: encoders + EKF predict + OTOS
//     3. BUS DRAIN        — safety + dequeueOne + drainCommandBatch
//     4. planner.tick(now)        — advance goal; returns CommandBatch{TWIST}
//     5. BUS DRAIN        — route planner batch → drive2.apply
//     6. drive2.tickAction(now)   — ACT: BVC → wheel PID → motor output
//     7. sensors.tick(now)        — timed line/color reads
//     8. TELEMETRY        — emit from state.actual (parity-shared state)
//
// PARITY GAPS (documented for follow-on ticket):
//   a) Drive2 operates on its own private _hw (not robot.state.actual).
//      The TLM frame still reads from robot.state.actual (Drive::periodic
//      keeps that path live on #ifdef USE_ORDERED_TICK).  A full cutover
//      requires Drive2 to write its state into robot.state.actual, OR
//      buildTlmFrame must be updated to read from drive2.state() — which
//      would require regenerating the golden_tlm_capture.json.
//   b) The MotorController's setCommandsRef is wired to robot.state.outputs
//      in the Robot constructor, not Drive2's private _outputs.  Drive2's
//      tickAction BVC path writes motor output through bvc2 → _mc which
//      writes to robot.state.outputs (re-wiring is needed in a follow-on).
//   c) Sensors facade (sensors.tick) drives line/color reads through its own
//      lag timers, independent of LoopTickState.lastLine/lastColor.
//
// Both paths compile and pass their respective tests.  The full-robot suite
// stays green under the default (legacy) path.  Ticket 059-006 (or a
// follow-on) will resolve the parity gaps and enable live cutover.
// ---------------------------------------------------------------------------

#ifndef USE_ORDERED_TICK

// ===========================================================================
// LEGACY LOOP (default, USE_ORDERED_TICK not defined)
//
// Mostly unchanged from sprint 043.  060-001 adds a transitional bridge at
// the end (before TLM): drive2.projectFromLegacy() mirrors state.actual into
// drive2._state, and sensors.tick() mirrors it into sensors._state.  This
// keeps buildTlmFrame (which now reads drive2.state() / sensors.state())
// byte-identical to the previous state.actual path.
// Deleted together with this legacy branch in ticket 060-005.
// ===========================================================================
void loopTickOnce(Robot& robot, CommandProcessor& cmd, CommandQueue& queue,
                  LoopTickState& ts, uint32_t now)
{
    const RobotConfig& cfg = robot.config;

    // ===== CONTROL COLLECT: outlier filter → PID → wedge push ===============
    //
    // 043-002 (Phase E): the CONTROL COLLECT block (~100 lines — outlier filter,
    // motorController.controlTick(), and the wedge push into PhysicalStateEstimate)
    // moved VERBATIM into Drive::periodic(now, fn, ctx).  The five filter-streak
    // members (_filterRejectStreakL/R, _prevDriving, _prevAnyWedged,
    // _lastControlMs) moved from Robot onto Drive as value members.  Same position
    // (before cmd.dequeueOne(queue)), same order, same numerics — the golden-TLM
    // canary is the byte-exact oracle.
    //
    // The EVT enc_filter_hold emission inside the block uses Robot's TLM sink
    // (_tlmBoundFn/_tlmBoundCtx); those are passed as the fn/ctx parameters
    // (architecture-update.md OQ-2) so the emission is byte-identical.
    robot.drive.periodic(now, robot._tlmBoundFn, robot._tlmBoundCtx);

    // ===== QUEUE: dispatch one enqueued command per tick ===================
    // Commands arrive via cmd.process() → queue.push_back().
    // dequeueOne() dispatches the front command, keeping behaviour identical
    // to the former immediate-dispatch path (enqueue + dequeue in same tick).
    cmd.dequeueOne(queue);

    // ===== SAFETY: centralized per-tick safety evaluation (042-003) =========
    // Formerly two consecutive inline blocks here — (1) the keepalive/system
    // watchdog and (2) the halt-controller (SAFE/X/ESTOP) evaluation.  Both
    // bodies moved VERBATIM, in the SAME ORDER, into Superstructure::evaluateSafety.
    // This single call sits in the SAME position the blocks did: after
    // dequeueOne(queue), before driveAdvance.  driveAdvance stays below, in the
    // same order.  The golden-TLM canary is the byte-exact oracle.
    robot.superstructure.evaluateSafety(cmd, queue, ts, robot.state.actual, now);

    // ===== DRIVE: advance drive state machine =================================
    robot.motionController.driveAdvance(
        robot.state.actual, robot.state.outputs, robot.state.desired, now);

    // ===== ODOMETRY: dead-reckon pose from encoder deltas ====================
    robot.estimate.addOdometryObservation(robot.state.actual, cfg.trackwidthMm,
                           cfg.rotationalSlip, now);

    // ===== HAL ACTUATOR TICK: deliver commanded velocity to the HAL ===========
    // Pass the commanded actuator state to the HAL so a bench-mode sensor plant
    // (BenchOtosSensor in NezhaHAL) can integrate it.  Must run before the OTOS
    // block so the plant advances its accumulators before otosCorrect() calls
    // readTransformed().  Production NezhaHAL / MockHAL implement this as a
    // near-no-op when bench mode is off; the robot core no longer reaches into
    // the concrete HAL to do this (034-002, replaces robot.benchOtosTick).
    robot.hal.tick(now, robot.state.outputs);

    // ===== OTOS: timed I2C pose read + EKF fusion ============================
    // In firmware: run when enOtos is set and lagOtosMs has elapsed.
    // In sim: the lagOtosMs gate is bypassed; fusion runs every tick when
    // fuseOtos is true (matches the original sim_tick() behaviour which called
    // otosCorrect() unconditionally each tick).
    if (ts.fuseOtos) {
        robot.otosCorrect(now);
    } else if (cfg.lagOtosMs > 0 &&
               (int32_t)(now - ts.lastOtos) >= (int32_t)cfg.lagOtosMs) {
        robot.otosCorrect(now);
        ts.lastOtos = now;
    }

    // ===== LINE: timed I2C read ===============================================
    // 043-001 (Phase E): the lag gate + read + timer bump moved VERBATIM into
    // LineSensor::periodic(ts, now).  Same order/position, same numerics — the
    // golden-TLM canary is the byte-exact oracle.
    robot.lineSensor.periodic(ts, now);

    // ===== COLOUR: timed read =================================================
    // 043-001: verbatim COLOUR block now in ColorSensor::periodic(ts, now).
    // (Robot member is colorSensor_ — the IColorSensor& device ref keeps the
    // colorSensor name to avoid macro collisions; see Robot.cpp annotation.)
    robot.colorSensor_.periodic(ts, now);

    // ===== PORTS: timed GPIO read =============================================
    // 043-001: verbatim PORTS block now in Ports::periodic(ts, now).
    robot.ports.periodic(ts, now);

    // ===== BRIDGE: project legacy state into message-contract subsystems ======
    // 060-001: buildTlmFrame now reads from drive2.state() and sensors.state()
    // instead of robot.state.actual.  In this legacy loop path, state.actual is
    // the authoritative source; drive2 and sensors must be explicitly updated to
    // mirror it so that TLM frames carry live values.
    //
    // drive2.projectFromLegacy: copies enc/vel/fused/optical/otos fields from
    // state.actual into drive2._state without running any motor control or EKF.
    // Deleted together with this legacy loop branch in ticket 060-005.
    //
    // sensors.tick: populates sensors._state from state.actual (which lineSensor
    // and colorSensor_ just updated above via their periodic() calls). Safe to
    // call alongside lineSensor.periodic / colorSensor_.periodic because it reads
    // the same HardwareState fields they wrote.
    robot.drive2.projectFromLegacy(robot.state.actual);
    robot.sensors.tick(now);

    // ===== TELEMETRY: timed TLM frame emit ====================================
    // N3 fix (030-003): emit with the STREAM-bound fn+ctx pair, not ts.activeCtx
    // (which is the last *command* channel, not the bound stream channel).
    // Mixed serial+radio field setup: STREAM over serial then a radio command
    // would have passed ts.activeCtx = &radio to serialReplyTlm, casting Radio*
    // to SerialPort* — UB.  Using the bound pair keeps TLM on the channel that
    // issued STREAM regardless of which channel subsequent commands arrive on.
    // telemetryEmit guards fn == nullptr, so SET tlmPeriod without STREAM is safe.
    if (cfg.tlmPeriodMs > 0 &&
        (int32_t)(now - ts.lastTlm) >= (int32_t)cfg.tlmPeriodMs) {
        robot.telemetryEmit(now, robot._tlmBoundFn, robot._tlmBoundCtx);
        ts.lastTlm = now;
    }
}

#else  // USE_ORDERED_TICK

// ===========================================================================
// ORDERED-TICK PATH (059-005) — enabled with -DUSE_ORDERED_TICK
//
// Eight-step message-driven sequence per the Phase-3 architecture.
// See comment block above for parity gaps relative to the legacy loop.
//
// Additional headers needed for the ordered-tick path.
// ===========================================================================

#include "robot/BusDrain.h"
#include "subsystems/drive/Drive2.h"
#include "superstructure/MotionController2.h"

void loopTickOnce(Robot& robot, CommandProcessor& cmd, CommandQueue& queue,
                  LoopTickState& ts, uint32_t now)
{
    const RobotConfig& cfg = robot.config;

    // =========================================================
    // STEP 1 — COMMS DRAIN: outlier filter + control collect
    //
    // 060-001: drive.periodic() crutch removed. buildTlmFrame now
    // reads encoder/pose/vel/twist/otos from drive2.state() and
    // sensor fields from sensors.state(); robot.state.actual is no
    // longer populated by the ordered-tick path.
    // =========================================================

    // =========================================================
    // STEP 2 — drive2.tickUpdate(now): SENSE
    //
    // Runs the Drive2 outlier filter + EKF predict + OTOS
    // correction on Drive2's private _hw.  The Drive2 estimate
    // is the authoritative source in the ordered-tick path.
    // =========================================================
    // fuseOtos: bypass Drive2's internal OTOS lag gate when ts.fuseOtos is set
    // (mirrors the ts.fuseOtos bypass in the legacy path; used by sim tests that
    // call sim_set_otos_fusion(1) to force per-tick OTOS updates).
    robot.drive2.tickUpdate(now, ts.fuseOtos);

    // =========================================================
    // STEP 2b — Sync robot.state.actual from drive2.state()
    //
    // 060-004: legacy command handlers (GET VEL, DBG OTOS, HALT
    // conditions, the D-decel hook in driveAdvance, stop conditions
    // in HaltController::evaluate / StopCondition::evaluate, and
    // SystemCommands::handleZero) all read robot.state.actual.
    // In the ordered-tick path, drive2 is the authoritative source
    // for encoder, velocity, and pose data.  Sync state.actual from
    // drive2.state() after tickUpdate so legacy code sees live values.
    //
    // This sync is the LAST step before drive2 state diverges again;
    // it runs once per tick, keeping state.actual consistent with the
    // most recent SENSE phase.  The full cutover (060-005) will
    // replace all state.actual reads with drive2.state() calls and
    // remove this sync.
    // =========================================================
    {
        const msg::DrivetrainState& ds = robot.drive2.state();
        // Per-wheel encoder accumulator (mm).  [0]=R (FR), [1]=L (FL).
        robot.state.actual.encMm[0] = ds.enc()[0];
        robot.state.actual.encMm[1] = ds.enc()[1];
        // Per-wheel velocity (mm/s).
        robot.state.actual.velMms[0] = ds.vel()[0];
        robot.state.actual.velMms[1] = ds.vel()[1];
        // Fused pose (EKF output).
        robot.state.actual.fused.pose.x   = ds.fused.pose.x;
        robot.state.actual.fused.pose.y   = ds.fused.pose.y;
        robot.state.actual.fused.pose.h   = ds.fused.pose.h;
        // Fused twist (EKF body-frame velocity).
        robot.state.actual.fused.twist.vx_mmps    = ds.fused.twist.v_x;
        robot.state.actual.fused.twist.vy_mmps    = ds.fused.twist.v_y;
        robot.state.actual.fused.twist.omega_rads = ds.fused.twist.omega;
        // Encoder pose (dead-reckoning only, no OTOS).
        robot.state.actual.encoder.pose.x = ds.encoder.pose.x;
        robot.state.actual.encoder.pose.y = ds.encoder.pose.y;
        robot.state.actual.encoder.pose.h = ds.encoder.pose.h;
        // Optical pose (raw OTOS reading, pre-EKF).
        robot.state.actual.optical.pose.x = ds.optical.pose.x;
        robot.state.actual.optical.pose.y = ds.optical.pose.y;
        robot.state.actual.optical.pose.h = ds.optical.pose.h;
        // OTOS freshness envelope.
        robot.state.actual.otos.lagMs      = ds.otos.lag;
        robot.state.actual.otos.lastUpdMs  = ds.otos.last_upd;
        robot.state.actual.otos.valid      = ds.otos.valid;
    }

    // =========================================================
    // STEP 3 — BUS DRAIN: safety + dequeueOne + motion verbs
    //
    // Safety is evaluated first (keepalive watchdog + halt
    // controller), then one enqueued command is dispatched
    // (dequeueOne), matching the legacy ordering.  The comms
    // batch from step 1 would normally arrive here; in the
    // current implementation the comms drain produces no
    // explicit CommandBatch, so we run evaluateSafety +
    // dequeueOne directly.
    // =========================================================
    robot.superstructure.evaluateSafety(cmd, queue, ts, robot.state.actual, now);
    cmd.dequeueOne(queue);

    // =========================================================
    // STEP 3b — sync wire reply context into planner
    //
    // 060-004: G/TURN/D wire commands are dispatched by
    // dequeueOne → handleGoTo/handleTurn/handleDistance which
    // call mc.beginGoTo/beginTurn/beginDistance(... replyFn ...).
    // Those calls store the real replyFn into robot.state.desired.
    // MC2's _desired uses _noopReply (set via MC2::apply()).
    // driveAdvance() line-275 calls:
    //   _activeCmd.setReplySink(target.replyFn, ...)
    // where target = MC2._desired — overwriting the real replyFn
    // stored in _activeCmd from beginGoTo with _noopReply.
    // syncWireContext copies the live replyFn/replyCtx/corrId
    // from state.desired into MC2._desired so driveAdvance emits
    // EVT done G / EVT done TURN on the correct channel.
    // =========================================================
    robot.planner.syncWireContext(robot.state.desired);

    // =========================================================
    // STEP 4 — planner.tick(now): advance goal state machine
    //
    // MotionController2::tick() calls _mc.driveAdvance() with
    // its own _hw (populated from drive2.state()) and returns a
    // CommandBatch containing a DrivetrainCommand{TWIST}.
    // =========================================================
    msg::CommandBatch plannerBatch = robot.planner.tick(now);

    // =========================================================
    // STEP 5 — BUS DRAIN: route planner batch → drive2.apply
    //
    // drainCommandBatch routes the TWIST OutCommand to
    // drive2.apply(DrivetrainCommand{TWIST}).
    // =========================================================
    drainCommandBatch(plannerBatch, robot.drive2, robot.planner, queue, cmd);

    // =========================================================
    // STEP 6 — drive2.tickAction(now): ACT
    //
    // Applies the staged DrivetrainCommand via BVC → wheel PID
    // → motor output.  Drive2's BVC (bvc2) is separate from
    // MotionController's internal BVC.
    // =========================================================
    robot.drive2.tickAction(now);

    // =========================================================
    // STEP 6b — HAL ACTUATOR TICK
    //
    // The HAL tick delivers the motor commands to the plant.
    // 060-002: Drive2's constructor binds MotorController to
    // drive2._outputs (setCommandsRef(&_outputs)), and Robot.cpp
    // no longer overrides that binding in the ordered-tick path.
    // So the buffer the MotorController actually wrote to is
    // drive2._outputs — pass drive2.outputs() here.
    // =========================================================
    robot.hal.tick(now, robot.drive2.outputs());

    // =========================================================
    // STEP 7 — sensors.tick(now): timed line/color reads
    //
    // sensors.tick() is the SOLE sensor-schedule authority in the
    // ordered-tick path.  It drives both sensor reads when their
    // lag gates (_lastLineTick / _lastColorTick in Sensors.h) fire,
    // independent of LoopTickState.lastLine / lastColor (which are
    // NOT read or written here).  lineSensor.periodic() and
    // colorSensor_.periodic() are NOT called in this path.
    // =========================================================
    robot.sensors.tick(now);
    // ports.periodic: Ports is not yet wrapped in a Ports2 facade;
    // keep it here until a Ports2 subsystem replaces it (ticket 060-005+).
    robot.ports.periodic(ts, now);

    // =========================================================
    // STEP 8 — TELEMETRY
    //
    // 060-001: buildTlmFrame now reads from drive2.state() and
    // sensors.state() — robot.state.actual is no longer required.
    // =========================================================
    if (cfg.tlmPeriodMs > 0 &&
        (int32_t)(now - ts.lastTlm) >= (int32_t)cfg.tlmPeriodMs) {
        robot.telemetryEmit(now, robot._tlmBoundFn, robot._tlmBoundCtx);
        ts.lastTlm = now;
    }
}

#endif  // USE_ORDERED_TICK
