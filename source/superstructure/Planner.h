#pragma once
// =============================================================================
// Planner.h — Planner subsystem.
//
// Owns and advances the S/T/D/G drive state machines, S-mode watchdog, and
// odometry delta tracking directly as native members.
//
// Role: GOAL CLOSURE only. Planner generates a time-varying body-twist setpoint
// from a goal + pose estimate and decides when the goal is reached.
// Velocity loops live in Drive (subsystems::Drive) / MotorController.
//
// Contract (no virtual dispatch, no heap allocation in tick()):
//   apply(PlannerCommand)     — STAGES the goal, no emission
//   tick(now) → CommandBatch — advances goal logic, returns DrivetrainCommand
//   state() → PlannerState   — const-ref to live planner state
//   configure(PlannerConfig) — delta-apply motion params
//
// See: clasi/issues/message-based-subsystem-architecture.md §Planner
// =============================================================================

#include "types/Inputs.h"          // HardwareState, MotorCommands, TargetState
#include "types/Config.h"          // RobotConfig, DriveMode
#include "types/CommandTypes.h"    // ReplyFn
#include "messages/planner.h"      // msg::PlannerCommand, PlannerState, PlannerConfig
#include "messages/common.h"       // msg::CommandBatch
#include "messages/drivetrain.h"   // msg::DrivetrainCommand
#include "BodyVelocityController.h"  // BodyVelocityController
#include "MotionCommand.h"           // MotionCommand

namespace subsystems {
class Drive;
}

class MotorController;
class Odometry;
struct Robot;

// ---------------------------------------------------------------------------
// Planner — message-driven Planner subsystem.
//
// Construction: takes MotorController& (for wheel output / encoder access),
// Odometry& (for pose reads in begin* entry points), Drive (for fused pose in
// tick()), and the RobotConfig for geometry / limits.
//
// apply() STAGES only — no hardware I/O.
// tick(now) does all work and RETURNS a CommandBatch containing a
// DrivetrainCommand{TWIST} setpoint. Caller (the bus dispatcher) forwards
// this to Drive.apply().
// ---------------------------------------------------------------------------
class Planner {
public:
    // Constructor — takes MotorController + Odometry + Drive + config directly.
    // cfg: motion-limits source (aMax, vBodyMax, etc.); stored as local copy
    // so configure() can update it without disturbing the original.
    Planner(MotorController& mc_ctrl, Odometry& odo,
            const subsystems::Drive& drive,
            const RobotConfig& cfg);

    // ---- 4-verb contract (no virtual dispatch) ----

    // Stage the goal command. No hardware I/O, no emission.
    // Dispatches on PlannerCommand::GoalKind → the appropriate begin*() call.
    void apply(const msg::PlannerCommand& cmd);

    // Advance goal closure one tick.
    // 1. Populate _hw from _drive.state() (fused pose + twist).
    // 2. Call driveAdvance(_hw, _cmds, _target, now) — now a direct method.
    // 3. Read commanded body twist from _desired.bodyTwist.
    // 4. Pack DrivetrainCommand{TWIST} into returned CommandBatch.
    // 5. Update _state.
    msg::CommandBatch tick(uint32_t now);

    // Read-only state snapshot — no I/O, no copy.
    const msg::PlannerState& state() const { return _state; }

    // Store updated planner config (motion limits only).
    void configure(const msg::PlannerConfig& cfg);

    // syncWireContext — copy the reply fn/ctx/corrId from robot.state.desired
    // into _desired before tick() calls driveAdvance().
    //
    // 060-004: G/TURN/D commands are dispatched through the wire path
    // (handleGoTo, handleTurn, handleDistance → mc.beginGoTo / beginTurn /
    // beginDistance) which writes the real replyFn into robot.state.desired.
    // But driveAdvance() uses Planner's _desired (= _target), which has _noopReply.
    // When PRE_ROTATE completes, driveAdvance line 275 calls:
    //   _activeCmd.setReplySink(target.replyFn, target.replyCtx, target.corrId)
    // overwriting the activeCmd's real replyFn with _noopReply.
    // This sync propagates the real reply context from the wire path into
    // Planner's _desired so driveAdvance uses the correct reply sink for EVT emission.
    void syncWireContext(const DesiredState& wire) {
        _desired.replyFn  = wire.replyFn;
        _desired.replyCtx = wire.replyCtx;
        // corrId is a char array — copy element-by-element (no strncpy in header).
        for (int i = 0; i < 16; ++i) _desired.corrId[i] = wire.corrId[i];
        _desired.sink = wire.sink;
    }

    // -------------------------------------------------------------------------
    // Planner native motion API — begin*(), stop, cancel, etc.
    // -------------------------------------------------------------------------

    DriveMode mode() const { return _mode; }

    void beginStream(float leftMms, float rightMms, uint32_t now_ms,
                     TargetState& target, ReplyFn fn, void* ctx);

    void beginVelocity(float v_mms, float omega_rads, uint32_t now_ms,
                       TargetState& target, ReplyFn fn, void* ctx,
                       const char* corr_id = nullptr, bool seedImmediate = false);

    void beginTimed(float leftMms, float rightMms, uint32_t durationMs, uint32_t now_ms,
                    TargetState& target, ReplyFn fn, void* ctx,
                    const char* corr_id = nullptr);

    void beginDistance(float leftMms, float rightMms, int32_t targetMm, uint32_t now_ms,
                       TargetState& target, ReplyFn fn, void* ctx,
                       const char* corr_id = nullptr);

    void beginGoTo(float tx, float ty, float speedMms, uint32_t now_ms,
                   TargetState& target, ReplyFn fn, void* ctx,
                   const char* corr_id = nullptr);

    void beginTurn(float headingCdeg, float epsCdeg, uint32_t now_ms,
                   TargetState& target, ReplyFn fn, void* ctx,
                   const char* corr_id = nullptr);

    void beginRotation(float relCdeg, uint32_t now_ms,
                       TargetState& target, ReplyFn fn, void* ctx,
                       const char* corr_id = nullptr);

    void stop(uint32_t now_ms, ReplyFn fn, void* ctx);

    void cancel(uint32_t now_ms, ReplyFn fn, void* ctx);

    void softStop(uint32_t now_ms);

    void beginRawVelocity(float v_mms, float omega_rads, uint32_t now_ms);

    void disableSafetyOneShot();

    // 065-003 / CR-05b: timestamp of the last genuine open-ended
    // velocity-target refresh (stamped by beginVelocity() and
    // beginRawVelocity() — the only two call sites that legitimately
    // create/refresh an S/VW/R/_VW target). Read by
    // Superstructure::evaluateSafety() as a second, `+`-independent
    // watchdog signal: an ambient keepalive alone must not be sufficient
    // to keep an open-ended command alive if the velocity-issuing layer
    // itself has stalled.
    uint32_t lastVelocityRefreshMs() const { return _lastVelocityRefreshMs; }

    // 065-003 / CR-05b: stamp a fresh velocity-target refresh WITHOUT going
    // through begin*(). The "D6 origin guard" VW-keepalive path
    // (MotionCommands.cpp handleVW) updates an already-active RETARGETABLE
    // command's target via activeCmd().setTarget() directly — a deliberate
    // bypass of beginVelocity() to avoid cancel/reconfigure churn on every
    // resend (see the D6 comment at that call site). That resend is exactly
    // a "genuine refresh" for staleness purposes (it is the KeyboardDriver
    // resend pattern this ticket exists to keep alive), so the wire-layer
    // caller marks it explicitly here.
    void markVelocityRefreshed(uint32_t now_ms) { _lastVelocityRefreshMs = now_ms; }

    bool hasActiveCommand() const { return _activeCmd.active(); }

    void emitToActiveChannel(const char* evt, TargetState& target) {
        if (_activeCmd.active()) {
            emitEvt(evt, target);
        }
    }

    MotionCommand& activeCmd() { return _activeCmd; }

    void setHardwareState(HardwareState* s) { _hwState = s; }

    void setRobotCtx(Robot* r) { _robot = r; }

    void setBvcStateRef(DesiredState* ds) { _bvc.setStateRef(ds); }

    const HardwareState* hardwareState() const { return _hwState; }

private:
    // ---- Primary control members ----
    MotorController&   _mc_ctrl;    // wheel output / encoder access
    Odometry&          _odo;        // pose reads in begin*() (via getPoseFloat())
    RobotConfig        _cfg;        // local copy of motion limits; configure() updates it
    HardwareState*     _hwState;    // authoritative state; set by setHardwareState()

    // Robot pointer for _checkSafeOneShot (re-arming config.safetyEnabled).
    // Set by setRobotCtx() from Robot constructor.
    struct Robot*      _robot;

    // _bvc MUST be declared before _activeCmd: Planner's constructor
    // passes &_bvc to _activeCmd.configure(), so _bvc must be fully constructed
    // first.  Do not reorder.
    BodyVelocityController _bvc;        // body-level (v,ω) profiler
    MotionCommand          _activeCmd;  // the single active MotionCommand

    // SAFE one-shot disable flag (sprint 024-003).
    bool _safeOneShotDisable = false;

    // 065-003 / CR-05b: last time an open-ended velocity target (S/VW/R/_VW)
    // was genuinely refreshed by beginVelocity()/beginRawVelocity(). Persists
    // across commands by design (see architecture-update.md Decision 3) —
    // its only purpose is "how long since a velocity target was last set."
    uint32_t _lastVelocityRefreshMs = 0;

    // Drive mode
    DriveMode _mode;

    // Current speed targets (kept for internal use only)
    float _tgtL;
    float _tgtR;

    // D-command state for per-tick decel hook
    float _dDistTarget;  // target distance in mm
    float _dOmega;       // commanded yaw rate at begin (from forward kinematics)
    float _dEnc0;        // encoder average at begin (baseline for decel cap)

    // G go-to state machine
    enum class GPhase { IDLE, PRE_ROTATE, PURSUE };
    GPhase  _gPhase;
    float   _gTargetXWorld;  // goal x in world frame (mm), set at beginGoTo()
    float   _gTargetYWorld;  // goal y in world frame (mm), set at beginGoTo()
    float   _gSpeed;

    // PURSUE re-gate counter (D8 027-004).
    uint8_t _pursueBacktrackTicks = 0;

    // Tick timing
    uint32_t _lastTickMs;
    uint32_t _currentTimeMs;

    // ---- Drive subsystem (for fused pose/twist in tick()) ----
    const subsystems::Drive&  _drive;

    // Internal state used by tick() (passed to driveAdvance).
    HardwareState             _hw      = {};  // populated from _drive.state() each tick
    MotorCommands             _cmds    = {};  // sink for driveAdvance motor outputs (discarded)
    DesiredState              _desired = {};  // BVC publish target (wired via setBvcStateRef)
    TargetState&              _target;   // alias to _desired (TargetState = DesiredState)

    // Published planner state.
    msg::PlannerState         _state   = {};

    // Stored planner config snapshot (set by configure()).
    msg::PlannerConfig        _planCfg = {};

    // No-op reply sink used for apply() begin*() calls.
    static void _noopReply(const char* /*msg*/, void* /*ctx*/) {}

    // ---- Private helpers (implementations in Planner.cpp / PlannerBegin.cpp) ----
    void driveAdvance(HardwareState& inputs, MotorCommands& cmds,
                      TargetState& target, uint32_t now_ms);
    void fullStop(ReplyFn fn, void* ctx);
    void getPoseFloat(float& x, float& y, float& h_rad) const;
    void _checkSafeOneShot(ReplyFn fn, void* ctx);
    void _startPreRotate(float bearingRad, float speed,
                         uint32_t now_ms, TargetState& target);
    static void emitEvt(const char* base, TargetState& target);
};
