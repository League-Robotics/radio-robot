#pragma once
#include <stdint.h>
#include "Inputs.h"
#include "state/EKFTiny.h"

// ---------------------------------------------------------------------------
// effectiveSlip — migration-safe rotationalSlip clamp.
//
// Used by Odometry::predict() and Planner::beginRotation() so both
// apply the same clamped slip factor from RobotConfig::rotationalSlip.
//
// Value semantics:
//   0.0 (or negative, or unset) → 1.0  (no correction; legacy config-safe)
//   (0.0, 0.5)                  → 0.5  (clamp floor — unrealistic slip)
//   [0.5, 1.0]                  → pass-through
//   > 1.0                       → 1.0  (clamp ceiling)
//
// Sprint 024-006: rotationalSlip is now wired into firmware logic (was dead).
// ---------------------------------------------------------------------------
inline float effectiveSlip(float rawSlip) {
    if (rawSlip <= 0.0f) return 1.0f;
    if (rawSlip < 0.5f)  return 0.5f;
    if (rawSlip > 1.0f)  return 1.0f;
    return rawSlip;
}

/**
 * Odometry — differential-drive dead-reckoning pose tracker.
 *
 * Heading convention: 0 = +X axis, positive = CCW (standard math).
 * Output convention: centidegrees (360 degrees = 36000 cdeg).
 *
 * De-threading (070-003): Odometry no longer takes a HardwareState&/
 * ActualState& on any (reachable) method. Each method takes exactly the
 * inputs it reads (raw encoder readings, OTOS readings) and exactly the
 * PoseEstimate& output slot(s) it writes; the caller owns where those slots
 * live (Drive::_hw, Robot::state.actual, etc. — Odometry no longer knows or
 * cares). Kinematics config (trackwidth, rotational slip) is set via
 * setKinematics(), called every tick by Drive::tickUpdate() from its own
 * live RobotConfig reference — this is "set once, refreshed live" rather
 * than per-call because Drive is the only caller of predict(). Odometry owns
 * only its previous-encoder snapshot (_prevEncL/_prevEncR) as intermediate
 * compute state — separate from the control task's counters.
 *
 * Primary API: call predict(encLeft, encRight, now, encoderOut,
 * fusedOut) once per odometry-predict task tick (after setKinematics() has
 * been called with the live trackwidth/rotationalSlip for this tick).
 * Computes deltas against _prevEncL/_prevEncR, applies midpoint (exact-arc)
 * integration per docs/kinematics-model.md §2.4, and writes the updated pose
 * into fusedOut.pose.{x,y,h} (before the EKF predict step overwrites it):
 *
 *   dC = (dL + dR)/2 ;  dθ = (dR − dL)/b
 *   θ_mid = θ + dθ/2
 *   x += dC·cos(θ_mid) ;  y += dC·sin(θ_mid) ;  θ = wrapπ(θ + dθ)
 *
 * Sprint 010, Ticket 005.
 * Sprint 010, Ticket 006: adds correct() for OTOS complementary fusion.
 * Sprint 014, Ticket 004: pose moved to HardwareState; struct-based API.
 * Sprint 022, Ticket 003: EKF integration — predict() drives EKF each tick;
 *   initEKF()/correctEKF() wire noise params and OTOS position updates.
 * Sprint 023, Ticket 003: 5-state EKF wiring — initEKF() extended with
 *   velocity noise params; predict() gains now for dt computation;
 *   correctEKF() extended for OTOS + encoder velocity fusion. setPose
 *   re-baseline fix (_prevEncL = s.encPos[], not 0) prevents spurious
 *   encoder-delta jumps after camera fixes.
 * Sprint 047, Ticket 002: encoder-only dead-reckoning accumulator added
 *   (_encPoseX/Y/H, _encVx/Vy/Omega); predict() dual-writes encoder and
 *   fused estimates; correctEKF() captures raw OTOS into actual.optical.
 * Sprint 070, Ticket 003: de-threaded — HardwareState&/ActualState&
 *   parameters replaced with explicit encoder-input/PoseEstimate-output
 *   parameters; trackwidth/rotationalSlip moved to setKinematics() members;
 *   setCtx() deleted (was already a documented no-op). correct() (dead, zero
 *   callers) is untouched and still takes HardwareState& — see Decision 6.
 * Sprint 071, Ticket 005: stripped unit suffixes from parameter/member names
 *   (encLeftMm/encRightMm → encLeft/encRight, now_ms → now, etc.) — identifier
 *   rename only, no behavioral change; see docs/coding-standards.md.
 */
class Odometry {
public:
    Odometry();

    // ---------------------------------------------------------------------------
    // Kinematics config (070-003)
    // ---------------------------------------------------------------------------

    // Live trackwidth/rotational-slip update, read by predict() on every
    // subsequent call. Called every tick from Drive::tickUpdate() (Drive is
    // the only caller of predict()), so this preserves the pre-070-003
    // every-tick freshness — and sprint 067's live-SET-reaches-the-estimator
    // guarantee — exactly.
    void setKinematics(float trackwidth, float rotationalSlip) {
        _trackwidth     = trackwidth;
        _rotationalSlip = rotationalSlip;
    }

    // ---------------------------------------------------------------------------
    // Primary API — struct-based (014-004)
    // ---------------------------------------------------------------------------

    // Midpoint (exact-arc) integration — primary predict step.
    // encLeft/encRight: raw cumulative encoder readings (mm) — computes
    // deltas against _prevEncL/_prevEncR, then writes the updated pose into
    // fusedOut.pose.{x,y,h} and the encoder-only dead-reckoning accumulator
    // into encoderOut. Also advances the EKF and writes EKF state into
    // fusedOut.{pose,twist,stamp}.
    //
    // Reads trackwidth/rotationalSlip from the members set by the most
    // recent setKinematics() call (070-003).
    //   dTheta = ((dR-dL)/trackwidth) * clamp(rotationalSlip, 0.5, 1.0).
    //   0 / unset → 1.0 (migration-safe: no change for legacy exact-profile configs).
    // now: robot system clock timestamp (ms). Used to compute dt for the
    // EKF and encoder-rate velocity. Signed delta cast avoids uint32 underflow
    // (see watchdog-uint32-underflow finding).
    void predict(float encLeft,          // [mm]
                 float encRight,         // [mm]
                 uint32_t now,           // [ms]
                 PoseEstimate& encoderOut, PoseEstimate& fusedOut);

    // EKF initialisation — set process and measurement noise parameters.
    // Must be called once at startup (e.g. from Robot constructor) before
    // predict() or correctEKF() are called.
    //   q_xy         — process noise variance for x and y (mm^2)
    //   q_theta      — process noise variance for heading (rad^2)
    //   q_v          — process noise variance for linear velocity (mm/s)^2
    //   q_omega      — process noise variance for angular velocity (rad/s)^2
    //   r_otos_xy    — OTOS position measurement noise variance (mm^2)
    //   r_otos_v     — OTOS velocity measurement noise variance ((mm/s)^2)
    //   r_enc_v      — encoder velocity measurement noise variance ((mm/s)^2)
    //   r_otos_theta — OTOS heading measurement noise variance (rad^2)
    void initEKF(float q_xy, float q_theta, float q_v, float q_omega,
                 float r_otos_xy, float r_otos_v, float r_enc_v,
                 float r_otos_theta);

    // Live noise update — forwards to EKFTiny::setNoise() (does NOT reset
    // EKF state/covariance) and refreshes Odometry's own cached
    // _rOtosTheta (read by correctEKF() when fusing OTOS heading). Safe to
    // call at any time, including mid-mission, e.g. from a "drive"-annotated
    // SET of an EKF noise key. Parameters match initEKF()'s of the same name.
    // Sprint 067, Ticket 003.
    void setNoise(float q_xy, float q_theta, float q_v, float q_omega,
                  float r_otos_xy, float r_otos_v, float r_enc_v,
                  float r_otos_theta);

    // EKF correction — fuse OTOS position, heading, and velocity measurements
    // into the 5-state EKF.
    // Call order: updatePosition → updateHeading → updateVelocity(OTOS).
    // All channels apply Mahalanobis gating internally. Writes all EKF
    // outputs into fusedOut; the raw OTOS observation into opticalOut.
    //   x_otos, y_otos — OTOS position observation (mm)
    //   thetaOtos      — OTOS heading observation (rad) — sprint 024-004
    //   vOtos          — OTOS body-frame linear velocity (mm/s)
    //   omegaOtos      — OTOS angular velocity (rad/s)
    //   vyOtos         — OTOS lateral velocity (mm/s); always 0.0f on
    //                    differential builds (captured into optical.twist
    //                    for logging; not fused into fused.twist.vy).
    //   now            — robot system clock (ms); used to stamp
    //                    opticalOut.stamp.lastUpdMs (047-002)
    //
    // 033-003: encoder-derived velocity is fused unconditionally in predict(),
    // NOT here — fusing it in both paths would double-count it per OTOS tick.
    //
    // 047-002: now added for optical.stamp.lastUpdMs; callers must pass
    // the same now they use for the corresponding predict() call.
    void correctEKF(float x_otos, float y_otos,
                    float thetaOtos,     // [rad]
                    float vOtos,         // [mm/s]
                    float omegaOtos,     // [rad/s]
                    float vyOtos,        // [mm/s]
                    uint32_t now,        // [ms]
                    PoseEstimate& opticalOut, PoseEstimate& fusedOut);

    // OTOS complementary correction — correct step of predict/correct.
    // (docs/kinematics-model.md §2.4; EKF upgrade path replaces this later.)
    //
    // Parameters x_otos, y_otos, thetaOtos are passed by the caller
    // (who reads them from s.otosX/Y/H after unit conversion) for testability.
    //
    // Outlier gate: if distance(otos, predicted pose) > otosGate, the sample
    // is rejected; _otosRejected is incremented and s.poseX/Y/Hrad are unchanged.
    //
    // If accepted:
    //   s.poseX    += alphaPos * (x_otos - s.poseX)
    //   s.poseY    += alphaPos * (y_otos - s.poseY)
    //   s.poseHrad += alphaYaw * wrapπ(θ_otos - s.poseHrad)   [angle-wrap safe]
    //
    // alphaPos, alphaYaw: blend gains in [0, 1] (from RobotConfig).
    // otosGate: rejection distance threshold in mm (from RobotConfig).
    //
    // DEAD CODE (070-003 Decision 6): confirmed zero callers (superseded by
    // the EKF cutover, per sprint 067's audit, re-confirmed this sprint). Not
    // reachable via any PhysicalStateEstimate method, so it is out of this
    // ticket's de-threading scope and is left untouched, still taking
    // HardwareState& — mirrors sprint 067 Decision 5's "document dead
    // things, don't fix them" precedent.
    void correct(HardwareState& s,
                 float x_otos, float y_otos, float thetaOtos,   // [rad]
                 float alphaPos, float alphaYaw, float otosGate);

    // ---------------------------------------------------------------------------
    // Pose read / write helpers (070-003: PoseEstimate-based, not HardwareState&).
    // ---------------------------------------------------------------------------

    // Read current pose from fused.  x and y are integer mm; h is
    // centidegrees (-18000..+18000 clamped).
    static void getPose(const PoseEstimate& fused,
                        int32_t& x,          // [mm]
                        int32_t& y,          // [mm]
                        int32_t& h);         // [cdeg]

    // Overwrite pose (used by Robot::distanceDrive / SI command).
    // h is centidegrees; stored internally as radians.
    // encLeft/encRight: current encoder readings — re-baselines
    // _prevEncL/_prevEncR to these (not 0) so the next predict() sees a delta
    // of ~0 instead of a spurious jump.
    // (Sprint 023: was = 0.0f, which caused encoder-delta corruption after
    // every camera fix when encoders were non-zero.)
    void setPose(float encLeft,          // [mm]
                 float encRight,         // [mm]
                 int32_t x,              // [mm]
                 int32_t y,              // [mm]
                 int32_t h,              // [cdeg]
                 PoseEstimate& encoderOut, PoseEstimate& fusedOut);

    // Zero pose: equivalent to setPose(encLeft, encRight, 0, 0, 0, ...).
    void zero(float encLeft, float encRight,
              PoseEstimate& encoderOut, PoseEstimate& fusedOut);

    // rebaselinePrev — reset the internal encoder snapshot to (encL, encR) without
    // touching pose.  Called by Robot::resetEncoders() so that the next predict()
    // sees a delta of 0 after a hardware accumulator reset, preventing the
    // large-negative-delta backward teleport on every D command and ZERO enc.
    // (N1 fix, sprint 030-001.)
    void rebaselinePrev(float encL, float encR) {
        _prevEncL = encL;
        _prevEncR = encR;
    }

    // ---------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------

    // Number of OTOS samples rejected by the outlier gate since boot.
    uint32_t otosRejectedCount() const { return _otosRejected; }

    // Cumulative EKF gate rejection count (all channels: position, heading, velocity).
    // Sprint 024-005: exposed for TLM ekf_rej= field.
    int ekfRejectCount() const { return _ekf.getRejectCount(); }

    // N15 test helper: return P[idx][idx] from the EKF (idx in [0..4]).
    float ekfPDiag(int idx) const { return _ekf.pDiag(idx); }

    // Encoder-rate velocity from the most recent predict() call.
    // Zero-initialised; updated each predict() tick.  predict() fuses these into
    // the EKF velocity channels unconditionally (033-003); the accessors remain
    // for telemetry / tests.
    float lastEncV()     const { return _lastEncV; }     // body linear speed, mm/s
    float lastEncOmega() const { return _lastEncOmega; } // yaw rate, rad/s

    // Encoder-omega health gate (033-003 / 033-005).  predict() fuses the
    // encoder yaw-rate observation into the EKF every tick; when a wheel is
    // wedged the differential term is phantom, so the wedge detector (033-005)
    // sets this false to suppress the omega observation.  Linear v still fuses.
    // Defaults true (both encoders assumed healthy).
    void setEncOmegaHealthy(bool healthy) { _encOmegaHealthy = healthy; }
    bool encOmegaHealthy() const { return _encOmegaHealthy; }

    // Wedge-active gate (033-005e).  When true, predict() suppresses the
    // differential term dTheta from the pose integration and the EKF predict
    // step, preventing a frozen wheel from injecting phantom heading rotation.
    // Robot::controlCollectSplitPhase() drives this from wheelWedgedL/R().
    // Defaults false (no wedge active at startup).
    void setWedgeActive(bool active) { _wedgeActive = active; }
    bool wedgeActive() const { return _wedgeActive; }


private:
    // Kinematics config (070-003): trackwidth/rotational-slip, set by
    // setKinematics() and read by predict(). Refreshed every tick by
    // Drive::tickUpdate() from its own live RobotConfig reference (was
    // passed as explicit predict() parameters before this refactor) — sprint
    // 067's live-SET-reaches-predict() guarantee is unaffected, since Drive
    // still reads _robCfg live and calls setKinematics() immediately before
    // addOdometryObservation() every tick.
    // Defaults (128mm / 0-unset) are never actually exercised in production
    // (Drive always calls setKinematics() before the first predict()); they
    // exist only so a construction-time predict() call would not divide by
    // zero.
    float    _trackwidth     = 128.0f; // [mm] default matches Config.h's trackwidth default.
    float    _rotationalSlip = 0.0f;   // 0 = unset -> effectiveSlip() returns 1.0.

    // Intermediate compute state: previous encoder snapshot (not in HardwareState
    // because Odometry runs at a different cadence than the control task).
    float    _prevEncL;   // [mm] last encoder snapshot
    float    _prevEncR;   // [mm] last encoder snapshot

    // Encoder-only dead-reckoning accumulator (047-002).
    // Integrated from wheel deltas only; the EKF NEVER writes here.
    // Used to populate actual.encoder.{pose,twist,stamp} every predict() tick.
    // Reset by setPose()/zero() to the new pose value.
    float    _encPoseX;    // [mm] encoder-only pose X
    float    _encPoseY;    // [mm] encoder-only pose Y
    float    _encPoseH;    // [rad] encoder-only heading
    float    _encVx;       // [mm/s] encoder-only body linear speed
    float    _encVy;       // [mm/s] encoder-only lateral speed (always 0 for differential)
    float    _encOmega;    // [rad/s] encoder-only yaw rate

    uint32_t _otosRejected; // count of OTOS samples rejected by outlier gate

    // dt tracking for encoder-rate velocity and EKF timestep.
    // uint32_t but delta is cast to int32_t before arithmetic to avoid underflow.
    uint32_t _lastPredict;  // [ms] timestamp of last predict() call (0 = not yet called)

    // Noise params stored from initEKF() — passed to update*() methods.
    // Using _rOtosV for both v and omega of OTOS source (symmetric simplification),
    // and _rEncV for both v and omega of encoder source.
    float _rOtosV;     // OTOS velocity measurement noise variance ((mm/s)^2)
    float _rEncV;      // encoder velocity measurement noise variance ((mm/s)^2)
    float _rOtosTheta; // OTOS heading measurement noise variance (rad^2) — sprint 024-004

    // Encoder-derived velocity from the most recent predict() call.
    // Set to 0 until the first valid predict() tick (dt > 0).
    float _lastEncV;     // body linear speed (mm/s)
    float _lastEncOmega; // yaw rate (rad/s)

    // Encoder-omega health gate (033-003).  When false, predict() suppresses the
    // encoder yaw-rate observation (wedged wheel → phantom omega).  Driven by the
    // wedge detector (033-005); defaults true.
    bool _encOmegaHealthy = true;

    // Wedge-active gate (033-005e).  When true, predict() zeroes dTheta before
    // the pose integration and EKF predict step.  Driven by the wedge detector
    // (Robot::controlCollectSplitPhase() via wheelWedgedL/R()).  Defaults false.
    bool _wedgeActive = false;

    EKFTiny _ekf;          // Extended Kalman Filter — fuses encoder odometry with OTOS


    // Wrap heading to (-π, π] using atan2f identity.
    static float wrapPi(float theta);

    static constexpr float PI_F         = 3.14159265f;
    static constexpr float kAngleScale    = 18000.0f / 3.14159265f;  // [cdeg/rad]
    static constexpr float kAngleScaleInv = 3.14159265f / 18000.0f;  // [rad/cdeg]
};
