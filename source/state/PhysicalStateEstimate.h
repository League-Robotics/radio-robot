#pragma once
#include <stdint.h>
#include "Odometry.h"         // pulls in EKFTiny.h, types/Inputs.h transitively

// PhysicalStateEstimate — the single fused-belief object for the robot's
// physical state (Phase C, Sprint 041). Wraps Odometry by composition.
//
// Observations in: addOdometryObservation, addOtosObservation, resetPose.
// Belief out:      getPose.
//
// De-threading (070-003): every observation/reset method now takes exactly
// the inputs it reads (encoder readings, OTOS readings) and exactly the
// PoseEstimate& output slot(s) it writes, instead of the whole HardwareState/
// ActualState blob. This is an explicit, per-call contract rather than a
// "bind once at construction" one because resetPose()/zero() have TWO
// independent live destinations in the current call graph (Drive::_hw vs.
// Robot::state.actual — see architecture-update.md Decision 3); a single
// bound destination would silently collapse them. Config (trackwidth,
// rotational slip) is genuinely single-destination, so it uses "set once,
// refreshed live" instead — see setKinematics().
//
// Dependency rule: this header includes no CommandTypes.h, Commandable,
// MicroBit.h, or Protocol.h. (Odometry.h still pulls CommandTypes.h until T2
// strips Commandable from Odometry — the grep-gate baseline update is timed
// to T2, not here.)
class PhysicalStateEstimate {
public:
    PhysicalStateEstimate();

    // --- Config (070-003) ---

    // Live trackwidth/rotational-slip update — forwards to
    // Odometry::setKinematics(). Drive is the only caller of
    // addOdometryObservation(), so this config is genuinely single-
    // destination; called every tick from Drive::tickUpdate() (matching the
    // pre-070-003 every-tick freshness exactly) so sprint 067's live-SET-
    // reaches-the-estimator guarantee is preserved.
    void setKinematics(float trackwidthMm, float rotationalSlip);

    // --- Observations in ---

    // Encoder dead-reckoning + EKF predict (= Odometry::predict).
    // encLeftMm/encRightMm: raw cumulative encoder readings (mm).
    // encoderOut/fusedOut: caller-owned PoseEstimate slots written in place
    // (e.g. Drive's own _hw.encoder/_hw.fused, or Robot::state.actual's).
    void addOdometryObservation(float encLeftMm, float encRightMm,
                                uint32_t now_ms,
                                PoseEstimate& encoderOut, PoseEstimate& fusedOut);

    // OTOS EKF correction (= Odometry::correctEKF).
    // vy_otos_mmps: OTOS lateral velocity (mm/s); always 0.0f on differential builds.
    // now_ms: robot system clock (ms); used to stamp opticalOut.stamp (047-002).
    void addOtosObservation(float x_otos, float y_otos, float theta_otos_rad,
                            float v_otos_mmps, float omega_otos_rads,
                            float vy_otos_mmps, uint32_t now_ms,
                            PoseEstimate& opticalOut, PoseEstimate& fusedOut);

    // External camera re-anchor / SI verb (= Odometry::setPose).
    // encLeftMm/encRightMm: current encoder readings, used to re-baseline the
    // internal previous-encoder snapshot (see Odometry::setPose).
    void resetPose(float encLeftMm, float encRightMm,
                   int32_t x_mm, int32_t y_mm, int32_t h_cdeg,
                   PoseEstimate& encoderOut, PoseEstimate& fusedOut);

    // Zero the fused pose (= Odometry::zero). Used by the ZERO command
    // (SystemCommands::handleZero) when the pose component is reset.
    // OQ-2 (041-001): added so the ZERO call-site can repoint to estimate in
    // T3 without dangling — robot->odometry.zero() has no other forwarder.
    void zero(float encLeftMm, float encRightMm,
              PoseEstimate& encoderOut, PoseEstimate& fusedOut);

    // --- Belief out ---

    // Read current fused pose (integer mm + centidegrees) from the one
    // PoseEstimate struct this reads (070-003: narrowed from the whole
    // HardwareState).
    static void getPose(const PoseEstimate& fused,
                        int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg);

    // --- Initialisation / wiring ---
    void initEKF(float q_xy, float q_theta, float q_v, float q_omega,
                 float r_otos_xy, float r_otos_v, float r_enc_v,
                 float r_otos_theta);

    // Live noise update — forwards to Odometry::setNoise() (does NOT reset
    // EKF state/covariance). Safe to call mid-mission. Sprint 067, Ticket 003.
    void setNoise(float q_xy, float q_theta, float q_v, float q_omega,
                  float r_otos_xy, float r_otos_v, float r_enc_v,
                  float r_otos_theta);

    // --- Forwarded accessors (used by RobotTelemetry, LoopTickOnce, etc.) ---
    uint32_t otosRejectedCount() const;
    int      ekfRejectCount()    const;
    float    ekfPDiag(int idx)   const;
    float    lastEncV()          const;
    float    lastEncOmega()      const;

    bool     encOmegaHealthy()        const;
    void     setEncOmegaHealthy(bool healthy);

    bool     wedgeActive()            const;
    void     setWedgeActive(bool active);

    void     rebaselinePrev(float encL, float encR);

    // --- Access to the wrapped Odometry (for OtosCommands context in T2) ---
    Odometry& odometry() { return _odometry; }

private:
    Odometry _odometry;
};
