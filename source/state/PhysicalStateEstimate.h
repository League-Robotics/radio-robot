#pragma once
#include <stdint.h>
#include "Odometry.h"         // pulls in EKF.h, types/Inputs.h transitively

// PhysicalStateEstimate — the single fused-belief object for the robot's
// physical state (Phase C, Sprint 041). Wraps Odometry by composition.
//
// Observations in: addOdometryObservation, addOtosObservation, resetPose.
// Belief out:      getPose, getVelocity.
//
// HardwareState back-compat: each observation method mirrors the fused pose
// back into HardwareState fields (poseX/Y/poseHrad/fusedV/fusedOmega) so
// existing readers (buildTlmFrame, getPoseFloat) work unchanged until Phase F.
//
// Dependency rule: this header includes no CommandTypes.h, Commandable,
// MicroBit.h, or Protocol.h. (Odometry.h still pulls CommandTypes.h until T2
// strips Commandable from Odometry — the grep-gate baseline update is timed
// to T2, not here.)
class PhysicalStateEstimate {
public:
    PhysicalStateEstimate();

    // --- Observations in ---

    // Encoder dead-reckoning + EKF predict (= Odometry::predict, verbatim).
    void addOdometryObservation(HardwareState& s, float trackwidthMm,
                                float rotationalSlip, uint32_t now_ms);

    // OTOS EKF correction (= Odometry::correctEKF, verbatim).
    // vy_otos_mmps: OTOS lateral velocity (mm/s); mecanum build only (046-006).
    // now_ms: robot system clock (ms); used to stamp actual.optical.stamp (047-002).
    void addOtosObservation(HardwareState& s,
                            float x_otos, float y_otos,
                            float theta_otos_rad,
                            float v_otos_mmps, float omega_otos_rads,
                            float vy_otos_mmps, uint32_t now_ms);

    // External camera re-anchor / SI verb (= Odometry::setPose, verbatim).
    void resetPose(HardwareState& s,
                   int32_t x_mm, int32_t y_mm, int32_t h_cdeg);

    // Zero the fused pose (= Odometry::zero, verbatim). Used by the ZERO
    // command (SystemCommands::handleZero) when the pose component is reset.
    // OQ-2 (041-001): added so the ZERO call-site can repoint to estimate in
    // T3 without dangling — robot->odometry.zero() has no other forwarder.
    void zero(HardwareState& s);

    // --- Belief out ---

    // Read current fused pose (integer mm + centidegrees).
    static void getPose(const HardwareState& s,
                        int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg);

    // Read fused velocity (mm/s, rad/s) from HardwareState back-compat fields.
    static void getVelocity(const HardwareState& s,
                            float& v_mmps, float& omega_rads);

    // --- Initialisation / wiring ---
    void initEKF(float q_xy, float q_theta, float q_v, float q_omega,
                 float r_otos_xy, float r_otos_v, float r_enc_v,
                 float r_otos_theta);

    // Bind IOdometer* and HardwareState* for the OTOS command context.
    // (Passed through to _odometry.setCtx(); also stored for OtosCommands
    // wiring in T2.)
    void setCtx(IOdometer* otos, const HardwareState* hwState = nullptr);

    // --- Three-estimate forwarders (047-002) ---
    // Return const references into the ActualState passed to each observation
    // method — callers read whichever estimate they need without copying.
    // NOTE: the returned reference is only valid as long as the ActualState
    //       passed to the observation methods is in scope.
    const PoseEstimate& encoderEstimate(const HardwareState& s) const {
        return s.encoder;
    }
    const PoseEstimate& opticalEstimate(const HardwareState& s) const {
        return s.optical;
    }
    const PoseEstimate& fusedEstimate(const HardwareState& s) const {
        return s.fused;
    }

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

#ifdef ROBOT_DRIVETRAIN_MECANUM
    // 046-006: forward the OTOS lateral velocity filter gain to Odometry.
    void     setOtosAlphaVy(float alpha);
#endif  // ROBOT_DRIVETRAIN_MECANUM

    // --- Access to the wrapped Odometry (for OtosCommands context in T2) ---
    Odometry& odometry() { return _odometry; }

private:
    Odometry _odometry;
};
