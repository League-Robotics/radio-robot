#pragma once
#include "PhysicsWorld.h"

// HardwareState is now a using-alias for ActualState (sprint 047-001);
// cannot be forward-declared as a struct. Include the full definition.
#include "types/Inputs.h"

/**
 * WorldView — read-only bridge from PhysicsWorld ground truth into the
 * sim_api.cpp C ABI (Sprint 040 Phase B, 040-003).
 *
 * This is the ONE component that crosses the plant/estimate boundary: it holds a
 * `const PhysicsWorld&` (the single source of ground truth) and a
 * `const HardwareState&` (the firmware's fused/dead-reckoned pose estimate, written
 * by Odometry into robot.state.inputs).  It computes:
 *   - truePoseX/Y/H()       — the plant truth, for the sim_get_true_pose_* ABI.
 *   - estimationErrorXY()   — Euclidean distance (mm) between true pose and the
 *                             firmware estimate (state.inputs.poseX/Y).
 *   - estimationErrorH()    — heading error (rad), wrapped to [-pi, pi].
 *
 * WorldView holds REFERENCES (not copies) so it always reflects the current state:
 * once the plant integrates a step or Odometry updates the estimate, the next read
 * sees the new values with no explicit refresh.
 *
 * Read-only: WorldView never mutates the plant or the state.  Zero-heap,
 * single-threaded, value-member ownership.  No CODAL dependency; compiles
 * host-side (HOST_BUILD) with plain clang++ -std=c++11 -I source.
 */
class WorldView {
public:
    WorldView(const PhysicsWorld& plant, const HardwareState& inputs)
        : _plant(plant), _inputs(inputs) {}

    // --- True pose (plant truth) --------------------------------------------
    float truePoseX() const { return _plant.truePoseX(); }
    float truePoseY() const { return _plant.truePoseY(); }
    float truePoseH() const { return _plant.truePoseH(); }

    // --- True wheel travel / velocity (plant truth) -------------------------
    float trueEncLMm() const { return _plant.trueEncLMm(); }
    float trueEncRMm() const { return _plant.trueEncRMm(); }
    float trueVelLMms() const { return _plant.trueVelLMms(); }
    float trueVelRMms() const { return _plant.trueVelRMms(); }

    // --- Estimation error: firmware estimate vs. plant truth ----------------
    // Euclidean distance (mm) between (truePoseX, truePoseY) and the firmware's
    // fused/dead-reckoned pose (state.inputs.poseX/poseY).
    float estimationErrorXY() const;
    // Heading error (rad), wrapped to [-pi, pi].
    float estimationErrorH() const;

private:
    const PhysicsWorld&  _plant;    // ground truth (read-only)
    const HardwareState& _inputs;   // firmware pose estimate (read-only)
};
