#pragma once
#include "PhysicsWorld.h"

// ActualState (with three PoseEstimate members) is needed for the reference
// member _actual.  Include the direct header rather than Inputs.h so clangd
// does not warn "included header not used directly".
#include "state/ActualState.h"

/**
 * WorldView — read-only bridge from PhysicsWorld ground truth into the
 * sim_api.cpp C ABI (Sprint 040 Phase B, 040-003).
 *
 * This is the ONE component that crosses the plant/estimate boundary: it holds a
 * `const PhysicsWorld&` (the single source of ground truth) and a
 * `const ActualState&` (the firmware's fused/dead-reckoned pose estimate, written
 * by Odometry into robot.state.actual).  It computes:
 *   - truePoseX/Y/H()       — the plant truth, for the sim_get_true_pose_* ABI.
 *   - estimationErrorXY()   — Euclidean distance (mm) between true pose and the
 *                             firmware fused estimate (actual.fused.pose.x/y).
 *   - estimationErrorH()    — heading error (rad), wrapped to [-pi, pi].
 *
 * WorldView holds REFERENCES (not copies) so it always reflects the current state:
 * once the plant integrates a step or Odometry updates the estimate, the next read
 * sees the new values with no explicit refresh.
 *
 * Read-only: WorldView never mutates the plant or the state.  Zero-heap,
 * single-threaded, value-member ownership.  No CODAL dependency; compiles
 * host-side (HOST_BUILD) with plain clang++ -std=c++11 -I source.
 *
 * 047-002: constructor param renamed from `inputs` to `actual`; estimationError*
 * now reads actual.fused.pose.{x,y,h} instead of legacy scalar poseX/poseHrad.
 */
class WorldView {
public:
    WorldView(const PhysicsWorld& plant, const ActualState& actual)
        : _plant(plant), _actual(actual) {}

    // --- True pose (plant truth) --------------------------------------------
    float truePoseX() const { return _plant.truePoseX(); }
    float truePoseY() const { return _plant.truePoseY(); }
    float truePoseH() const { return _plant.truePoseH(); }

    // --- True wheel travel / velocity (plant truth) -------------------------
    float trueEncL() const { return _plant.trueEncL(); }
    float trueEncR() const { return _plant.trueEncR(); }
    float trueVelL() const { return _plant.trueVelL(); }
    float trueVelR() const { return _plant.trueVelR(); }

    // --- Estimation error: firmware estimate vs. plant truth ----------------
    // Euclidean distance (mm) between (truePoseX, truePoseY) and the firmware's
    // fused pose (actual.fused.pose.x/y — 047-002).
    float estimationErrorXY() const;
    // Heading error (rad), wrapped to [-pi, pi].
    float estimationErrorH() const;

private:
    const PhysicsWorld& _plant;    // ground truth (read-only)
    const ActualState&  _actual;   // firmware pose estimate (read-only) — 047-002
};
