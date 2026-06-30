// message_test_api.cpp — extern "C" shims for testing generated message types.
//
// (ticket 056-003) Provides testable C-ABI functions that exercise the
// generated source/messages/*.h types from Python via ctypes.
//
// This file intentionally does NOT include hal/capability/Pose2D.h.
// Including both messages/common.h and Pose2D.h in one TU causes a
// redefinition of struct Pose2D (same global name, different fields).
// The compile-time layout-compatibility proof is split:
//   - This file verifies the generated types (Pose2D, BodyTwist3) are the
//     correct sizes.
//   - source/messages/bridges.h verifies the HAL types are the same sizes.
// Together the static_asserts prove layout compatibility without needing to
// include both in one TU.

// Include generated message headers (no HAL headers — see comment above).
#include "messages/common.h"       // Pose2D, BodyTwist3, Opt<T>, CommandBatch, OutCommand, etc.
#include "messages/drivetrain.h"   // DrivetrainCommand (includes common.h)
#include "messages/motor.h"        // MotorCommand (includes common.h)
#include "messages/planner.h"      // PlannerCommand, PlannerConfig (includes common.h)

// --- Generated-side size checks (ticket 003 bridges verify) ---
// These are the generated counterparts to the HAL-side checks in bridges.h.
// Both sides must be sizeof(float)*N for layout compatibility.

// Generated Pose2D: { float x_mm, y_mm, h_rad } — must match HAL Pose2D { float x, y, h }.
static_assert(sizeof(Pose2D) == sizeof(float) * 3,
    "Generated Pose2D must be 3 floats {x_mm,y_mm,h_rad} — "
    "layout compat with HAL Pose2D broken; check common.proto");

// Generated BodyTwist3: { float vx_mmps, vy_mmps, omega_rads } — must match HAL BodyTwist3.
static_assert(sizeof(BodyTwist3) == sizeof(float) * 3,
    "Generated BodyTwist3 must be 3 floats — "
    "layout compat with HAL BodyTwist3 broken; check common.proto");

// CommandBatch uses OutCommand[8] (common.h); verify OutCommand has verb_id + args[4].
// (Not a cross-HAL check; guards against generator regressions.)
static_assert(sizeof(CommandBatch) >= sizeof(OutCommand) * 8,
    "CommandBatch must hold at least 8 OutCommands — check common.proto max_count");

extern "C" {

// ---------------------------------------------------------------------------
// Test 1: DrivetrainCommand fluent builder round-trip.
//
// Constructs a default DrivetrainCommand, calls setTwist(vx, vy, omega),
// then reads back control.twist.{vx_mmps, vy_mmps, omega_rads} and the
// control_kind discriminant.
//
// Returns 1 on success, 0 on failure.
// out_vx, out_vy, out_omega: the read-back values.
// out_kind: the ControlKind value (1 == TWIST).
// ---------------------------------------------------------------------------
int msg_test_drivetrain_twist_roundtrip(
    float vx, float vy, float omega,
    float* out_vx, float* out_vy, float* out_omega,
    int* out_kind)
{
    DrivetrainCommand cmd;
    BodyTwist3 t;
    t.vx_mmps    = vx;
    t.vy_mmps    = vy;
    t.omega_rads = omega;
    cmd.setTwist(t);

    if (out_vx)    *out_vx    = cmd.control.twist.vx_mmps;
    if (out_vy)    *out_vy    = cmd.control.twist.vy_mmps;
    if (out_omega) *out_omega = cmd.control.twist.omega_rads;
    if (out_kind)  *out_kind  = static_cast<int>(cmd.control_kind);
    return 1;
}

// ---------------------------------------------------------------------------
// Test 2: MotorCommand Opt<float> feedforward — present case.
//
// Constructs a MotorCommand, calls setFeedforward(val).
// Returns has flag and val via out pointers.
// ---------------------------------------------------------------------------
int msg_test_motor_feedforward_present(
    float val,
    int* out_has,
    float* out_val)
{
    MotorCommand m;
    m.setFeedforward(val);

    if (out_has) *out_has = m.feedforward.has ? 1 : 0;
    if (out_val) *out_val = m.feedforward.val;
    return 1;
}

// ---------------------------------------------------------------------------
// Test 3: MotorCommand Opt<float> feedforward — absent (default) case.
//
// Constructs a default MotorCommand and reads the feedforward Opt.
// out_has should be 0.
// ---------------------------------------------------------------------------
int msg_test_motor_feedforward_absent(int* out_has)
{
    MotorCommand m;
    if (out_has) *out_has = m.feedforward.has ? 1 : 0;
    return 1;
}

// ---------------------------------------------------------------------------
// Test 4: CommandBatch repeated-field count.
//
// Appends n_cmds default OutCommand entries to a CommandBatch and returns
// the cmds_count value.
// ---------------------------------------------------------------------------
int msg_test_command_batch_count(int n_cmds, int* out_count)
{
    CommandBatch batch;
    int max_cap = 8;  // OutCommand cmds_[8]
    int to_add = n_cmds < max_cap ? n_cmds : max_cap;
    for (int i = 0; i < to_add; ++i) {
        batch.cmds_[batch.cmds_count++] = OutCommand{};
    }
    if (out_count) *out_count = static_cast<int>(batch.cmds_count);
    return 1;
}

// ---------------------------------------------------------------------------
// Test 5: PlannerConfig chainable setters.
//
// Calls cfg.setAMax(a_max).setVBodyMax(v_body_max) and reads back the values.
// ---------------------------------------------------------------------------
int msg_test_planner_config_chained(
    float a_max, float v_body_max,
    float* out_a_max, float* out_v_body_max)
{
    PlannerConfig cfg;
    cfg.setAMax(a_max).setVBodyMax(v_body_max);

    if (out_a_max)      *out_a_max      = cfg.a_max;
    if (out_v_body_max) *out_v_body_max = cfg.v_body_max;
    return 1;
}

} // extern "C"
