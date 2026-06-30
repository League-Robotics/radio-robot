// message_test_api.cpp — extern "C" shims for testing generated message types.
//
// (ticket 056-003) Provides testable C-ABI functions that exercise the
// generated source/messages/*.h types from Python via ctypes.
//
// Phase 2 (ticket 057-001): all generated types are now in namespace msg::,
// so this file can safely include BOTH messages/common.h AND
// hal/capability/Pose2D.h in the same TU — msg::Pose2D and ::Pose2D are
// now distinct names.  The static_asserts below verify cross-namespace
// layout compatibility directly (replacing the split-TU workaround).

// Include both generated message headers AND the HAL Pose2D header in one TU.
// This was impossible before namespace migration (name collision); it works now.
#include "hal/capability/Pose2D.h"    // ::Pose2D, ::BodyTwist3, ::RobotGeometry
#include "messages/common.h"          // msg::Pose2D, msg::BodyTwist3, msg::Opt<T>, etc.
#include "messages/drivetrain.h"      // msg::DrivetrainCommand
#include "messages/motor.h"           // msg::MotorCommand
#include "messages/planner.h"         // msg::PlannerCommand, msg::PlannerConfig

// --- Cross-namespace layout-compatibility checks (Phase 2, ticket 057-001) ---
// Both msg:: and ::Pose2D/BodyTwist3 are now visible in this TU.
// Prove they have the same memory layout (same size, same field count).

// msg::Pose2D { float x_mm, y_mm, h_rad } vs ::Pose2D { float x, y, h }
static_assert(sizeof(msg::Pose2D) == sizeof(::Pose2D),
    "msg::Pose2D and ::Pose2D must have the same size — layout compat broken");
static_assert(sizeof(msg::Pose2D) == sizeof(float) * 3,
    "Generated msg::Pose2D must be 3 floats {x_mm,y_mm,h_rad} — "
    "layout compat with HAL Pose2D broken; check common.proto");

// msg::BodyTwist3 { float vx_mmps, vy_mmps, omega_rads }
// vs ::BodyTwist3 { float vx_mmps, vy_mmps, omega_rads }
static_assert(sizeof(msg::BodyTwist3) == sizeof(::BodyTwist3),
    "msg::BodyTwist3 and ::BodyTwist3 must have the same size — layout compat broken");
static_assert(sizeof(msg::BodyTwist3) == sizeof(float) * 3,
    "Generated msg::BodyTwist3 must be 3 floats — "
    "layout compat with HAL BodyTwist3 broken; check common.proto");

// msg::CommandBatch uses msg::OutCommand[8]; guard against generator regressions.
static_assert(sizeof(msg::CommandBatch) >= sizeof(msg::OutCommand) * 8,
    "msg::CommandBatch must hold at least 8 OutCommands — check common.proto max_count");

extern "C" {

// ---------------------------------------------------------------------------
// Test 1: DrivetrainCommand fluent builder round-trip.
//
// Constructs a default msg::DrivetrainCommand, calls setTwist(vx, vy, omega),
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
    msg::DrivetrainCommand cmd;
    msg::BodyTwist3 t;
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
// Constructs a msg::MotorCommand, calls setFeedforward(val).
// Returns has flag and val via out pointers.
// ---------------------------------------------------------------------------
int msg_test_motor_feedforward_present(
    float val,
    int* out_has,
    float* out_val)
{
    msg::MotorCommand m;
    m.setFeedforward(val);

    if (out_has) *out_has = m.feedforward.has ? 1 : 0;
    if (out_val) *out_val = m.feedforward.val;
    return 1;
}

// ---------------------------------------------------------------------------
// Test 3: MotorCommand Opt<float> feedforward — absent (default) case.
//
// Constructs a default msg::MotorCommand and reads the feedforward Opt.
// out_has should be 0.
// ---------------------------------------------------------------------------
int msg_test_motor_feedforward_absent(int* out_has)
{
    msg::MotorCommand m;
    if (out_has) *out_has = m.feedforward.has ? 1 : 0;
    return 1;
}

// ---------------------------------------------------------------------------
// Test 4: CommandBatch repeated-field count.
//
// Appends n_cmds default msg::OutCommand entries to a msg::CommandBatch and
// returns the cmds_count value.
// ---------------------------------------------------------------------------
int msg_test_command_batch_count(int n_cmds, int* out_count)
{
    msg::CommandBatch batch;
    int max_cap = 8;  // msg::OutCommand cmds_[8]
    int to_add = n_cmds < max_cap ? n_cmds : max_cap;
    for (int i = 0; i < to_add; ++i) {
        batch.cmds_[batch.cmds_count++] = msg::OutCommand{};
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
    msg::PlannerConfig cfg;
    cfg.setAMax(a_max).setVBodyMax(v_body_max);

    if (out_a_max)      *out_a_max      = cfg.a_max;
    if (out_v_body_max) *out_v_body_max = cfg.v_body_max;
    return 1;
}

// ---------------------------------------------------------------------------
// Test 6b: Namespace isolation — msg::DrivetrainCommand::ControlKind::TWIST.
//
// Verifies that the ControlKind enum is accessible as
// msg::DrivetrainCommand::ControlKind::TWIST and equals 1.
// Returns 1 on success, 0 on failure.
// ---------------------------------------------------------------------------
int msg_test_drivetrain_control_kind_enum(int* out_twist_val)
{
    // Verify msg::DrivetrainCommand::ControlKind::TWIST == 1
    int twist_val = static_cast<int>(msg::DrivetrainCommand::ControlKind::TWIST);
    if (out_twist_val) *out_twist_val = twist_val;
    return (twist_val == 1) ? 1 : 0;
}

} // extern "C"
