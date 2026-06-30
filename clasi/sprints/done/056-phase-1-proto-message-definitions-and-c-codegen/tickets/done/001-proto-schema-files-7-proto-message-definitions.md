---
id: '001'
title: "Proto schema files \u2014 7 .proto message definitions"
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Proto schema files — 7 .proto message definitions

## Description

Create the `protos/` directory and author all 7 proto3 schema files that define the
canonical message vocabulary for the message-based subsystem architecture. These
files are the SSOT for the message contract; subsequent tickets depend on them.

This is purely a schema authoring task. No C++ code, no build hooks, no tests yet.
The deliverable is proto3 text that `protoc` accepts without error.

Also create `protos/options.proto` defining the custom `(units)` and `(max_count)`
options (both require importing `google/protobuf/descriptor.proto`).

## Message inventory to implement

**`protos/options.proto`** — custom option declarations:
- `extend google.protobuf.FieldOptions { optional string units = 50000; optional uint32 max_count = 50001; }`

**`protos/common.proto`** — shared value types:
- `Pose2D { float x_mm; float y_mm; float h_rad; }`
- `BodyTwist { float v_mmps; float omega_rads; }`
- `BodyTwist3 { float vx_mmps; float vy_mmps; float omega_rads; }`
- `BodyAccel { float ax_mmps2; float ay_mmps2; }`
- `ValueSet { uint32 lag_ms; uint32 last_upd_ms; bool valid; }`
- `PoseEstimate { Pose2D pose; BodyTwist3 twist; ValueSet stamp; }`
- `WheelTarget { optional float speed_mmps; optional float position_mm; }`
- `Gains { float kp; float ki; float kff; float i_max; float kaw; }`
- `enum Neutral { BRAKE = 0; COAST = 1; }`
- `OutCommand { uint32 verb_id; repeated float args = 1 [(max_count) = 4]; uint32 argc; bool priority; }`
- `CommandBatch { repeated OutCommand cmds = 1 [(max_count) = 8]; uint32 count; }`
- `Capabilities { repeated uint32 command_modes = 1 [(max_count) = 8]; repeated uint32 state_fields = 1 [(max_count) = 16]; bool holonomic; bool onboard_position; uint32 wheel_count; }`

**`protos/motor.proto`** — Motor subsystem:
- `MotorCommand { oneof control { float duty_cycle = 1; float voltage = 2; float velocity_mmps = 3; float position_mm = 4; Neutral neutral = 5; } optional float feedforward; }`
- `MotorState { bool connected; optional float position_mm; optional float velocity_mmps; optional float applied_pct; optional bool wedged; }`
- `MotorConfig { float mm_per_deg; int32 fwd_sign; }`
- `MotorCapabilities { bool onboard_position; bool has_encoder; }`

**`protos/drivetrain.proto`** — Drivetrain subsystem:
- `SetPose { float x_mm; float y_mm; float h_rad; }`
- `WheelTargets { repeated WheelTarget w = 1 [(max_count) = 4]; }`
- `DrivetrainCommand { oneof control { BodyTwist3 twist = 1; WheelTargets wheels = 2; Neutral neutral = 3; SetPose pose = 4; } optional bool seed; }`
- `DrivetrainState { PoseEstimate fused; PoseEstimate encoder; PoseEstimate optical; repeated float enc_mm = 1 [(max_count) = 4]; repeated float vel_mms = 1 [(max_count) = 4]; ValueSet enc; ValueSet otos; repeated bool wheel_wedged = 1 [(max_count) = 4]; bool connected; }`
- `DrivetrainConfig` — full drive config slice (geometry, PID gains, OTOS fusion, EKF noise; excludes motion limits `aMax`/`vBodyMax`/`yawRateMax` which belong to PlannerConfig):
  `{ int32 fwd_sign_l; int32 fwd_sign_r; float mm_per_deg_l; float mm_per_deg_r; float trackwidth_mm; float half_track_mm; float half_wheelbase_mm; repeated float mm_per_deg_wheel = 1 [(max_count) = 4]; repeated int32 fwd_sign_wheel = 1 [(max_count) = 4]; float v_wheel_max; float steer_headroom; Gains vel_gains; float vel_filt_alpha; float sync_gain; float min_wheel_mms; float alpha_pos; float alpha_yaw; float otos_gate; float otos_linear_scale; float otos_angular_scale; float rotation_gain_pos; float rotation_gain_neg; float rotation_offset_deg; float rotation_offset_deg_neg; float rotational_slip; float odom_off_x; float odom_off_y; float odom_yaw_deg; bool odom_upside_down; float ekf_q_xy; float ekf_q_theta; float ekf_r_otos_xy; float ekf_r_otos_theta; float ekf_q_v; float ekf_q_omega; float ekf_r_otos_v; float ekf_r_enc_v; uint32 lag_otos_ms; int32 drivetrain_type; }`
- `DrivetrainCapabilities { bool holonomic; bool onboard_position; uint32 wheel_count; }`

**`protos/sensors.proto`** — Sensors subsystem (pure-observation, no Command):
- `LineSensorState { repeated uint32 raw = 1 [(max_count) = 4]; repeated uint32 normalized = 1 [(max_count) = 4]; ValueSet stamp; bool connected; }`
- `LineSensorConfig { uint32 lag_line_ms; uint32 threshold; uint32 norm_min; uint32 norm_max; repeated uint32 channel_map = 1 [(max_count) = 4]; }`
- `ColorSensorState { uint32 r; uint32 g; uint32 b; uint32 c; ValueSet stamp; bool connected; }`
- `ColorSensorConfig { uint32 lag_color_ms; uint32 integration; uint32 gain; float cal_r; float cal_g; float cal_b; }`

**`protos/gripper.proto`** — Gripper subsystem:
- `GripperCommand { optional float angle_deg; }`
- `GripperState { optional float angle_deg; bool connected; }`
- `GripperConfig { bool has_gripper; float gripper_offset_mm; float min_deg; float max_deg; }`

**`protos/ports.proto`** — Ports subsystem:
- `DigitalOut { repeated bool value = 1 [(max_count) = 4]; uint32 mask; }`
- `AnalogOut { repeated int32 value = 1 [(max_count) = 4]; uint32 mask; }`
- `PortCommand { oneof control { DigitalOut digital_out = 1; AnalogOut analog_out = 2; } }`
- `PortState { repeated bool digital_in = 1 [(max_count) = 4]; repeated int32 analog_in = 1 [(max_count) = 4]; ValueSet stamp; }`
- `PortConfig { uint32 lag_ports_ms; repeated uint32 direction = 1 [(max_count) = 4]; }`

**`protos/planner.proto`** — Planner subsystem:
- `enum DriveMode { IDLE = 0; STREAMING = 1; DISTANCE = 3; GO_TO = 4; VELOCITY = 5; }`
- `enum StopStyle { SMOOTH = 0; ABRUPT = 1; }`
- `enum Origin { USER = 0; AUTONOMOUS = 1; }`
- `enum CmpOp { LT = 0; GT = 1; EQ = 2; }`
- `enum StopKind { NONE = 0; TIME = 1; DISTANCE = 2; HEADING = 3; POSITION = 4; SENSOR = 5; COLOR = 6; LINE_ANY = 7; ROTATION = 8; }`
- `StopCondition { StopKind kind; float a; float b; float ax; float ay; uint32 sensor; CmpOp cmp; }`
- `VelocityGoal { float vx_mmps; float vy_mmps; float omega_rads; uint32 duration_ms; }`
- `GotoGoal { float x_mm; float y_mm; float speed_mmps; }`
- `TurnGoal { float heading_rad; float speed_mmps; }`
- `DistanceGoal { float distance_mm; float speed_mmps; }`
- `TimedGoal { float vx_mmps; float omega_rads; uint32 duration_ms; }`
- `RotationGoal { float angle_rad; float speed_mmps; }`
- `StreamGoal { float vx_mmps; float vy_mmps; float omega_rads; }`
- `PlannerCommand { oneof goal { VelocityGoal velocity = 1; GotoGoal goto_goal = 2; TurnGoal turn = 3; DistanceGoal distance = 4; TimedGoal timed = 5; RotationGoal rotation = 6; StreamGoal stream = 7; bool stop = 8; } repeated StopCondition stops = 9 [(max_count) = 4]; StopStyle style; Origin origin; string corr_id [(units) = "none"]; }`
- `PlannerState { DriveMode mode; float target_x_mm; float target_y_mm; float target_speed_mms; float distance_target_mm; uint32 deadline_ms; BodyTwist3 body_twist; bool active; }`
- `PlannerConfig { float a_max [(units) = "mm/s^2"]; float a_decel [(units) = "mm/s^2"]; float v_body_max [(units) = "mm/s"]; float yaw_rate_max [(units) = "rad/s"]; float yaw_acc_max [(units) = "rad/s^2"]; float j_max; float yaw_jerk_max; float arrive_tol_mm; float turn_in_place_gate [(units) = "deg"]; float turn_threshold_mm; float done_tol_mm; float min_speed_mms; }`

## Acceptance Criteria

- [x] `protos/` directory created at repo root.
- [x] `protos/options.proto` defines `(units)` and `(max_count)` custom field options.
- [x] All 7 proto files exist: `common.proto`, `motor.proto`, `drivetrain.proto`,
      `sensors.proto`, `gripper.proto`, `ports.proto`, `planner.proto`.
- [x] `protoc --proto_path=protos --descriptor_set_out=/dev/null protos/*.proto`
      (or grpcio-tools equivalent) exits 0 without errors.
- [x] Every `repeated` field in every file carries `(max_count) = N`.
- [x] `DrivetrainConfig` excludes motion limits (`a_max`, `v_body_max`, `yaw_rate_max`)
      — those are in `PlannerConfig`.
- [x] `SetPose` is a variant in `DrivetrainCommand.control` oneof (not in PlannerCommand).
- [x] `Sensors` subsystem has no Command message (pure-observation contract).

## Implementation Plan

### Approach

Create `protos/options.proto` first (dependency of all others), then create each
`.proto` in dependency order: `common.proto` → `motor.proto` → `drivetrain.proto`
→ `sensors.proto` → `gripper.proto` → `ports.proto` → `planner.proto`.

Verify with `python3 -c "from grpc_tools import protoc; protoc.main(['-I', 'protos', '--descriptor_set_out=/dev/null', '--include_imports', 'protos/common.proto', ...])"` or install `protoc` binary and run directly.

### Files to create

- `protos/options.proto`
- `protos/common.proto`
- `protos/motor.proto`
- `protos/drivetrain.proto`
- `protos/sensors.proto`
- `protos/gripper.proto`
- `protos/ports.proto`
- `protos/planner.proto`

### Files to modify

None.

### Testing plan

No host test changes needed in this ticket. The acceptance check is `protoc` parsing.
Run: `uv run python -c "from grpc_tools import protoc; ..."` to parse all 7 files.
If `grpcio-tools` is not yet installed, install it: `uv pip install grpcio-tools`.
Ticket 002 adds it as a proper dev dependency.

### Documentation updates

None in this ticket (traceability table is ticket 004).

## Verification Command

`uv run python -m pytest` (no new tests in this ticket; run to confirm no regressions).
