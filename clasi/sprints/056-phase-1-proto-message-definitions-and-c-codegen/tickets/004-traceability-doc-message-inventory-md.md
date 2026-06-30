---
id: '004'
title: "Traceability doc \u2014 message-inventory.md"
status: done
use-cases:
- SUC-005
depends-on:
- '003'
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Traceability doc — message-inventory.md

## Description

Populate the hand-authored mapping dict inside `scripts/gen_messages.py` that
maps every generated message field to its corresponding member in the existing
codebase (`ActualState`, `DesiredState`, `RobotConfig`, or the portable-motor-
interface spec). Then run `gen_messages.py --emit-inventory` to generate
`docs/design/message-inventory.md` and check it in.

This is the final Phase 1 deliverable. It closes the loop between the proto
schema (ticket 001), the generated types (ticket 002), and the existing firmware
state model — de-risking Phase 2 by proving every generated field has a known
home in the current codebase.

## Mapping dict specification

Inside `scripts/gen_messages.py`, add (or extend) a Python dict:

```python
# Maps (MessageName, field_name) -> existing codebase location string.
# Used by --emit-inventory to populate the traceability table.
FIELD_TRACE: dict[tuple[str,str], str] = {
    # DrivetrainState
    ("DrivetrainState", "fused"):        "ActualState::fused (PoseEstimate)",
    ("DrivetrainState", "encoder"):      "ActualState::encoder (PoseEstimate)",
    ("DrivetrainState", "optical"):      "ActualState::optical (PoseEstimate)",
    ("DrivetrainState", "enc_mm"):       "ActualState::encMm[kWheelCount]",
    ("DrivetrainState", "vel_mms"):      "ActualState::velMms[kWheelCount]",
    ("DrivetrainState", "enc"):          "ActualState::enc (ValueSet)",
    ("DrivetrainState", "otos"):         "ActualState::otos (ValueSet)",
    ("DrivetrainState", "wheel_wedged"): "ActualState::wheelWedged (not yet in ActualState — new field)",
    ("DrivetrainState", "connected"):    "ActualState (implicit — new field)",

    # DrivetrainCommand
    ("DrivetrainCommand", "twist"):   "DesiredState::bodyTwist (BodyTwist3)",
    ("DrivetrainCommand", "wheels"):  "DesiredState::wheelMms[kWheelCount] (per-wheel speed)",
    ("DrivetrainCommand", "neutral"): "OutputState / HaltController (new message wrapping existing BRAKE/COAST logic)",
    ("DrivetrainCommand", "pose"):    "Superstructure::handleSI() / estimate.resetPose (SI verb re-anchor)",
    ("DrivetrainCommand", "seed"):    "SI immediate-seed flag (new field)",

    # DrivetrainConfig — full RobotConfig drive slice
    ("DrivetrainConfig", "fwd_sign_l"):        "RobotConfig::fwdSignL",
    ("DrivetrainConfig", "fwd_sign_r"):        "RobotConfig::fwdSignR",
    ("DrivetrainConfig", "mm_per_deg_l"):      "RobotConfig::mmPerDegL",
    ("DrivetrainConfig", "mm_per_deg_r"):      "RobotConfig::mmPerDegR",
    ("DrivetrainConfig", "trackwidth_mm"):     "RobotConfig::trackwidthMm",
    ("DrivetrainConfig", "v_wheel_max"):       "RobotConfig::vWheelMax",
    ("DrivetrainConfig", "steer_headroom"):    "RobotConfig::steerHeadroom",
    ("DrivetrainConfig", "vel_gains"):         "RobotConfig::{velKp,velKi,velKff,velIMax,velKaw}",
    ("DrivetrainConfig", "vel_filt_alpha"):    "RobotConfig::velFiltAlpha",
    ("DrivetrainConfig", "sync_gain"):         "RobotConfig::syncGain",
    ("DrivetrainConfig", "min_wheel_mms"):     "RobotConfig::minWheelMms",
    ("DrivetrainConfig", "alpha_pos"):         "RobotConfig::alphaPos",
    ("DrivetrainConfig", "alpha_yaw"):         "RobotConfig::alphaYaw",
    ("DrivetrainConfig", "otos_gate"):         "RobotConfig::otosGate",
    ("DrivetrainConfig", "otos_linear_scale"): "RobotConfig::otosLinearScale",
    ("DrivetrainConfig", "otos_angular_scale"):"RobotConfig::otosAngularScale",
    ("DrivetrainConfig", "rotation_gain_pos"): "RobotConfig::rotationGainPos",
    ("DrivetrainConfig", "rotation_gain_neg"): "RobotConfig::rotationGainNeg",
    ("DrivetrainConfig", "ekf_q_xy"):          "RobotConfig::ekfQxy",
    ("DrivetrainConfig", "ekf_q_theta"):       "RobotConfig::ekfQtheta",
    ("DrivetrainConfig", "ekf_r_otos_xy"):     "RobotConfig::ekfROtosXy",
    ("DrivetrainConfig", "ekf_r_otos_theta"):  "RobotConfig::ekfROtosTheta",
    ("DrivetrainConfig", "lag_otos_ms"):       "RobotConfig::lagOtosMs",
    ("DrivetrainConfig", "drivetrain_type"):   "RobotConfig::drivetrain",
    ("DrivetrainConfig", "half_track_mm"):     "RobotConfig::halfTrackMm",
    ("DrivetrainConfig", "half_wheelbase_mm"): "RobotConfig::halfWheelbaseMm",

    # MotorCommand — portable-motor-interface verbs
    ("MotorCommand", "duty_cycle"):    "portable-motor-interface: DUTY_CYCLE verb",
    ("MotorCommand", "voltage"):       "portable-motor-interface: VOLTAGE verb",
    ("MotorCommand", "velocity_mmps"): "portable-motor-interface: VELOCITY verb / IVelocityMotor::setSpeed()",
    ("MotorCommand", "position_mm"):   "portable-motor-interface: POSITION verb / IPositionMotor",
    ("MotorCommand", "neutral"):       "portable-motor-interface: NEUTRAL verb (BRAKE/COAST)",
    ("MotorCommand", "feedforward"):   "MotorController vel loop kFF term",

    # MotorState
    ("MotorState", "connected"):      "IVelocityMotor::connected() (new capability query)",
    ("MotorState", "position_mm"):    "IPositionMotor / ActualState::encMm per wheel",
    ("MotorState", "velocity_mmps"):  "ActualState::velMms per wheel / IVelocityMotor::readSpeed()",
    ("MotorState", "applied_pct"):    "OutputState::motorPct (or equivalent) — new field",
    ("MotorState", "wedged"):         "ActualState::wheelWedged / WedgeTest",

    # PlannerCommand — DesiredState / GoalRequest
    ("PlannerCommand", "velocity"):   "DesiredState DriveMode::VELOCITY / bodyTwistRaw (VW/VX verbs)",
    ("PlannerCommand", "goto_goal"):  "DesiredState DriveMode::GO_TO / targetXWorld,targetYWorld",
    ("PlannerCommand", "turn"):       "DesiredState DriveMode::GO_TO (heading goal)",
    ("PlannerCommand", "distance"):   "DesiredState DriveMode::DISTANCE / distanceTargetMm",
    ("PlannerCommand", "timed"):      "DesiredState DriveMode::STREAMING + deadlineMs (T verb)",
    ("PlannerCommand", "rotation"):   "MotionCommand::handleRotation / DriveMode::DISTANCE",
    ("PlannerCommand", "stream"):     "DesiredState DriveMode::STREAMING (S verb, continuous)",
    ("PlannerCommand", "stop"):       "MotionCommand::handleStop / DriveMode::IDLE",
    ("PlannerCommand", "stops"):      "StopCondition stops[4] (control/StopCondition.h)",
    ("PlannerCommand", "corr_id"):    "DesiredState::corrId[16]",

    # PlannerState
    ("PlannerState", "mode"):                 "DesiredState::mode (DriveMode enum)",
    ("PlannerState", "target_x_mm"):          "DesiredState::targetXWorld",
    ("PlannerState", "target_y_mm"):          "DesiredState::targetYWorld",
    ("PlannerState", "target_speed_mms"):     "DesiredState::targetSpeedMms",
    ("PlannerState", "distance_target_mm"):   "DesiredState::distanceTargetMm",
    ("PlannerState", "deadline_ms"):          "DesiredState::deadlineMs",
    ("PlannerState", "body_twist"):           "DesiredState::bodyTwist (profiled live setpoint)",
    ("PlannerState", "active"):               "DesiredState (implicit — new convenience field)",

    # PlannerConfig — motion-only RobotConfig subset
    ("PlannerConfig", "a_max"):            "RobotConfig::aMax",
    ("PlannerConfig", "a_decel"):          "RobotConfig::aDecel",
    ("PlannerConfig", "v_body_max"):       "RobotConfig::vBodyMax",
    ("PlannerConfig", "yaw_rate_max"):     "RobotConfig::yawRateMax",
    ("PlannerConfig", "yaw_acc_max"):      "RobotConfig::yawAccMax",
    ("PlannerConfig", "arrive_tol_mm"):    "RobotConfig::arriveTolMm",
    ("PlannerConfig", "turn_in_place_gate"):"RobotConfig::turnInPlaceGate",
    ("PlannerConfig", "turn_threshold_mm"):"RobotConfig::turnThresholdMm",
    ("PlannerConfig", "done_tol_mm"):      "RobotConfig::doneTolMm",
    ("PlannerConfig", "min_speed_mms"):    "RobotConfig::minSpeedMms",

    # Sensors subsystem
    ("LineSensorState", "raw"):        "ActualState::line[4] (raw ADC)",
    ("LineSensorState", "normalized"): "ActualState::line[4] (post-normalization — new split)",
    ("LineSensorState", "stamp"):      "ActualState::lineVS (ValueSet)",
    ("LineSensorState", "connected"):  "LineSensor::connected() (new capability query)",
    ("LineSensorConfig", "lag_line_ms"):"RobotConfig::lagLineMs",
    ("ColorSensorState", "r"):         "ActualState::colorR",
    ("ColorSensorState", "g"):         "ActualState::colorG",
    ("ColorSensorState", "b"):         "ActualState::colorB",
    ("ColorSensorState", "c"):         "ActualState::colorC",
    ("ColorSensorState", "stamp"):     "ActualState::colorVS (ValueSet)",
    ("ColorSensorState", "connected"): "ColorSensor::connected() (new capability query)",
    ("ColorSensorConfig", "lag_color_ms"):"RobotConfig::lagColorMs",

    # Ports subsystem
    ("PortState", "digital_in"):  "ActualState::digitalIn[4]",
    ("PortState", "analog_in"):   "ActualState::analogIn[4]",
    ("PortState", "stamp"):       "ActualState::portsVS (ValueSet)",
    ("PortConfig", "lag_ports_ms"):"RobotConfig::lagPortsMs",

    # Gripper subsystem
    ("GripperState", "angle_deg"):     "OutputState::servoAngle / ServoController (new message wrapping)",
    ("GripperConfig", "has_gripper"):  "RobotConfig (not present — new field for has-gripper capability)",
    ("GripperConfig", "gripper_offset_mm"):"RobotConfig (not present — new field)",
    ("GripperConfig", "min_deg"):      "RobotConfig (not present — new field)",
    ("GripperConfig", "max_deg"):      "RobotConfig (not present — new field)",
}
```

The implementer should complete and correct this mapping dict based on the actual
current state of the codebase — the mapping above is a starting point from reading
`ActualState.h`, `DesiredState.h`, `Config.h`, and the issue spec. Fields marked
"new field" are genuinely new (no existing home) and should be noted as such in
the generated table.

## Acceptance Criteria

- [x] The `FIELD_TRACE` dict in `scripts/gen_messages.py` covers every field of
      every message in the 7 proto files (or marks fields as "new" if no existing
      home exists).
- [x] `python scripts/gen_messages.py --emit-inventory` runs without error and writes
      `docs/design/message-inventory.md`.
- [x] `docs/design/message-inventory.md` is checked into the repo.
- [x] The generated table includes all spot-check rows:
      - `DrivetrainState::fused` → `ActualState::fused`
      - `PlannerCommand::goto_goal` → `DesiredState DriveMode::GO_TO / targetXWorld,targetYWorld`
      - `MotorCommand::velocity_mmps` → `portable-motor-interface VELOCITY verb / IVelocityMotor::setSpeed()`
      - `DrivetrainConfig::vel_gains` → `RobotConfig::{velKp,velKi,...}`
      - `PlannerConfig::a_max` → `RobotConfig::aMax`
- [x] Fields with no existing home are annotated "new field" (not left blank).
- [x] `uv run python -m pytest` remains green (no new failures).

## Implementation Plan

### Approach

1. Read `source/state/ActualState.h`, `source/state/DesiredState.h`,
   `source/types/Config.h`, `source/control/StopCondition.h`, and the proto files
   from ticket 001 to build an accurate mapping.
2. Populate `FIELD_TRACE` in `gen_messages.py` (starting from the dict above).
3. Implement the `--emit-inventory` path in `gen_messages.py`: walk all messages
   in the descriptor, emit a Markdown table row per field, look up `FIELD_TRACE`
   for the "Maps to existing" column.
4. Run `python scripts/gen_messages.py --emit-inventory` and review the output.
5. Check in `docs/design/message-inventory.md`.

### Files to modify

- `scripts/gen_messages.py` — add `FIELD_TRACE` dict and `--emit-inventory` CLI flag.

### Files to create

- `docs/design/message-inventory.md` — generated and checked in.

### Testing plan

No new pytest tests. Spot-check the generated table rows manually. Run
`uv run python -m pytest` to confirm no regressions from the gen_messages.py edits.

### Documentation updates

`docs/design/message-inventory.md` IS the documentation deliverable of this ticket.

## Verification Command

`uv run python -m pytest`
