#!/usr/bin/env python3
"""Generate source/messages/*.h — C++11 POD headers from proto3 message definitions.

Run:  python3 scripts/gen_messages.py [--dry-run] [--emit-inventory]

Reads protos/*.proto via grpcio-tools (host-only; the device never sees protobuf)
and emits one header per proto file to source/messages/.

Generated code targets CODAL/C++11 with -fno-rtti -fno-exceptions.  No STL
containers, no heap, no exceptions, no RTTI.

Flags
-----
--dry-run        Print what would be written without touching the filesystem.
--emit-inventory Write docs/design/message-inventory.md (traceability table).
"""

import argparse
import os
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
PROTO_DIR  = REPO_ROOT / "protos"
OUT_DIR    = REPO_ROOT / "source" / "messages"
INVENTORY_OUT = REPO_ROOT / "docs" / "design" / "message-inventory.md"

# ---------------------------------------------------------------------------
# Extension field numbers defined in options.proto
# ---------------------------------------------------------------------------
_FIELD_OPT_UNITS     = 50000
_FIELD_OPT_MAX_COUNT = 50001

# Message types that get chainable setters (Command and Config).
_SETTER_TYPES = frozenset([
    "DrivetrainCommand", "MotorCommand", "PlannerCommand",
    "DrivetrainConfig",  "MotorConfig",  "PlannerConfig",
    "LineSensorConfig",  "ColorSensorConfig",
    "GripperConfig",     "PortConfig",
    "GripperCommand",
])

# ---------------------------------------------------------------------------
# Hand-authored field-to-existing-symbol mapping (used by --emit-inventory).
# Format: {(MessageName, field_name): "ExistingSymbol::path"}
#
# Fields annotated "(new field)" have no home in the current codebase and will
# be introduced as part of the Phase 2 message-based subsystem migration.
# Fields annotated "(shared type)" are common geometric/utility types whose
# fields are defined by the message type, not a single firmware member.
# ---------------------------------------------------------------------------
_INVENTORY_MAP: dict = {
    # -----------------------------------------------------------------------
    # common.proto — shared geometric and utility types
    # -----------------------------------------------------------------------

    # Pose2D: 2-D pose value type matching hal/capability/Pose2D.h {x,y,h}
    ("Pose2D", "x"):     "Pose2D::x (hal/capability/Pose2D.h)",
    ("Pose2D", "y"):     "Pose2D::y (hal/capability/Pose2D.h)",
    ("Pose2D", "h"):     "Pose2D::h (hal/capability/Pose2D.h)",

    # BodyTwist: 2-DOF differential twist; not used standalone in ActualState
    # (BodyTwist3 is used everywhere); BodyTwist is retained for wire-compat.
    ("BodyTwist", "v"):     "(new field — BodyTwist2 not in ActualState; BodyTwist3 used instead)",
    ("BodyTwist", "omega"): "(new field — BodyTwist2 not in ActualState; BodyTwist3 used instead)",

    # BodyTwist3: 3-DOF holonomic twist matching hal/capability/Pose2D.h BodyTwist3
    ("BodyTwist3", "v_x"):   "BodyTwist3::vx_mmps (hal/capability/Pose2D.h)",
    ("BodyTwist3", "v_y"):   "BodyTwist3::vy_mmps (hal/capability/Pose2D.h)",
    ("BodyTwist3", "omega"): "BodyTwist3::omega_rads (hal/capability/Pose2D.h)",

    # BodyAccel: used by OTOS passthrough telemetry; not a named member of ActualState
    ("BodyAccel", "a_x"): "ActualState::otosAccelX (passthroughtelemetry)",
    ("BodyAccel", "a_y"): "ActualState::otosAccelY (passthroughtelemetry)",

    # ValueSet: sensor freshness/validity stamp matching types/ValueSet.h ValueSet
    ("ValueSet", "lag"):      "ValueSet::lagMs (types/ValueSet.h)",
    ("ValueSet", "last_upd"): "ValueSet::lastUpdMs (types/ValueSet.h)",
    ("ValueSet", "valid"):       "ValueSet::valid (types/ValueSet.h)",

    # PoseEstimate: pose+twist+stamp; matches source/state/PoseEstimate.h
    ("PoseEstimate", "pose"):  "PoseEstimate::pose (state/PoseEstimate.h)",
    ("PoseEstimate", "twist"): "PoseEstimate::twist (state/PoseEstimate.h)",
    ("PoseEstimate", "stamp"): "PoseEstimate::stamp (state/PoseEstimate.h)",

    # WheelTarget: per-wheel nullable speed/position target
    ("WheelTarget", "speed"):    "DesiredState::wheelMms[] (per-wheel speed target)",
    ("WheelTarget", "position"): "IPositionMotor::setAngleDeg (position-motor interface)",

    # Gains: velocity PID gains matching RobotConfig vel* fields
    ("Gains", "kp"):    "RobotConfig::velKp",
    ("Gains", "ki"):    "RobotConfig::velKi",
    ("Gains", "kff"):   "RobotConfig::velKff",
    ("Gains", "i_max"): "RobotConfig::velIMax",
    ("Gains", "kaw"):   "RobotConfig::velKaw",

    # OutCommand: internal command-bus message (new architecture type)
    ("OutCommand", "verb_id"):  "(new field — command-bus verb ID, Phase 2 architecture)",
    ("OutCommand", "args"):     "(new field — command-bus payload floats, Phase 2 architecture)",
    ("OutCommand", "argc"):     "(new field — argument count, Phase 2 architecture)",
    ("OutCommand", "priority"): "(new field — priority flag for safety/ESTOP commands, Phase 2)",

    # CommandBatch: batch of OutCommands returned by tick()
    ("CommandBatch", "cmds"):  "(new field — command batch array, Phase 2 architecture)",
    ("CommandBatch", "count"): "(new field — batch count, Phase 2 architecture)",

    # Capabilities: declared subsystem capabilities
    ("Capabilities", "command_modes"):   "(new field — accepted command mode bitmask, Phase 2)",
    ("Capabilities", "state_fields"):    "(new field — populated state field bitmask, Phase 2)",
    ("Capabilities", "holonomic"):       "(new field — holonomic flag, Phase 2)",
    ("Capabilities", "onboard_position"):"(new field — onboard position flag, Phase 2)",
    ("Capabilities", "wheel_count"):     "(new field — wheel count, Phase 2)",

    # -----------------------------------------------------------------------
    # drivetrain.proto
    # -----------------------------------------------------------------------

    # SetPose: imperative pose re-anchor (SI verb in the new message surface)
    ("SetPose", "x"): "Superstructure::handleSI() / estimate.resetPose x",
    ("SetPose", "y"): "Superstructure::handleSI() / estimate.resetPose y",
    ("SetPose", "h"): "Superstructure::handleSI() / estimate.resetPose h",

    # WheelTargets: wrapper for repeated WheelTarget (per-wheel speed/position)
    ("WheelTargets", "w"): "DesiredState::wheelMms[] (array of per-wheel targets)",

    # DrivetrainCommand: oneof variants map to DesiredState / OutputState
    ("DrivetrainCommand", "twist"):   "DesiredState::bodyTwist (BodyTwist3 profiled setpoint)",
    ("DrivetrainCommand", "wheels"):  "DesiredState::wheelMms[kWheelCount] (per-wheel speed)",
    ("DrivetrainCommand", "neutral"): "OutputState::pwm[] (BRAKE/COAST via HaltController)",
    ("DrivetrainCommand", "pose"):    "Superstructure::handleSI() / estimate.resetPose",
    ("DrivetrainCommand", "seed"):    "(new field — immediate-seed flag, SI S-command semantics)",

    # DrivetrainState: maps to ActualState members
    ("DrivetrainState", "fused"):        "ActualState::fused (PoseEstimate)",
    ("DrivetrainState", "encoder"):      "ActualState::encoder (PoseEstimate)",
    ("DrivetrainState", "optical"):      "ActualState::optical (PoseEstimate)",
    ("DrivetrainState", "enc"):          "ActualState::encMm[kWheelCount]",
    ("DrivetrainState", "vel"):          "ActualState::velMms[kWheelCount]",
    ("DrivetrainState", "enc_stamp"):    "ActualState::enc (ValueSet)",
    ("DrivetrainState", "otos"):         "ActualState::otos (ValueSet)",
    ("DrivetrainState", "wheel_wedged"): "(new field — wheel-stall flag, not yet in ActualState)",
    ("DrivetrainState", "connected"):    "(new field — drivetrain connected flag, not in ActualState)",
    ("DrivetrainState", "otos_status"):         "Drive::_lastOtosStatus (subsystems/drive/Drive.h, 074-004)",
    ("DrivetrainState", "otos_fusion_blocked"):  "Drive::_otosFusionBlocked (subsystems/drive/Drive.h, 074-004)",

    # DrivetrainConfig: maps to RobotConfig members
    ("DrivetrainConfig", "fwd_sign_l"):           "RobotConfig::fwdSignL",
    ("DrivetrainConfig", "fwd_sign_r"):           "RobotConfig::fwdSignR",
    ("DrivetrainConfig", "travel_calib_l"):       "RobotConfig::wheelTravelCalibL",
    ("DrivetrainConfig", "travel_calib_r"):       "RobotConfig::wheelTravelCalibR",
    ("DrivetrainConfig", "trackwidth"):         "RobotConfig::trackwidth",
    ("DrivetrainConfig", "half_track"):         "RobotConfig::halfTrack",
    ("DrivetrainConfig", "half_wheelbase"):     "RobotConfig::halfWheelbase",
    ("DrivetrainConfig", "travel_calib_wheel"):    "RobotConfig::{wheelTravelCalibFR,wheelTravelCalibFL,wheelTravelCalibBR,wheelTravelCalibBL}",
    ("DrivetrainConfig", "fwd_sign_wheel"):        "RobotConfig::{fwdSignFR,fwdSignFL,fwdSignBR,fwdSignBL}",
    ("DrivetrainConfig", "v_wheel_max"):           "RobotConfig::vWheelMax",
    ("DrivetrainConfig", "steer_headroom"):        "RobotConfig::steerHeadroom",
    ("DrivetrainConfig", "vel_gains"):             "RobotConfig::{velKp,velKi,velKff,velIMax,velKaw}",
    ("DrivetrainConfig", "vel_filt_alpha"):        "RobotConfig::velFiltAlpha",
    ("DrivetrainConfig", "sync_gain"):             "RobotConfig::syncGain",
    ("DrivetrainConfig", "min_wheel"):              "RobotConfig::minWheelSpeed",
    ("DrivetrainConfig", "alpha_pos"):             "RobotConfig::alphaPos",
    ("DrivetrainConfig", "alpha_yaw"):             "RobotConfig::alphaYaw",
    ("DrivetrainConfig", "otos_gate"):             "RobotConfig::otosGate",
    ("DrivetrainConfig", "otos_linear_scale"):     "RobotConfig::otosLinearScale",
    ("DrivetrainConfig", "otos_angular_scale"):    "RobotConfig::otosAngularScale",
    ("DrivetrainConfig", "rotation_gain_pos"):     "RobotConfig::rotationGainPos",
    ("DrivetrainConfig", "rotation_gain_neg"):     "RobotConfig::rotationGainNeg",
    ("DrivetrainConfig", "rotation_offset"):        "RobotConfig::rotationOffset",
    ("DrivetrainConfig", "rotation_offset_neg"):    "RobotConfig::rotationOffsetNeg",
    ("DrivetrainConfig", "rotational_slip"):       "RobotConfig::rotationalSlip",
    ("DrivetrainConfig", "odom_off_x"):            "RobotConfig::odomOffX",
    ("DrivetrainConfig", "odom_off_y"):            "RobotConfig::odomOffY",
    ("DrivetrainConfig", "odom_yaw"):               "RobotConfig::odomYaw",
    ("DrivetrainConfig", "odom_upside_down"):      "RobotConfig::odomUpsideDown",
    ("DrivetrainConfig", "ekf_q_xy"):              "RobotConfig::ekfQxy",
    ("DrivetrainConfig", "ekf_q_theta"):           "RobotConfig::ekfQtheta",
    ("DrivetrainConfig", "ekf_r_otos_xy"):         "RobotConfig::ekfROtosXy",
    ("DrivetrainConfig", "ekf_r_otos_theta"):      "RobotConfig::ekfROtosTheta",
    ("DrivetrainConfig", "ekf_q_v"):               "RobotConfig::ekfQv",
    ("DrivetrainConfig", "ekf_q_omega"):           "RobotConfig::ekfQomega",
    ("DrivetrainConfig", "ekf_r_otos_v"):          "RobotConfig::ekfROtosV",
    ("DrivetrainConfig", "ekf_r_enc_v"):           "RobotConfig::ekfREncV",
    ("DrivetrainConfig", "lag_otos"):               "RobotConfig::lagOtos",
    ("DrivetrainConfig", "drivetrain_type"):       "RobotConfig::drivetrain",

    # DrivetrainCapabilities: capability declaration (new Phase 2 type)
    ("DrivetrainCapabilities", "holonomic"):        "(new field — holonomic capability flag, Phase 2)",
    ("DrivetrainCapabilities", "onboard_position"): "(new field — onboard position capability, Phase 2)",
    ("DrivetrainCapabilities", "wheel_count"):      "(new field — wheel count declaration, Phase 2)",

    # -----------------------------------------------------------------------
    # gripper.proto
    # -----------------------------------------------------------------------

    # GripperCommand: servo angle command (via ServoController / IPositionMotor)
    ("GripperCommand", "angle"): "IPositionMotor::setAngleDeg (hal/capability/IPositionMotor.h)",

    # GripperState: servo angle feedback (not yet in OutputState/ActualState)
    ("GripperState", "angle"):     "(new field — servo angle readback, not yet in OutputState)",
    ("GripperState", "connected"): "(new field — gripper connected flag, not yet in ActualState)",

    # GripperConfig: gripper capability config (new fields, not in RobotConfig)
    ("GripperConfig", "has_gripper"):      "(new field — has-gripper capability flag, not in RobotConfig)",
    ("GripperConfig", "gripper_offset"):   "(new field — gripper mounting offset, not in RobotConfig)",
    ("GripperConfig", "min"):              "(new field — servo minimum angle, not in RobotConfig)",
    ("GripperConfig", "max"):              "(new field — servo maximum angle, not in RobotConfig)",

    # -----------------------------------------------------------------------
    # motor.proto
    # -----------------------------------------------------------------------

    # MotorCommand: portable-motor-interface verbs
    ("MotorCommand", "duty_cycle"):  "portable-motor-interface: DUTY_CYCLE verb / IVelocityMotor::setSpeed()",
    ("MotorCommand", "voltage"):     "portable-motor-interface: VOLTAGE verb (new control mode)",
    ("MotorCommand", "velocity"):    "portable-motor-interface: VELOCITY verb / IVelocityMotor::setSpeed()",
    ("MotorCommand", "position"):    "portable-motor-interface: POSITION verb / IPositionMotor::setAngleDeg()",
    ("MotorCommand", "neutral"):     "portable-motor-interface: NEUTRAL verb (BRAKE/COAST) / OutputState::pwm[]",
    ("MotorCommand", "feedforward"): "RobotConfig::velKff (feed-forward coefficient in vel loop)",
    ("MotorCommand", "reset_position"): "(new field — zero the encoder this tick, Motor::resetPosition(), 077-002)",

    # MotorState: per-motor observable state
    ("MotorState", "connected"): "(new field — motor I2C connected flag, via IBusDiagnostics::errorCount())",
    ("MotorState", "position"):  "IVelocityMotor::positionMm() / ActualState::encMm[]",
    ("MotorState", "velocity"):  "IVelocityMotor::velocityMmps() / ActualState::velMms[]",
    ("MotorState", "applied"):   "OutputState::pwm[] (applied PWM % — new dedicated readback field)",
    ("MotorState", "wedged"):    "(new field — motor stall flag, related to IBusDiagnostics wedge detection)",

    # MotorConfig: per-motor calibration parameters (077-002 accuracy pass)
    ("MotorConfig", "travel_calib"): "RobotConfig::{wheelTravelCalibL,wheelTravelCalibR} (per-motor, indexed by channel)",
    ("MotorConfig", "fwd_sign"):   "RobotConfig::{fwdSignL,fwdSignR} (per-motor, indexed by channel)",
    ("MotorConfig", "vel_gains"):      "RobotConfig::{velKp,velKi,velKff,velIMax,velKaw} (per-motor velocity loop, moved from DrivetrainConfig)",
    ("MotorConfig", "vel_filt_alpha"): "RobotConfig::velFiltAlpha (moved from DrivetrainConfig, now per-motor)",
    ("MotorConfig", "min_duty"):       "RobotConfig::minWheelSpeed (stiction floor, moved from DrivetrainConfig.min_wheel, now duty-domain)",
    ("MotorConfig", "slew_rate"):      "hal/real/MotorSlew.h clampStep() kMaxDeltaPwmPerWrite (duty slew limit)",
    ("MotorConfig", "port"):           "(new field — Nezha motor port 1..4, identity moved from class to Config, 077-002)",

    # MotorCapabilities: capability declaration (077-002: one bool per control mode)
    ("MotorCapabilities", "duty_cycle"): "(new field — duty-cycle control mode capability flag)",
    ("MotorCapabilities", "voltage"):    "(new field — voltage control mode capability flag; false on Nezha)",
    ("MotorCapabilities", "velocity"):   "(new field — velocity control mode capability flag; true on Nezha)",
    ("MotorCapabilities", "position"):   "(new field — onboard position-move capability flag, Nezha 0x5D)",
    ("MotorCapabilities", "has_encoder"): "(new field — encoder capability flag, Phase 2)",

    # -----------------------------------------------------------------------
    # planner.proto
    # -----------------------------------------------------------------------

    # StopCondition: maps to control/StopCondition.h StopCondition struct
    ("StopCondition", "kind"):   "StopCondition::Kind (control/StopCondition.h)",
    ("StopCondition", "a"):      "StopCondition::a (primary param: time ms / distance mm / heading rad / sensor threshold)",
    ("StopCondition", "b"):      "StopCondition::b (secondary param: heading eps / position radius / color saturation)",
    ("StopCondition", "ax"):     "StopCondition::ax (POSITION: target X mm; COLOR: HSV distance threshold)",
    ("StopCondition", "ay"):     "StopCondition::ay (COLOR: target value/brightness)",
    ("StopCondition", "sensor"): "StopCondition::sensor (channel index into HardwareState)",
    ("StopCondition", "cmp"):    "StopCondition::Cmp (GE/LE comparison direction)",

    # Goal variant messages: map to DesiredState / MotionCommand handler
    ("VelocityGoal", "v_x"):      "DesiredState::bodyTwistRaw.vx_mmps (VW/VX verb body velocity)",
    ("VelocityGoal", "v_y"):      "DesiredState::bodyTwistRaw.vy_mmps (holonomic lateral)",
    ("VelocityGoal", "omega"):    "DesiredState::bodyTwistRaw.omega_rads",
    ("VelocityGoal", "duration"): "DesiredState::deadlineMs (T-command deadline)",

    ("GotoGoal", "x"):     "DesiredState::targetXWorld",
    ("GotoGoal", "y"):     "DesiredState::targetYWorld",
    ("GotoGoal", "speed"): "DesiredState::targetSpeedMms",

    ("TurnGoal", "heading"): "DesiredState::targetXWorld (heading encoded as target; DriveMode::GO_TO)",
    ("TurnGoal", "speed"):   "DesiredState::targetSpeedMms",

    ("DistanceGoal", "distance"): "DesiredState::distanceTargetMm",
    ("DistanceGoal", "speed"):    "DesiredState::targetSpeedMms",

    ("TimedGoal", "v_x"):      "DesiredState::bodyTwistRaw.vx_mmps (T verb velocity)",
    ("TimedGoal", "omega"):    "DesiredState::bodyTwistRaw.omega_rads",
    ("TimedGoal", "duration"): "DesiredState::deadlineMs",

    ("RotationGoal", "angle"): "DesiredState::distanceTargetMm (rotation arc encoded as distance; DriveMode::DISTANCE)",
    ("RotationGoal", "speed"): "DesiredState::targetSpeedMms",

    ("StreamGoal", "v_x"):   "DesiredState::bodyTwistRaw.vx_mmps (S verb streaming velocity)",
    ("StreamGoal", "v_y"):   "DesiredState::bodyTwistRaw.vy_mmps",
    ("StreamGoal", "omega"): "DesiredState::bodyTwistRaw.omega_rads",

    # PlannerCommand: oneof goal + metadata
    ("PlannerCommand", "velocity"):  "DesiredState DriveMode::VELOCITY / bodyTwistRaw (VW/VX verbs)",
    ("PlannerCommand", "goto_goal"): "DesiredState DriveMode::GO_TO / targetXWorld,targetYWorld",
    ("PlannerCommand", "turn"):      "DesiredState DriveMode::GO_TO (heading goal variant)",
    ("PlannerCommand", "distance"):  "DesiredState DriveMode::DISTANCE / distanceTargetMm",
    ("PlannerCommand", "timed"):     "DesiredState DriveMode::VELOCITY + deadlineMs (T verb)",
    ("PlannerCommand", "rotation"):  "MotionCommand::handleRotation / DriveMode::DISTANCE",
    ("PlannerCommand", "stream"):    "DesiredState DriveMode::STREAMING (S verb, continuous)",
    ("PlannerCommand", "stop"):      "MotionCommand::handleStop / DriveMode::IDLE",
    ("PlannerCommand", "stops"):     "StopCondition stops[] (control/StopCondition.h, up to 4)",
    ("PlannerCommand", "style"):     "(new field — stop deceleration style SMOOTH/ABRUPT, Phase 2)",
    ("PlannerCommand", "origin"):    "(new field — command origin USER/AUTONOMOUS, Phase 2)",
    ("PlannerCommand", "corr_id"):   "DesiredState::corrId[16] (correlation ID for reply tracking)",

    # PlannerState: maps to DesiredState members
    ("PlannerState", "mode"):            "DesiredState::mode (DriveMode enum)",
    ("PlannerState", "target_x"):        "DesiredState::targetXWorld",
    ("PlannerState", "target_y"):        "DesiredState::targetYWorld",
    ("PlannerState", "target_speed"):    "DesiredState::targetSpeedMms",
    ("PlannerState", "distance_target"): "DesiredState::distanceTargetMm",
    ("PlannerState", "deadline"):        "DesiredState::deadlineMs",
    ("PlannerState", "body_twist"):      "DesiredState::bodyTwist (profiled live setpoint)",
    ("PlannerState", "active"):          "(new field — planner active flag, not a DesiredState member)",

    # PlannerConfig: motion-only RobotConfig subset
    ("PlannerConfig", "a_max"):             "RobotConfig::aMax",
    ("PlannerConfig", "a_decel"):           "RobotConfig::aDecel",
    ("PlannerConfig", "v_body_max"):        "RobotConfig::vBodyMax",
    ("PlannerConfig", "yaw_rate_max"):      "RobotConfig::yawRateMax",
    ("PlannerConfig", "yaw_acc_max"):       "RobotConfig::yawAccMax",
    ("PlannerConfig", "j_max"):             "RobotConfig::jMax",
    ("PlannerConfig", "yaw_jerk_max"):      "RobotConfig::yawJerkMax",
    ("PlannerConfig", "arrive_tol"):        "RobotConfig::arriveTolerance",
    ("PlannerConfig", "turn_in_place_gate"):"RobotConfig::turnInPlaceGate",
    ("PlannerConfig", "min_speed"):         "RobotConfig::minSpeed",

    # -----------------------------------------------------------------------
    # ports.proto
    # -----------------------------------------------------------------------

    # DigitalOut: digital output command
    ("DigitalOut", "value"): "DesiredState::digitalOut[4] / OutputState::digitalOut[4]",
    ("DigitalOut", "mask"):  "(new field — channel-enable mask for digital output, Phase 2)",

    # AnalogOut: analog output command
    ("AnalogOut", "value"): "DesiredState::analogOut[4] / OutputState::analogOut[4]",
    ("AnalogOut", "mask"):  "(new field — channel-enable mask for analog output, Phase 2)",

    # PortCommand: oneof digital/analog output command
    ("PortCommand", "digital_out"): "DesiredState::digitalOut[4] / OutputState::digitalOut[4]",
    ("PortCommand", "analog_out"):  "DesiredState::analogOut[4] / OutputState::analogOut[4]",

    # PortState: read-only port input state
    ("PortState", "digital_in"): "ActualState::digitalIn[4]",
    ("PortState", "analog_in"):  "ActualState::analogIn[4]",
    ("PortState", "stamp"):      "ActualState::portsVS (ValueSet)",

    # PortConfig: ports configuration (077-002: direction field removed — no
    # hardware counterpart, see protos/ports.proto comment)
    ("PortConfig", "lag_ports"): "RobotConfig::lagPorts",

    # -----------------------------------------------------------------------
    # sensors.proto
    # -----------------------------------------------------------------------

    # LineSensorState: line sensor read-only state
    ("LineSensorState", "raw"):        "ActualState::line[4] (raw ADC values)",
    ("LineSensorState", "normalized"): "(new field — normalized line values, not yet split in ActualState)",
    ("LineSensorState", "stamp"):      "ActualState::lineVS (ValueSet)",
    ("LineSensorState", "connected"):  "(new field — line sensor connected flag, capability query)",

    # LineSensorConfig: line sensor configuration (077-002 accuracy pass:
    # threshold/norm_min/norm_max/channel_map removed — no hardware
    # counterpart; cal_min/cal_max/filt_alpha added — real per-channel
    # calibration + EMA smoothing, see protos/sensors.proto comment)
    ("LineSensorConfig", "lag_line"):   "RobotConfig::lagLine",
    ("LineSensorConfig", "cal_min"):    "hal/real/LineSensor.h LineSensor::_calMin[4] (captureCalibMin())",
    ("LineSensorConfig", "cal_max"):    "hal/real/LineSensor.h LineSensor::_calMax[4] (captureCalibMax())",
    ("LineSensorConfig", "filt_alpha"): "hal/real/LineSensor.h LineSensor::_alpha (setSmoothingAlpha())",

    # ColorSensorState: color sensor read-only state
    ("ColorSensorState", "r"):         "ActualState::colorR",
    ("ColorSensorState", "g"):         "ActualState::colorG",
    ("ColorSensorState", "b"):         "ActualState::colorB",
    ("ColorSensorState", "c"):         "ActualState::colorC",
    ("ColorSensorState", "stamp"):     "ActualState::colorVS (ValueSet)",
    ("ColorSensorState", "connected"): "(new field — color sensor connected flag, capability query)",

    # ColorSensorConfig: color sensor configuration (077-002 accuracy pass:
    # cal_r/cal_g/cal_b removed — no RGBC scaling exists anywhere in
    # source_old; integration/gain kept — real APDS9960 registers, currently
    # hardcoded in ColorSensor::initApds(), fallback-chip-only)
    ("ColorSensorConfig", "lag_color"):   "RobotConfig::lagColor",
    ("ColorSensorConfig", "integration"): "hal/real/ColorSensor.cpp initApds() ATIME write 0x81 (APDS9960 fallback only)",
    ("ColorSensorConfig", "gain"):        "hal/real/ColorSensor.cpp initApds() CONTROL write 0x8F (APDS9960 fallback only)",
}

# ---------------------------------------------------------------------------
# protobuf type constants
# ---------------------------------------------------------------------------
try:
    from google.protobuf import descriptor_pb2 as _dpb2
    _TYPE_FLOAT   = _dpb2.FieldDescriptorProto.TYPE_FLOAT
    _TYPE_DOUBLE  = _dpb2.FieldDescriptorProto.TYPE_DOUBLE
    _TYPE_INT32   = _dpb2.FieldDescriptorProto.TYPE_INT32
    _TYPE_INT64   = _dpb2.FieldDescriptorProto.TYPE_INT64
    _TYPE_UINT32  = _dpb2.FieldDescriptorProto.TYPE_UINT32
    _TYPE_UINT64  = _dpb2.FieldDescriptorProto.TYPE_UINT64
    _TYPE_BOOL    = _dpb2.FieldDescriptorProto.TYPE_BOOL
    _TYPE_STRING  = _dpb2.FieldDescriptorProto.TYPE_STRING
    _TYPE_BYTES   = _dpb2.FieldDescriptorProto.TYPE_BYTES
    _TYPE_MESSAGE = _dpb2.FieldDescriptorProto.TYPE_MESSAGE
    _TYPE_ENUM    = _dpb2.FieldDescriptorProto.TYPE_ENUM
    _LABEL_REPEATED = _dpb2.FieldDescriptorProto.LABEL_REPEATED
except ImportError as exc:
    print(f"gen_messages: google.protobuf not found — install grpcio-tools: {exc}",
          file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Helper: read a varint from a bytes buffer
# ---------------------------------------------------------------------------
def _read_varint(buf: bytes, pos: int):
    val = 0
    shift = 0
    while pos < len(buf):
        b = buf[pos]; pos += 1
        val |= (b & 0x7f) << shift
        shift += 7
        if not (b & 0x80):
            return val, pos
    return val, pos


def _read_max_count(field) -> int | None:
    """Return the (max_count) option value from a FieldDescriptorProto, or None."""
    raw = field.options.SerializeToString()
    pos = 0
    while pos < len(raw):
        tag, pos = _read_varint(raw, pos)
        field_num = tag >> 3
        wire_type = tag & 7
        if wire_type == 0:   # varint
            val, pos = _read_varint(raw, pos)
            if field_num == _FIELD_OPT_MAX_COUNT:
                return val
        elif wire_type == 2:  # length-delimited
            vlen, pos = _read_varint(raw, pos)
            pos += vlen
        elif wire_type == 5:  # 32-bit fixed
            pos += 4
        elif wire_type == 1:  # 64-bit fixed
            pos += 8
        else:
            break
    return None


# ---------------------------------------------------------------------------
# Proto scalar → C++ type mapping
# ---------------------------------------------------------------------------
def _scalar_cpp_type(field) -> str:
    """Map a proto scalar field type to a C++ type name."""
    t = field.type
    if t == _TYPE_FLOAT:   return "float"
    if t == _TYPE_DOUBLE:  return "double"
    if t == _TYPE_INT32:   return "int32_t"
    if t == _TYPE_INT64:   return "int64_t"
    if t == _TYPE_UINT32:  return "uint32_t"
    if t == _TYPE_UINT64:  return "uint64_t"
    if t == _TYPE_BOOL:    return "bool"
    if t == _TYPE_STRING:  return "char"   # char arrays, size N=64 by default
    if t == _TYPE_BYTES:   return "uint8_t"
    return "uint8_t"  # fallback


def _scalar_default(field) -> str:
    """Return a sensible zero-initialiser literal for a scalar field."""
    t = field.type
    if t == _TYPE_FLOAT:   return "0.0f"
    if t == _TYPE_DOUBLE:  return "0.0"
    if t in (_TYPE_INT32, _TYPE_INT64, _TYPE_UINT32, _TYPE_UINT64): return "0"
    if t == _TYPE_BOOL:    return "false"
    return "0"


def _short_type_name(type_name: str) -> str:
    """Strip .robot. prefix from a message/enum type_name."""
    if type_name.startswith(".robot."):
        return type_name[len(".robot."):]
    if type_name.startswith("."):
        return type_name[1:].replace(".", "::")
    return type_name


def _cpp_field_type(field) -> str:
    """Return the C++ type for a field (scalar, enum, or message)."""
    t = field.type
    if t in (_TYPE_MESSAGE, _TYPE_ENUM):
        return _short_type_name(field.type_name)
    return _scalar_cpp_type(field)


# ---------------------------------------------------------------------------
# Identify oneof classification
# ---------------------------------------------------------------------------
def _classify_oneofs(md):
    """
    Return (real_oneofs, proto3_optional_oneof_indices).

    Proto3 `optional T` fields get a synthetic oneof whose name starts with
    '_'.  These are *not* code-gen'd as union-based oneofs — they map to
    Opt<T> instead.

    Returns:
        real_oneofs: list of (oneof_index, oneof_name, [field_descriptor])
            for real (non-synthetic) oneofs.
        opt_field_indices: set of field oneof_index values that are
            synthetic proto3-optional wrappers.
    """
    oneof_is_synthetic = {}
    for idx, od in enumerate(md.oneof_decl):
        oneof_is_synthetic[idx] = od.name.startswith("_")

    # Group fields by oneof index (real oneofs only)
    real_oneof_fields: dict[int, list] = {}
    for field in md.field:
        if field.HasField("oneof_index"):
            oi = field.oneof_index
            if not oneof_is_synthetic[oi]:
                real_oneof_fields.setdefault(oi, []).append(field)

    real_oneofs = []
    for idx, od in enumerate(md.oneof_decl):
        if not oneof_is_synthetic[idx] and idx in real_oneof_fields:
            real_oneofs.append((idx, od.name, real_oneof_fields[idx]))

    opt_indices = {idx for idx, synth in oneof_is_synthetic.items() if synth}
    return real_oneofs, opt_indices


# ---------------------------------------------------------------------------
# Enum code generation
# ---------------------------------------------------------------------------
def _emit_enum(ed, lines: list[str]) -> None:
    """Emit a C++11 enum class for a proto enum descriptor."""
    lines.append(f"enum class {ed.name} : uint8_t {{")
    for val in ed.value:
        lines.append(f"    {val.name} = {val.number},")
    lines.append("};")
    lines.append("")


# ---------------------------------------------------------------------------
# Message code generation
# ---------------------------------------------------------------------------
def _emit_message(md, want_setters: bool, lines: list[str],
                  all_enums: set[str]) -> None:
    """Emit a C++11 POD struct for a proto message descriptor.

    Rules (from ticket 002 specification):
      - scalar/message fields         → plain members
      - proto3_optional fields        → Opt<T> members
      - repeated with (max_count)=N  → T name[N]; uint8_t name_count = 0;
      - real oneof                    → KindName enum + union
      - setters                       → only if want_setters (Command/Config)
    """
    real_oneofs, opt_indices = _classify_oneofs(md)
    real_oneof_field_indices: set[int] = set()
    for oi, _name, fields in real_oneofs:
        for f in fields:
            real_oneof_field_indices.add(f.number)

    struct_name = md.name

    lines.append(f"// {struct_name}")
    lines.append(f"struct {struct_name} {{")

    # --- Emit real oneof kinds first (before the fields that reference them) ---
    for oi, oneof_name, oneof_fields in real_oneofs:
        kind_name = f"{_cap_camel(oneof_name)}Kind"
        lines.append(f"    enum class {kind_name} : uint8_t {{")
        lines.append(f"        NONE = 0,")
        for idx_f, f in enumerate(oneof_fields, 1):
            lines.append(f"        {f.name.upper()} = {idx_f},")
        lines.append("    };")
        lines.append(f"    {kind_name} {oneof_name}_kind = {kind_name}::NONE;")
        lines.append(f"    union {{")
        for f in oneof_fields:
            ft = _cpp_field_type(f)
            if f.type == _TYPE_BOOL:
                # bools in unions need padding to avoid UB on some compilers;
                # use uint8_t to be safe in C++11
                lines.append(f"        uint8_t {f.name}_v;  // bool")
            elif f.type == _TYPE_STRING:
                lines.append(f"        char {f.name}[64];")
            else:
                lines.append(f"        {ft} {f.name};")
        lines.append(f"    }} {oneof_name} = {{}};")
        lines.append("")

    # --- Emit regular fields ---
    for field in md.field:
        fname = field.name
        is_repeated = field.label == _LABEL_REPEATED
        is_opt      = field.HasField("oneof_index") and field.oneof_index in opt_indices
        in_real_oneof = field.HasField("oneof_index") and (
            field.oneof_index not in opt_indices
        )

        # Real oneof fields were emitted above as part of the union
        if in_real_oneof:
            continue

        if is_repeated:
            max_n = _read_max_count(field)
            if max_n is None:
                print(f"  WARNING: repeated field {struct_name}.{fname} has no "
                      f"(max_count) option; defaulting to 8", file=sys.stderr)
                max_n = 8
            ft = _cpp_field_type(field)
            if ft == "bool":
                # bool arrays in C++11 embedded: use uint8_t for ABI clarity
                ft_arr = "uint8_t"
                comment = "  // bool[]"
            else:
                ft_arr = ft
                comment = ""
            # Use trailing _ on data member to avoid collision with getter method.
            if field.type == _TYPE_STRING:
                # repeated string → not expected in our protos; treat as char[][64]
                lines.append(f"    char {fname}_[{max_n}][64] = {{}};")
            else:
                lines.append(f"    {ft_arr} {fname}_[{max_n}] = {{}};{comment}")
            lines.append(f"    uint8_t {fname}_count = 0;")

        elif is_opt:
            ft = _cpp_field_type(field)
            if field.type == _TYPE_STRING:
                # optional string → Opt<char[64]> is awkward; use Opt<char*> stub
                # but we have no heap; use a fixed array with a has flag
                lines.append(f"    bool {fname}_has = false;")
                lines.append(f"    char {fname}[64] = {{}};")
            else:
                lines.append(f"    Opt<{ft}> {fname} = {{}};")

        elif field.type == _TYPE_MESSAGE:
            ft = _short_type_name(field.type_name)
            lines.append(f"    {ft} {fname} = {{}};")

        elif field.type == _TYPE_ENUM:
            ft = _short_type_name(field.type_name)
            # emit with default value of 0 cast to the enum type
            lines.append(f"    {ft} {fname} = static_cast<{ft}>(0);")

        elif field.type == _TYPE_STRING:
            lines.append(f"    char {fname}[64] = {{}};")

        else:
            # plain scalar
            default = _scalar_default(field)
            ft = _scalar_cpp_type(field)
            lines.append(f"    {ft} {fname} = {default};")

    # --- Getters ---
    lines.append("")
    lines.append("    // --- getters ---")
    for oi, oneof_name, oneof_fields in real_oneofs:
        kind_name = f"{_cap_camel(oneof_name)}Kind"
        lines.append(f"    {kind_name} get_{oneof_name}_kind() const"
                     f" {{ return {oneof_name}_kind; }}")

    for field in md.field:
        fname = field.name
        is_repeated = field.label == _LABEL_REPEATED
        is_opt      = field.HasField("oneof_index") and field.oneof_index in opt_indices
        in_real_oneof = field.HasField("oneof_index") and (
            field.oneof_index not in opt_indices
        )
        if in_real_oneof:
            continue  # access via union directly

        if is_repeated:
            max_n = _read_max_count(field) or 8
            ft = _cpp_field_type(field)
            if ft == "bool":
                ft_arr = "uint8_t"
            else:
                ft_arr = ft
            if field.type == _TYPE_STRING:
                pass  # skip getter for char[][] — too messy
            else:
                # Data member is {fname}_ to avoid collision with getter method.
                lines.append(f"    const {ft_arr}* {fname}() const {{ return {fname}_; }}")
                lines.append(f"    uint8_t {fname}_count_val() const"
                             f" {{ return {fname}_count; }}")
        elif is_opt:
            if field.type == _TYPE_STRING:
                lines.append(f"    bool has_{fname}() const {{ return {fname}_has; }}")
                lines.append(f"    const char* get_{fname}() const {{ return {fname}; }}")
            else:
                ft = _cpp_field_type(field)
                lines.append(f"    const Opt<{ft}>& get_{fname}() const"
                             f" {{ return {fname}; }}")
        elif field.type == _TYPE_MESSAGE:
            ft = _short_type_name(field.type_name)
            lines.append(f"    const {ft}& get_{fname}() const {{ return {fname}; }}")
        elif field.type == _TYPE_ENUM:
            ft = _short_type_name(field.type_name)
            lines.append(f"    {ft} get_{fname}() const {{ return {fname}; }}")
        elif field.type == _TYPE_STRING:
            lines.append(f"    const char* get_{fname}() const {{ return {fname}; }}")
        else:
            ft = _scalar_cpp_type(field)
            lines.append(f"    {ft} get_{fname}() const {{ return {fname}; }}")

    # --- Setters (Command/Config types only) ---
    if want_setters:
        lines.append("")
        lines.append("    // --- chainable setters (Command/Config only) ---")
        for oi, oneof_name, oneof_fields in real_oneofs:
            kind_name = f"{_cap_camel(oneof_name)}Kind"
            for f in oneof_fields:
                ft = _cpp_field_type(f)
                cap = _cap_camel(f.name)
                if f.type == _TYPE_BOOL:
                    lines.append(f"    {struct_name}& set{cap}(bool v) {{")
                    lines.append(f"        {oneof_name}_kind = {kind_name}::{f.name.upper()};")
                    lines.append(f"        {oneof_name}.{f.name}_v = v ? 1 : 0;")
                    lines.append(f"        return *this;")
                    lines.append("    }")
                elif f.type == _TYPE_STRING:
                    lines.append(f"    // set{cap}: string oneof arm — use"
                                 f" {oneof_name}.{f.name} directly")
                else:
                    if ft in ("float", "double") or "int" in ft or ft == "uint8_t":
                        lines.append(f"    {struct_name}& set{cap}({ft} v) {{")
                        lines.append(f"        {oneof_name}_kind = {kind_name}::{f.name.upper()};")
                        lines.append(f"        {oneof_name}.{f.name} = v;")
                        lines.append(f"        return *this;")
                        lines.append("    }")
                    else:
                        # message type
                        lines.append(f"    {struct_name}& set{cap}(const {ft}& v) {{")
                        lines.append(f"        {oneof_name}_kind = {kind_name}::{f.name.upper()};")
                        lines.append(f"        {oneof_name}.{f.name} = v;")
                        lines.append(f"        return *this;")
                        lines.append("    }")

        for field in md.field:
            fname = field.name
            is_repeated = field.label == _LABEL_REPEATED
            is_opt      = field.HasField("oneof_index") and field.oneof_index in opt_indices
            in_real_oneof = field.HasField("oneof_index") and (
                field.oneof_index not in opt_indices
            )
            if in_real_oneof:
                continue

            cap = _cap_camel(fname)

            if is_repeated:
                # Setters for repeated are tricky — just expose a clear helper
                max_n = _read_max_count(field) or 8
                ft = _cpp_field_type(field)
                if ft == "bool":
                    ft_s = "uint8_t"
                else:
                    ft_s = ft
                if field.type == _TYPE_STRING:
                    pass
                else:
                    lines.append(f"    {struct_name}& clear{cap}()"
                                 f" {{ {fname}_count = 0; return *this; }}")
            elif is_opt:
                if field.type == _TYPE_STRING:
                    lines.append(f"    // set{cap}: optional string — set"
                                 f" {fname}_has and {fname} directly")
                else:
                    ft = _cpp_field_type(field)
                    lines.append(f"    {struct_name}& set{cap}({ft} v) {{")
                    lines.append(f"        {fname}.has = true; {fname}.val = v;")
                    lines.append(f"        return *this;")
                    lines.append("    }")
            elif field.type == _TYPE_MESSAGE:
                ft = _short_type_name(field.type_name)
                lines.append(f"    {struct_name}& set{cap}(const {ft}& v)"
                             f" {{ {fname} = v; return *this; }}")
            elif field.type == _TYPE_ENUM:
                ft = _short_type_name(field.type_name)
                lines.append(f"    {struct_name}& set{cap}({ft} v)"
                             f" {{ {fname} = v; return *this; }}")
            elif field.type == _TYPE_STRING:
                lines.append(f"    // set{cap}: set {fname}[] directly (char array)")
            elif field.type == _TYPE_BOOL:
                lines.append(f"    {struct_name}& set{cap}(bool v)"
                             f" {{ {fname} = v; return *this; }}")
            else:
                ft = _scalar_cpp_type(field)
                lines.append(f"    {struct_name}& set{cap}({ft} v)"
                             f" {{ {fname} = v; return *this; }}")

    lines.append("};")
    lines.append("")


def _cap_camel(name: str) -> str:
    """Convert snake_case to CapCamelCase for setter names."""
    return "".join(w.capitalize() for w in name.split("_"))


# ---------------------------------------------------------------------------
# File-level code generation
# ---------------------------------------------------------------------------
_BANNER = """\
// AUTO-GENERATED — do not edit by hand.
// Regenerated by scripts/gen_messages.py before each firmware build.
// Source: protos/{proto_name}
#pragma once
"""

# For common.proto: emitted BEFORE the namespace msg block (system include only).
_COMMON_SYSTEM_INCLUDE = "#include <stdint.h>\n"

# For common.proto: emitted INSIDE namespace msg (Opt<T> is a msg:: type).
_COMMON_NAMESPACE_PREAMBLE = """\
// Opt<T> — nullable wrapper for proto3 optional fields.
// Replaces std::optional<T> (which requires RTTI / exceptions).
// Target: CODAL C++11, -fno-rtti -fno-exceptions, no heap.
template<class T>
struct Opt { bool has = false; T val{}; };
"""

_OTHER_INCLUDE = '#include "messages/common.h"\n'


def _emit_file(fd, file_messages: dict, file_enums: dict,
               all_enums: set[str]) -> str:
    """Emit the full content of one generated header.

    All generated types (Opt<T>, enums, structs) are wrapped in
    ``namespace msg { ... }`` so ``msg::Pose2D`` and ``::Pose2D`` (HAL) are
    distinct names and can coexist in a single translation unit.
    """
    lines: list[str] = []
    proto_name = fd.name

    # Banner + pragma once
    lines.append("// AUTO-GENERATED — do not edit by hand.")
    lines.append("// Regenerated by scripts/gen_messages.py before each firmware build.")
    lines.append(f"// Source: protos/{proto_name}")
    lines.append("#pragma once")
    lines.append("")

    is_common = (proto_name == "common.proto")

    if is_common:
        # <stdint.h> goes outside the namespace (it defines global typedefs).
        lines.append(_COMMON_SYSTEM_INCLUDE)
    else:
        lines.append(_OTHER_INCLUDE)
        lines.append("")

    # Open namespace msg — ALL generated types live in msg::
    lines.append("namespace msg {")
    lines.append("")

    if is_common:
        # Opt<T> is a generated utility template; it goes inside msg::
        lines.append(_COMMON_NAMESPACE_PREAMBLE)

    # Emit top-level enums in this file
    for ed in fd.enum_type:
        _emit_enum(ed, lines)

    # Emit messages in this file
    for md in fd.message_type:
        want_setters = md.name in _SETTER_TYPES
        _emit_message(md, want_setters, lines, all_enums)

    # Close namespace msg
    lines.append("}  // namespace msg")
    lines.append("")

    return "\n".join(lines) + "\n"


def _emit_bridges_header() -> str:
    """Emit source/messages/bridges.h — static_assert compatibility checks."""
    return """\
// AUTO-GENERATED — do not edit by hand.
// Regenerated by scripts/gen_messages.py before each firmware build.
//
// bridges.h — compile-time compatibility checks between the generated
// message headers (source/messages/*.h) and the existing HAL types.
//
// Phase 2 namespace strategy (ticket 057-001):
//   All generated types are now in namespace msg::, so msg::Pose2D and
//   ::Pose2D (HAL) are distinct names.  Both can be included in the SAME
//   translation unit without a redefinition error.  This file therefore
//   includes BOTH headers and asserts cross-namespace layout compatibility
//   directly: sizeof(msg::Pose2D) == sizeof(::Pose2D) etc.
//
// Usage:
//   Include bridges.h from any firmware TU that needs the HAL types or
//   the generated message types — both are now safe to include together.
#pragma once
#include "hal/capability/Pose2D.h"
#include "messages/common.h"

// --- Cross-namespace layout-compatibility checks (Phase 2) ---
// Now that generated types live in msg:: the compiler can see BOTH
// msg::Pose2D and ::Pose2D in one TU.  These static_asserts prove that
// the two structs are bit-for-bit compatible (same size, same alignment)
// so they can be safely reinterpret_cast<> across the subsystem boundary.

// msg::Pose2D { float x, y, h } vs ::Pose2D { float x, y, h }
// Same layout: 3 floats, trivially copyable.
static_assert(sizeof(msg::Pose2D) == sizeof(::Pose2D),
              "msg::Pose2D and ::Pose2D must have the same size — layout compat broken");
static_assert(sizeof(msg::Pose2D) == sizeof(float) * 3,
              "msg::Pose2D must be 3 floats {x,y,h}");
static_assert(sizeof(::Pose2D) == sizeof(float) * 3,
              "HAL Pose2D must be 3 floats {x,y,h}");

// msg::BodyTwist3 { float v_x, v_y, omega }
// vs ::BodyTwist3 { float vx_mmps, vy_mmps, omega_rads } — identical layout (3 floats, same size).
static_assert(sizeof(msg::BodyTwist3) == sizeof(::BodyTwist3),
              "msg::BodyTwist3 and ::BodyTwist3 must have the same size — layout compat broken");
static_assert(sizeof(msg::BodyTwist3) == sizeof(float) * 3,
              "msg::BodyTwist3 must be 3 floats");
static_assert(sizeof(::BodyTwist3) == sizeof(float) * 3,
              "HAL BodyTwist3 must be 3 floats");

// HAL RobotGeometry: { float halfTrack, halfWheelbase } — must remain 2 floats.
// Corresponds to DrivetrainConfig::half_track / half_wheelbase in the
// generated drivetrain.h (no direct generated RobotGeometry message yet).
static_assert(sizeof(::RobotGeometry) == sizeof(float) * 2,
              "HAL RobotGeometry must be 2 floats — check hal/capability/Pose2D.h");
"""


# ---------------------------------------------------------------------------
# Inventory emission
# ---------------------------------------------------------------------------
def _emit_inventory(file_descriptors) -> str:
    """Generate docs/design/message-inventory.md.

    Walks every message in every non-options proto file, looks up the
    field in _INVENTORY_MAP, and emits a Markdown traceability table.
    Fields with no mapping are flagged as MISSING so coverage gaps are
    immediately visible.
    """
    rows = []
    for fd in file_descriptors:
        if fd.name.startswith("google"):
            continue
        if fd.name == "options.proto":
            continue
        for md in fd.message_type:
            for field in md.field:
                cpp_type = _cpp_field_type(field)
                if field.type in (_TYPE_MESSAGE, _TYPE_ENUM):
                    cpp_type = _short_type_name(field.type_name)
                if field.proto3_optional:
                    cpp_type = f"Opt<{cpp_type}>"
                elif field.label == _LABEL_REPEATED:
                    max_n = _read_max_count(field) or "?"
                    cpp_type = f"{cpp_type}[{max_n}]"
                existing = _INVENTORY_MAP.get((md.name, field.name), "**MISSING**")
                rows.append((fd.name, md.name, field.name, cpp_type, existing))

    # --- Coverage statistics ---
    total = len(rows)
    mapped = sum(1 for *_, e in rows if e and e != "**MISSING**")
    new_fields = sum(1 for *_, e in rows if e and "(new field" in e)
    missing = total - mapped

    lines = [
        "<!-- AUTO-GENERATED by scripts/gen_messages.py --emit-inventory -->",
        "<!-- Do not edit by hand. -->",
        "# Message Inventory — Phase 1 Traceability",
        "",
        "This table maps every generated message field to its existing source-of-truth",
        "home in the current firmware codebase (`ActualState`, `DesiredState`,",
        "`OutputState`, `RobotConfig`, the portable-motor-interface spec, or",
        "`StopCondition`). Fields annotated **(new field)** are genuinely new — no",
        "existing firmware member corresponds to them; they will be introduced in Phase 2.",
        "Fields annotated **(shared type)** are common value types defined by the message",
        "schema itself rather than a single firmware member.",
        "",
        f"**Coverage: {mapped}/{total} fields mapped "
        f"({new_fields} new, {missing} missing)**",
        "",
        "| Proto file | Message | Field | C++ type | Maps to existing |",
        "|---|---|---|---|---|",
    ]
    for proto_file, msg, field, cpp_type, existing in rows:
        lines.append(f"| {proto_file} | {msg} | {field} | {cpp_type} | {existing} |")
    lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate C++11 POD headers from proto3 message definitions."
    )
    parser.add_argument("--dry-run", action="store_true",
                        help="Print what would be written; do not touch files.")
    parser.add_argument("--emit-inventory", action="store_true",
                        help="Also write docs/design/message-inventory.md.")
    args = parser.parse_args(argv)

    # ------------------------------------------------------------------
    # Locate grpcio-tools _proto directory for well-known imports
    # ------------------------------------------------------------------
    try:
        import grpc_tools
    except ImportError:
        print("gen_messages: grpcio-tools is not installed.\n"
              "  Run: uv sync  (grpcio-tools is in the 'codegen' dependency group)",
              file=sys.stderr)
        return 1

    well_known_dir = str(Path(grpc_tools.__file__).parent / "_proto")

    # ------------------------------------------------------------------
    # Collect all proto files (deterministic order)
    # ------------------------------------------------------------------
    proto_names = sorted(p.name for p in PROTO_DIR.glob("*.proto"))
    proto_paths = [str(PROTO_DIR / n) for n in proto_names]

    # ------------------------------------------------------------------
    # Run protoc to get a FileDescriptorSet
    # ------------------------------------------------------------------
    from grpc_tools import protoc
    from google.protobuf import descriptor_pb2

    with tempfile.NamedTemporaryFile(suffix=".pb", delete=False) as tmp_f:
        tmp_path = tmp_f.name

    try:
        ret = protoc.main([
            "protoc",
            "-I", str(PROTO_DIR),
            "-I", well_known_dir,
            f"--descriptor_set_out={tmp_path}",
            "--include_imports",
        ] + proto_paths)

        if ret != 0:
            print("gen_messages: protoc failed — check proto syntax.", file=sys.stderr)
            return 1

        fds_bytes = Path(tmp_path).read_bytes()
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass

    fds = descriptor_pb2.FileDescriptorSet()
    fds.ParseFromString(fds_bytes)

    # ------------------------------------------------------------------
    # Build index: file_name → FileDescriptorProto
    # Collect all enum names for type resolution
    # ------------------------------------------------------------------
    all_enums: set[str] = set()
    file_map: dict[str, object] = {}
    for fd in fds.file:
        file_map[fd.name] = fd
        for ed in fd.enum_type:
            all_enums.add(ed.name)
        for md in fd.message_type:
            for ed in md.enum_type:
                all_enums.add(ed.name)

    # ------------------------------------------------------------------
    # Emit one header per proto file (skip options.proto — no messages)
    # ------------------------------------------------------------------
    emit_order = [n for n in proto_names if n != "options.proto"]

    outputs: dict[str, str] = {}
    for proto_name in emit_order:
        fd = file_map.get(proto_name)
        if fd is None:
            print(f"gen_messages: WARNING — {proto_name} not found in descriptor set",
                  file=sys.stderr)
            continue
        header_name = proto_name.replace(".proto", ".h")
        content = _emit_file(fd, {}, {}, all_enums)
        outputs[header_name] = content

    # bridges.h is hand-authored in this generator (not from a proto file)
    outputs["bridges.h"] = _emit_bridges_header()

    # ------------------------------------------------------------------
    # Write or dry-run
    # ------------------------------------------------------------------
    if not args.dry_run:
        OUT_DIR.mkdir(parents=True, exist_ok=True)

    for header_name, content in sorted(outputs.items()):
        out_path = OUT_DIR / header_name
        if args.dry_run:
            print(f"[dry-run] would write {out_path.relative_to(REPO_ROOT)}")
            print("  first 5 lines:")
            for line in content.splitlines()[:5]:
                print(f"    {line}")
        else:
            out_path.write_text(content)
            print(f"gen_messages: wrote {out_path.relative_to(REPO_ROOT)}",
                  file=sys.stderr)

    # ------------------------------------------------------------------
    # Optional inventory
    # ------------------------------------------------------------------
    if args.emit_inventory:
        inv_content = _emit_inventory(list(fds.file))
        if args.dry_run:
            print(f"[dry-run] would write {INVENTORY_OUT.relative_to(REPO_ROOT)}")
        else:
            INVENTORY_OUT.parent.mkdir(parents=True, exist_ok=True)
            INVENTORY_OUT.write_text(inv_content)
            print(f"gen_messages: wrote {INVENTORY_OUT.relative_to(REPO_ROOT)}",
                  file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
