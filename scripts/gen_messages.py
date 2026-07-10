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
from collections import deque
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
_FIELD_OPT_MIN       = 50002
_FIELD_OPT_MAX       = 50003
_FIELD_OPT_ABS_MAX   = 50004
_FIELD_OPT_REQ       = 50005
_FIELD_OPT_STR_LEN   = 50006

# Default fixed-array width for a `string` field with no `(str_len)`
# override (095-005 Step 0b: minimal per-field string-width mechanism,
# scoped to DeviceId's three shrunk strings -- see options.proto's
# `(str_len)` doc comment).
_DEFAULT_STR_LEN = 64

# Message types that get chainable setters (Command and Config).
_SETTER_TYPES = frozenset([
    "DrivetrainCommand", "MotorCommand", "PlannerCommand",
    "DrivetrainConfig",  "MotorConfig",  "PlannerConfig",
    "LineSensorConfig",  "ColorSensorConfig",
    "GripperConfig",     "PortConfig",
    "GripperCommand",
    "CommunicatorConfig",
    "OdometerCommand",   "OdometerConfig",
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

    # Pose2D: 2-D pose value type — canonical definition is msg::Pose2D {x,y,h}
    ("Pose2D", "x"):     "msg::Pose2D::x (messages/common.h)",
    ("Pose2D", "y"):     "msg::Pose2D::y (messages/common.h)",
    ("Pose2D", "h"):     "msg::Pose2D::h (messages/common.h)",

    # BodyTwist: 2-DOF differential twist; not used standalone in ActualState
    # (BodyTwist3 is used everywhere); BodyTwist is retained for wire-compat.
    ("BodyTwist", "v"):     "(new field — BodyTwist2 not in ActualState; BodyTwist3 used instead)",
    ("BodyTwist", "omega"): "(new field — BodyTwist2 not in ActualState; BodyTwist3 used instead)",

    # BodyTwist3: 3-DOF holonomic twist — canonical definition is msg::BodyTwist3
    ("BodyTwist3", "v_x"):   "msg::BodyTwist3::v_x (messages/common.h)",
    ("BodyTwist3", "v_y"):   "msg::BodyTwist3::v_y (messages/common.h)",
    ("BodyTwist3", "omega"): "msg::BodyTwist3::omega (messages/common.h)",

    # BodyAccel: used by OTOS passthrough telemetry; not a named member of ActualState
    ("BodyAccel", "a_x"): "ActualState::otosAccelX (passthroughtelemetry)",
    ("BodyAccel", "a_y"): "ActualState::otosAccelY (passthroughtelemetry)",

    # ValueSet: sensor freshness/validity stamp matching types/value_set.h ValueSet
    ("ValueSet", "lag"):      "ValueSet::lagMs (types/value_set.h)",
    ("ValueSet", "last_upd"): "ValueSet::lastUpdMs (types/value_set.h)",
    ("ValueSet", "valid"):       "ValueSet::valid (types/value_set.h)",

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
    ("DrivetrainCommand", "standby"): "(new field — sprint 079: relinquish drivetrain authority side-channel, Subsystems::Drivetrain::standby(), architecture-update.md \"Authority arbitration\")",

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
    ("DrivetrainState", "active"):        "Subsystems::Drivetrain::active_ (subsystems/drivetrain.h, 087-003)",

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
    ("DrivetrainConfig", "left_port"):             "(new field — sprint 079: bound wheel-motor port, moved from DevLoopState::leftPort, architecture-update.md \"Authority arbitration\")",
    ("DrivetrainConfig", "right_port"):            "(new field — sprint 079: bound wheel-motor port, moved from DevLoopState::rightPort, architecture-update.md \"Authority arbitration\")",

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
    ("MotorState", "wedge_suspect"): "(new field — sprint 078: motion-qualified wedge signal, wedged AND |appliedDuty| > output_deadband, Hal::Motor write-path armor)",
    ("MotorState", "hard_reset_count"): "(new field — sprint 078: cumulative hard-reset count, ported idea from source_old 064-003 Motor::hardResetCount())",
    ("MotorState", "soft_reset_count"): "(new field — sprint 078: cumulative soft-reset count, ported idea from source_old 064-003 Motor::softResetCount())",

    # MotorConfig: per-motor calibration parameters (077-002 accuracy pass)
    ("MotorConfig", "travel_calib"): "RobotConfig::{wheelTravelCalibL,wheelTravelCalibR} (per-motor, indexed by channel)",
    ("MotorConfig", "fwd_sign"):   "RobotConfig::{fwdSignL,fwdSignR} (per-motor, indexed by channel)",
    ("MotorConfig", "vel_gains"):      "RobotConfig::{velKp,velKi,velKff,velIMax,velKaw} (per-motor velocity loop, moved from DrivetrainConfig)",
    ("MotorConfig", "vel_filt_alpha"): "RobotConfig::velFiltAlpha (moved from DrivetrainConfig, now per-motor)",
    ("MotorConfig", "min_duty"):       "RobotConfig::minWheelSpeed (stiction floor, moved from DrivetrainConfig.min_wheel, now duty-domain)",
    ("MotorConfig", "slew_rate"):      "hal/real/MotorSlew.h clampStep() kMaxDeltaPwmPerWrite (duty slew limit)",
    ("MotorConfig", "port"):           "(new field — Nezha motor port 1..4, identity moved from class to Config, 077-002)",
    ("MotorConfig", "reversal_dwell"): "(new field — sprint 078: reversal-dwell hold time, Opt<float> unset -> ship default 100 ms applied in Hal::Motor::configure(), ticket 078-002)",
    ("MotorConfig", "output_deadband"): "(new field — sprint 078: write-path output deadband fraction, Opt<float> unset -> ship default 0.03 applied in Hal::Motor::configure(), ticket 078-002)",
    ("MotorConfig", "polled"): "(new field — sprint 091: I2C flip-flop poll-schedule membership, moved off command-derived NezhaHardware::portInUse_ onto a configured, config-plane-only mask, ticket 091-002)",

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

    # -----------------------------------------------------------------------
    # communicator.proto — no CommunicatorCommand by design (the Communicator
    # is a source of commands; its faceplate has no command-in channel)
    # -----------------------------------------------------------------------

    # CommunicatorConfig: comms configuration
    ("CommunicatorConfig", "radio_channel"): "com/radio.h Radio::_channel (begin()/setChannel(); radiochan::clamp bounds)",

    # CommunicatorState: read-only comms snapshot
    ("CommunicatorState", "radio_channel"): "com/radio.h Radio::channel()",
    ("CommunicatorState", "serial_lines"):  "(new field — received-line counter, subsystems/communicator.cpp tick())",
    ("CommunicatorState", "radio_lines"):   "(new field — received-line counter, subsystems/communicator.cpp tick())",

    # CommunicatorCapabilities: declared comms channels
    ("CommunicatorCapabilities", "serial"): "com/serial_port.h SerialPort (owned by value)",
    ("CommunicatorCapabilities", "radio"):  "com/radio.h Radio (owned by value)",
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


def _parse_field_options(field) -> dict:
    """Parse a FieldDescriptorProto's serialized FieldOptions extension bytes
    into {field_number: raw_value} for every extension option
    options.proto declares (varint/fixed32/fixed64/length-delimited).

    Generalizes `_read_max_count()`'s hand-rolled single-option walk (left
    untouched above -- it is exercised by every existing generated header,
    095-005 does not risk touching it) for the ticket 005 additions that
    need MULTIPLE options off the same field (`(str_len)`, and
    `(min)`/`(max)`/`(abs_max)`/`(req)` for the FieldDesc/kMaxEncodedSize
    work) without re-walking `field.options.SerializeToString()` once per
    option.
    """
    import struct

    raw = field.options.SerializeToString()
    pos = 0
    out: dict = {}
    while pos < len(raw):
        tag, pos = _read_varint(raw, pos)
        field_num = tag >> 3
        wire_type = tag & 7
        if wire_type == 0:  # varint (uint32/bool options: max_count, req, str_len)
            val, pos = _read_varint(raw, pos)
            out[field_num] = val
        elif wire_type == 1:  # fixed64 (double options: min/max/abs_max)
            (val,) = struct.unpack_from("<d", raw, pos)
            pos += 8
            out[field_num] = val
        elif wire_type == 2:  # length-delimited (string option: units)
            vlen, pos = _read_varint(raw, pos)
            out[field_num] = raw[pos:pos + vlen]
            pos += vlen
        elif wire_type == 5:  # fixed32 (unused by any current option, handled for completeness)
            (val,) = struct.unpack_from("<f", raw, pos)
            pos += 4
            out[field_num] = val
        else:
            break
    return out


def _read_str_len(field) -> int:
    """Return the (str_len) option value for a `string` field, or the
    generator's default width (095-005 Step 0b)."""
    opts = _parse_field_options(field)
    val = opts.get(_FIELD_OPT_STR_LEN)
    return int(val) if val is not None else _DEFAULT_STR_LEN


def _read_bound(field, opt_field_number: int):
    """Return a (min)/(max)/(abs_max) option value as a float, or None."""
    opts = _parse_field_options(field)
    val = opts.get(opt_field_number)
    return float(val) if val is not None else None


def _read_req(field) -> bool:
    """Return whether the (req) option is set true on a field."""
    opts = _parse_field_options(field)
    return bool(opts.get(_FIELD_OPT_REQ, 0))


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
                lines.append(f"        char {f.name}[{_read_str_len(f)}];")
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
                lines.append(f"    char {fname}[{_read_str_len(field)}] = {{}};")
            else:
                lines.append(f"    Opt<{ft}> {fname} = {{}};")

        elif field.type == _TYPE_BYTES:
            # 095-005 (ticket 001's own flagged deviation, now fixed): a
            # singular `bytes` field with `(max_count)` used to fall through
            # to the generic scalar branch below and emit a ONE-byte
            # `uint8_t {fname} = 0;` -- not the intended fixed-capacity
            # buffer. `bytes` is a proto3 scalar type, not inherently
            # repeated, so it needs its own branch (mirrors the repeated-field
            # array+count shape, minus the extra `[max_n]` array-of-arrays
            # dimension repeated fields need).
            max_n = _read_max_count(field)
            if max_n is None:
                print(f"  WARNING: bytes field {struct_name}.{fname} has no "
                      f"(max_count) option; defaulting to 64", file=sys.stderr)
                max_n = 64
            lines.append(f"    uint8_t {fname}_[{max_n}] = {{}};")
            lines.append(f"    uint8_t {fname}_count = 0;")

        elif field.type == _TYPE_MESSAGE:
            ft = _short_type_name(field.type_name)
            lines.append(f"    {ft} {fname} = {{}};")

        elif field.type == _TYPE_ENUM:
            ft = _short_type_name(field.type_name)
            # emit with default value of 0 cast to the enum type
            lines.append(f"    {ft} {fname} = static_cast<{ft}>(0);")

        elif field.type == _TYPE_STRING:
            lines.append(f"    char {fname}[{_read_str_len(field)}] = {{}};")

        else:
            # plain scalar
            default = _scalar_default(field)
            ft = _scalar_cpp_type(field)
            lines.append(f"    {ft} {fname} = {default};")

    # --- Array / optional-string accessors ---
    # 080-001: every get_*-prefixed accessor (oneof-kind discriminator,
    # Opt<T>, message, enum, string, plain scalar) has been removed -- each of
    # those fields is already a plain public struct member with no invariant
    # or computation behind it, so callers read the field directly
    # (x.foo, x.foo_kind, x.foo.has / x.foo.val for Opt<T>). The two accessors
    # below are NOT get_-prefixed and stay in scope (architecture-update.md
    # "Unchanged"): the repeated-field array pair exists only because its
    # backing member is suffixed `{field}_` to dodge a name collision with the
    # accessor, and has_{field}() is the only way to read the has-flag of an
    # optional string (the value itself is still a bare field, {field}).
    lines.append("")
    lines.append("    // --- array / optional-string accessors ---")
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
                pass  # skip accessor for char[][] — too messy
            else:
                # Data member is {fname}_ to avoid collision with the accessor.
                lines.append(f"    const {ft_arr}* {fname}() const {{ return {fname}_; }}")
                lines.append(f"    uint8_t {fname}_count_val() const"
                             f" {{ return {fname}_count; }}")
        elif is_opt and field.type == _TYPE_STRING:
            lines.append(f"    bool has_{fname}() const {{ return {fname}_has; }}")
        elif field.type == _TYPE_BYTES:
            # Same accessor shape as the repeated-field array pair above --
            # the backing member is suffixed `{field}_` to dodge the
            # collision with this accessor's own name.
            lines.append(f"    const uint8_t* {fname}() const {{ return {fname}_; }}")
            lines.append(f"    uint8_t {fname}_count_val() const"
                         f" {{ return {fname}_count; }}")
        # Opt<T> (non-string), message, enum, string, and plain-scalar fields
        # are plain public members -- no accessor emitted here.

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
            elif field.type == _TYPE_BYTES:
                lines.append(f"    // set{cap}: set {fname}_[]/{fname}_count directly (bytes buffer)")
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


def _cross_file_include_block(fd) -> str:
    """Return the '#include "messages/X.h"\\n' block for a non-common
    generated header (095-003).

    Every non-common header unconditionally needs `messages/common.h` — it
    defines the `Opt<T>` template every proto3 `optional` field expands to,
    regardless of whether the `.proto` file itself `import`s common.proto.
    Beyond that fixed first include, emit one ADDITIONAL `#include` per
    OTHER proto file this file's own `import` lines name
    (`fd.dependency`), skipping `options.proto` (declares no messages, has
    no generated header) and `common.proto` (already unconditionally
    included). This is the ticket 095-003 fix for the cross-file
    struct-reference gap ticket 001 flagged: `envelope.proto` is the first
    file in this tree to `import` a subsystem-specific proto other than
    `common.proto`/`options.proto` (its own header comment + ticket 001's
    completion notes have the full verification), and the generator
    previously always emitted exactly one `#include "messages/common.h"`
    with no per-file cross-reference tracking, so `envelope.h` referenced
    `DrivetrainCommand`/`MotionSegment`/`PlannerCommand`/`SetPose`/
    `OdometerCommand` with no matching `#include`.

    For every PRE-EXISTING header (every file that, like every proto file
    before `envelope.proto`, imports at most `common.proto`/
    `options.proto`), this returns byte-identical output to the old fixed
    `_OTHER_INCLUDE` constant — verified by `gen_messages.py --dry-run`
    diffing zero change to any of them.
    """
    stems = ["common"]
    for dep in fd.dependency:
        if dep in ("options.proto", "common.proto"):
            continue
        if dep.startswith("google/"):
            continue
        stem = dep[: -len(".proto")] if dep.endswith(".proto") else dep
        if stem not in stems:
            stems.append(stem)
    # common.h stays first (matches the pre-existing single-include shape
    # exactly when there are no extra cross-file deps); any additional
    # cross-file includes are sorted for a deterministic, reviewable order.
    extra = sorted(s for s in stems if s != "common")
    ordered = ["common"] + extra
    return "".join(f'#include "messages/{s}.h"\n' for s in ordered)


# ---------------------------------------------------------------------------
# 095-003: day-one decision gate — std::is_standard_layout static_asserts.
#
# Decide, cheaply and BEFORE ticket 005's expensive field-table/wire.{h,cpp}
# codegen is written, whether the generator's planned offsetof-based
# FieldDesc tables are viable. This emits ONLY the check (an aggregator
# header, source/messages/layout_checks.h, chosen over inline-per-header
# asserts -- see _emit_layout_checks_header()'s own doc comment for why) --
# no field table, no offsetof call.
# ---------------------------------------------------------------------------

_LAYOUT_CHECK_ROOTS = ("CommandEnvelope", "ReplyEnvelope")


def _compute_layout_check_structs(fds) -> list[str]:
    """Return every msg:: struct name transitively reachable (via
    message-typed fields, including oneof-union arms) from
    CommandEnvelope/ReplyEnvelope, in stable BFS (first-seen) order.

    This is the exact set a future offsetof-based FieldDesc table (ticket
    005) would need to walk for THIS sprint's implemented + declared-only
    envelope arms -- computed generically from the full FileDescriptorSet
    (every message in every proto file, not just envelope.proto/
    motion.proto) rather than hand-enumerated, so it stays correct as the
    schema grows without this function needing an edit.
    """
    msg_map: dict[str, object] = {}
    for fd in fds.file:
        for md in fd.message_type:
            msg_map[md.name] = md

    order: list[str] = []
    discovered: set[str] = set(_LAYOUT_CHECK_ROOTS)
    queue: deque = deque(_LAYOUT_CHECK_ROOTS)
    while queue:
        name = queue.popleft()
        md = msg_map.get(name)
        if md is None:
            continue  # referenced type has no message descriptor (shouldn't happen)
        order.append(name)
        for field in md.field:
            if field.type != _TYPE_MESSAGE:
                continue  # enums/scalars/Opt<scalar> need no offsetof-layout check
            child = _short_type_name(field.type_name)
            if child not in discovered:
                discovered.add(child)
                queue.append(child)
    return order


_LAYOUT_CHECKS_BANNER = """\
// AUTO-GENERATED — do not edit by hand.
// Regenerated by scripts/gen_messages.py before each firmware build.
// Day-one decision gate (095-003, architecture-update.md SUC-002/Risk 2/
// Open Question 5): one std::is_standard_layout static_assert per msg::
// struct transitively reachable from CommandEnvelope/ReplyEnvelope -- the
// exact set a future offsetof-based FieldDesc table (ticket 005) would
// need to walk. This header emits ONLY the check, no field table, no
// offsetof call -- deciding whether the offsetof-table approach is viable
// BEFORE that generator work is written, per the issue's own "day-one
// decision gate" framing.
//
// Nuance (Open Question 5 / Risk 2, recorded here so ticket 005 can carry
// it into its own generated table's header comment): every struct checked
// below is standard-layout but NOT trivial (every generated field carries
// a default member initializer, e.g. `float x = 0.0f;`). Under strict
// C++11/C++14 wording, offsetof on a standard-layout-but-non-trivial type
// is CONDITIONALLY-SUPPORTED, not unconditionally guaranteed -- that
// guarantee only becomes unconditional from C++17 onward. GCC/Clang
// (arm-none-eabi-g++, this project's actual toolchain) define the
// behavior in practice, matching universal embedded-C++ practice (nanopb
// and protobuf-c generated bindings rely on the identical guarantee). The
// static_asserts below check the theoretically load-bearing property
// (standard-layout) -- the C++11-vs-C++17 offsetof-on-non-trivial
// technicality itself cannot be static_assert'ed.
#pragma once

#include <type_traits>

#include "messages/envelope.h"

namespace msg {

"""

_LAYOUT_CHECKS_FOOTER = "}  // namespace msg\n"

_LAYOUT_CHECKS_SOURCE = """\
// AUTO-GENERATED — do not edit by hand.
// Regenerated by scripts/gen_messages.py before each firmware build.
// Translation-unit anchor (095-003): forces layout_checks.h's
// static_asserts to be evaluated as part of the normal build (just build
// ARM + just build-sim) -- no new build step. Defines zero runtime
// symbols.
#include "messages/layout_checks.h"
"""


def _emit_layout_checks_header(struct_names: list) -> str:
    """Emit source/messages/layout_checks.h: one static_assert per struct.

    Aggregator-header choice (over inline-per-header asserts): the
    reachable-from-CommandEnvelope/ReplyEnvelope set spans SIX generated
    headers (envelope.h, motion.h, drivetrain.h, planner.h, odometer.h,
    common.h) and is a cross-cutting, whole-schema property computed from
    the full FileDescriptorSet -- emitting it inline would require every
    per-file `_emit_file()` call to know a global fact it doesn't otherwise
    need. A single aggregator that `#include`s envelope.h (which
    transitively pulls in every other file this check spans, via the
    095-003 cross-file `#include` fix above) and asserts on every reachable
    struct keeps that global computation in exactly one place.
    """
    asserts = "\n".join(
        f"static_assert(std::is_standard_layout<{name}>::value,\n"
        f'              "msg::{name} must be standard-layout for '
        f'offsetof-based field tables");'
        for name in struct_names
    )
    return _LAYOUT_CHECKS_BANNER + asserts + "\n\n" + _LAYOUT_CHECKS_FOOTER


def _emit_layout_checks_source() -> str:
    """Emit source/messages/layout_checks.cpp — the TU anchor above."""
    return _LAYOUT_CHECKS_SOURCE


# ---------------------------------------------------------------------------
# 095-005: FieldDesc tables + wire.{h,cpp} generation (M4 -- Generated
# Envelope Codec, architecture-update.md). Builds on ticket 003's
# _compute_layout_check_structs() (the exact reachable-from-CommandEnvelope/
# ReplyEnvelope struct set, already proven standard-layout/offsetof-safe by
# that ticket's day-one gate) and ticket 004's WireRuntime primitives.
#
# Two independent things are computed from the SAME field walk below:
#   1. kMaxEncodedSize (Step 0c / Step 4): a pure-PYTHON worst-case wire-size
#      calculator (varint max width, fixed sizes, nested-message worst case,
#      MAX across mutually exclusive oneof arms) -- the resulting numbers are
#      baked into wire.h as `constexpr` literals + `static_assert`s, so a
#      FUTURE schema change that grows a field is caught by re-running this
#      generator (which every `just build`/`just build-sim` already does)
#      and recompiling, not by any runtime check.
#   2. FieldDesc/MessageTable tables (Step 2): the RUNTIME offsetof-based
#      data the generated decode()/encode() engine in wire.cpp walks.
# ---------------------------------------------------------------------------

_WIRE_SCALAR_TYPE_BY_PROTO_TYPE = {}  # populated below once _TYPE_* constants exist


def _scalar_type_literal(field) -> str:
    """Map a proto scalar/enum field to its ScalarType C++ enum literal."""
    t = field.type
    if t == _TYPE_FLOAT:
        return "kFloat"
    if t == _TYPE_DOUBLE:
        return "kDouble"
    if t == _TYPE_INT32:
        return "kInt32"
    if t == _TYPE_INT64:
        return "kInt64"
    if t == _TYPE_UINT32:
        return "kUint32"
    if t == _TYPE_UINT64:
        return "kUint64"
    if t == _TYPE_BOOL:
        return "kBool"
    if t == _TYPE_ENUM:
        return "kEnum"
    raise GenMessagesError(
        f"wire.cpp codegen: field {field.name} has no ScalarType mapping (proto type={t})")


_WIRE_TYPE_BY_SCALAR = {
    "kFloat": "kFixed32",
    "kDouble": "kFixed64",
    "kInt32": "kVarint",
    "kInt64": "kVarint",
    "kUint32": "kVarint",
    "kUint64": "kVarint",
    "kBool": "kVarint",
    "kEnum": "kVarint",
}

# sizeof() for each ScalarType's C++ storage -- used for repeated-scalar
# elemStride (unused by this sprint's reachable schema, see FieldKind's own
# doc comment in wire.cpp, but computed correctly for engine completeness).
_SCALAR_CPP_SIZE = {
    "kFloat": 4, "kDouble": 8, "kInt32": 4, "kInt64": 8,
    "kUint32": 4, "kUint64": 8, "kBool": 1, "kEnum": 1,
}


def _oneof_member_expr(field) -> str:
    """Return the C++ member-access suffix for a real-oneof union member.

    Mirrors _emit_message()'s own union-emission rule exactly: a BOOL oneof
    member is emitted as `uint8_t {name}_v` (not a plain `{name}`) to dodge
    union-of-bool UB on some compilers; every other type keeps its bare
    field name.
    """
    if field.type == _TYPE_BOOL:
        return f"{field.name}_v"
    return field.name


def _varint_len(value: int) -> int:
    """Byte width of `value` (already the target wire representation, e.g. a
    zigzag/sign-extended 64-bit pattern) as a protobuf base-128 varint."""
    if value < 0:
        value &= 0xFFFFFFFFFFFFFFFF
    n = 1
    while value >= 0x80:
        value >>= 7
        n += 1
    return n


def _field_wire_type_bits(field, packed: bool = False) -> int:
    if packed:
        return 2  # length-delimited (the packed payload blob)
    t = field.type
    if t == _TYPE_FLOAT:
        return 5
    if t == _TYPE_DOUBLE:
        return 1
    if t in (_TYPE_INT32, _TYPE_INT64, _TYPE_UINT32, _TYPE_UINT64, _TYPE_BOOL, _TYPE_ENUM):
        return 0
    if t in (_TYPE_STRING, _TYPE_BYTES, _TYPE_MESSAGE):
        return 2
    raise GenMessagesError(f"wire.cpp codegen: no wire-type mapping for field type {t}")


def _tag_size_for_field(field, packed: bool = False) -> int:
    wt = _field_wire_type_bits(field, packed)
    return _varint_len((field.number << 3) | wt)


def _tag_and_len_size(field, content_len: int, packed: bool = False) -> int:
    return _tag_size_for_field(field, packed) + _varint_len(content_len) + content_len


def _worst_case_scalar_size(field) -> int:
    """Worst-case ON-WIRE size (bytes) of ONE occurrence of this field's
    VALUE ONLY (excludes its tag) -- conservative protobuf wire-format
    widths. Not narrowed by any (min)/(max)/(abs_max) bound: every bounded
    field in this schema is a fixed-width `float`, and a bound never changes
    a float's wire width; a future bounded VARINT field would need this
    revisited.
    """
    t = field.type
    if t == _TYPE_FLOAT:
        return 4
    if t == _TYPE_DOUBLE:
        return 8
    if t == _TYPE_BOOL:
        return 1
    if t == _TYPE_ENUM:
        # Every enum reachable from CommandEnvelope/ReplyEnvelope today has
        # a small, non-negative value set (<32 enumerators) -- exactly 1
        # byte. Protobuf enums are wire-identical to int32; a hypothetical
        # future enum with a negative or huge value would need the int32
        # case below instead -- flagged here rather than silently assumed
        # forever.
        return 1
    if t == _TYPE_UINT32:
        return 5  # ceil(32/7)
    if t == _TYPE_INT32:
        # protobuf's own well-known gotcha: a NEGATIVE int32 is sign-
        # extended to 64 bits before varint encoding by the reference
        # implementation (unless the field is sint32, which this schema does
        # not use) -- worst case is the full 10-byte int64 varint width.
        return 10
    if t in (_TYPE_UINT64, _TYPE_INT64):
        return 10
    raise GenMessagesError(f"wire.cpp codegen: no worst-case scalar size rule for field type {t}")


def _worst_case_field_contribution(field, msg_map: dict, memo: dict, is_repeated: bool) -> int:
    """Worst-case tag(s)+value(s) contribution of ONE field declaration (a
    single non-repeated field, or a repeated field's full max_count
    occurrences) to its enclosing message's own worst-case size."""
    if field.type == _TYPE_MESSAGE:
        nested_name = _short_type_name(field.type_name)
        payload = _worst_case_message_size(nested_name, msg_map, memo)
        one = _tag_and_len_size(field, payload)
        if is_repeated:
            max_n = _read_max_count(field) or 0
            return one * max_n  # repeated MESSAGE is never packed -- each element separately tagged
        return one
    if field.type == _TYPE_STRING:
        cap = _read_str_len(field)
        content = max(cap - 1, 0)  # capacity reserves room for the decoder's null terminator
        return _tag_and_len_size(field, content)
    if field.type == _TYPE_BYTES:
        cap = _read_max_count(field)
        if cap is None:
            cap = 64
        return _tag_and_len_size(field, cap)
    # scalar (float/double/int/uint/bool/enum)
    value_size = _worst_case_scalar_size(field)
    if is_repeated:
        max_n = _read_max_count(field) or 0
        payload = value_size * max_n
        return _tag_and_len_size(field, payload, packed=True)
    return _tag_size_for_field(field) + value_size


def _worst_case_message_size(struct_name: str, msg_map: dict, memo: dict) -> int:
    """Worst-case wire-encoded size of ONE `struct_name` instance, standalone
    (no enclosing tag/length prefix) -- SUM across every non-oneof field's
    own worst-case contribution, plus the MAX (not sum) across each real
    oneof's mutually-exclusive arms. Matches the hand-computation methodology
    this ticket's own thrown exception used (nanopb's PB_SIZE_MAX
    convention), memoized per struct name since the same nested type (e.g.
    Pose2D) can be reached from multiple parents.
    """
    if struct_name in memo:
        return memo[struct_name]
    md = msg_map[struct_name]
    real_oneofs, opt_indices = _classify_oneofs(md)
    oneof_field_numbers = {f.number for _oi, _name, fields in real_oneofs for f in fields}

    total = 0
    for field in md.field:
        if field.number in oneof_field_numbers:
            continue
        is_repeated = field.label == _LABEL_REPEATED
        total += _worst_case_field_contribution(field, msg_map, memo, is_repeated)

    for _oi, _name, fields in real_oneofs:
        arm_worst = 0
        for f in fields:
            arm_worst = max(arm_worst, _worst_case_field_contribution(f, msg_map, memo, is_repeated=False))
        total += arm_worst

    memo[struct_name] = total
    return total


def _envelope_worst_case_report(struct_name: str, oneof_field_name: str, msg_map: dict, memo: dict) -> dict:
    """Per-arm + total worst-case breakdown for a top-level envelope type's
    named oneof (`cmd` for CommandEnvelope, `body` for ReplyEnvelope) --
    Step 0c's "individually computed and reported" requirement. Each arm's
    reported size is its OWN wrapped contribution (tag+len+payload) as it
    sits inside the envelope, matching the exception's own reporting
    convention; `total` (non-oneof fields + the single worst arm) is the
    number actually checked against the 186-byte budget.
    """
    md = msg_map[struct_name]
    real_oneofs, _opt = _classify_oneofs(md)
    oneof_fields = None
    for _oi, name, fields in real_oneofs:
        if name == oneof_field_name:
            oneof_fields = fields
            break
    if oneof_fields is None:
        raise GenMessagesError(f"wire.cpp codegen: {struct_name} has no oneof named {oneof_field_name!r}")

    oneof_number_set = {f.number for f in oneof_fields}
    non_oneof_total = 0
    for field in md.field:
        if field.number in oneof_number_set:
            continue
        is_repeated = field.label == _LABEL_REPEATED
        non_oneof_total += _worst_case_field_contribution(field, msg_map, memo, is_repeated)

    arm_sizes = {f.name: _worst_case_field_contribution(f, msg_map, memo, is_repeated=False) for f in oneof_fields}
    worst_arm_name = max(arm_sizes, key=arm_sizes.get) if arm_sizes else None
    worst_arm = arm_sizes[worst_arm_name] if worst_arm_name else 0

    return {
        "struct": struct_name,
        "non_oneof": non_oneof_total,
        "arms": arm_sizes,
        "worst_arm_name": worst_arm_name,
        "worst_arm": worst_arm,
        "total": non_oneof_total + worst_arm,
    }


def _float_literal(v: float) -> str:
    return f"{float(v)!r}f"


def _fd_entry(field, kind: str, scalar_type: str | None = None, offset_expr: str = "0",
              offset2_expr: str = "0", oneof_kind_value_expr: str = "0", cap: int = 0,
              table_index_expr: str = "0xFF", elem_stride_expr: str = "0") -> dict:
    min_v = _read_bound(field, _FIELD_OPT_MIN)
    max_v = _read_bound(field, _FIELD_OPT_MAX)
    abs_v = _read_bound(field, _FIELD_OPT_ABS_MAX)
    req = _read_req(field)

    flag_names = []
    if min_v is not None:
        flag_names.append("kHasMin")
    if max_v is not None:
        flag_names.append("kHasMax")
    if abs_v is not None:
        flag_names.append("kHasAbsMax")
    if req:
        flag_names.append("kIsReq")
    flags_expr = " | ".join(flag_names) if flag_names else "0"

    if kind in ("kMessage", "kOneofMessage", "kString", "kBytes", "kRepeatedMessage", "kRepeatedScalar"):
        wire_type = "kLengthDelimited"
    else:
        wire_type = _WIRE_TYPE_BY_SCALAR[scalar_type]

    return {
        "number": field.number,
        "wire_type": wire_type,
        "kind": kind,
        "scalar_type": scalar_type or "kNone",
        "offset_expr": offset_expr,
        "offset2_expr": offset2_expr,
        "oneof_kind_value_expr": oneof_kind_value_expr,
        "cap": cap,
        "table_index_expr": table_index_expr,
        "elem_stride_expr": elem_stride_expr,
        "flags_expr": flags_expr,
        "min": min_v if min_v is not None else 0.0,
        "max": max_v if max_v is not None else 0.0,
        "abs_max": abs_v if abs_v is not None else 0.0,
        "field_name": field.name,
    }


def _build_field_table(struct_name: str, md, msg_map: dict, table_index_of: dict) -> list[dict]:
    """Build the list of FieldDesc entries for one reachable struct.

    Every entry's `offset_expr`/`offset2_expr` is a literal `offsetof(...)`
    C++ expression against the ACTUAL generated struct shape in
    source/messages/*.h (union-member paths for oneof arms, the `_`-suffixed
    array/count members repeated and bytes fields already use, `.has`/`.val`
    for Opt<T>) -- see _emit_message()'s own field-emission rules, which
    this function mirrors rather than duplicates a second struct-shape
    decision from.
    """
    real_oneofs, opt_indices = _classify_oneofs(md)
    oneof_field_map = {}
    for _oi, oneof_name, fields in real_oneofs:
        kind_enum = f"{struct_name}::{_cap_camel(oneof_name)}Kind"
        for f in fields:
            oneof_field_map[f.number] = (oneof_name, kind_enum, f.name.upper())

    entries: list[dict] = []
    for field in md.field:
        is_repeated = field.label == _LABEL_REPEATED
        is_opt = field.HasField("oneof_index") and field.oneof_index in opt_indices

        if field.number in oneof_field_map:
            oneof_name, kind_enum, kind_member = oneof_field_map[field.number]
            member = _oneof_member_expr(field)
            member_path = f"{oneof_name}.{member}"
            offset2_expr = f"offsetof({struct_name}, {oneof_name}_kind)"
            kind_value_expr = f"static_cast<uint16_t>({kind_enum}::{kind_member})"
            if field.type == _TYPE_MESSAGE:
                nested = _short_type_name(field.type_name)
                entries.append(_fd_entry(
                    field, kind="kOneofMessage",
                    offset_expr=f"offsetof({struct_name}, {member_path})",
                    offset2_expr=offset2_expr,
                    oneof_kind_value_expr=kind_value_expr,
                    table_index_expr=str(table_index_of[nested]),
                ))
            else:
                entries.append(_fd_entry(
                    field, kind="kOneofScalar", scalar_type=_scalar_type_literal(field),
                    offset_expr=f"offsetof({struct_name}, {member_path})",
                    offset2_expr=offset2_expr,
                    oneof_kind_value_expr=kind_value_expr,
                ))
            continue

        if is_repeated:
            max_n = _read_max_count(field) or 8
            if field.type == _TYPE_MESSAGE:
                nested = _short_type_name(field.type_name)
                entries.append(_fd_entry(
                    field, kind="kRepeatedMessage",
                    offset_expr=f"offsetof({struct_name}, {field.name}_)",
                    offset2_expr=f"offsetof({struct_name}, {field.name}_count)",
                    cap=max_n, table_index_expr=str(table_index_of[nested]),
                    elem_stride_expr=f"sizeof({nested})",
                ))
            else:
                scalar_type = _scalar_type_literal(field)
                entries.append(_fd_entry(
                    field, kind="kRepeatedScalar", scalar_type=scalar_type,
                    offset_expr=f"offsetof({struct_name}, {field.name}_)",
                    offset2_expr=f"offsetof({struct_name}, {field.name}_count)",
                    cap=max_n, elem_stride_expr=f"sizeof({_scalar_cpp_type(field)})",
                ))
            continue

        if is_opt:
            if field.type == _TYPE_STRING:
                # No optional-string field is reachable from CommandEnvelope/
                # ReplyEnvelope in this sprint's schema -- flag rather than
                # guess at an untested wire shape (mirrors ticket 001/003's
                # own "flag and stop rather than hand-patch" precedent).
                print(f"  WARNING: {struct_name}.{field.name} is an optional string -- "
                      f"wire.cpp codegen does not implement this FieldKind yet (unreached "
                      f"by this sprint's schema); no FieldDesc emitted for it.",
                      file=sys.stderr)
                continue
            entries.append(_fd_entry(
                field, kind="kOpt", scalar_type=_scalar_type_literal(field),
                offset_expr=f"offsetof({struct_name}, {field.name}.has)",
                offset2_expr=f"offsetof({struct_name}, {field.name}.val)",
            ))
            continue

        if field.type == _TYPE_MESSAGE:
            nested = _short_type_name(field.type_name)
            entries.append(_fd_entry(
                field, kind="kMessage",
                offset_expr=f"offsetof({struct_name}, {field.name})",
                table_index_expr=str(table_index_of[nested]),
            ))
            continue

        if field.type == _TYPE_STRING:
            entries.append(_fd_entry(
                field, kind="kString",
                offset_expr=f"offsetof({struct_name}, {field.name})",
                cap=_read_str_len(field),
            ))
            continue

        if field.type == _TYPE_BYTES:
            cap = _read_max_count(field)
            if cap is None:
                cap = 64
            entries.append(_fd_entry(
                field, kind="kBytes",
                offset_expr=f"offsetof({struct_name}, {field.name}_)",
                offset2_expr=f"offsetof({struct_name}, {field.name}_count)",
                cap=cap,
            ))
            continue

        # plain scalar / enum
        entries.append(_fd_entry(
            field, kind="kScalar", scalar_type=_scalar_type_literal(field),
            offset_expr=f"offsetof({struct_name}, {field.name})",
        ))

    return entries


def _render_field_desc(e: dict) -> str:
    return (
        "    { "
        f".number = {e['number']}, "
        f".wireType = WireRuntime::WireType::{e['wire_type']}, "
        f".kind = FieldKind::{e['kind']}, "
        f".scalarType = ScalarType::{e['scalar_type']}, "
        f".offset = {e['offset_expr']}, "
        f".offset2 = {e['offset2_expr']}, "
        f".oneofKindValue = {e['oneof_kind_value_expr']}, "
        f".cap = {e['cap']}, "
        f".tableIndex = {e['table_index_expr']}, "
        f".elemStride = {e['elem_stride_expr']}, "
        f".flags = {e['flags_expr']}, "
        f".minVal = {_float_literal(e['min'])}, "
        f".maxVal = {_float_literal(e['max'])}, "
        f".absMaxVal = {_float_literal(e['abs_max'])} "
        "},"
        f"  // {e['field_name']}"
    )


def _render_message_table(name: str, entries: list[dict]) -> list[str]:
    lines = []
    if entries:
        lines.append(f"constexpr FieldDesc kFields_{name}[] = {{")
        for e in entries:
            lines.append(_render_field_desc(e))
        lines.append("};")
        lines.append(f"constexpr MessageTable kTable_{name} = {{ kFields_{name}, {len(entries)} }};")
    else:
        lines.append(f"constexpr MessageTable kTable_{name} = {{ nullptr, 0 }};")
    lines.append("")
    return lines


_WIRE_H_BANNER = """\
// AUTO-GENERATED — do not edit by hand.
// Regenerated by scripts/gen_messages.py before each firmware build.
// Generated envelope codec (095-005, M4 "Generated Envelope Codec",
// architecture-update.md). Declares decode()/encode(), which walk the
// FieldDesc tables generated into wire.cpp to decode, encode, and validate
// CommandEnvelope/ReplyEnvelope and every message reachable from them.
// Built on source/messages/wire_runtime.{h,cpp} (M3, hand-written,
// schema-agnostic byte-level primitives) -- this file adds the schema
// knowledge (field numbers/offsets/bounds) WireRuntime deliberately does
// not have.
//
// Open Question 1 (resolved): FieldDesc.minVal/maxVal/absMaxVal (wire.cpp)
// are stored as `float` (4 bytes), NOT `double` (the (min)/(max)/(abs_max)
// proto option's own declared width) -- halves the flash cost of the field
// tables and matches every generated scalar field's own `float` type; no
// schema field this sprint needs more than `float` precision for a bound.
//
// offsetof()-on-non-trivial-standard-layout nuance: every struct wire.cpp's
// tables index into is standard-layout but NOT trivial (every field has a
// default member initializer). This is CONDITIONALLY-SUPPORTED under strict
// C++11/C++14 wording but unconditionally well-defined from C++17 onward --
// and this project's actual compiled standard is `-std=gnu++20` (root
// CMakeLists.txt / tests/_infra/sim/CMakeLists.txt both override the
// vendored codal-microbit-v2 target's nominal C++11 pin), so offsetof here
// is standard-guaranteed, not merely "GCC/Clang define it in practice". See
// source/messages/layout_checks.h (095-003) for the day-one gate that
// verified every struct below is standard-layout.
#pragma once

#include <cstdint>

#include "messages/envelope.h"

namespace msg {
namespace wire {

struct Result {
  bool ok;
  uint16_t field;
  ErrCode code;
};

// kMaxEncodedSize (095-005 Step 0c / Step 4, architecture-update-r1.md
// Decision 6): worst-case wire-encoded size of the largest CommandEnvelope/
// ReplyEnvelope oneof arm, computed by gen_messages.py from the schema's own
// field widths (varint max width, fixed sizes, nested-message worst case,
// MAX -- not sum -- across mutually exclusive oneof arms). Recomputed on
// every regeneration (gen_messages.py runs before every `just build`/`just
// build-sim`), so a future schema change that pushes an envelope over the
// 186-byte budget fails one of the two static_asserts below at build time,
// not at runtime on a truncated wire line.
"""

_WIRE_H_FOOTER = """
// decode(): walks CommandEnvelope's generated FieldDesc table per incoming
// wire tag, validating (min)/(max)/(abs_max)/(req) inline during the same
// pass. Returns {false, fieldNumber, ErrCode} on the first violation
// (missing (req) field, out-of-bound value, or malformed wire bytes -- see
// ErrCode's own doc comment in envelope.proto for which code means which).
// Unknown field numbers are skipped (forward-compatible with a future
// schema that declares a field number this build doesn't recognize yet).
Result decode(CommandEnvelope& out, const uint8_t* buf, uint16_t len);

// encode(): walks ReplyEnvelope's generated FieldDesc table, emitting only
// the currently-selected `body` oneof arm (plus `corr_id` if nonzero --
// proto3 implicit presence: a plain scalar field equal to its zero default
// is omitted from the wire, exactly as a real protobuf encoder would, so
// ticket 006's differential fuzz suite sees byte-identical output against
// google.protobuf). Returns 0 (never a truncated/corrupt buffer) if `cap`
// is smaller than the required output.
uint16_t encode(const ReplyEnvelope& in, uint8_t* buf, uint16_t cap);

}  // namespace wire
}  // namespace msg
"""

_WIRE_CPP_PART1 = """\
// AUTO-GENERATED — do not edit by hand.
// Regenerated by scripts/gen_messages.py before each firmware build.
// Generated envelope codec implementation (095-005, M4). The FieldKind/
// ScalarType/FieldDesc/MessageTable types and the decodeInto()/
// encodeInto()/encodeNestedMessage() generic walkers below are EMITTED
// VERBATIM by gen_messages.py (identical on every regeneration -- this is
// schema-INDEPENDENT engine code, unlike the per-message kFields_*/kTable_*
// tables further down, which ARE regenerated from the current
// protos/*.proto schema every run). It lives here rather than in
// wire_runtime.{h,cpp} (M3) because M3's own header comment draws the line:
// M3 "knows nothing about field numbers, offsets, or bounds belonging to
// any specific message -- that is M4's job" -- FieldDesc/MessageTable and
// the code that walks them ARE that job, even though the walker's own text
// happens to be schema-agnostic (only the DATA varies per message). See
// wire.h's header comment for the offsetof/C++17-vs-C++11 and
// float-vs-double (Open Question 1) notes.
#include "messages/wire.h"

#include <cstddef>
#include <cstring>

#include "messages/wire_runtime.h"

namespace msg {
namespace wire {

namespace {

// --- Generic field-table types -------------------------------------------

enum class FieldKind : uint8_t {
  kScalar,          // plain (non-oneof, non-Opt) scalar/enum field
  kOpt,             // Opt<T> (proto3 `optional` scalar)
  kOneofScalar,     // scalar/enum member of a real oneof union
  kMessage,         // plain (non-oneof) nested-message field (unreached by
                     // this sprint's schema -- every message-typed field
                     // reachable from CommandEnvelope/ReplyEnvelope is a
                     // oneof member or a repeated element -- kept for
                     // engine completeness/future schema growth)
  kOneofMessage,    // nested-message member of a real oneof union
  kString,          // fixed-capacity char[N] (non-oneof, non-Opt)
  kBytes,           // fixed-capacity uint8_t[N] + count (Echo.payload)
  kRepeatedScalar,  // T[N] + count, PACKED on the wire (unreached by this
                     // sprint's schema -- kept for engine completeness)
  kRepeatedMessage, // T[N] + count, each element separately tagged
                     // (WheelTargets.w -- the repeated field this sprint's
                     // schema actually reaches)
};

enum class ScalarType : uint8_t {
  kNone = 0,
  kFloat,
  kDouble,
  kInt32,
  kInt64,
  kUint32,
  kUint64,
  kBool,
  kEnum,
};

// FieldDesc: one entry per proto field, offsetof-based (see wire.h's header
// comment for the standard-layout/offsetof/C++17 discussion). `minVal`/
// `maxVal`/`absMaxVal` are `float` (Open Question 1). `flags` bits:
// kHasMin=0x01, kHasMax=0x02, kHasAbsMax=0x04, kIsReq=0x08.
struct FieldDesc {
  uint16_t number;
  WireRuntime::WireType wireType;
  FieldKind kind;
  ScalarType scalarType;
  uint16_t offset;         // primary data offset (meaning depends on `kind` -- see decodeInto/encodeInto)
  uint16_t offset2;        // secondary offset: Opt<T>.val / oneof `_kind` discriminator / count field
  uint16_t oneofKindValue; // discriminator value this field's oneof arm sets (kOneofScalar/kOneofMessage only)
  uint16_t cap;            // capacity: char[N]/uint8_t[N] width, or a repeated field's max_count
  uint8_t tableIndex;      // index into kMessageTables[] (kMessage/kOneofMessage/kRepeatedMessage only); 0xFF = n/a
  uint16_t elemStride;     // sizeof(element) for repeated fields; 0 = n/a
  uint8_t flags;
  float minVal;
  float maxVal;
  float absMaxVal;
};

struct MessageTable {
  const FieldDesc* fields;
  uint8_t fieldCount;
};

constexpr uint8_t kHasMin = 0x01;
constexpr uint8_t kHasMax = 0x02;
constexpr uint8_t kHasAbsMax = 0x04;
constexpr uint8_t kIsReq = 0x08;

// Bound sized generously above every struct's actual field count in this
// schema (CommandEnvelope's own table, the largest, has 12 entries) -- used
// to size the decode-time "which fields have we seen" bitmap for the (req)
// completeness check.
constexpr int kMaxFieldsPerMessage = 32;

// --- Scalar read/write/compare helpers ------------------------------------

bool decodeScalarValue(ScalarType type, const uint8_t* buf, size_t len, size_t* pos, void* dst) {
  switch (type) {
    case ScalarType::kFloat: {
      float v;
      if (!WireRuntime::decodeFloat(buf, len, pos, &v)) return false;
      std::memcpy(dst, &v, sizeof(v));
      return true;
    }
    case ScalarType::kDouble:
      // fixed64/double is unreached from CommandEnvelope/ReplyEnvelope in
      // this sprint's schema (every bounded/float field is a protobuf
      // `float`, per wire_runtime.h's own item-3 note) and WireRuntime (M3)
      // has no decodeFixed64 primitive to build on -- reject cleanly rather
      // than mis-decode if a future schema adds a double field before M3
      // grows the primitive.
      return false;
    case ScalarType::kBool:
    case ScalarType::kEnum: {
      uint64_t v;
      if (!WireRuntime::decodeVarint(buf, len, pos, &v)) return false;
      *static_cast<uint8_t*>(dst) = static_cast<uint8_t>(v);
      return true;
    }
    case ScalarType::kInt32: {
      uint64_t v;
      if (!WireRuntime::decodeVarint(buf, len, pos, &v)) return false;
      int32_t sv = static_cast<int32_t>(static_cast<uint32_t>(v));
      std::memcpy(dst, &sv, sizeof(sv));
      return true;
    }
    case ScalarType::kUint32: {
      uint64_t v;
      if (!WireRuntime::decodeVarint(buf, len, pos, &v)) return false;
      uint32_t uv = static_cast<uint32_t>(v);
      std::memcpy(dst, &uv, sizeof(uv));
      return true;
    }
    case ScalarType::kInt64: {
      uint64_t v;
      if (!WireRuntime::decodeVarint(buf, len, pos, &v)) return false;
      int64_t sv = static_cast<int64_t>(v);
      std::memcpy(dst, &sv, sizeof(sv));
      return true;
    }
    case ScalarType::kUint64: {
      uint64_t v;
      if (!WireRuntime::decodeVarint(buf, len, pos, &v)) return false;
      std::memcpy(dst, &v, sizeof(v));
      return true;
    }
    case ScalarType::kNone:
      return false;
  }
  return false;
}

bool encodeScalarValue(ScalarType type, const void* src, uint8_t* buf, size_t cap, size_t* pos) {
  switch (type) {
    case ScalarType::kFloat: {
      float v;
      std::memcpy(&v, src, sizeof(v));
      return WireRuntime::encodeFloat(v, buf, cap, pos);
    }
    case ScalarType::kDouble:
      return false;  // see decodeScalarValue's kDouble note
    case ScalarType::kBool:
    case ScalarType::kEnum: {
      uint8_t v = *static_cast<const uint8_t*>(src);
      return WireRuntime::encodeVarint(v, buf, cap, pos);
    }
    case ScalarType::kInt32: {
      int32_t v;
      std::memcpy(&v, src, sizeof(v));
      // protobuf's own int32 gotcha: a NEGATIVE int32 is sign-extended to
      // 64 bits before varint encoding (unless the field is sint32, which
      // this schema does not use) -- mirrored via the int64 cast.
      return WireRuntime::encodeVarint(static_cast<uint64_t>(static_cast<int64_t>(v)), buf, cap, pos);
    }
    case ScalarType::kUint32: {
      uint32_t v;
      std::memcpy(&v, src, sizeof(v));
      return WireRuntime::encodeVarint(v, buf, cap, pos);
    }
    case ScalarType::kInt64: {
      int64_t v;
      std::memcpy(&v, src, sizeof(v));
      return WireRuntime::encodeVarint(static_cast<uint64_t>(v), buf, cap, pos);
    }
    case ScalarType::kUint64: {
      uint64_t v;
      std::memcpy(&v, src, sizeof(v));
      return WireRuntime::encodeVarint(v, buf, cap, pos);
    }
    case ScalarType::kNone:
      return false;
  }
  return false;
}

// True if `src` holds this scalar type's zero/default value -- proto3
// implicit presence: encodeInto() omits a plain (non-oneof, non-Opt) scalar
// field from the wire when this is true, exactly as a real protobuf encoder
// would (needed for ticket 006's differential fuzz suite to see byte-
// identical output against google.protobuf).
bool scalarIsDefault(ScalarType type, const void* src) {
  switch (type) {
    case ScalarType::kFloat: {
      float v;
      std::memcpy(&v, src, sizeof(v));
      return v == 0.0f;
    }
    case ScalarType::kDouble:
      return true;
    case ScalarType::kBool:
    case ScalarType::kEnum:
      return *static_cast<const uint8_t*>(src) == 0;
    case ScalarType::kInt32: {
      int32_t v;
      std::memcpy(&v, src, sizeof(v));
      return v == 0;
    }
    case ScalarType::kUint32: {
      uint32_t v;
      std::memcpy(&v, src, sizeof(v));
      return v == 0;
    }
    case ScalarType::kInt64: {
      int64_t v;
      std::memcpy(&v, src, sizeof(v));
      return v == 0;
    }
    case ScalarType::kUint64: {
      uint64_t v;
      std::memcpy(&v, src, sizeof(v));
      return v == 0;
    }
    case ScalarType::kNone:
      return true;
  }
  return true;
}

double scalarAsDouble(ScalarType type, const void* src) {
  switch (type) {
    case ScalarType::kFloat: {
      float v;
      std::memcpy(&v, src, sizeof(v));
      return static_cast<double>(v);
    }
    case ScalarType::kDouble: {
      double v;
      std::memcpy(&v, src, sizeof(v));
      return v;
    }
    case ScalarType::kBool:
    case ScalarType::kEnum:
      return static_cast<double>(*static_cast<const uint8_t*>(src));
    case ScalarType::kInt32: {
      int32_t v;
      std::memcpy(&v, src, sizeof(v));
      return static_cast<double>(v);
    }
    case ScalarType::kUint32: {
      uint32_t v;
      std::memcpy(&v, src, sizeof(v));
      return static_cast<double>(v);
    }
    case ScalarType::kInt64: {
      int64_t v;
      std::memcpy(&v, src, sizeof(v));
      return static_cast<double>(v);
    }
    case ScalarType::kUint64: {
      uint64_t v;
      std::memcpy(&v, src, sizeof(v));
      return static_cast<double>(v);
    }
    case ScalarType::kNone:
      return 0.0;
  }
  return 0.0;
}

// Validates (min)/(max)/(abs_max) INLINE during decodeInto()'s single pass
// (no second validation walk) -- returns false on the FIRST bound this
// value violates.
bool validateBounds(const FieldDesc& fd, const void* src) {
  if ((fd.flags & (kHasMin | kHasMax | kHasAbsMax)) == 0) return true;
  const double v = scalarAsDouble(fd.scalarType, src);
  if (fd.flags & kHasAbsMax) {
    const double av = v < 0.0 ? -v : v;
    if (av > static_cast<double>(fd.absMaxVal)) return false;
  }
  if (fd.flags & kHasMin) {
    if (v < static_cast<double>(fd.minVal)) return false;
  }
  if (fd.flags & kHasMax) {
    if (v > static_cast<double>(fd.maxVal)) return false;
  }
  return true;
}

// --- Generated per-message field tables (regenerated from protos/*.proto
// on every run -- everything above this point is fixed engine text,
// everything from here through kMessageTables[] is schema-derived data). --
"""

_WIRE_CPP_PART2 = """\
// --- Generic recursive decode/encode walkers ------------------------------
// (fixed engine text again -- kMessageTables[] above is now fully defined,
// so no forward declaration is needed for the mutual nested-message
// recursion below.)

Result decodeInto(void* base, const MessageTable& table, const uint8_t* buf, size_t len, int depth) {
  uint32_t seen = 0;
  uint8_t* const baseBytes = static_cast<uint8_t*>(base);
  size_t pos = 0;

  while (pos < len) {
    uint32_t fieldNumber = 0;
    WireRuntime::WireType wireType = WireRuntime::WireType::kVarint;
    if (!WireRuntime::decodeTag(buf, len, &pos, &fieldNumber, &wireType)) {
      return Result{false, 0, ErrCode::ERR_DECODE};
    }

    const FieldDesc* fd = nullptr;
    int fdIndex = -1;
    for (uint8_t i = 0; i < table.fieldCount; ++i) {
      if (table.fields[i].number == fieldNumber) {
        fd = &table.fields[i];
        fdIndex = static_cast<int>(i);
        break;
      }
    }
    if (fd == nullptr) {
      if (!WireRuntime::skipField(buf, len, &pos, wireType)) return Result{false, 0, ErrCode::ERR_DECODE};
      continue;
    }
    if (fdIndex >= 0 && fdIndex < kMaxFieldsPerMessage) seen |= (1u << fdIndex);

    uint8_t* const fieldPtr = baseBytes + fd->offset;

    switch (fd->kind) {
      case FieldKind::kScalar: {
        if (!decodeScalarValue(fd->scalarType, buf, len, &pos, fieldPtr)) return Result{false, fd->number, ErrCode::ERR_DECODE};
        if (!validateBounds(*fd, fieldPtr)) return Result{false, fd->number, ErrCode::ERR_RANGE};
        break;
      }
      case FieldKind::kOpt: {
        *reinterpret_cast<bool*>(fieldPtr) = true;
        uint8_t* const valPtr = baseBytes + fd->offset2;
        if (!decodeScalarValue(fd->scalarType, buf, len, &pos, valPtr)) return Result{false, fd->number, ErrCode::ERR_DECODE};
        if (!validateBounds(*fd, valPtr)) return Result{false, fd->number, ErrCode::ERR_RANGE};
        break;
      }
      case FieldKind::kOneofScalar: {
        *(baseBytes + fd->offset2) = static_cast<uint8_t>(fd->oneofKindValue);
        if (!decodeScalarValue(fd->scalarType, buf, len, &pos, fieldPtr)) return Result{false, fd->number, ErrCode::ERR_DECODE};
        if (!validateBounds(*fd, fieldPtr)) return Result{false, fd->number, ErrCode::ERR_RANGE};
        break;
      }
      case FieldKind::kMessage:
      case FieldKind::kOneofMessage: {
        if (fd->kind == FieldKind::kOneofMessage) {
          *(baseBytes + fd->offset2) = static_cast<uint8_t>(fd->oneofKindValue);
        }
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, depth, &payloadLen)) return Result{false, fd->number, ErrCode::ERR_DECODE};
        const MessageTable& nested = kMessageTables[fd->tableIndex];
        const Result r = decodeInto(fieldPtr, nested, buf + pos, payloadLen, depth + 1);
        if (!r.ok) return r;
        pos += payloadLen;
        break;
      }
      case FieldKind::kString: {
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, depth, &payloadLen)) return Result{false, fd->number, ErrCode::ERR_DECODE};
        // fd->cap is the array's FULL capacity, including room for the null
        // terminator this decoder always writes.
        if (fd->cap == 0 || payloadLen > static_cast<size_t>(fd->cap) - 1) {
          return Result{false, fd->number, ErrCode::ERR_DECODE};
        }
        std::memcpy(fieldPtr, buf + pos, payloadLen);
        fieldPtr[payloadLen] = '\\0';
        pos += payloadLen;
        break;
      }
      case FieldKind::kBytes: {
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, depth, &payloadLen)) return Result{false, fd->number, ErrCode::ERR_DECODE};
        const size_t n = payloadLen < static_cast<size_t>(fd->cap) ? payloadLen : static_cast<size_t>(fd->cap);
        std::memcpy(fieldPtr, buf + pos, n);
        *(baseBytes + fd->offset2) = static_cast<uint8_t>(n);
        pos += payloadLen;
        break;
      }
      case FieldKind::kRepeatedScalar: {
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, depth, &payloadLen)) return Result{false, fd->number, ErrCode::ERR_DECODE};
        uint8_t* const countPtr = baseBytes + fd->offset2;
        size_t outCount = 0;
        const bool ok = (fd->scalarType == ScalarType::kFloat)
                            ? WireRuntime::decodePackedFixed32(buf + pos, payloadLen, reinterpret_cast<float*>(fieldPtr),
                                                                fd->cap, &outCount)
                            : WireRuntime::decodePackedVarint(buf + pos, payloadLen, reinterpret_cast<uint32_t*>(fieldPtr),
                                                               fd->cap, &outCount);
        if (!ok) return Result{false, fd->number, ErrCode::ERR_DECODE};
        *countPtr = static_cast<uint8_t>(outCount);
        pos += payloadLen;
        break;
      }
      case FieldKind::kRepeatedMessage: {
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, depth, &payloadLen)) return Result{false, fd->number, ErrCode::ERR_DECODE};
        uint8_t* const countPtr = baseBytes + fd->offset2;
        const uint8_t count = *countPtr;
        if (count < fd->cap) {
          const MessageTable& nested = kMessageTables[fd->tableIndex];
          uint8_t* const elemPtr = fieldPtr + static_cast<size_t>(count) * fd->elemStride;
          const Result r = decodeInto(elemPtr, nested, buf + pos, payloadLen, depth + 1);
          if (!r.ok) return r;
          *countPtr = static_cast<uint8_t>(count + 1);
        }
        // else: clamped at max_count -- silently dropped, matching
        // WireRuntime's own packed-repeated max_count-clamp convention
        // (095-004). Unlike that packed-scalar reader, a clamped repeated-
        // MESSAGE element's bytes are NOT structurally re-validated here
        // (would need a per-element-type scratch instance the generic
        // offsetof walker does not have) -- a deliberate, documented engine
        // limitation, not an oversight.
        pos += payloadLen;
        break;
      }
    }
  }

  for (uint8_t i = 0; i < table.fieldCount; ++i) {
    if ((table.fields[i].flags & kIsReq) != 0) {
      const bool wasSeen = (i < kMaxFieldsPerMessage) && ((seen >> i) & 1u);
      if (!wasSeen) return Result{false, table.fields[i].number, ErrCode::ERR_BADARG};
    }
  }
  return Result{true, 0, ErrCode::ERR_NONE};
}

// Scratch cap for a nested message's own encoded payload -- comfortably
// above the 186-byte whole-envelope budget (no nested message this schema
// declares can itself approach 186B; the largest, DeviceId, is ~171B and is
// never nested inside another message -- it IS the top-level reply body).
constexpr size_t kEncodeScratchCap = 220;

bool encodeInto(const void* base, const MessageTable& table, uint8_t* buf, size_t cap, size_t* pos);

bool encodeNestedMessage(const FieldDesc& fd, const void* src, uint8_t* buf, size_t cap, size_t* pos) {
  uint8_t scratch[kEncodeScratchCap];
  size_t scratchPos = 0;
  const MessageTable& nested = kMessageTables[fd.tableIndex];
  if (!encodeInto(src, nested, scratch, sizeof(scratch), &scratchPos)) return false;
  if (!WireRuntime::encodeTag(fd.number, WireRuntime::WireType::kLengthDelimited, buf, cap, pos)) return false;
  if (!WireRuntime::encodeVarint(scratchPos, buf, cap, pos)) return false;
  const size_t p = *pos;
  if (cap - p < scratchPos) return false;
  std::memcpy(buf + p, scratch, scratchPos);
  *pos = p + scratchPos;
  return true;
}

bool encodeInto(const void* base, const MessageTable& table, uint8_t* buf, size_t cap, size_t* pos) {
  const uint8_t* const baseBytes = static_cast<const uint8_t*>(base);

  for (uint8_t i = 0; i < table.fieldCount; ++i) {
    const FieldDesc& fd = table.fields[i];
    const uint8_t* const fieldPtr = baseBytes + fd.offset;

    switch (fd.kind) {
      case FieldKind::kScalar: {
        if (scalarIsDefault(fd.scalarType, fieldPtr)) break;  // proto3 implicit presence
        if (!WireRuntime::encodeTag(fd.number, fd.wireType, buf, cap, pos)) return false;
        if (!encodeScalarValue(fd.scalarType, fieldPtr, buf, cap, pos)) return false;
        break;
      }
      case FieldKind::kOpt: {
        if (!*reinterpret_cast<const bool*>(fieldPtr)) break;
        const uint8_t* const valPtr = baseBytes + fd.offset2;
        if (!WireRuntime::encodeTag(fd.number, fd.wireType, buf, cap, pos)) return false;
        if (!encodeScalarValue(fd.scalarType, valPtr, buf, cap, pos)) return false;
        break;
      }
      case FieldKind::kOneofScalar: {
        const uint8_t activeKind = *(baseBytes + fd.offset2);
        if (activeKind != static_cast<uint8_t>(fd.oneofKindValue)) break;
        if (!WireRuntime::encodeTag(fd.number, fd.wireType, buf, cap, pos)) return false;
        if (!encodeScalarValue(fd.scalarType, fieldPtr, buf, cap, pos)) return false;
        break;
      }
      case FieldKind::kOneofMessage: {
        const uint8_t activeKind = *(baseBytes + fd.offset2);
        if (activeKind != static_cast<uint8_t>(fd.oneofKindValue)) break;
        if (!encodeNestedMessage(fd, fieldPtr, buf, cap, pos)) return false;
        break;
      }
      case FieldKind::kMessage: {
        if (!encodeNestedMessage(fd, fieldPtr, buf, cap, pos)) return false;
        break;
      }
      case FieldKind::kString: {
        // 095-006 (SUC-005, differential-suite finding): scan strictly up to
        // `fd.cap - 1`, NEVER `fd.cap` -- the LAST byte of a char[cap] field
        // is reserved for the null terminator by the SAME convention
        // decodeInto()'s own kString case documents and enforces ("fd->cap
        // is the array's FULL capacity, including room for the null
        // terminator"). Scanning through `fd.cap` (the pre-fix bound) reads
        // that reserved byte too: for a MAX-LENGTH string (content ==
        // cap - 1 bytes, e.g. a 47-char DeviceId.model at str_len=48) there
        // is no guaranteed '\\0' anywhere in [0, cap), so the scan fell
        // through to the reserved byte and used ITS value -- for a struct
        // that was never round-tripped through decode() (every
        // ReplyEnvelope this sprint IS hand-constructed by firmware code,
        // never decoded; decode() is CommandEnvelope-only per wire.h's own
        // asymmetric API, Decision 4), that reserved byte can be
        // uninitialized (a default-constructed message only zero-initializes
        // the ACTIVE union alternative's own bytes, not a not-yet-active
        // alternative's tail past a smaller sibling -- see decode()'s own
        // 095-006 fix comment above for the general shape of this hazard),
        // leaking 1 byte of stack/heap garbage onto the wire and reporting a
        // length one byte longer than the field's own documented maximum
        // content length. Confirmed via the differential harness: encoding
        // DeviceId{model: 47 'M' characters} emitted a 48-byte string field
        // instead of 47.
        size_t slen = 0;
        while (slen < static_cast<size_t>(fd.cap) - 1 && reinterpret_cast<const char*>(fieldPtr)[slen] != '\\0') ++slen;
        if (slen == 0) break;  // proto3 implicit presence
        if (!WireRuntime::encodeTag(fd.number, fd.wireType, buf, cap, pos)) return false;
        if (!WireRuntime::encodeVarint(slen, buf, cap, pos)) return false;
        const size_t p = *pos;
        if (cap - p < slen) return false;
        std::memcpy(buf + p, fieldPtr, slen);
        *pos = p + slen;
        break;
      }
      case FieldKind::kBytes: {
        const uint8_t n = *(baseBytes + fd.offset2);
        if (n == 0) break;  // proto3 implicit presence
        if (!WireRuntime::encodeTag(fd.number, fd.wireType, buf, cap, pos)) return false;
        if (!WireRuntime::encodeVarint(n, buf, cap, pos)) return false;
        const size_t p = *pos;
        if (cap - p < n) return false;
        std::memcpy(buf + p, fieldPtr, n);
        *pos = p + n;
        break;
      }
      case FieldKind::kRepeatedScalar: {
        const uint8_t count = *(baseBytes + fd.offset2);
        if (count == 0) break;
        uint8_t scratch[kEncodeScratchCap];
        size_t scratchPos = 0;
        for (uint8_t e = 0; e < count; ++e) {
          const uint8_t* const elemPtr = fieldPtr + static_cast<size_t>(e) * fd.elemStride;
          if (!encodeScalarValue(fd.scalarType, elemPtr, scratch, sizeof(scratch), &scratchPos)) return false;
        }
        if (!WireRuntime::encodeTag(fd.number, fd.wireType, buf, cap, pos)) return false;
        if (!WireRuntime::encodeVarint(scratchPos, buf, cap, pos)) return false;
        const size_t p = *pos;
        if (cap - p < scratchPos) return false;
        std::memcpy(buf + p, scratch, scratchPos);
        *pos = p + scratchPos;
        break;
      }
      case FieldKind::kRepeatedMessage: {
        const uint8_t count = *(baseBytes + fd.offset2);
        for (uint8_t e = 0; e < count; ++e) {
          const uint8_t* const elemPtr = fieldPtr + static_cast<size_t>(e) * fd.elemStride;
          if (!encodeNestedMessage(fd, elemPtr, buf, cap, pos)) return false;
        }
        break;
      }
    }
  }
  return true;
}

}  // namespace

Result decode(CommandEnvelope& out, const uint8_t* buf, uint16_t len) {
  // 095-006 (SUC-005, differential-suite finding, fixed at the generator so
  // every regeneration carries the fix): `out = CommandEnvelope{}` does NOT
  // zero the whole object. Per the C++ aggregate-init rules for a union
  // data member, `= {}` value-initializes only the union's FIRST named
  // alternative (`cmd.drive`, and recursively `control.twist` inside IT);
  // any union alternative that is not first AND is larger than the first
  // alternative's own size (e.g. `cmd.drive.control.wheels`, a WheelTargets,
  // is far larger than `control`'s first alternative BodyTwist3) has its
  // extra bytes left INDETERMINATE. decodeInto()'s repeated-message clamp
  // (`kRepeatedMessage`) reads that indeterminate byte as the field's
  // starting element COUNT before ever writing to it -- confirmed by the
  // differential harness (095-006) reproducing a decode of a valid 2-element
  // `drive.wheels` envelope that came back with a garbage element count and
  // uninitialized element data, an uninitialized-memory read, not merely a
  // stale-value cosmetic bug. `std::memset` zeroes every byte of `out`
  // (well-defined: CommandEnvelope is standard-layout AND trivially
  // copyable -- ticket 003's day-one gate -- so a full-object memset is the
  // same "zero it all, unconditionally" idiom nanopb/protobuf-c use for the
  // identical union-of-message-types shape), independent of which union
  // alternative ends up decoded. Destination cast to `void*` explicitly:
  // GCC's `-Wclass-memaccess` (part of `-Wall`/`-Wextra`) flags a
  // `memset()`/`memcpy()` whose destination argument's OWN declared type is
  // a non-trivial class -- exactly what CommandEnvelope is, for the same
  // "has default member initializers" reason noted above (ticket 003's own
  // day-one gate: standard-layout AND trivially copyable, but NOT trivial).
  // The warning's usual concern (memset-ing a type with vtables/virtual
  // bases/non-POD members) does not apply here -- ticket 003 already proved
  // this exact property -- but the diagnostic can't see that, only the
  // static type; casting to `void*` states the intent explicitly instead of
  // leaving an unexplained warning at every regeneration.
  std::memset(static_cast<void*>(&out), 0, sizeof(out));
  if (buf == nullptr && len != 0) return Result{false, 0, ErrCode::ERR_DECODE};
  return decodeInto(&out, kTable_CommandEnvelope, buf, static_cast<size_t>(len), 0);
}

uint16_t encode(const ReplyEnvelope& in, uint8_t* buf, uint16_t cap) {
  if (buf == nullptr) return 0;
  size_t pos = 0;
  if (!encodeInto(&in, kTable_ReplyEnvelope, buf, static_cast<size_t>(cap), &pos)) return 0;
  return static_cast<uint16_t>(pos);
}

}  // namespace wire
}  // namespace msg
"""


def _render_message_tables_array(struct_order: list[str]) -> list[str]:
    lines = ["constexpr MessageTable kMessageTables[] = {"]
    for i, name in enumerate(struct_order):
        lines.append(f"    kTable_{name},  // {i}")
    lines.append("};")
    lines.append("")
    return lines


def _render_arm_report(report: dict) -> str:
    arm_text = ", ".join(f"{name}={size}B" for name, size in report["arms"].items())
    return (f"//   {report['struct']}: {arm_text} "
            f"(worst={report['worst_arm_name']}={report['worst_arm']}B) + "
            f"non-oneof={report['non_oneof']}B => total={report['total']}B")


def _emit_wire_files(fds):
    """Emit source/messages/wire.{h,cpp}. Returns (wire_h, wire_cpp, cmd_report,
    reply_report) -- the reports are also printed to stderr by the caller
    (095-005 Step 0c's "reported in completion notes" requirement)."""
    struct_order = _compute_layout_check_structs(fds)
    msg_map: dict = {}
    for f in fds.file:
        for md in f.message_type:
            msg_map[md.name] = md
    table_index_of = {name: i for i, name in enumerate(struct_order)}

    memo: dict = {}
    field_tables = {name: _build_field_table(name, msg_map[name], msg_map, table_index_of)
                     for name in struct_order}

    cmd_report = _envelope_worst_case_report("CommandEnvelope", "cmd", msg_map, memo)
    reply_report = _envelope_worst_case_report("ReplyEnvelope", "body", msg_map, memo)

    wire_h = (_WIRE_H_BANNER
              + _render_arm_report(cmd_report) + "\n"
              + _render_arm_report(reply_report) + "\n"
              + f"constexpr uint16_t kCommandEnvelopeMaxEncodedSize = {cmd_report['total']};\n"
              + f"constexpr uint16_t kReplyEnvelopeMaxEncodedSize = {reply_report['total']};\n"
              + "static_assert(kCommandEnvelopeMaxEncodedSize <= 186,\n"
              + '              "CommandEnvelope worst-case encoded size exceeds the 186-byte envelope budget");\n'
              + "static_assert(kReplyEnvelopeMaxEncodedSize <= 186,\n"
              + '              "ReplyEnvelope worst-case encoded size exceeds the 186-byte envelope budget");\n'
              + _WIRE_H_FOOTER)

    cpp_parts = [_WIRE_CPP_PART1]
    for name in struct_order:
        cpp_parts.extend(_render_message_table(name, field_tables[name]))
    cpp_parts.extend(_render_message_tables_array(struct_order))
    cpp_parts.append(_WIRE_CPP_PART2)
    wire_cpp = "\n".join(cpp_parts)

    return wire_h, wire_cpp, cmd_report, reply_report


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
        lines.append(_cross_file_include_block(fd))
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


class GenMessagesError(RuntimeError):
    """Raised by _run_codegen_pipeline() on a codegen pipeline failure."""


def _run_codegen_pipeline():
    """Run protoc over protos/*.proto and emit every header's content in-memory.

    Returns (outputs, fds):
      outputs: {header_name: content} for the proto-derived headers -- no
        filesystem writes.
      fds: the parsed FileDescriptorSet (needed by --emit-inventory).

    Raises GenMessagesError on any pipeline failure (missing grpcio-tools,
    protoc syntax error) instead of printing + returning an exit code, so this
    function is reusable outside main()'s CLI shape.
    """
    # ------------------------------------------------------------------
    # Locate grpcio-tools _proto directory for well-known imports
    # ------------------------------------------------------------------
    try:
        import grpc_tools
    except ImportError as exc:
        raise GenMessagesError(
            "grpcio-tools is not installed.\n"
            "  Run: uv sync  (grpcio-tools is in the 'codegen' dependency group)"
        ) from exc

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
            raise GenMessagesError("protoc failed — check proto syntax.")

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

    # ------------------------------------------------------------------
    # 095-003: day-one decision gate — the standard-layout static_assert
    # aggregator header + its translation-unit anchor .cpp. Computed from
    # the full FileDescriptorSet (every message in every proto file), not
    # just envelope.proto/motion.proto, so the reachable set stays correct
    # as the schema grows.
    # ------------------------------------------------------------------
    layout_structs = _compute_layout_check_structs(fds)
    outputs["layout_checks.h"] = _emit_layout_checks_header(layout_structs)
    outputs["layout_checks.cpp"] = _emit_layout_checks_source()

    # ------------------------------------------------------------------
    # 095-005: FieldDesc tables + wire.{h,cpp} (M4 -- Generated Envelope
    # Codec). See _emit_wire_files()'s own doc comment.
    # ------------------------------------------------------------------
    wire_h, wire_cpp, cmd_report, reply_report = _emit_wire_files(fds)
    outputs["wire.h"] = wire_h
    outputs["wire.cpp"] = wire_cpp
    print("gen_messages: kMaxEncodedSize report (095-005 Step 0c):", file=sys.stderr)
    print(f"  {_render_arm_report(cmd_report)}", file=sys.stderr)
    print(f"  {_render_arm_report(reply_report)}", file=sys.stderr)

    return outputs, fds


def generate_headers() -> dict:
    """Run the codegen pipeline and return {header_name: content} in-memory.

    Public entry point for anything that needs generated header text without
    writing to source/messages/ — currently the getter regression-guard test
    (tests/unit/test_gen_messages_no_getters.py), which scans this output for
    a reintroduced get_*-prefixed method (architecture-update.md Decision 3:
    the guard must run the same codegen path a real build runs, not a grep
    over the checked-in headers alone).
    """
    outputs, _fds = _run_codegen_pipeline()
    return outputs


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

    try:
        outputs, fds = _run_codegen_pipeline()
    except GenMessagesError as exc:
        print(f"gen_messages: {exc}", file=sys.stderr)
        return 1

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
