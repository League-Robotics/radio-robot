#pragma once

#include <cstdint>

#include "app/comms.h"
#include "firm/types/robot_state.h"
#include "messages/telemetry.h"

namespace App {

class Drive;


constexpr uint32_t kFlagOtosPresent = 1u << 0;
constexpr uint32_t kFlagOtosConnected = 1u << 1;
constexpr uint32_t kFlagActive = 1u << 2;
constexpr uint32_t kFlagConnLeft = 1u << 3;
constexpr uint32_t kFlagConnRight = 1u << 4;
constexpr uint32_t kFlagFaultI2CSafetyNet = 1u << 6;
constexpr uint32_t kFlagFaultWedgeLatch = 1u << 7;
constexpr uint32_t kFlagFaultI2CNak = 1u << 8;
constexpr uint32_t kFlagFaultCommsMalformed = 1u << 9;
constexpr uint32_t kFlagEventDeadmanExpired = 1u << 10;
constexpr uint32_t kFlagEventConfigApplied = 1u << 12;
constexpr uint32_t kFlagLinePresent = 1u << 13;
constexpr uint32_t kFlagColorPresent = 1u << 14;
constexpr uint32_t kFlagFaultMoveTimeout = 1u << 15;
constexpr uint32_t kFlagFaultShapingDisabled = 1u << 16;
constexpr uint32_t kFlagFaultPositionClamped = 1u << 17;
constexpr uint32_t kFlagFaultCommandsDropped = 1u << 18;
constexpr uint32_t kFlagFaultWheelFrozenLeft = 1u << 19;
constexpr uint32_t kFlagFaultWheelFrozenRight = 1u << 20;
constexpr uint32_t kFlagFaultWheelDeficitLeft = 1u << 21;
constexpr uint32_t kFlagFaultWheelDeficitRight = 1u << 22;

constexpr uint32_t kPrimaryPeriod = 40;  // [ms] floor; robot_loop.cpp's kCycle (50ms) exceeds it every cycle

constexpr uint8_t kAckRingDepth = 12;

constexpr uint8_t kAckRepeats = 3;

enum class TlmMode : uint8_t { kOff, kAuto, kOn };

constexpr uint32_t kCoastHoldoff = 2000;  // [ms]

class Telemetry {
 public:
  struct Frame {
    msg::DriveMode mode = msg::DriveMode::IDLE;

    msg::EncoderReading encLeft{};
    msg::EncoderReading encRight{};

    msg::OtosReading otos{};

    msg::Pose2D pose{};
    msg::BodyTwist3 twist{};

    uint32_t line = 0;
    uint32_t color = 0;

    uint32_t cycleBusy = 0;  // [us] cycleStart -> frame-staging instant, THIS cycle
    uint32_t cyclePeriod = 0;  // [us] this cycle's own cycleStart minus the previous cycle's

    float dutyPerSpeedLeft = 0.0f;  // [duty/(mm/s)]
    float dutyPerSpeedRight = 0.0f;  // [duty/(mm/s)]
    float biasLeft = 0.0f;  // [mm/s] Stage C's adapted parameter
    float biasRight = 0.0f;  // [mm/s]
    float pidLeft = 0.0f;  // [mm/s] Stage B's last-computed output
    float pidRight = 0.0f;  // [mm/s]
  };

  explicit Telemetry(Comms& comms);

  void update(const Types::RobotState& state, const Drive& drive);

  void setLiveFlag(uint32_t bit, bool active);

  uint32_t flags() const { return flags_; }

  void setMode(TlmMode mode) { mode_ = mode; }
  TlmMode mode() const { return mode_; }

  bool applyAction(Comms::TlmAction action);

  void ack(uint32_t corrId, uint32_t errCode);

  void emit(uint32_t now, bool force = false);

  uint32_t primaryEmitCount() const { return primaryEmitCount_; }
  uint32_t lastPrimaryEmit() const { return lastPrimaryEmit_; }  // [ms]

 private:
  bool primaryDue(uint32_t now) const;
  bool pendingAckDeliveries() const;
  void emitPrimary(uint32_t now);
  void pushAckRing(uint32_t corrId, uint32_t errCode);

  void setFlag(uint32_t bit, bool active);

  static uint32_t ageOf(uint32_t now, uint32_t sampleTime);  // [ms] [ms] -> [ms], clamped to 255

  Comms& comms_;

  Frame frame_;

  uint32_t flags_ = 0;

  uint32_t ackRing_[kAckRingDepth]{};
  uint8_t ackRingHead_ = 0;
  uint8_t ackRingCount_ = 0;

  uint8_t ackSends_[kAckRingDepth]{};

  TlmMode mode_ = TlmMode::kAuto;

  bool everMoved_ = false;

  uint32_t lastActivity_ = 0;  // [ms]

  uint32_t seq_ = 0;

  bool everEmittedPrimary_ = false;
  uint32_t lastPrimaryEmit_ = 0;  // [ms]
  uint32_t primaryEmitCount_ = 0;
};

}
