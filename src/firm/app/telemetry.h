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

// [ms] primary-frame emit floor, deliberately BELOW App::RobotLoop::kCycle
// (32) so every cycle clears it and telemetry stays ONE FRAME PER CYCLE
// (~31 fps). It was 40 against the old 50 ms kCycle for exactly the same
// reason -- the value tracks kCycle down, it is not an independent rate.
//
// ONE FRAME PER CYCLE IS A CORRECTNESS REQUIREMENT, NOT A PREFERENCE.
// robot_loop.cpp's pace block alternates which perception leaf it ticks by
// cycle parity (line on odd cycles, color on even). If the emit floor is
// longer than one cycle, the emit cadence ALIASES with that parity and
// telemetry samples the same phase forever. Measured on tovez 2026-08-07 at
// kPrimaryPeriod=40/kCycle=32: `line_present 93, color_present 0` over 93
// idle frames -- the colour sensor vanished from the wire completely. Any
// future change that lets this exceed kCycle must first decouple perception
// reporting from emit parity (report the CACHED reading with its own age,
// rather than a per-cycle freshness flag).
//
// KNOWN, MEASURED COST -- read before raising the frame rate further. The
// nRF link is HALF DUPLEX, so outbound airtime eats the window in which the
// robot can hear the host. Over the getez relay (channel 3, 2026-08-07):
//
//   kPrimaryPeriod   telemetry   radio_bench_gate   move_wheels   0x0A repro
//   25 (this)        31.4 fps    30/35              command LOST  8/10
//   40               15.8 fps    31/35              ok            9/10
//
// The loss is INBOUND (host->robot commands), not outbound: outbound is
// 99.2% ok with zero unparseable and zero CRC mismatches at BOTH rates, and
// the lost move_wheels was proven to be the command itself, not its ack --
// the encoders never moved (338,358 before and after). Acks are already
// redundant against frame loss (kAckRepeats below: every ack rides three
// consecutive frames), so a dropped telemetry frame cannot lose one.
//
// Halving the emit rate DOES buy back that inbound reliability, and it was
// tried -- but it is a blunt workaround pointed at the wrong direction, and
// it breaks the perception alternation above. The real fix is inbound
// reliability (sequence + NACK + retransmit-from-N, host->robot; the COBS
// framing and CRC needed for it already exist) -- see clasi/issues/
// inbound-command-loss-needs-retransmit-not-a-slower-telemetry-stream.md.
// Until that lands, the relay path carries a known inbound-loss risk that
// pre-dates this constant (clasi/issues/later/radio-bench-gate-fault-latch-
// check-contradicts-inbound-loss-budget.md, sprint 128's own 31/35 run).
// USB is unaffected -- 31 fps is ~2400 B/s, ~21% of the link, and every
// move_protocol_bench scenario passes over it.
constexpr uint32_t kPrimaryPeriod = 25;

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
