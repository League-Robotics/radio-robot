// motor.h — the Motor faceplate: the four-verb contract (configure/apply/
// tick/state, plus capabilities) applied to one wheel-motor channel.
//
// Ported from the locked interface sketch in clasi/sprints/077-greenfield-
// faceplate-hal-drivetrain-and-dev-bench-system/issues/greenfield-rebuild-
// faceplate-hal-in-a-fresh-source-old-tree-parked.md, Step 3. Concrete
// leaves (NezhaMotor today; a future SimMotor/MockMotor could follow) supply
// the primitive setters/getters and the faceplate verbs configure()/tick()/
// capabilities(); apply()/state() are the message plane, implemented ONCE
// here on top of those primitives so no leaf re-derives the oneof-unpack /
// capability-gating logic.
//
// Deviation from the issue's sketch: apply() returns bool here (true =
// accepted), not void. The issue's code sketch shows `void apply(...)`, but
// ticket 003's acceptance criteria requires that a command whose mode is
// absent from capabilities() (e.g. VOLT on Nezha) be rejected "before
// touching hardware" in a way ticket 5's DEV command layer can turn into
// "ERR unsupported" — a void return gives the caller no way to distinguish
// "accepted" from "silently ignored" without re-deriving it from state().
// Returning bool is the minimal change that preserves the rest of the
// locked shape (same parameters, same non-virtual/shared-once
// implementation, same capability-gating description) while making
// rejection observable to the command layer that needs it.
#pragma once

#include <stdint.h>

#include "messages/motor.h"

namespace Hal {

class Motor {
 public:
  virtual ~Motor() = default;
  virtual void begin() {}

  // Primitive setters — the real implementations, one per command mode.
  // setVelocity() sets the target the embedded PID chases in tick();
  // setDutyCycle() stages the duty the slew limiter walks toward; etc.
  // Wheel motors have one degree of freedom, so a directionless magnitude is
  // a speed; velocity here is the signed scalar along that axis.
  virtual void setDutyCycle(float dutyCycle) = 0;      // [-1, 1]
  virtual void setVoltage(float voltage) = 0;          // [V] Nezha: unsupported (capability)
  virtual void setVelocity(float velocity) = 0;        // [mm/s] signed
  virtual void setPosition(float position) = 0;        // [deg]
  virtual void setNeutral(msg::Neutral mode) = 0;
  virtual void setFeedforward(float feedforward) = 0;  // [V]
  virtual void resetPosition() = 0;                    // zero encoder

  // Primitive getters — the real reads, served from what tick() last sampled.
  virtual float position() const = 0;      // [mm]
  virtual float velocity() const = 0;      // [mm/s] signed
  virtual float appliedDuty() const = 0;   // [-1, 1]
  virtual bool connected() const = 0;
  virtual bool wedged() const = 0;

  // Faceplate verbs.
  virtual void configure(const msg::MotorConfig& config) = 0;
  virtual void tick(uint32_t now) = 0;   // [ms] sample encoder; run the active mode
  virtual msg::MotorCapabilities capabilities() const = 0;

  // Message plane — implemented ONCE in this base class on top of the
  // primitives: apply() validates vs capabilities(), unpacks the oneof, and
  // calls the matching setter; state() assembles a MotorState from the
  // getters. See the file-level comment for the bool-return deviation from
  // the issue's void sketch.
  bool apply(const msg::MotorCommand& command);
  msg::MotorState state() const;
};

// --- apply()/state(): the shared message plane, defined once here (inline
// so this header stays the only translation unit backing every Motor leaf;
// no capability/motor.cpp exists — see the sprint's Build acceptance
// criterion, which lists capability/ as headers-only). ---

inline bool Motor::apply(const msg::MotorCommand& command) {
  const msg::MotorCapabilities caps = capabilities();
  bool accepted = true;

  switch (command.get_control_kind()) {
    case msg::MotorCommand::ControlKind::DUTY_CYCLE:
      if (!caps.duty_cycle) { accepted = false; break; }
      setDutyCycle(command.control.duty_cycle);
      break;
    case msg::MotorCommand::ControlKind::VOLTAGE:
      if (!caps.voltage) { accepted = false; break; }
      setVoltage(command.control.voltage);
      break;
    case msg::MotorCommand::ControlKind::VELOCITY:
      if (!caps.velocity) { accepted = false; break; }
      setVelocity(command.control.velocity);
      break;
    case msg::MotorCommand::ControlKind::POSITION:
      if (!caps.position) { accepted = false; break; }
      setPosition(command.control.position);
      break;
    case msg::MotorCommand::ControlKind::NEUTRAL:
      // Neutral is always accepted — every motor must be able to go
      // neutral regardless of which drive modes it otherwise supports.
      setNeutral(command.control.neutral);
      break;
    case msg::MotorCommand::ControlKind::NONE:
    default:
      // No control arm set — e.g. a command whose only payload is
      // reset_position (DEV M <n> RESET). Nothing to dispatch here; the
      // reset/feedforward side-channels below still apply.
      break;
  }

  // Feedforward and reset_position ride beside whichever arm (or lack of
  // one) was just processed — never gated by capabilities(), since they
  // are not control modes themselves.
  if (command.get_feedforward().has) {
    setFeedforward(command.get_feedforward().val);
  }
  if (command.get_reset_position().has && command.get_reset_position().val) {
    resetPosition();
  }

  return accepted;
}

inline msg::MotorState Motor::state() const {
  msg::MotorState s;
  const msg::MotorCapabilities caps = capabilities();

  s.connected = connected();
  s.applied.has = true;
  s.applied.val = appliedDuty();

  // Position/velocity/wedged only mean something for a motor with an
  // encoder; a future capability-less leaf would leave these unset rather
  // than report a fabricated zero.
  if (caps.has_encoder) {
    s.position.has = true;
    s.position.val = position();
    s.velocity.has = true;
    s.velocity.val = velocity();
    s.wedged.has = true;
    s.wedged.val = wedged();
  }

  return s;
}

}  // namespace Hal
