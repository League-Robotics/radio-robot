// motor.h — Devices::Motor: the pure motor interface every consumer takes.
//
// Stakeholder design (2026-07-18): "either the armor has exactly a motor,
// or … the armor has precisely a motor interface … it composes a motor. If
// you want the armor, you construct a motor, then you give it to the motor
// armor, and then you give the motor armor to the thing that's looking for
// a motor. If you don't want the armor, you construct the motor and you
// give it to the thing that wants the motor directly."
//
// This is that interface. Two implementations exist:
//   - Devices::NezhaMotor (nezha_motor.h) — the bare concrete leaf: register
//     map, split-phase encoder sequencing, velocity PID, and its OWN
//     device-intrinsic write shaping (slew cap, write throttle,
//     write-on-change, reversal dwell, output deadband — all Nezha-brick
//     protection, see nezha_motor.cpp).
//   - Devices::MotorArmor (motor_armor.h) — a decorator: composes a Motor&,
//     forwards everything, and adds the observation/recovery policies
//     (wedge detection, standstill-guarded resets).
//
// The sim composes the bare NezhaMotor directly (src/sim/sim_harness.h —
// no armor in the loop); the ARM build wraps each NezhaMotor in a
// MotorArmor (src/firm/main.cpp) before handing it to the app graph.
//
// Surface: exactly the union of what the app graph calls today —
// App::Drive (setVelocity), App::Odometry (position/velocity),
// App::Preamble (begin), App::RobotLoop (requestSample/tick/
// position/velocity/connected/wedged/gains/applyGains) — plus the raw-duty
// and reset verbs the bench/test surface uses. wedged()/wedgeSuspect()
// default to false so a bare (armor-less) motor is honest: nothing is
// watching for a wedge. (App::HeadingSource, a former position/velocity
// consumer, is DELETED -- 115-002, gut-to-minimal-firmware S1 motion-stack
// excision.)
#pragma once

#include <cstdint>

#include "devices/device_config.h"
#include "devices/device_types.h"

namespace Devices {

class Motor {
 public:
  virtual ~Motor() = default;

  // Primes the device (e.g. the Nezha 0x46 encoder register sits frozen at
  // 0 until its first atomic read). Called once per motor by the preamble
  // before the cycle starts.
  virtual void begin() = 0;

  // Cycle-level "prepare this cycle's sample" hook (split-phase phase 1 on
  // the Nezha — the 0x46 select write). The loop calls this, waits the
  // device's settle window, then tick() collects.
  virtual void requestSample() = 0;

  // --- Command staging — tick() executes. ---
  virtual void setVelocity(float velocity) = 0;   // [mm/s] signed
  virtual void setDuty(float duty) = 0;           // [-1, 1] raw duty
  virtual void setNeutral(Neutral mode) = 0;
  virtual void setPidEnabled(bool on) = 0;

  // Live gain-apply / read-back (RobotLoop's CONFIG merge path).
  virtual void applyGains(const Gains& gains, Opt<float> travelCalib = {}) = 0;
  virtual const Gains& gains() const = 0;

  // reconfigure — REVISION 1 (114-001): a guarded, post-construction,
  // WHOLE-config replacement (port/fwdSign/velGains/velFiltAlpha/slewRate/
  // wheelTravelCalib/reversalDwell/outputDeadband — every MotorConfig
  // field). This is NOT applyGains()'s live wire CFG-patch surface above —
  // applyGains() is the always-safe, narrow, bounded-retuning path
  // RobotLoop::handleConfig() uses at any time; reconfigure() replaces
  // fields (fwdSign, port) that are a runaway-direction hazard if flipped
  // mid-drive, so it is guarded: an implementation must refuse (return
  // false, leave its config unchanged) unless the motor has never yet been
  // commanded or is independently verified at rest, and must succeed
  // otherwise. Exists so a composition root that constructs a motor before
  // its real configuration is known (TestSim::SimHarness, whose bare
  // NezhaMotor starts at Devices::MotorConfig{}'s all-zero default) can
  // still reach a genuinely working motor once configuration arrives — see
  // sprint 114's sprint.md Architecture Revision 1 / Decision 6 for the
  // full rationale.
  [[nodiscard]] virtual bool reconfigure(const MotorConfig& config) = 0;

  virtual void tick(uint64_t nowUs) = 0;   // [us]

  // --- Getters ---
  virtual float position() const = 0;        // [mm]
  virtual float velocity() const = 0;        // [mm/s] signed, filtered
  virtual float velocityTarget() const = 0;  // [mm/s] signed — last setVelocity()
  virtual float appliedDuty() const = 0;     // [-1, 1] last successfully written
  virtual bool connected() const = 0;

  // --- Resets ---
  // Bare motor: resetPosition() acts IMMEDIATELY (the caller owns any
  // at-rest discipline). MotorArmor overrides it with the staged,
  // standstill-guarded dispatch (hard at verified rest, rebaseline
  // otherwise).
  virtual void resetPosition() = 0;
  virtual void rebaseline() = 0;   // software-only re-anchor; no bus traffic

  // --- Observability — armor-provided; a bare motor reports false (nothing
  // is watching). ---
  virtual bool wedged() const { return false; }
  virtual bool wedgeSuspect() const { return false; }
};

}  // namespace Devices
