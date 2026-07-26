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
//     map, split-phase encoder sequencing, and its OWN device-intrinsic
//     write shaping (slew cap, write throttle, write-on-change, reversal
//     dwell, output deadband — all Nezha-brick protection, see
//     nezha_motor.cpp).
//   - Devices::MotorArmor (motor_armor.h) — a decorator: composes a Motor&,
//     forwards everything, and adds the observation/recovery policies
//     (wedge detection, standstill-guarded resets).
//
// The sim composes the bare NezhaMotor directly (src/sim/sim_harness.h —
// no armor in the loop); the ARM build wraps each NezhaMotor in a
// MotorArmor (src/firm/main.cpp) before handing it to the app graph.
//
// 125-003 (sprint 125 Decision 2, "protection vs. control" — see
// sprint.md): the staged-velocity setter, its target-readback accessor,
// the gain-set getter, and the gain-bearing half of the old gain-apply
// method are DELETED from this interface — the velocity control law (a
// control DECISION, not hardware protection) moved to the motion library
// (relocated from its old devices-local home). `setDuty()` is now the
// ONLY command-staging verb: every Motor takes a duty target and reports
// what the encoder actually said, nothing else. The old gain-apply method
// narrows to applyTravelCalib(float) — the one MotorConfig field this
// interface still live-applies (calibration, not control); gain routing
// (kp/ki/kff/iMax/kaw) is ticket 008's CONFIG-routing-split job, landing
// on whoever owns the relocated control law. The PID-enable toggle and its
// readback are deleted outright — there is no PID here to enable or
// disable.
//
// Surface: exactly the union of what the app graph calls today —
// App::Drive (setDuty), App::Odometry (position/velocity),
// App::Preamble (begin), App::RobotLoop (requestSample/tick/
// position/velocity/connected/wedged/applyTravelCalib) — plus the raw-duty
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

  // --- Command staging — tick() executes. The ONLY staging verb left
  // (125-003: the staged-velocity setter and the embedded PID it drove are
  // gone — see this file's own header). ---
  virtual void setDuty(float duty) = 0;           // [-1, 1] raw duty
  virtual void setNeutral(Neutral mode) = 0;

  // Live travel-calibration apply (RobotLoop's CONFIG merge path, side-
  // selected — config.proto's MotorConfigPatch.side). Narrowed from the
  // pre-125-003 applyGains(Gains, Opt<float>) — the ONE MotorConfig field
  // this interface still live-applies; gain routing moved out with the PID
  // (this file's own header).
  virtual void applyTravelCalib(float travelCalib) = 0;

  // reconfigure — REVISION 1 (114-001): a guarded, post-construction,
  // WHOLE-config replacement (port/fwdSign/slewRate/wheelTravelCalib/
  // reversalDwell/outputDeadband — every MotorConfig field). This is NOT
  // applyTravelCalib()'s live wire CFG-patch surface above — that is the
  // always-safe, narrow, bounded-retuning path RobotLoop::handleConfig()
  // uses at any time; reconfigure() replaces fields (fwdSign, port) that
  // are a runaway-direction hazard if flipped mid-drive, so it is guarded:
  // an implementation must refuse (return false, leave its config
  // unchanged) unless the motor has never yet been commanded or is
  // independently verified at rest, and must succeed otherwise. Exists so
  // a composition root that constructs a motor before its real
  // configuration is known (TestSim::SimHarness, whose bare NezhaMotor
  // starts at Devices::MotorConfig{}'s all-zero default) can still reach a
  // genuinely working motor once configuration arrives — see sprint 114's
  // sprint.md Architecture Revision 1 / Decision 6 for the full rationale.
  [[nodiscard]] virtual bool reconfigure(const MotorConfig& config) = 0;

  virtual void tick(uint64_t nowUs) = 0;   // [us]

  // --- Getters ---
  virtual float position() const = 0;        // [mm]
  virtual float velocity() const = 0;        // [mm/s] signed -- 125-003: a
                                              // naive per-tick difference
                                              // quotient now (the freshness
                                              // gate/glitch rejection/EMA-
                                              // or-line-fit estimator pair
                                              // are DELETED, not replaced,
                                              // pending ticket 004's
                                              // App::WheelObserver -- see
                                              // nezha_motor.cpp's own tick()
                                              // comment)
  virtual float appliedDuty() const = 0;     // [-1, 1] last successfully written
  virtual bool connected() const = 0;

  // The nowUs of the tick() call that produced the CURRENTLY-cached
  // position()/velocity() reading — the last ACCEPTED fresh sample's own
  // timestamp, never "now" at call time (NezhaMotor's lastFreshUs_ is the
  // reference implementation). Lets a caller compute a real per-sample age
  // instead of stamping every reading with the same cycle's now — see
  // protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md §B2.
  virtual uint64_t sampleTime() const = 0;  // [us]

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
