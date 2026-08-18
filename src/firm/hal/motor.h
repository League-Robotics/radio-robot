// motor.h — Hal::Motor: the pure motor interface every consumer takes.
// Either construct a bare motor and hand it directly to whatever wants a
// Motor, or wrap it in the armor decorator and hand that along instead —
// the armor composes a Motor&, it is not a subclass of one.
//
// Two implementations exist:
//   - Hardware::NezhaMotor (nezha_motor.h) — the bare concrete leaf: register
//     map, split-phase encoder sequencing, and its OWN device-intrinsic
//     write shaping (slew cap, write throttle, write-on-change, reversal
//     dwell, output deadband — all Nezha-brick protection, see
//     nezha_motor.cpp).
//   - Hardware::MotorArmor (motor_armor.h) — a decorator: composes a Motor&,
//     forwards everything, and adds the observation/recovery policies
//     (wedge detection, standstill-guarded resets).
//
// The sim composes the bare NezhaMotor directly (src/firm/platform/host/sim_harness.h —
// no armor in the loop); the ARM build wraps each NezhaMotor in a
// MotorArmor (src/firm/main.cpp) before handing it to the app graph.
//
// `setDuty()` is the ONLY command-staging verb: every Motor takes a duty
// target and reports what the encoder actually said, nothing else --
// there is no velocity control law or PID here; that lives in
// Control::DifferentialDrive (the wheel kernel), the sole consumer.
//
// COUNTS-NATIVE (exploratory kernel, 2026-08-15): position()/velocity()
// report shaft encoder counts / counts-per-second (1 count = 0.1 deg, the
// Nezha 0x46 register's own unit). The old applyTravelCalib() mm seam is
// deleted — no mm value exists at or below this interface; travel
// calibration (`travel_calib` in the robot JSON) belongs to the
// application layer. wedged()/wedgeSuspect() default to false so a bare
// (armor-less) motor is honest: nothing is watching for a wedge.
#pragma once

#include <cstdint>

#include "hal/device_config.h"
#include "hal/device_types.h"

namespace Hal {

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
  virtual void setDuty(float duty) = 0;           // [-1, 1] raw duty
  virtual void setNeutral(Neutral mode) = 0;

  // emergencyStop() — write zero to the device NOW, unstaged.
  //
  // This is the ONE method on this interface that both stages AND
  // executes. setDuty(0) would not do: it only STAGES, and the thing that
  // executes a stage is tick(), which is called by the kernel fiber — so
  // if the kernel fiber is the thing that died, a staged zero is never
  // written and the brick keeps its last latched speed forever (the
  // measured 936 mm runaway class; the Nezha brick does not even reset on
  // an nRF52 reset).
  //
  // Zero is never shaped anyway — it already short-circuits reversal
  // dwell, the write throttle, slew and the dedupe cache — so this adds
  // no new bypass, it only removes the dependency on a healthy tick().
  //
  // EMERGENCY ONLY. It is the single sanctioned exception to "the kernel
  // fiber is the only 0x10 client": called from the main fiber, it can
  // land between a 0x46 select and its read and destroy that pending
  // encoder sample. That is the right trade when the alternative is a
  // robot driving with nobody in control, and it is safe to interleave
  // because CODAL I2C transactions are fiber-atomic.
  virtual void emergencyStop() = 0;

  // applyTravelCalib() is DELETED (counts-native leaf — this file's own
  // header). No Config::Robot-consuming configure() on THIS interface
  // either (132-007, the-configuration-object.md): the devices isolation
  // invariant (DESIGN.md §3) forbids devices/ from naming a Config:: type
  // at all — Core::-layer helpers convert and call reconfigure() below.

  // reconfigure — a guarded, post-construction, WHOLE-config replacement
  // (port/fwdSign/slewRate/reversalDwell/outputDeadband/writeThrottle
  // — every MotorConfig field). reconfigure() replaces fields (fwdSign,
  // port) that are a
  // runaway-direction hazard if flipped mid-drive, so it is guarded: an
  // implementation must refuse (return false, leave its config unchanged)
  // unless the motor has never yet been commanded or is independently
  // verified at rest, and must succeed otherwise. Exists so a composition
  // root that constructs a motor before its real configuration is known
  // (TestSim::SimHarness, whose bare NezhaMotor starts at
  // Hal::MotorConfig{}'s all-zero default) can still reach a
  // genuinely working motor once configuration arrives.
  [[nodiscard]] virtual bool reconfigure(const MotorConfig& config) = 0;

  virtual void tick(uint64_t nowUs) = 0;   // [us]

  // DBG fault injection (system test, ROBOT_DEBUG builds): force the wedge
  // latch so the whole wedge-response machinery runs against an induced
  // fault. Default no-op -- only MotorArmor implements it; the wire path
  // that calls it is compiled out of shipped images.
  virtual void setForcedWedge(bool) {}

  // --- Getters ---
  virtual float position() const = 0;        // [counts] shaft, accumulated
  virtual float velocity() const = 0;        // [counts/s] signed -- a naive
                                              // per-tick difference
                                              // quotient: no freshness
                                              // gate, glitch rejection, or
                                              // smoothing (see
                                              // nezha_motor.cpp's own
                                              // tick() comment)
  virtual float appliedDuty() const = 0;     // [-1, 1] last successfully written
  virtual bool connected() const = 0;

  // The nowUs of the tick() call that produced the CURRENTLY-cached
  // position()/velocity() reading — the last ACCEPTED fresh sample's own
  // timestamp, never "now" at call time. IMPLEMENTED (131-002 — this
  // comment used to promise a lastFreshUs_ field that did not yet exist;
  // it now does): NezhaMotor::lastFreshUs_ (nezha_motor.h) advances only
  // when that tick's collectEncoder() actually succeeded, and
  // NezhaMotor::sampleTime() returns exactly that field, not every
  // tick() call's own nowUs. Lets a caller compute a real per-sample age
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

}  // namespace Hal
