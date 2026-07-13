// handles.h — Devices::Motor / ColorSensor / LineSensor / Odometer: the
// per-device handle classes DeviceBus (device_bus.h) hands out.
//
// Ticket DB-007 (device-bus-tickets.md). Implements clasi/issues/
// device-bus-fiber-owned-self-contained-device-subsystem.md's "The public
// surface" handle sketch exactly: private constructors, `friend class
// DeviceBus`, non-copyable, returned BY REFERENCE from DeviceBus's
// motor()/color()/line()/odometer() getters. Header-only (no handles.cpp) —
// every method is defined inline right here, mirroring motor_armor.h's own
// "small concrete accessors... defined inline here (headers-only, matching
// the original's own 'no capability/motor.cpp exists' precedent)" — these
// handle methods are just as small (thin delegation to a leaf/ring
// reference), so the same precedent applies.
//
// --- The one contract every handle honors (issue "The public surface") ---
// Getters serve the MOST RECENT PUBLISHED sample and NEVER touch the bus:
// latest()/sample(age)/sampleAt(t)/updatedAt()/connected() are all plain
// reads of a MeasurementRing<T> (DB-002) or a cheap leaf accessor
// (connected()) — no I2C, no yield, matching the concurrency contract's
// rule 2 ("No yield inside... a consumer-side sample copy"). Setters STAGE a
// request the fiber applies at its next cycle top; see each handle's own
// section below for exactly what "staged" means for that handle (it is NOT
// the same mechanism for every handle — see the Motor section's design note).
//
// --- Renamed leaf classes (DB-006 -> DB-007) ---
// ColorSensorLeaf/LineSensorLeaf are DB-006's ColorSensor/LineSensor leaves,
// renamed in this ticket to free up the bare `ColorSensor`/`LineSensor`
// names for the HANDLE classes below (the issue's public-surface sketch
// names both the OTOS handle `Odometer` and the color/line HANDLES
// `ColorSensor`/`LineSensor` — the same names DB-006 had already given its
// LEAVES, an unavoidable collision since both tickets independently followed
// the issue's own vocabulary). See color_sensor.h's "Renamed ColorSensor ->
// ColorSensorLeaf in DB-007" note for the full reasoning. NezhaMotor (DB-004)
// and Otos (DB-005) never collided with their handle names (`Motor`/
// `Odometer`) because they already carried a distinct vendor/chip name.
#pragma once

#include <cstdint>

#include "devices/clock.h"
#include "devices/color_sensor.h"
#include "devices/device_types.h"
#include "devices/interpolation.h"
#include "devices/line_sensor.h"
#include "devices/measurement_ring.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"

namespace Devices {

class DeviceBus;  // handles are constructed only by DeviceBus (friend, below)

// lerpUint — the integer-reading counterpart to interpolation.h's float
// lerp(), for ColorReading/LineReading's uint32_t channel fields (raw ADC
// counts — see device_types.h). Rounds to nearest, clamped at 0 (frac is
// already clamped to [0,1] by lerpFraction(), and both endpoints are
// non-negative, so the only real edge case is round-to-nearest at frac==0/1
// landing exactly on `older`/`newer`).
inline uint32_t lerpUint(uint32_t older, uint32_t newer, float frac) {
  float v = lerp(static_cast<float>(older), static_cast<float>(newer), frac);
  if (v < 0.0f) v = 0.0f;
  return static_cast<uint32_t>(v + 0.5f);
}

// ---------------------------------------------------------------------------
// Motor — the differential-drive wheel handle (NezhaMotor leaf, DB-004).
//
// --- Design note: setters are THIN PASSTHROUGHS onto the leaf's own staged
// fields, not a second duplicate staging layer ---
// NezhaMotor::setVelocity()/setDuty()/setNeutral()/setPidEnabled()/
// resetPosition() (nezha_motor.h/.cpp, DB-004) are ALREADY exactly "stage a
// command; tick() executes it" — velocityTarget_/dutyTarget_/mode_/
// pidEnabled_/resetPending_ are plain fields NezhaMotor's own tick() reads
// at the top of its next call. A caller-context store into those fields
// touches no bus and has no yield, so it satisfies the concurrency
// contract's staged-input rule (rule 2) exactly as well as a second,
// DeviceBus-owned staging cell would — duplicating that state here would add
// indirection without adding safety (this mirrors Odometer::setPose()
// below, which relays directly into Otos's own posePending_ cell for the
// identical reason). The ONE piece of state this handle adds beyond a pure
// passthrough is velocityStaged_/velocityStagedUs_ — NezhaMotor has no
// notion of "when was I last told a velocity", and DeviceBus's
// drainStagedInputs() (device_bus.cpp) needs exactly that timestamp to
// implement the stale-target/RX-watchdog neutralize gate (device-bus-
// tickets.md's DB-007 acceptance criteria).
class Motor {
 public:
  // --- Setters: stage the command (via the leaf's own fields); the
  // fiber's next collect+PID+write step (DeviceBus::collectAndDrive(),
  // device_bus.cpp) executes it. ---
  void setVelocity(float velocity);  // [mm/s] signed — PID target
  void setDuty(float duty);          // [-1, 1] raw duty target (PID-off only)
  void setNeutral(Neutral mode);     // coast / brake
  void resetPosition();              // zero encoder (staged, at-rest-guarded)
  void setPidEnabled(bool on);       // default true — armor applies in both modes

  // --- Getters: latest published sample, never touch the bus. ---
  Sample<MotorReading> latest() const { return ring_.latest(); }
  Sample<MotorReading> sample(uint8_t age) const { return ring_.sample(age); }
  bool sampleAt(uint64_t t, MotorReading& out) const;  // [us] bracketed lerp
  uint64_t updatedAt() const { return ring_.latest().stamp; }  // [us]
  bool connected() const { return leaf_.connected(); }

  // --- Diagnostics pass-through (issue "The public surface"). ---
  bool wedged() const { return leaf_.wedged(); }
  bool wedgeSuspect() const { return leaf_.wedgeSuspect(); }
  uint32_t encGlitchCount() const { return leaf_.encGlitchCount(); }

 private:
  Motor(NezhaMotor& leaf, MeasurementRing<MotorReading>& ring, const Clock& clock)
      : leaf_(leaf), ring_(ring), clock_(clock) {}
  Motor(const Motor&) = delete;
  Motor& operator=(const Motor&) = delete;
  friend class DeviceBus;

  NezhaMotor& leaf_;
  MeasurementRing<MotorReading>& ring_;
  const Clock& clock_;

  // Stale-target/RX-watchdog bookkeeping (read by DeviceBus::
  // drainStagedInputs()/applyStaleGate(), friend access) — see this class's
  // own design note above. velocityStaged_ becomes true on the first
  // setVelocity() call and stays true (re-armed on every subsequent
  // setVelocity()) until an explicit setNeutral() cancels it; it is NOT
  // cleared by the watchdog firing (device_bus.cpp's applyStaleGate() keeps
  // re-asserting neutral every cycle for as long as it stays stale — see
  // that function's own comment for why that is the more robust choice).
  bool velocityStaged_ = false;
  uint64_t velocityStagedUs_ = 0;  // [us] time of the most recent setVelocity() call
};

inline void Motor::setVelocity(float velocity) {
  leaf_.setVelocity(velocity);
  velocityStaged_ = true;
  velocityStagedUs_ = clock_.nowMicros();
}

inline void Motor::setDuty(float duty) { leaf_.setDuty(duty); }

inline void Motor::setNeutral(Neutral mode) {
  leaf_.setNeutral(mode);
  velocityStaged_ = false;  // explicit neutral command cancels the velocity watchdog
}

inline void Motor::resetPosition() { leaf_.resetPosition(); }

inline void Motor::setPidEnabled(bool on) { leaf_.setPidEnabled(on); }

inline bool Motor::sampleAt(uint64_t t, MotorReading& out) const {
  Sample<MotorReading> older, newer;
  if (!ring_.bracket(t, older, newer)) return false;
  float frac = lerpFraction(older.stamp, newer.stamp, t);
  out.position = lerp(older.value.position, newer.value.position, frac);
  out.velocity = lerp(older.value.velocity, newer.value.velocity, frac);
  out.appliedDuty = lerp(older.value.appliedDuty, newer.value.appliedDuty, frac);
  return true;
}

// ---------------------------------------------------------------------------
// ColorSensor — the RGBC handle (ColorSensorLeaf leaf, DB-006). No setters —
// this leaf has no staged command surface (its detection/read cadence is
// entirely DeviceBus-scheduled, see device_bus.cpp's perceptionSlotStep()).
// ---------------------------------------------------------------------------
class ColorSensor {
 public:
  Sample<ColorReading> latest() const { return ring_.latest(); }
  Sample<ColorReading> sample(uint8_t age) const { return ring_.sample(age); }
  bool sampleAt(uint64_t t, ColorReading& out) const;  // [us] bracketed lerp
  uint64_t updatedAt() const { return ring_.latest().stamp; }  // [us]
  bool connected() const { return leaf_.connected(); }

 private:
  ColorSensor(ColorSensorLeaf& leaf, MeasurementRing<ColorReading>& ring)
      : leaf_(leaf), ring_(ring) {}
  ColorSensor(const ColorSensor&) = delete;
  ColorSensor& operator=(const ColorSensor&) = delete;
  friend class DeviceBus;

  ColorSensorLeaf& leaf_;
  MeasurementRing<ColorReading>& ring_;
};

inline bool ColorSensor::sampleAt(uint64_t t, ColorReading& out) const {
  Sample<ColorReading> older, newer;
  if (!ring_.bracket(t, older, newer)) return false;
  float frac = lerpFraction(older.stamp, newer.stamp, t);
  out.r = lerpUint(older.value.r, newer.value.r, frac);
  out.g = lerpUint(older.value.g, newer.value.g, frac);
  out.b = lerpUint(older.value.b, newer.value.b, frac);
  out.c = lerpUint(older.value.c, newer.value.c, frac);
  return true;
}

// ---------------------------------------------------------------------------
// LineSensor — the 4-channel handle (LineSensorLeaf leaf, DB-006). No
// setters, same reasoning as ColorSensor above.
// ---------------------------------------------------------------------------
class LineSensor {
 public:
  Sample<LineReading> latest() const { return ring_.latest(); }
  Sample<LineReading> sample(uint8_t age) const { return ring_.sample(age); }
  bool sampleAt(uint64_t t, LineReading& out) const;  // [us] bracketed lerp
  uint64_t updatedAt() const { return ring_.latest().stamp; }  // [us]
  bool connected() const { return leaf_.connected(); }

 private:
  LineSensor(LineSensorLeaf& leaf, MeasurementRing<LineReading>& ring)
      : leaf_(leaf), ring_(ring) {}
  LineSensor(const LineSensor&) = delete;
  LineSensor& operator=(const LineSensor&) = delete;
  friend class DeviceBus;

  LineSensorLeaf& leaf_;
  MeasurementRing<LineReading>& ring_;
};

inline bool LineSensor::sampleAt(uint64_t t, LineReading& out) const {
  Sample<LineReading> older, newer;
  if (!ring_.bracket(t, older, newer)) return false;
  float frac = lerpFraction(older.stamp, newer.stamp, t);
  for (int i = 0; i < 4; ++i) {
    out.raw[i] = lerpUint(older.value.raw[i], newer.value.raw[i], frac);
    out.normalized[i] = lerpUint(older.value.normalized[i], newer.value.normalized[i], frac);
  }
  return true;
}

// ---------------------------------------------------------------------------
// Odometer — the OTOS pose handle (Otos leaf, DB-005).
//
// setPose() relays DIRECTLY onto Otos::setPose() (otos.h/.cpp) — that leaf
// already implements exactly "stage an (x,y,heading) re-anchor; touch no
// bus; the next tick() drains it first, ahead of its periodic read" (see
// otos.h's own "Staged setPose() re-anchor" file-header section). This IS
// the issue's "staged setPose() re-anchor request ... drained by the fiber
// at a safe slot" — Otos::tick() is only ever called by DeviceBus's fiber
// (device_bus.cpp's perceptionSlotStep(), at this handle's designated
// round-robin turn), so "the fiber drains it at a safe slot" falls
// straight out of tick()'s own call-order with no extra DeviceBus-level
// staging cell needed — the identical reasoning Motor's own design note
// above gives for its passthrough setters.
// ---------------------------------------------------------------------------
class Odometer {
 public:
  void setPose(float x, float y, float heading) {  // [mm] [mm] [rad]
    leaf_.setPose(x, y, heading);
  }

  Sample<PoseReading> latest() const { return ring_.latest(); }
  Sample<PoseReading> sample(uint8_t age) const { return ring_.sample(age); }
  bool sampleAt(uint64_t t, PoseReading& out) const;  // [us] bracketed lerp (wrap-aware heading)
  uint64_t updatedAt() const { return ring_.latest().stamp; }  // [us]
  bool connected() const { return leaf_.connected(); }

 private:
  Odometer(Otos& leaf, MeasurementRing<PoseReading>& ring)
      : leaf_(leaf), ring_(ring) {}
  Odometer(const Odometer&) = delete;
  Odometer& operator=(const Odometer&) = delete;
  friend class DeviceBus;

  Otos& leaf_;
  MeasurementRing<PoseReading>& ring_;
};

inline bool Odometer::sampleAt(uint64_t t, PoseReading& out) const {
  Sample<PoseReading> older, newer;
  if (!ring_.bracket(t, older, newer)) return false;
  float frac = lerpFraction(older.stamp, newer.stamp, t);
  out.x = lerp(older.value.x, newer.value.x, frac);
  out.y = lerp(older.value.y, newer.value.y, frac);
  // heading is the issue's flagged wrap-aware case (interpolation.h's own
  // lerpAngle() header comment) -- every other PoseReading field is an
  // ordinary linear quantity.
  out.heading = lerpAngle(older.value.heading, newer.value.heading, frac);
  out.v_x = lerp(older.value.v_x, newer.value.v_x, frac);
  out.v_y = lerp(older.value.v_y, newer.value.v_y, frac);
  out.omega = lerp(older.value.omega, newer.value.omega, frac);
  return true;
}

}  // namespace Devices
