// sim_plant.h -- TestSim::SimPlant: the ONE honest simulator I2C bus.
//
// Sprint 108 ticket 002 (clasi/issues/plan-pure-i2cbus-clock-interfaces-a-
// real-simplant-simulator.md, Stage 2). Ticket 001 reduced
// `Devices::I2CBus` to a pure interface (source/devices/i2c_bus.h);
// `SimPlant` is the SECOND concrete implementation, alongside
// `Devices::MicroBitI2CBus` (source/devices/microbit_i2c_bus.h) on the ARM
// side.
//
// This class RESPONDS to whatever bytes firmware actually put on the wire
// instead of PREDICTING them from a write count (the old
// tests/sim/support/sim_api.h `SimApi`/`DutyPredictor` desync bug this
// class replaces -- under an arbitrary twist stream the predictor and the
// firmware's actual write sequence could drift apart; a bus that just
// parses the real frame cannot desync).
//
// Two-layer split (architecture-update.md Decision 3): SimPlant owns the
// wire PROTOCOL only -- parsing the Nezha 0x60/0x46 frame and the OTOS
// register-pointer protocol, dispatching by 8-bit wire address, and
// packing/unpacking the exact byte layouts the real chips use. The PHYSICS
// (duty->velocity->position integration, wheel-position-to-pose
// accumulation) is reused VERBATIM from tests/sim/plant/{wheel,otos}_plant.
// {h,cpp} -- SimPlant owns two TestSim::WheelPlant (left = Nezha port 1,
// right = Nezha port 2) and one TestSim::OtosPlant, and calls only their
// physics-facing surface (step()/position()/velocity()/x()/y()/heading())
// -- never their scriptEncoderResponse()/scriptPoseResponse() helpers,
// which target the now-deleted scripted-FIFO I2CBus fake and do not apply
// here.
//
// The hook wrapper (architecture-update.md's "The hook (middleware, on
// SimPlant -- not on I2CBus)"): read()/write() (the Devices::I2CBus
// overrides) each check for a registered Python-facing hook and, if
// present, call IT instead of the default protocol handler; the hook may
// call defaultRead()/defaultWrite() itself for pass-through (full access to
// the plant's real response), or return without calling it to fully
// override or swallow the transaction. defaultRead()/defaultWrite() never
// re-enter the hook -- there is no recursion path.
//
//   int SimPlant::read(addr, data, len, ...)  { return readHook_  ? readHook_(addr, data, len)  : defaultRead(addr, data, len); }
//   int SimPlant::write(addr, data, len, ...) { return writeHook_ ? writeHook_(addr, data, len) : defaultWrite(addr, data, len); }
//
// Intended ctypes bridge (ticket 005, NOT built here): `ReadHook`/
// `WriteHook` below are `std::function<int(uint16_t,uint8_t*,int)>` -- fine
// for pure C++ callers (this ticket's own standalone driver, ticket 003's
// sim_harness). Ticket 005's `sim_ctypes.cpp` bridges a Python-registered
// hook by wrapping a flat C callback shape
// (`int(*)(void* ctx, uint16_t addr, uint8_t* data, int len)` + a `void*
// ctx`) in a lambda capturing the callback pointer and ctx, and passes THAT
// lambda to setReadHook()/setWriteHook() -- this class never needs to know
// about ctypes/Python at all. The matching pass-through exports
// (`sim_default_read`/`sim_default_write`) are thin call-throughs straight
// to defaultRead()/defaultWrite() below, which is why those two methods
// are public rather than private-plus-friend.
//
// Source placement: HOST_BUILD-only test infrastructure -- this file does
// NOT live in source/ (architecture-update.md Decision 2, "source/ holds
// only interfaces + ARM impls").
#pragma once

#include <cstdint>
#include <functional>

#include "devices/i2c_bus.h"
#include "otos_plant.h"
#include "wheel_plant.h"

namespace TestSim {

// Ship-default wheelbase used to construct this SimPlant's own OtosPlant --
// matches the TestGUI's own default trackwidth (testgui-revival program,
// simset-max-args-truncation.md's "GUI trackwidth default 128"). A caller
// composing SimPlant against a differently-configured App::Odometry MUST
// pass the matching trackWidth to the constructor -- see OtosPlant's own
// header comment on why the two must agree.
constexpr float kDefaultTrackWidth = 128.0f;  // [mm]

class SimPlant : public Devices::I2CBus {
 public:
  // addr/data/len only -- no repeated/preClear/postClear parameter. Those
  // three exist on Devices::I2CBus::write()/read() purely to schedule real-
  // bus clearance timing (MicroBitI2CBus's own concern); SimPlant has no
  // clearance timers (clearanceSafetyNetCount() below always returns 0), so
  // a hook has nothing useful to do with them. See this file's header for
  // the intended ctypes bridge shape.
  using ReadHook = std::function<int(uint16_t address, uint8_t* data, int len)>;
  using WriteHook = std::function<int(uint16_t address, uint8_t* data, int len)>;

  explicit SimPlant(float trackWidth = kDefaultTrackWidth);

  // ---- Devices::I2CBus overrides -- hook wrappers only, see file header ----
  int write(uint16_t address, uint8_t* data, int len, bool repeated = false,
            uint32_t preClear = 0, uint32_t postClear = 0) override;
  int read(uint16_t address, uint8_t* data, int len, bool repeated = false,
           uint32_t preClear = 0, uint32_t postClear = 0) override;

  // SimPlant never trips a real bus's clearance safety net -- there is no
  // spinning wait here to trip.
  uint32_t clearanceSafetyNetCount() const override { return 0; }

  // ---- Default (non-hooked) protocol handlers -- public so a registered
  // hook (or this ticket's own standalone driver) can call them directly
  // for pass-through WITHOUT re-entering the hook. See file header. ----
  int defaultWrite(uint16_t address, uint8_t* data, int len);
  int defaultRead(uint16_t address, uint8_t* data, int len);

  // Steps both WheelPlants (from their last wire-parsed duty) and the
  // OtosPlant (from the two wheel plants' resulting positions) by dt of
  // virtual time. Called once per cycle by the harness (ticket 003) --
  // SimPlant never ticks itself.
  void tick(float dt);  // [s]

  void setReadHook(ReadHook hook) { readHook_ = std::move(hook); }
  void setWriteHook(WriteHook hook) { writeHook_ = std::move(hook); }
  void clearReadHook() { readHook_ = nullptr; }
  void clearWriteHook() { writeHook_ = nullptr; }

  // ---- Fault-injection knobs -- plain SimPlant/plant-owned methods, NOT
  // on Devices::I2CBus (architecture-update.md Decision 1). port: 1 = left
  // (matches Nezha motorId 1), 2 = right (motorId 2) -- the same port
  // numbering the real Nezha frame's byte [2] carries. ----
  void setDisconnected(int port, bool disconnected);
  void freezePosition(int port, bool freeze);
  void setDropoutRate(int port, float fraction);  // [0,1]

  // Rest-encoder jitter (108-011) -- fans out to BOTH WheelPlants (left and
  // right); there is no per-port knob here, unlike the three fault-injection
  // knobs above, because jitter is a plant-fidelity default for a whole
  // SimPlant, not a scenario-scripted per-wheel fault. See
  // WheelPlant::setEncoderJitter()'s own comment (wheel_plant.h) for why
  // this defaults OFF and who turns it on.
  void setEncoderJitter(bool enabled);

  // OTOS drift/bias -- deterministic, per OtosPlant::setDrift()'s own
  // comment (no RNG anywhere in either plant).
  void setOtosDrift(float xDrift, float yDrift, float headingDrift);  // [mm] [mm] [rad]

  // Plant teleport (sim command-surface fix): snaps the OtosPlant's ground-
  // truth pose to (x, y, heading) AND resets both WheelPlants' positions to
  // 0 in the same call -- see OtosPlant::reset()'s own comment for why the
  // two resets must be coupled (a wheel-position reset with no matching
  // OtosPlant re-baseline, or vice versa, injects a phantom one-cycle jump
  // on the very next tick()). This is what backs the TestGUI Sim mode's
  // "reset to origin"/SI-verb pose reset (host/robot_radio/io/sim_loop.py's
  // set_true_pose()) -- there is no operator to physically place the robot
  // the way real-hardware "Set Robot @ 0,0" assumes.
  void setTruePose(float x, float y, float heading);  // [mm] [mm] [rad]

  // Read-only accessors, mainly for tests/a future harness's true-pose
  // export.
  const WheelPlant& wheelPlant(int port) const;
  const OtosPlant& otosPlant() const { return otos_; }

 private:
  WheelPlant& mutableWheelPlant(int port);

  int handleMotorWrite(uint8_t* data, int len);
  int handleMotorRead(uint8_t* data, int len);
  int handleOtosWrite(uint8_t* data, int len);
  int handleOtosRead(uint8_t* data, int len);

  WheelPlant left_;
  WheelPlant right_;
  OtosPlant otos_;

  // Wire-parsed duty, off the 0x60 frame only -- NEVER read back from
  // Devices::NezhaMotor::appliedDuty() (no back-channel of any kind, per
  // this ticket's own acceptance criteria).
  float leftDuty_ = 0.0f;   // [-1,1]
  float rightDuty_ = 0.0f;  // [-1,1]

  // The port most recently selected by a 0x46 encoder-select write --
  // defaultRead()'s motor branch reports THIS port's encoder. Nezha
  // motorId 1 (left) matches this class's own reset default.
  int selectedPort_ = 1;

  // OTOS register pointer most recently written -- decides what
  // defaultRead()'s OTOS branch returns.
  uint8_t otosRegPtr_ = 0;

  ReadHook readHook_;
  WriteHook writeHook_;
};

}  // namespace TestSim
