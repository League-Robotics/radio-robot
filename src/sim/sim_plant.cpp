#include "sim_plant.h"

#include <cmath>

namespace TestSim {

namespace {
// ---------------------------------------------------------------------------
// Wire constants -- duplicated from the real device leaves' own private
// register-map constants (source/devices/nezha_motor.{h,cpp},
// source/devices/otos.h), the SAME per-file duplication convention
// tests/sim/plant/otos_plant.cpp's own kPosMmPerLsb/kHdgRadPerLsb already
// established ("this codebase's established per-file fixture-duplication
// convention, NOT a second, independently-derived formula"). SimPlant is
// the one place a NAK'd probe or a malformed frame is reasoned about
// without touching physics code (architecture-update.md Decision 3) -- it
// duplicates the WIRE FORMAT for the same reason OtosPlant duplicates the
// LSB scale factors: no dependency from tests/_infra/sim/ onto the private
// internals of a source/devices/ leaf class.
// ---------------------------------------------------------------------------

// Nezha motor-controller channel -- source/devices/nezha_motor.h.
constexpr uint8_t kNezhaDeviceAddr = 0x10;                                    // 7-bit
constexpr uint16_t kMotorWireAddr = static_cast<uint16_t>(kNezhaDeviceAddr << 1);
constexpr uint8_t kNezhaCmdRun = 0x60;
constexpr uint8_t kNezhaCmdEncoderSelect = 0x46;
constexpr uint8_t kNezhaDirCw = 1;   // positive
constexpr uint8_t kNezhaDirCcw = 2;  // negative
constexpr int kNezhaFrameLen = 8;

// OTOS -- source/devices/otos.h.
constexpr uint8_t kOtosDeviceAddr = 0x17;                                  // 7-bit
constexpr uint16_t kOtosWireAddr = static_cast<uint16_t>(kOtosDeviceAddr << 1);
constexpr uint8_t kOtosRegProductId = 0x00;
constexpr uint8_t kOtosRegLinearScalar = 0x04;   // 109-007 -- see handleOtosWrite()
constexpr uint8_t kOtosRegAngularScalar = 0x05;  // 109-007 -- see handleOtosWrite()
constexpr uint8_t kOtosRegPositionXl = 0x20;
constexpr uint8_t kOtosExpectedProductId = 0x5F;
constexpr float kPosMmPerLsb = 0.305f;                              // [mm/LSB]
constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);  // [rad/LSB]

// Color/line sensors -- source/devices/{color_sensor,line_sensor}.h. These
// are never simulated devices (no plant models them); every transaction to
// one of these wire addresses NAKs, matching the real bus's own behavior
// for an absent/uninitialized device and feeding ticket 008's regression
// test (color_sensor.cpp's APDS presence-probe fix).
constexpr uint16_t kLineWireAddr = static_cast<uint16_t>(0x1A << 1);
constexpr uint16_t kColorApdsWireAddr = static_cast<uint16_t>(0x39 << 1);
constexpr uint16_t kColorAltWireAddr = static_cast<uint16_t>(0x43 << 1);

// CODAL's well-known convention, duplicated per nezha_motor.cpp/otos.cpp's
// own local `kOk`.
constexpr int kOk = 0;
constexpr int kNakStatus = -1;

int32_t lroundToTenthsMm(float positionMm) {
  return static_cast<int32_t>(std::lround(positionMm * 10.0f));
}

void writeLeInt32(uint8_t* data, int32_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

void writeLeInt16(uint8_t* data, int16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

}  // namespace

SimPlant::SimPlant(float trackWidth)
    : left_(kDefaultDutyVelMax, kDefaultTau),
      right_(kDefaultDutyVelMax, kDefaultTau),
      otos_(trackWidth) {}

// ---------------------------------------------------------------------------
// Hook wrappers -- the middleware seam. Never re-entered by default*().
// ---------------------------------------------------------------------------

int SimPlant::write(uint16_t address, uint8_t* data, int len, bool /*repeated*/,
                     uint32_t /*preClear*/, uint32_t /*postClear*/) {
  return writeHook_ ? writeHook_(address, data, len) : defaultWrite(address, data, len);
}

int SimPlant::read(uint16_t address, uint8_t* data, int len, bool /*repeated*/,
                    uint32_t /*preClear*/, uint32_t /*postClear*/) {
  return readHook_ ? readHook_(address, data, len) : defaultRead(address, data, len);
}

// ---------------------------------------------------------------------------
// Default protocol handlers -- dispatch by 8-bit wire address.
// ---------------------------------------------------------------------------

int SimPlant::defaultWrite(uint16_t address, uint8_t* data, int len) {
  if (address == kMotorWireAddr) return handleMotorWrite(data, len);
  if (address == kOtosWireAddr) return handleOtosWrite(data, len);
  if (address == kLineWireAddr || address == kColorApdsWireAddr ||
      address == kColorAltWireAddr) {
    return kNakStatus;
  }
  return kNakStatus;  // unknown device -- absent, per the real bus's behavior.
}

int SimPlant::defaultRead(uint16_t address, uint8_t* data, int len) {
  if (address == kMotorWireAddr) return handleMotorRead(data, len);
  if (address == kOtosWireAddr) return handleOtosRead(data, len);
  return kNakStatus;
}

int SimPlant::handleMotorWrite(uint8_t* data, int len) {
  if (len != kNezhaFrameLen) return kNakStatus;
  // [0xFF, 0xF9, port, dir, cmd, speed, 0xF5, 0x00]
  uint8_t port = data[2];
  uint8_t dir = data[3];
  uint8_t cmd = data[4];
  uint8_t speed = data[5];

  if (cmd == kNezhaCmdRun) {
    // kNezhaDirCw -> positive; kNezhaDirCcw -> negative; anything else
    // (should not occur -- firmware only ever sends one of the two) holds
    // duty at 0, the safe default. A speed-0 coast write (writeMotorRun()'s
    // own kDirCw-with-speed-0 convention, nezha_motor.cpp) yields duty 0
    // either way.
    float magnitude = static_cast<float>(speed) / 100.0f;
    float duty = 0.0f;
    if (dir == kNezhaDirCw) {
      duty = magnitude;
    } else if (dir == kNezhaDirCcw) {
      duty = -magnitude;
    }
    if (port == 1) {
      leftDuty_ = duty;
    } else if (port == 2) {
      rightDuty_ = duty;
    }
    return kOk;
  }
  if (cmd == kNezhaCmdEncoderSelect) {
    selectedPort_ = port;
    return kOk;
  }
  return kOk;  // unrecognized command byte -- swallow, matching an ACK'd bus.
}

// The real Nezha 0x46 encoder register returns raw COUNTS (1 count == 1
// motor-shaft degree, 360/rev), NOT millimetres -- the firmware multiplies by
// wheelTravelCalib [mm/count] (~0.70486 for the tovez wheel) to recover mm.
// WheelPlant integrates in mm, so convert mm -> counts here before packing, so
// the firmware's real calibration round-trips to true mm exactly like hardware
// (previously the plant packed mm, which only read true when travelCalib==1.0
// and silently under-read by ~30% the moment the real ml/mr calibration was
// pushed). counts = mm * 360/(pi*80.77) = mm * 1.4187 for the tovez wheel.
constexpr float kEncoderCountsPerMm = 1.4187f;

int SimPlant::handleMotorRead(uint8_t* data, int len) {
  if (len != 4) return kNakStatus;
  WheelPlant& plant = mutableWheelPlant(selectedPort_);
  if (plant.disconnected()) return kNakStatus;
  writeLeInt32(data, lroundToTenthsMm(plant.reportedPosition() * kEncoderCountsPerMm));
  return kOk;
}

int SimPlant::handleOtosWrite(uint8_t* data, int len) {
  if (len < 1) return kNakStatus;
  // Track the register pointer written; swallow init/config/pose payload
  // bytes (data[1..]) -- SimPlant's OtosPlant is driven purely from wheel
  // positions (architecture-update.md Decision 3), never from a write.
  otosRegPtr_ = data[0];
  // 109-007: the ONE exception to "swallow every OTOS write payload" above
  // -- a write to the chip's own linear/angular calibration-scalar
  // registers (the REAL Devices::Otos::setLinearScalar()/setAngularScalar()
  // wire path, driven by begin()'s boot-config push, a live OtosConfigPatch,
  // or the OL/OA text verb) is captured and applied to this plant's
  // OtosPlant, so a firmware-pushed calibration genuinely corrects the
  // raw-scale-error fault knob's effect on subsequent reads (see
  // otos_plant.h's own setLinearScalarReg()/setAngularScalarReg() comment).
  // This is NOT a second, independent model of chip behavior -- it is the
  // exact register the real chip documents multiplying its raw measurement
  // by, ported here the same way handleOtosRead() already ports the
  // POSITION_XL+VELOCITY_XL burst layout.
  if (len >= 2 && otosRegPtr_ == kOtosRegLinearScalar) {
    otos_.setLinearScalarReg(static_cast<int8_t>(data[1]));
  } else if (len >= 2 && otosRegPtr_ == kOtosRegAngularScalar) {
    otos_.setAngularScalarReg(static_cast<int8_t>(data[1]));
  }
  return kOk;
}

int SimPlant::handleOtosRead(uint8_t* data, int len) {
  if (otosRegPtr_ == kOtosRegProductId) {
    if (len < 1) return kNakStatus;
    data[0] = kOtosExpectedProductId;
    for (int i = 1; i < len; ++i) data[i] = 0;
    return kOk;
  }
  if (otosRegPtr_ == kOtosRegPositionXl) {
    if (len < 12) return kNakStatus;
    // Same 12-byte POSITION_XL+VELOCITY_XL burst layout
    // Devices::Otos::readPositionVelocity() decodes (otos.cpp); packed
    // here directly since there is no I2CBus FIFO left for a
    // scriptPoseResponse()-style helper to target. reportedX/Y/Heading()
    // (not the bare x()/y()/heading() ground truth) apply OtosPlant's own
    // drift/bias fault knob.
    int16_t rx = static_cast<int16_t>(std::lround(otos_.reportedX() / kPosMmPerLsb));
    int16_t ry = static_cast<int16_t>(std::lround(otos_.reportedY() / kPosMmPerLsb));
    int16_t rh = static_cast<int16_t>(std::lround(otos_.reportedHeading() / kHdgRadPerLsb));
    writeLeInt16(data + 0, rx);
    writeLeInt16(data + 2, ry);
    writeLeInt16(data + 4, rh);
    // 109-010: VELOCITY_XL's own angular-rate word (rvh, decoded by
    // Devices::Otos::readPositionVelocity() as `whF` -- see that method's
    // own "reuses the SAME kPosMmPerLsb/kHdgRadPerLsb" comment for why the
    // SAME kHdgRadPerLsb scale applies here too) is OtosPlant::omega(), a
    // real finite-difference rate estimate -- App::HeadingSource's own
    // measurement-age projection (locus 1) needed a real omega_meas to
    // characterize/validate at all; before ticket 109-010 this word was
    // always zero ("no scenario asserts on OTOS's twist").
    //
    // 115-006 (gut S1 optional stretch): the linear-velocity words (rvx,
    // rvy) are likewise now OtosPlant::v_x()/v_y() -- the SAME
    // kPosMmPerLsb scale Devices::Otos::readPositionVelocity() decodes
    // vxF/vyF with (otos.cpp) -- instead of the hard-zero this word used to
    // carry ("no consumer reads pose().v_x/v_y, only pose().omega" was true
    // until this ticket; OtosReading.v_x/v_y now ride the primary telemetry
    // frame -- telemetry.proto's OtosReading message). v_y() is always 0
    // (no lateral-slip model, see OtosPlant::v_y()'s own comment), so ry's
    // encoded word is always exactly 0 regardless.
    int16_t rvx = static_cast<int16_t>(std::lround(otos_.v_x() / kPosMmPerLsb));
    int16_t rvy = static_cast<int16_t>(std::lround(otos_.v_y() / kPosMmPerLsb));
    int16_t rvh = static_cast<int16_t>(std::lround(otos_.omega() / kHdgRadPerLsb));
    writeLeInt16(data + 6, rvx);
    writeLeInt16(data + 8, rvy);
    writeLeInt16(data + 10, rvh);
    return kOk;
  }
  // Any other register pointer -- zeros, ACK.
  for (int i = 0; i < len; ++i) data[i] = 0;
  return kOk;
}

// ---------------------------------------------------------------------------
// Physics step -- called once per cycle by the harness, never by SimPlant.
// ---------------------------------------------------------------------------

void SimPlant::tick(float dt) {
  left_.step(leftDuty_, dt);
  right_.step(rightDuty_, dt);
  // 114-007 (Decision 7): correct each wheel's own physical (wire-frame)
  // position into the shared vehicle-forward convention OtosPlant requires
  // ONLY here, at the OtosPlant-feeding boundary -- left_.position()/
  // right_.position() themselves stay untouched, so handleMotorRead()'s
  // wire-level encoder simulation still reports exactly what a real chip's
  // raw encoder would for a mirror-mounted motor. See setFwdSign()'s own
  // comment (sim_plant.h).
  otos_.step(static_cast<float>(leftFwdSign_) * left_.position(),
             static_cast<float>(rightFwdSign_) * right_.position(),
             dt);  // 109-010: dt drives omega()
}

// ---------------------------------------------------------------------------
// Fault-injection knobs -- plain methods, not on Devices::I2CBus.
// ---------------------------------------------------------------------------

void SimPlant::setDisconnected(int port, bool disconnected) {
  mutableWheelPlant(port).setDisconnected(disconnected);
}

void SimPlant::freezePosition(int port, bool freeze) {
  mutableWheelPlant(port).freezePosition(freeze);
}

void SimPlant::setDropoutRate(int port, float fraction) {
  mutableWheelPlant(port).setDropoutRate(fraction);
}

void SimPlant::setEncScaleErr(int port, float fraction) {
  mutableWheelPlant(port).setScaleErr(fraction);
}

void SimPlant::setEncTickQuantization(int port, float tickSizeMm) {
  mutableWheelPlant(port).setTickQuantization(tickSizeMm);
}

void SimPlant::setEncSlip(int port, float rate, float magnitudeMm) {
  mutableWheelPlant(port).setSlip(rate, magnitudeMm);
}

void SimPlant::setEncoderJitter(bool enabled) {
  left_.setEncoderJitter(enabled);
  right_.setEncoderJitter(enabled);
}

void SimPlant::setOtosDrift(float xDrift, float yDrift, float headingDrift) {
  otos_.setDrift(xDrift, yDrift, headingDrift);
}

void SimPlant::setOtosRawScaleErr(float linearFraction, float angularFraction) {
  otos_.setRawScaleErr(linearFraction, angularFraction);
}

void SimPlant::setTruePose(float x, float y, float heading) {
  // Do NOT zero the wheel plants. Keeping their encoder raw continuous is what
  // lets the firmware motors' hardReset() (SimHarness::setTruePose()) re-zero
  // their software offset with no discontinuity on the next collectEncoder()
  // -- zeroing the wheels here made the firmware read a fresh 0 against a
  // stale motor offset and jump. Re-anchor the OTOS truth to (x,y,heading)
  // with its wheel-delta baseline at the wheels' CURRENT positions so its next
  // step() integrates a zero delta, not a phantom jump.
  //
  // 114-007: the baseline passed here MUST be in the SAME corrected
  // (fwdSign-applied) frame tick()'s own otos_.step() call uses above --
  // otherwise the very next tick() would compute its delta against an
  // uncorrected baseline and inject a phantom one-cycle jump, exactly the
  // failure mode this method's own comment above already warns about.
  otos_.reset(x, y, heading, static_cast<float>(leftFwdSign_) * left_.position(),
              static_cast<float>(rightFwdSign_) * right_.position());
}

void SimPlant::setFwdSign(int port, int sign) {
  if (port == 2) {
    rightFwdSign_ = sign;
  } else {
    leftFwdSign_ = sign;
  }
}

const WheelPlant& SimPlant::wheelPlant(int port) const {
  return (port == 2) ? right_ : left_;
}

WheelPlant& SimPlant::mutableWheelPlant(int port) {
  return (port == 2) ? right_ : left_;
}

}  // namespace TestSim
