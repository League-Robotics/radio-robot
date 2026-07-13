#include "devices/color_sensor.h"

namespace Devices {

namespace {
// CODAL's well-known convention: 0 == success (matches nezha_motor.cpp's
// and otos.cpp's identical local kOk).
constexpr int kOk = 0;
}  // namespace

ColorSensor::ColorSensor(I2CBus& bus, const ColorConfig& config)
    : bus_(bus), config_(config) {
  // ColorConfig's fields all zero-default (device_config.h) — the
  // "unconfigured" sentinel this leaf resolves to its ship default, mirroring
  // nezha_motor.cpp's identical `if (config_.slewRate <= 0.0f) ...` pattern.
  if (config_.lagColor == 0) config_.lagColor = kDefaultLagColor;
  if (config_.integration == 0) config_.integration = kDefaultIntegration;
  if (config_.gain == 0) config_.gain = kDefaultGain;
}

// ---------------------------------------------------------------------------
// beginStep() — non-blocking single detection step. See color_sensor.h's
// declaration comment for the full phase contract.
// ---------------------------------------------------------------------------

void ColorSensor::beginStep(uint64_t nowUs) {
  if (phase_ == DetectPhase::Done) return;

  if (phase_ == DetectPhase::AltProbe) {
    if (hasAttempted_ && (nowUs - lastAttemptUs_) < kAltRetryPeriod) {
      return;  // not due yet -- non-blocking, caller retries next call
    }
    hasAttempted_ = true;
    lastAttemptUs_ = nowUs;
    ++altAttempts_;

    // EXACT port of upstream PlanetX initColor: re-assert the wake writes
    // INSIDE every retry (see file header), then check the 16-bit value at
    // 0xA4/0xA5 is non-zero.
    writeReg8(kColorDeviceAddrAlt, 0x81, 0xCA);
    writeReg8(kColorDeviceAddrAlt, 0x80, 0x17);
    uint16_t probe = readReg16Alt(0xA4);
    if (probe != 0) {
      isAlt_ = true;
      initialized_ = true;
      connected_ = true;
      phase_ = DetectPhase::Done;
      return;
    }

    if (altAttempts_ >= kMaxAltAttempts) {
      phase_ = DetectPhase::ApdsProbe;  // next beginStep() call attempts APDS
    }
    return;
  }

  // phase_ == ApdsProbe: exactly one attempt (the pre-port fallback has no
  // retry loop for APDS either — see file header).
  writeReg8(kColorDeviceAddrApds, 0x80, 0x00);
  uint8_t en = readReg8(kColorDeviceAddrApds, 0x80);
  if (en == 0x00) {
    isAlt_ = false;
    initApds();
    initialized_ = true;
    connected_ = true;
  } else {
    initialized_ = false;
    connected_ = false;
  }
  phase_ = DetectPhase::Done;
}

bool ColorSensor::detectDone() const { return phase_ == DetectPhase::Done; }

bool ColorSensor::present() const { return initialized_; }

bool ColorSensor::connected() const { return initialized_ && connected_; }

// ---------------------------------------------------------------------------
// readDue() -- pure scheduling query, no I2C traffic.
// ---------------------------------------------------------------------------

bool ColorSensor::readDue(uint64_t nowUs) const {
  uint64_t periodUs = static_cast<uint64_t>(config_.lagColor) * 1000;
  return !hasRead_ || (nowUs - lastReadUs_) >= periodUs;
}

// ---------------------------------------------------------------------------
// tick() -- non-blocking poll-and-collect. See color_sensor.h's declaration
// comment for the full contract (ports pollRGBC(), not readRGBC()).
// ---------------------------------------------------------------------------

void ColorSensor::tick(uint64_t nowUs) {
  if (!initialized_) return;  // beginStep() never found a chip -- no bus traffic
  if (!readDue(nowUs)) return;  // rate-limited -- no bus traffic

  lastReadUs_ = nowUs;
  hasRead_ = true;

  if (isAlt_) {
    // Alt chip: non-zero clear channel (0xA6) means fresh data is ready --
    // mirrors pollRGBC()'s ALT branch exactly.
    uint16_t probe = 0;
    bool ok = readReg16AltStatus(0xA6, probe);
    if (!ok) {
      connected_ = false;
      readingFresh_ = false;
      return;
    }
    connected_ = true;
    if (probe == 0) {
      readingFresh_ = false;  // not ready yet -- retried next due tick()
      return;
    }
    uint16_t r = 0, g = 0, b = 0;
    bool okAll = readReg16AltStatus(0xA0, r) && readReg16AltStatus(0xA2, g) &&
                 readReg16AltStatus(0xA4, b);
    if (!okAll) {
      connected_ = false;
      readingFresh_ = false;
      return;
    }
    cachedReading_ = ColorReading{r, g, b, probe};
    readingFresh_ = true;
  } else {
    // APDS9960: AVALID bit (STATUS, 0x93) without blocking -- mirrors
    // pollRGBC()'s APDS branch exactly.
    uint8_t status = 0;
    bool ok = readReg8Status(kColorDeviceAddrApds, 0x93, status);
    if (!ok) {
      connected_ = false;
      readingFresh_ = false;
      return;
    }
    connected_ = true;
    if ((status & 0x01) == 0) {
      readingFresh_ = false;  // not ready yet -- retried next due tick()
      return;
    }
    uint16_t c = 0, r = 0, g = 0, b = 0;
    bool okAll = readReg16Status(kColorDeviceAddrApds, 0x94, c) &&
                 readReg16Status(kColorDeviceAddrApds, 0x96, r) &&
                 readReg16Status(kColorDeviceAddrApds, 0x98, g) &&
                 readReg16Status(kColorDeviceAddrApds, 0x9A, b);
    if (!okAll) {
      connected_ = false;
      readingFresh_ = false;
      return;
    }
    cachedReading_ = ColorReading{r, g, b, c};
    readingFresh_ = true;
  }
}

ColorReading ColorSensor::reading() const { return cachedReading_; }

bool ColorSensor::readingFresh() const { return readingFresh_; }

// ---------------------------------------------------------------------------
// initApds() -- ported unchanged, except ATIME/CONTROL now come from
// ColorConfig::integration/gain (device_config.h's own doc comments name
// these fields exactly as the pre-port literals below: "raw sensor
// integration-time register value" / "raw sensor gain register value") in
// place of the pre-port file's hardcoded 252 / 0x03.
// ---------------------------------------------------------------------------

void ColorSensor::initApds() {
  writeReg8(kColorDeviceAddrApds, 0x81, static_cast<uint8_t>(config_.integration));  // ATIME
  writeReg8(kColorDeviceAddrApds, 0x8F, static_cast<uint8_t>(config_.gain));         // CONTROL
  writeReg8(kColorDeviceAddrApds, 0x80, 0x00);                                       // ENABLE: power off
  writeReg8(kColorDeviceAddrApds, 0xAB, 0x00);
  writeReg8(kColorDeviceAddrApds, 0xE7, 0x00);
  writeReg8(kColorDeviceAddrApds, 0x80, 0x01);  // ENABLE: power on
  uint8_t en = readReg8(kColorDeviceAddrApds, 0x80);
  writeReg8(kColorDeviceAddrApds, 0x80, en | 0x02);  // AEN (ambient/colour enable)
}

// ---------------------------------------------------------------------------
// Private register-map helpers.
// ---------------------------------------------------------------------------

void ColorSensor::writeReg8(uint8_t addr, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  bus_.write(static_cast<uint16_t>(addr << 1), buf, 2, false);
}

bool ColorSensor::readReg8Status(uint8_t addr, uint8_t reg, uint8_t& out) {
  int writeStatus = bus_.write(static_cast<uint16_t>(addr << 1), &reg, 1, false);
  uint8_t result = 0;
  int readStatus = bus_.read(static_cast<uint16_t>(addr << 1), &result, 1, false);
  out = result;
  return writeStatus == kOk && readStatus == kOk;
}

uint8_t ColorSensor::readReg8(uint8_t addr, uint8_t reg) {
  uint8_t out = 0;
  readReg8Status(addr, reg, out);
  return out;
}

bool ColorSensor::readReg16Status(uint8_t addr, uint8_t regLo, uint16_t& out) {
  // Read two consecutive bytes: [regLo, regLo+1] -> little-endian uint16.
  uint8_t raw[2] = {0, 0};
  int writeStatus = bus_.write(static_cast<uint16_t>(addr << 1), &regLo, 1, false);
  int readStatus = bus_.read(static_cast<uint16_t>(addr << 1), raw, 2, false);
  out = static_cast<uint16_t>(raw[0] | (static_cast<uint16_t>(raw[1]) << 8));
  return writeStatus == kOk && readStatus == kOk;
}

uint16_t ColorSensor::readReg16(uint8_t addr, uint8_t regLo) {
  uint16_t out = 0;
  readReg16Status(addr, regLo, out);
  return out;
}

bool ColorSensor::readReg16AltStatus(uint8_t regLo, uint16_t& out) {
  // Alt-chip (0x43) single-byte protocol: two separate write-reg/read-1-byte
  // transactions instead of one 2-byte burst read -- mirrors the upstream
  // PlanetX driver exactly (i2cread_color lo + i2cread_color hi * 256).
  uint8_t lo = 0, hi = 0;
  bool okLo = readReg8Status(kColorDeviceAddrAlt, regLo, lo);
  bool okHi = readReg8Status(kColorDeviceAddrAlt, static_cast<uint8_t>(regLo + 1), hi);
  out = static_cast<uint16_t>(lo | (static_cast<uint16_t>(hi) << 8));
  return okLo && okHi;
}

uint16_t ColorSensor::readReg16Alt(uint8_t regLo) {
  uint16_t out = 0;
  readReg16AltStatus(regLo, out);
  return out;
}

}  // namespace Devices
