#include "devices/line_sensor.h"

namespace Devices {

namespace {
// CODAL's well-known convention: 0 == success (matches nezha_motor.cpp's,
// otos.cpp's, and color_sensor.cpp's identical local kOk).
constexpr int kOk = 0;
}  // namespace

LineSensor::LineSensor(I2CBus& bus, const LineConfig& config)
    : bus_(bus), config_(config) {
  // LineConfig::lagLine zero-defaults (device_config.h) -- the
  // "unconfigured" sentinel this leaf resolves to its ship default, mirroring
  // nezha_motor.cpp's identical `if (config_.slewRate <= 0.0f) ...` pattern.
  if (config_.lagLine == 0) config_.lagLine = kDefaultLagLine;

  // LineConfig::calMax[] ALSO zero-defaults (device_config.h), unlike the
  // pre-port file's own constructor, which defaulted _calMax[ch] to 255 per
  // channel. A calibration max of exactly 0 is never meaningful for a real
  // sensor (it would clamp every non-zero raw reading straight to 1000 --
  // see tick()'s normalize step), so 0 is treated as the same "unconfigured"
  // sentinel here and re-defaulted to match the pre-port ship default.
  // calMin[] needs no such fixup -- 0 is both DB-001's zero-default AND the
  // pre-port file's own default for that bound.
  for (uint8_t ch = 0; ch < 4; ++ch) {
    if (config_.calMax[ch] == 0) config_.calMax[ch] = kDefaultCalMax;
  }
}

// ---------------------------------------------------------------------------
// beginStep() -- non-blocking single detection step. See line_sensor.h's
// declaration comment for the full contract.
// ---------------------------------------------------------------------------

void LineSensor::beginStep(uint64_t nowUs) {
  if (phase_ == DetectPhase::Done) return;

  if (hasAttempted_ && (nowUs - lastAttemptUs_) < kRetryPeriod) {
    return;  // not due yet -- non-blocking, caller retries next call
  }
  hasAttempted_ = true;
  lastAttemptUs_ = nowUs;
  ++attempts_;

  bool ok = readRaw(nullptr);
  if (ok) {
    initialized_ = true;
    connected_ = true;
    phase_ = DetectPhase::Done;
    return;
  }

  if (attempts_ >= kMaxAttempts) {
    initialized_ = false;
    connected_ = false;
    phase_ = DetectPhase::Done;
  }
}

bool LineSensor::detectDone() const { return phase_ == DetectPhase::Done; }

bool LineSensor::present() const { return initialized_; }

bool LineSensor::connected() const { return initialized_ && connected_; }

// ---------------------------------------------------------------------------
// readDue() -- pure scheduling query, no I2C traffic.
// ---------------------------------------------------------------------------

bool LineSensor::readDue(uint64_t nowUs) const {
  uint64_t periodUs = static_cast<uint64_t>(config_.lagLine) * 1000;
  return !hasRead_ || (nowUs - lastReadUs_) >= periodUs;
}

// ---------------------------------------------------------------------------
// tick() -- single non-blocking raw read, then normalize + EMA smooth.
// Ported math from the pre-port file's readNormalized().
// ---------------------------------------------------------------------------

void LineSensor::tick(uint64_t nowUs) {
  if (!initialized_) return;    // beginStep() never found a chip -- no bus traffic
  if (!readDue(nowUs)) return;  // rate-limited -- no bus traffic

  lastReadUs_ = nowUs;
  hasRead_ = true;

  uint16_t raw[4] = {0, 0, 0, 0};
  bool ok = readRaw(raw);
  connected_ = ok;
  if (!ok) {
    readingFresh_ = false;
    return;
  }

  LineReading out{};
  for (uint8_t ch = 0; ch < 4; ++ch) {
    out.raw[ch] = raw[ch];

    uint32_t mn = config_.calMin[ch];
    uint32_t mx = config_.calMax[ch];
    uint32_t span = (mx > mn) ? (mx - mn) : 255u;

    int32_t norm;
    if (raw[ch] <= mn) {
      norm = 0;
    } else if (raw[ch] >= mx) {
      norm = 1000;
    } else {
      norm = (static_cast<int32_t>(raw[ch] - mn) * 1000) / static_cast<int32_t>(span);
    }
    if (norm < 0) norm = 0;
    if (norm > 1000) norm = 1000;

    if (config_.filtAlpha > 0.0f) {
      emaState_[ch] = config_.filtAlpha * emaState_[ch] +
                       (1.0f - config_.filtAlpha) * static_cast<float>(norm);
      norm = static_cast<int32_t>(emaState_[ch]);
      if (norm < 0) norm = 0;
      if (norm > 1000) norm = 1000;
    }

    out.normalized[ch] = static_cast<uint32_t>(norm);
  }

  cachedReading_ = out;
  readingFresh_ = true;
}

LineReading LineSensor::reading() const { return cachedReading_; }

bool LineSensor::readingFresh() const { return readingFresh_; }

// ---------------------------------------------------------------------------
// Calibration -- each issues its own fresh, non-blocking read.
// ---------------------------------------------------------------------------

bool LineSensor::captureCalibMin() {
  uint16_t raw[4] = {0, 0, 0, 0};
  if (!readRaw(raw)) return false;
  for (uint8_t ch = 0; ch < 4; ++ch) config_.calMin[ch] = raw[ch];
  return true;
}

bool LineSensor::captureCalibMax() {
  uint16_t raw[4] = {0, 0, 0, 0};
  if (!readRaw(raw)) return false;
  for (uint8_t ch = 0; ch < 4; ++ch) config_.calMax[ch] = raw[ch];
  return true;
}

void LineSensor::setSmoothingAlpha(float alpha) {
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha >= 1.0f) alpha = 0.99f;  // clamp to keep the EMA stable
  config_.filtAlpha = alpha;
}

// ---------------------------------------------------------------------------
// readRaw() -- ported byte-for-byte (already non-blocking in the pre-port
// file -- no fiber_sleep). Four write(channel-index)/read(1-byte) pairs.
// ---------------------------------------------------------------------------

bool LineSensor::readRaw(uint16_t out[4]) {
  for (uint8_t ch = 0; ch < 4; ++ch) {
    uint8_t chByte = ch;
    int writeStatus = bus_.write(static_cast<uint16_t>(kLineDeviceAddr << 1), &chByte, 1, false);
    if (writeStatus != kOk) return false;

    uint8_t val = 0;
    int readStatus = bus_.read(static_cast<uint16_t>(kLineDeviceAddr << 1), &val, 1, false);
    if (readStatus != kOk) return false;

    if (out) out[ch] = val;
  }
  return true;
}

}  // namespace Devices
