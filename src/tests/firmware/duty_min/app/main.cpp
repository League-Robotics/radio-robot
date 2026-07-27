// duty_min bench firmware -- find the REAL minimum duty that moves each
// wheel, with ZERO competing bus traffic during the hold. Motivated by a
// vendor-blocks rig running these motors at 1-5% duty while the robot
// firmware measured a 10-16% dead zone: this isolates whether the gap is
// the motors or the main loop's constant encoder polling perturbing the
// brick's narrow-pulse PWM.
//
// Per wheel, duty 1..14% forward: read encoder, write ONE motor-run
// command (byte-for-byte the vendor/main-firmware 0x60 wire format), hold
// 3 s with NO bus traffic at all, read encoder, stop, rest 1 s. Streams
// CSV over USB serial:
//   R,<L|R>,<duty%>,<delta_raw>     raw delta in tenths of shaft degrees
//   DONE
#include <cstdio>

#include "MicroBit.h"

static MicroBit uBit;

namespace {

constexpr uint16_t kNezhaAddress = 0x10 << 1;
constexpr uint8_t kPortLeft = 1;
constexpr uint8_t kPortRight = 2;
constexpr uint8_t kDirCw = 1;
constexpr uint32_t kHold = 3000;  // [ms] quiet hold per level
constexpr uint32_t kRest = 1000;  // [ms] full-stop rest between levels

bool readEncoderRaw(uint8_t port, int32_t& raw) {
  uint8_t cmd[8] = {0xFF, 0xF9, port, 0x00, 0x46, 0x00, 0xF5, 0x00};
  if (uBit.i2c.write(kNezhaAddress, cmd, 8) != MICROBIT_OK) return false;
  uBit.sleep(5);
  uint8_t data[4] = {0};
  if (uBit.i2c.read(kNezhaAddress, data, 4) != MICROBIT_OK) return false;
  raw = static_cast<int32_t>(data[0]) | (static_cast<int32_t>(data[1]) << 8) |
        (static_cast<int32_t>(data[2]) << 16) |
        (static_cast<int32_t>(data[3]) << 24);
  return true;
}

void writeMotorRun(uint8_t port, uint8_t dir, uint8_t speed) {
  uint8_t buf[8] = {0xFF, 0xF9, port, dir, 0x60, speed, 0xF5, 0x00};
  uBit.i2c.write(kNezhaAddress, buf, 8);
  uBit.sleep(5);
}

constexpr uint16_t kOtosAddress = 0x17 << 1;
constexpr uint16_t kLineAddress = 0x1A << 1;

// mode 0 (Q): ZERO bus traffic during the hold. mode 1 (B): brick-addressed
// encoder polling every ~47 ms. mode 2 (O): OTHER-device traffic -- an
// OTOS burst read + a line-sensor read every ~47 ms (the main loop's other
// bus tenants) with NO brick traffic during the hold.
void probeWheel(uint8_t port, char label, int mode) {
  for (uint8_t duty = 1; duty <= 14; ++duty) {
    int32_t before = 0, after = 0;
    if (!readEncoderRaw(port, before)) continue;
    writeMotorRun(port, kDirCw, duty);
    if (mode == 1) {
      uint32_t held = 0;
      while (held < kHold) {
        uBit.sleep(38);
        int32_t scratch = 0;
        readEncoderRaw(port, scratch);  // ~9 ms incl. clearance
        held += 47;
      }
    } else if (mode == 2) {
      uint32_t held = 0;
      while (held < kHold) {
        uBit.sleep(33);
        uint8_t reg = 0x00;
        uint8_t scratch[12] = {0};
        uBit.i2c.write(kOtosAddress, &reg, 1);   // OTOS-style register select
        uBit.i2c.read(kOtosAddress, scratch, 12);  // burst read
        uBit.sleep(4);
        uBit.i2c.write(kLineAddress, &reg, 1);   // line-sensor touch
        uBit.i2c.read(kLineAddress, scratch, 4);
        held += 47;
      }
    } else {
      uBit.sleep(kHold);
    }
    if (!readEncoderRaw(port, after)) continue;
    writeMotorRun(port, kDirCw, 0);  // coast stop
    uBit.sleep(kRest);
    char line[48];
    const char modeChar = (mode == 0) ? 'Q' : (mode == 1) ? 'B' : 'O';
    snprintf(line, sizeof(line), "R,%c%c,%u,%ld\n", modeChar, label,
             static_cast<unsigned>(duty), static_cast<long>(after - before));
    uBit.serial.send(line);
  }
}

}  // namespace

int main() {
  uBit.init();
  uBit.serial.setBaud(115200);
  uBit.sleep(2000);
  uBit.serial.send("# duty_min: quiet-hold minimum-duty probe\n");
  probeWheel(kPortRight, 'R', 0);
  probeWheel(kPortRight, 'R', 2);
  probeWheel(kPortLeft, 'L', 2);
  uBit.serial.send("DONE\n");
  while (true) uBit.sleep(1000);
}
