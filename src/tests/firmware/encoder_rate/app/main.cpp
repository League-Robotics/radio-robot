// encoder_rate bench firmware -- characterize the Nezha V2 brick's 0x46
// encoder-register refresh behavior (motion-planner issue §7.3). This is a
// COMPLETELY SEPARATE firmware from the robot's: no robot loop, no
// protocol, no telemetry -- it drives both wheels at a constant duty and
// tight-polls the raw encoder registers, streaming a change-log over USB
// serial for host-side analysis (analyze.py in this directory).
//
// The three questions it answers (motion-planner sketch/issue):
//   1. What is the actual refresh period, and its jitter? (folklore: ~80 ms,
//      inferred once during the freshness-gate bug fix, never characterized)
//   2. Are the LEFT and RIGHT encoder registers refreshed on the same
//      internal clock (correlated change instants), or independently?
//   3. Does the polling rate itself perturb the refresh (refresh-on-read)?
//
// Phases (fixed sequence, ~85 s total, then motors stop and it idles):
//   A1  tight-poll LEFT alone  (~9 ms/poll), 20 s
//   A2  tight-poll RIGHT alone (~9 ms/poll), 20 s
//   B   alternate LEFT/RIGHT   (~18 ms/poll per wheel), 20 s -- correlation
//   C   slow-poll LEFT (~25 ms/poll), 20 s -- read-rate influence vs A1
//
// Wire protocol (byte-for-byte from src/firm/devices/nezha_motor.cpp --
// this file deliberately re-implements it raw rather than linking the
// driver, so the driver's own conditioning can't color the measurement):
//   encoder read:  write [0xFF,0xF9,port,0x00,0x46,0x00,0xF5,0x00],
//                  >=4 ms clearance, read 4 bytes little-endian int32
//                  (tenths of motor-shaft degrees)
//   motor run:     write [0xFF,0xF9,port,dir,0x60,speed,0xF5,0x00]
//                  dir 1=CW 2=CCW, speed 0-100 [%], >=4 ms clearance after
//
// Output lines (CSV over 115200-baud USB serial):
//   #  <comment/header>
//   P,<phase>,<t>            phase start                 [t in us, 32-bit]
//   C,<phase>,<w>,<t>,<raw>,<polls>   register CHANGED: wheel L/R, poll
//                            timestamp, new raw count, polls since last change
//   N,<phase>,<w>,<polls>,<changes>,<errors>   phase summary per wheel
//   DONE                     all phases complete, motors stopped
#include "MicroBit.h"

static MicroBit uBit;

namespace {

constexpr uint16_t kNezhaAddress = 0x10 << 1;  // 8-bit wire address
constexpr uint8_t kPortLeft = 1;   // drive-pair ports (boot_config.cpp)
constexpr uint8_t kPortRight = 2;
constexpr uint8_t kDirCw = 1;
constexpr uint8_t kDirCcw = 2;
constexpr uint8_t kDutySpeed = 30;      // [%] constant test duty
constexpr uint32_t kClearance = 4;      // [ms] >=4 ms between 0x10 transactions
constexpr uint32_t kPhaseDuration = 20000000;  // [us] 20 s per phase

int i2cErrors = 0;

// One split encoder transaction: 0x46 select-write, clearance, 4-byte read.
// Returns true and fills `raw` on success; counts and swallows NAKs.
bool readEncoderRaw(uint8_t port, int32_t& raw) {
  uint8_t cmd[8] = {0xFF, 0xF9, port, 0x00, 0x46, 0x00, 0xF5, 0x00};
  if (uBit.i2c.write(kNezhaAddress, cmd, 8) != MICROBIT_OK) {
    ++i2cErrors;
    return false;
  }
  uBit.sleep(kClearance);
  uint8_t resp[4] = {0, 0, 0, 0};
  if (uBit.i2c.read(kNezhaAddress, resp, 4) != MICROBIT_OK) {
    ++i2cErrors;
    return false;
  }
  raw = static_cast<int32_t>((static_cast<uint32_t>(resp[3]) << 24) |
                             (static_cast<uint32_t>(resp[2]) << 16) |
                             (static_cast<uint32_t>(resp[1]) << 8) |
                             static_cast<uint32_t>(resp[0]));
  return true;
}

void writeMotorRun(uint8_t port, uint8_t direction, uint8_t speed) {
  uint8_t cmd[8] = {0xFF, 0xF9, port, direction, 0x60, speed, 0xF5, 0x00};
  if (uBit.i2c.write(kNezhaAddress, cmd, 8) != MICROBIT_OK) ++i2cErrors;
  uBit.sleep(kClearance);
}

uint32_t nowStamp() {
  return static_cast<uint32_t>(system_timer_current_time_us());
}

// Per-wheel change tracker shared by every phase loop.
struct ChangeTracker {
  char wheel;           // 'L' or 'R'
  int32_t lastRaw = 0;
  bool primed = false;
  uint32_t polls = 0;
  uint32_t changes = 0;
  uint32_t pollsSinceChange = 0;

  void poll(const char* phase, uint8_t port) {
    int32_t raw = 0;
    if (!readEncoderRaw(port, raw)) return;
    const uint32_t t = nowStamp();
    ++polls;
    ++pollsSinceChange;
    if (!primed) {
      primed = true;
      lastRaw = raw;
      pollsSinceChange = 0;
      return;
    }
    if (raw != lastRaw) {
      ++changes;
      uBit.serial.printf("C,%s,%c,%d,%d,%d\r\n", phase, wheel,
                         static_cast<int>(t), static_cast<int>(raw),
                         static_cast<int>(pollsSinceChange));
      lastRaw = raw;
      pollsSinceChange = 0;
    }
  }

  void summary(const char* phase) {
    uBit.serial.printf("N,%s,%c,%d,%d,%d\r\n", phase, wheel,
                       static_cast<int>(polls),
                       static_cast<int>(changes), i2cErrors);
    polls = 0;
    changes = 0;
    primed = false;
    i2cErrors = 0;
  }
};

ChangeTracker left{'L'};
ChangeTracker right{'R'};

void phaseMarker(const char* phase) {
  uBit.serial.printf("P,%s,%d\r\n", phase, static_cast<int>(nowStamp()));
}

// Single-wheel poll loop; extraDelay stretches the poll period (phase C);
// dutyEvery > 0 interleaves a same-value 0x60 duty write every N polls,
// emulating the robot loop's own periodic duty refresh (phase D -- the
// wedge history says writes can latch the 0x46 readback, making this the
// prime suspect for the loop's observed staleness).
void pollSingle(const char* phase, ChangeTracker& tracker, uint8_t port,
                uint32_t extraDelay, uint32_t duration,
                uint32_t dutyEvery, uint8_t duty) {  // [ms] [us]
  phaseMarker(phase);
  const uint32_t start = nowStamp();
  uint32_t polls = 0;
  while (nowStamp() - start < duration) {
    if (dutyEvery > 0 && polls % dutyEvery == 0) {
      writeMotorRun(port, kDirCw, duty);
    }
    ++polls;
    tracker.poll(phase, port);
    uBit.sleep(kClearance + extraDelay);  // post-read clearance (+ stretch)
  }
  tracker.summary(phase);
}

void pollAlternating(const char* phase) {
  phaseMarker(phase);
  const uint32_t start = nowStamp();
  while (nowStamp() - start < kPhaseDuration) {
    left.poll(phase, kPortLeft);
    uBit.sleep(kClearance);
    right.poll(phase, kPortRight);
    uBit.sleep(kClearance);
  }
  left.summary(phase);
  right.summary(phase);
}

}  // namespace

int main() {
  uBit.init();
  uBit.serial.setBaud(115200);
  uBit.sleep(500);  // let USB serial enumerate

  uBit.serial.printf("# encoder_rate bench v1\r\n");
  uBit.serial.printf("# duty=%d%% ports L=%d R=%d clearance=%dms phase=%ds\r\n",
                     kDutySpeed, kPortLeft, kPortRight,
                     static_cast<int>(kClearance),
                     static_cast<int>(kPhaseDuration / 1000000));

  // Prime: the 0x46 register sits frozen at 0 until the chip has seen a
  // read transaction per port (NezhaMotor::begin()'s hardReset does this
  // on the real firmware). Three reads per port, report connectivity.
  for (int i = 0; i < 3; ++i) {
    int32_t rawLeft = 0;
    int32_t rawRight = 0;
    const bool okLeft = readEncoderRaw(kPortLeft, rawLeft);
    uBit.sleep(kClearance);
    const bool okRight = readEncoderRaw(kPortRight, rawRight);
    uBit.sleep(kClearance);
    uBit.serial.printf("# prime %d: L ok=%d raw=%d | R ok=%d raw=%d\r\n", i,
                       okLeft ? 1 : 0, static_cast<int>(rawLeft),
                       okRight ? 1 : 0, static_cast<int>(rawRight));
  }
  if (i2cErrors > 4) {
    uBit.serial.printf("# ERROR: brick not responding (errors=%d) -- abort\r\n",
                       i2cErrors);
    uBit.display.print('X');
    while (true) uBit.sleep(1000);
  }
  i2cErrors = 0;

  // Spin both wheels (robot is on the stand; wheels free). Opposite chip
  // directions == both wheels "robot forward" (right motor is mirrored).
  writeMotorRun(kPortLeft, kDirCw, kDutySpeed);
  writeMotorRun(kPortRight, kDirCcw, kDutySpeed);
  uBit.sleep(1500);  // let speed settle so counts change every refresh

  // A1 baseline: tight poll, no interference (run 1 established: fresh
  // EVERY poll at 16/32 ms periods -- the register is live, ~80 ms is
  // false at this duty). A2/B/C from run 1 kept for reference in git.
  pollSingle("A1", left, kPortLeft, 0, kPhaseDuration / 2, 0, 0);
  pollAlternating("B");
  // D: same tight poll + a same-value duty write every 3rd poll (~50 ms
  // cadence, the robot loop's own write pattern).
  pollSingle("D", left, kPortLeft, 0, kPhaseDuration, 3, kDutySpeed);
  // E: very slow wheel -- does the register stay live near stall, or do
  // the count quanta (tenths of a degree) just stop moving?
  writeMotorRun(kPortLeft, kDirCw, 8);
  uBit.sleep(2000);
  pollSingle("E", left, kPortLeft, 0, kPhaseDuration / 2, 0, 0);

  // F: interpose a duty write BETWEEN the 0x46 select and the read -- the
  // robot loop's split-phase schedule can put 0x10 traffic inside that
  // window; does it invalidate/stale the pending select?
  writeMotorRun(kPortLeft, kDirCw, kDutySpeed);
  uBit.sleep(1500);
  phaseMarker("F");
  {
    const uint32_t start = nowStamp();
    while (nowStamp() - start < kPhaseDuration / 2) {
      uint8_t cmd[8] = {0xFF, 0xF9, kPortLeft, 0x00, 0x46, 0x00, 0xF5, 0x00};
      if (uBit.i2c.write(kNezhaAddress, cmd, 8) != MICROBIT_OK) ++i2cErrors;
      uBit.sleep(kClearance);
      writeMotorRun(kPortLeft, kDirCw, kDutySpeed);  // interposed 0x60
      uint8_t resp[4] = {0, 0, 0, 0};
      if (uBit.i2c.read(kNezhaAddress, resp, 4) != MICROBIT_OK) {
        ++i2cErrors;
      } else {
        const int32_t raw = static_cast<int32_t>(
            (static_cast<uint32_t>(resp[3]) << 24) |
            (static_cast<uint32_t>(resp[2]) << 16) |
            (static_cast<uint32_t>(resp[1]) << 8) |
            static_cast<uint32_t>(resp[0]));
        uBit.serial.printf("C,F,L,%d,%d,1\r\n",
                           static_cast<int>(nowStamp()),
                           static_cast<int>(raw));
      }
      uBit.sleep(kClearance);
    }
    uBit.serial.printf("N,F,L,0,0,%d\r\n", i2cErrors);
    i2cErrors = 0;
  }

  // G: interpose the OTHER port's 0x46 select between L's select and the
  // read -- documents which counter the read then returns (the select-
  // latch semantics the robot loop's "clear" slice guards against).
  phaseMarker("G");
  {
    const uint32_t start = nowStamp();
    while (nowStamp() - start < kPhaseDuration / 2) {
      uint8_t cmdLeftSel[8] = {0xFF, 0xF9, kPortLeft, 0x00, 0x46,
                               0x00, 0xF5, 0x00};
      uint8_t cmdRightSel[8] = {0xFF, 0xF9, kPortRight, 0x00, 0x46,
                                0x00, 0xF5, 0x00};
      if (uBit.i2c.write(kNezhaAddress, cmdLeftSel, 8) != MICROBIT_OK)
        ++i2cErrors;
      uBit.sleep(kClearance);
      if (uBit.i2c.write(kNezhaAddress, cmdRightSel, 8) != MICROBIT_OK)
        ++i2cErrors;
      uBit.sleep(kClearance);
      uint8_t resp[4] = {0, 0, 0, 0};
      if (uBit.i2c.read(kNezhaAddress, resp, 4) != MICROBIT_OK) {
        ++i2cErrors;
      } else {
        const int32_t raw = static_cast<int32_t>(
            (static_cast<uint32_t>(resp[3]) << 24) |
            (static_cast<uint32_t>(resp[2]) << 16) |
            (static_cast<uint32_t>(resp[1]) << 8) |
            static_cast<uint32_t>(resp[0]));
        uBit.serial.printf("C,G,X,%d,%d,1\r\n",
                           static_cast<int>(nowStamp()),
                           static_cast<int>(raw));
      }
      uBit.sleep(kClearance);
    }
    uBit.serial.printf("N,G,X,0,0,%d\r\n", i2cErrors);
    i2cErrors = 0;
  }

  writeMotorRun(kPortLeft, kDirCw, 0);
  writeMotorRun(kPortRight, kDirCw, 0);
  uBit.serial.printf("DONE\r\n");
  uBit.display.print('E');
  while (true) uBit.sleep(1000);
}
