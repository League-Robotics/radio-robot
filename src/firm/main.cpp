// main.cpp -- the ARM entry point. Owns the real MicroBit hardware
// singleton, constructs the ONE leaf that differs from the sim build (the
// real Devices::MicroBitI2CBus), and hands everything else to
// App::composeRobot() (app/boot_wiring.h) -- the SAME composition root
// TestSim::SimHarness (src/sim/sim_harness.h) calls. 130-002 (unify-sim-
// and-robot-composition-roots.md): before this ticket, this file hand-
// wired the whole App::/Motion:: graph itself, and TestSim::SimHarness
// hand-wired an independently-drifted copy of the same graph. See
// boot_wiring.h's own header for the composition-root contract and why
// that drift is now structurally impossible.
//
// No cycle logic lives here, and (as of this file's own extraction) no
// graph-construction logic either. Design/rationale: DESIGN.md.
#include "MicroBit.h"

#include <cstdio>
#include <cstring>

#include "app/boot_wiring.h"
#include "app/comms.h"
#include "com/banner.h"
#include "com/radio.h"
#include "com/serial_port.h"
#include "config/boot_config.h"
#include "config/persisted_tuning.h"
#include "devices/microbit_clock.h"
#include "devices/microbit_i2c_bus.h"
#include "types/version_generated.h"

static MicroBit uBit;

namespace {

// The boot tag: the DAY of the version's date field, then the build number.
// "0.20260726.1" -> "261" (day 26, build 1).
//
// A single trailing digit cannot distinguish two builds made on different
// days -- the board showed "1" for both 0.20260726.1 and 0.20260729.1, which
// is exactly the confusion that costs bench time. Day+build stays short
// enough to read off the matrix at a glance while being unique across any
// plausible session (stakeholder directive 2026-07-29).
//
// Parsed rather than indexed at fixed offsets, so it survives a format change
// in gen_version.py; emits "?" if the string lacks the two dots the
// major.date.build shape requires.
void versionTag(char* out, size_t cap) {
  if (cap == 0) return;
  const char* firstDot = nullptr;
  const char* lastDot = nullptr;
  for (const char* p = FIRMWARE_VERSION_STR; *p != '\0'; ++p) {
    if (*p == '.') {
      if (firstDot == nullptr) firstDot = p;
      lastDot = p;
    }
  }
  size_t n = 0;
  // Two distinct dots, and a date field of at least two characters to take a
  // day from.
  if (firstDot != nullptr && lastDot != firstDot && (lastDot - firstDot) >= 3) {
    if (n + 1 < cap) out[n++] = *(lastDot - 2);
    if (n + 1 < cap) out[n++] = *(lastDot - 1);
    for (const char* p = lastDot + 1; *p != '\0' && n + 1 < cap; ++p) out[n++] = *p;
  }
  if (n == 0 && cap > 1) out[n++] = '?';
  out[n] = '\0';
}

// Boot identity on the LED matrix: heart, the last digit of the firmware
// version, then the heart again -- and the heart STAYS LIT.
//
// This is the only way to tell WHICH build is actually on the board without
// opening a serial session, and "did that flash land?" is a question that
// has cost real debugging time. The trailing heart is the resting state: a
// lit display through boot means "powered, flashed, and running"; a dark
// one means it never got here.
//
// It is not left on forever. main() disables the display the instant boot
// completes and before the first control cycle: the LED matrix driver
// refreshes continuously off its own timer, and the loop runs to a measured
// ~47 ms budget whose motion tuning assumes those cycles are not being
// spent elsewhere. Boot is the one window where the cost is free.
void showBootIdentity() {
  // Pixel values are BRIGHTNESS 0..255, not booleans -- a 1 here renders
  // at 1/255 duty, i.e. invisibly dim next to the font's full-bright
  // digit (stakeholder observed "only the 1, no hearts", 2026-07-27).
  constexpr uint8_t kOn = 255;
  static const uint8_t kHeart[] = {
      0,   kOn, 0,   kOn, 0,
      kOn, kOn, kOn, kOn, kOn,
      kOn, kOn, kOn, kOn, kOn,
      0,   kOn, kOn, kOn, 0,
      0,   0,   kOn, 0,   0,
  };
  constexpr int kHeartHold = 500;  // [ms]
  constexpr int kDigitHold = 700;  // [ms] per digit -- it is the payload
  constexpr int kDigitGap = 150;   // [ms] blank between digits, so a repeated
                                   // digit ("22") reads as two, not one long

  uBit.display.enable();
  MicroBitImage heart(5, 5, kHeart);
  uBit.display.print(heart);
  uBit.sleep(kHeartHold);
  uBit.display.clear();

  char tag[8];
  versionTag(tag, sizeof(tag));
  for (const char* p = tag; *p != '\0'; ++p) {
    uBit.display.printChar(*p);
    uBit.sleep(kDigitHold);
    uBit.display.clear();
    uBit.sleep(kDigitGap);
  }

  uBit.display.print(heart);  // resting state -- left lit through boot
}

}  // namespace

int main() {
  uBit.init();

  // Before anything touches the buses: say which build this is.
  showBootIdentity();

  static SerialPort serial(uBit.serial);
  serial.begin();
  static Radio radio(uBit.radio, uBit.messageBus);
  radio.begin();

  static char banner[64];
  formatBanner(banner, sizeof(banner));
  serial.sendReliable(banner);
  radio.send(reinterpret_cast<const uint8_t*>(banner), static_cast<uint16_t>(std::strlen(banner)));

  // ID:<drivetrain>:<profile>:<version> -- configured-robot identity
  // (drivetrain type + calibration-profile name/version), distinct from
  // `banner`'s hardware identity. Both Config::kDrivetrainType and
  // Config::kRobotProfileName are generated string constants
  // (boot_config.h) baked from the robot JSON's own
  // identity.drivetrain_type/filename stem.
  static char idLine[96];
  snprintf(idLine, sizeof(idLine), "ID:%s:%s:%s", Config::kDrivetrainType,
           Config::kRobotProfileName, FIRMWARE_VERSION_STR);

  // The ONE leaf that differs from the sim composition root: the real I2C
  // bus. Everything downstream of `bus` is built by App::composeRobot(),
  // the SAME function TestSim::SimHarness calls with a TestSim::SimPlant
  // in this slot instead (app/boot_wiring.h).
  static Devices::MicroBitI2CBus bus(uBit.i2c);
  static Devices::MicroBitClock clock;
  static Devices::MicroBitSleeper sleeper;
  static App::SerialTransport serialLink(serial);
  static App::RadioTransport radioLink(radio);
  static Config::MicroBitTuningStore tuningStore(uBit.storage);

  static App::RobotGraph graph = App::composeRobot(bus, clock, sleeper, serialLink, radioLink,
                                                   &tuningStore, banner, idLine);

  graph.loadPersistedTuning();
  graph.robotLoop().markConfigured();

  // RobotLoop::run() is boot() followed by cycle() forever; it is spelled
  // out here instead so the display can be turned off in between.
  graph.robotLoop().boot();

  // Boot is done and the first control cycle is next: give the LED matrix's
  // refresh timer back to the loop. Everything from here runs to the
  // measured budget App::RobotGraph baked from the robot JSON (see
  // Motion::PlannerLimits::controlPeriod), and the motion tuning assumes
  // those cycles are the loop's.
  uBit.display.clear();
  uBit.display.disable();

  for (;;) {
    graph.robotLoop().cycle();
  }
}
