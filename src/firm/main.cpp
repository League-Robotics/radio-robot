// main.cpp -- the ARM entry point. Owns the real MicroBit hardware
// singleton, constructs the ONE leaf that differs from the sim build (the
// real Platform::MicroBitI2CBus), and hands everything else to
// Core::composeRobot() (app/boot_wiring.h) -- the SAME composition root
// TestSim::SimHarness (src/firm/platform/host/sim_harness.h) calls. 130-002 (unify-sim-
// and-robot-composition-roots.md): before this ticket, this file hand-
// wired the whole Core::/Motion:: graph itself, and TestSim::SimHarness
// hand-wired an independently-drifted copy of the same graph. See
// boot_wiring.h's own header for the composition-root contract and why
// that drift is now structurally impossible.
//
// No cycle logic lives here, and (as of this file's own extraction) no
// graph-construction logic either. Design/rationale: DESIGN.md.
#include "MicroBit.h"

#include <cstring>

#include "core/boot_wiring.h"
#include "config/boot_config.h"
#include "config/persisted_tuning.h"
#include "platform/microbit/microbit_banner.h"
#include "platform/microbit/microbit_boot_identity.h"
#include "platform/microbit/microbit_clock.h"
#include "platform/microbit/microbit_i2c_bus.h"
#include "platform/microbit/microbit_radio_link.h"
#include "platform/microbit/microbit_serial_port.h"
#include "types/version_generated.h"
#include "types/version_tag.h"

static MicroBit uBit;

int main() {
  uBit.init();

  // Before anything touches the buses: say which build this is. The tag is
  // computed HERE (not inside showBootIdentity() itself) because
  // platform/microbit/ may not reach types/ (test_layer_isolation.py's
  // platform-layer allowed-include set is platform/+hal/ only) -- main.cpp
  // is the one place FIRMWARE_VERSION_STR and the display routine are both
  // in scope.
  char bootTag[8];
  Types::versionTag(FIRMWARE_VERSION_STR, bootTag, sizeof(bootTag));
  Platform::showBootIdentity(uBit, bootTag);

  static Platform::MicroBitSerialPort serial(uBit.serial);
  serial.begin();
  static Platform::MicroBitRadioLink radio(uBit.radio, uBit.messageBus);
  // Channel from the robot JSON's connection.radio_channel (baked --
  // Config::kRadioChannel, default 0). Bake-only by design: no wire verb
  // can retune it, so two robots sharing a bench stay on the channels
  // their own config files say.
  radio.begin(Config::kRadioChannel);

  static char banner[64];
  Platform::formatBanner(banner, sizeof(banner));
  serial.sendReliable(banner);
  radio.send(reinterpret_cast<const uint8_t*>(banner), static_cast<uint16_t>(std::strlen(banner)));

  // ID:<drivetrain>:<profile>:<version> -- configured-robot identity
  // (drivetrain type + calibration-profile name/version), distinct from
  // `banner`'s hardware identity. Both Config::kDrivetrainType and
  // Config::kRobotProfileName are generated string constants
  // (boot_config.h) baked from the robot JSON's own
  // identity.drivetrain_type/filename stem.
  static char idLine[96];
  Platform::formatIdLine(idLine, sizeof(idLine), Config::kDrivetrainType,
                          Config::kRobotProfileName, FIRMWARE_VERSION_STR);

  // The ONE leaf that differs from the sim composition root: the real I2C
  // bus. Everything downstream of `bus` is built by Core::composeRobot(),
  // the SAME function TestSim::SimHarness calls with a TestSim::SimPlant
  // in this slot instead (app/boot_wiring.h). `serial`/`radio` above
  // implement Hal::Transport directly (136-005, "dissolve com/ into
  // Hal::Transport + platform/microbit/") -- no separate adapter objects
  // to construct here any more.
  static Platform::MicroBitI2CBus bus(uBit.i2c);
  static Platform::MicroBitClock clock;
  static Platform::MicroBitSleeper sleeper;
  static Config::MicroBitTuningStore tuningStore(uBit.storage);

  static Core::RobotGraph graph = Core::composeRobot(bus, clock, sleeper, serial, radio,
                                                   &tuningStore, banner, idLine);

  // RobotLoop::run() is boot() followed by cycle() forever; it is spelled
  // out here instead so the display can be turned off in between.
  graph.robotLoop().boot();

  // 132-015 (trap 1, the-configuration-object.md): loadPersistedTuning()
  // MUST run AFTER boot(), not before -- the pre-132-015 order here. Every
  // Hardware::RealOtos setter reapplyPersistedTuning()'s own OTOS branch
  // reaches (Core::configureOtos() -> setOffset()/setLinearScalar()/
  // setAngularScalar(), otos.cpp) is a no-op until RealOtos::begin() sets
  // initialized_ = true, and begin() itself only ever runs inside boot()'s
  // own Preamble::step() loop (preamble.cpp's Otos slot) -- calling
  // loadPersistedTuning() first meant a persisted OTOS scale/offset was
  // SILENTLY discarded on every real boot: the write no-op'd here, and
  // begin() then applied the BAKED scale anyway (it does not touch the
  // offset registers at all), so nothing downstream ever saw the failure.
  // begin() itself cannot be re-run to "fix" a too-early apply instead --
  // it zeroes the chip's tracked pose and kicks IMU bias calibration,
  // so re-running it would teleport the OTOS origin out from under
  // odometry -- so the ordering itself had to move, not be worked around.
  // boot() itself never reads tuningStore_/config_.otos, so this reorder
  // changes nothing about what boot() does; it only changes whether the
  // persisted OTOS values reach the chip once boot() is done. markConfigured()
  // only needs to land before the first cycle() call below, so it moves
  // down here with loadPersistedTuning() rather than staying split apart.
  graph.loadPersistedTuning();
  graph.robotLoop().markConfigured();

  // Boot is done and the first control cycle is next: give the LED matrix's
  // refresh timer back to the loop. Everything from here runs to the
  // measured budget Core::RobotGraph baked from the robot JSON (see
  // Motion::PlannerLimits::controlPeriod), and the motion tuning assumes
  // those cycles are the loop's.
  uBit.display.clear();
  uBit.display.disable();

  for (;;) {
    graph.robotLoop().cycle();
  }
}
