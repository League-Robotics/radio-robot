// trap1_persisted_otos_ordering_harness.cpp -- 132-015's own regression
// proof for trap 1 (the-configuration-object.md): a persisted OTOS scale
// must reach the (simulated) chip's own calibration-scalar register, and
// only does so when App::RobotGraph::loadPersistedTuning() runs AFTER
// App::RobotLoop::boot() -- never before.
//
// Root cause this proves closed: every Devices::RealOtos setter
// (setLinearScalar()/setAngularScalar()/setOffset()) is a no-op until
// begin() sets initialized_ = true (otos.cpp's own guards), and begin()
// itself only ever runs inside boot()'s own Preamble::step() loop
// (preamble.cpp's Otos slot). src/firm/main.cpp called
// graph.loadPersistedTuning() BEFORE graph.robotLoop().boot() prior to
// this ticket -- silently discarding any persisted OTOS tuning on every
// real boot (begin() then goes on to apply the BAKED scale from
// bootValues_.otosConfig regardless, since it never touches the offset
// registers and always writes the linear/angular scalar registers from
// its own construction-time config, independent of anything
// loadPersistedTuning() attempted moments earlier).
//
// Chip-level observable: TestSim::OtosPlant::linearScalarReg()/
// angularScalarReg() -- NOT App::Configurator::config().otos, which is
// set unconditionally by Configurator::reapplyPersistedTuning() regardless
// of whether the underlying RealOtos setter actually landed (that
// LOOKS-correct-but-isn't-applied gap is trap 1 itself: a GET_CONFIG
// read-back of the OTOS group would have reported the persisted value
// even while the real chip silently kept running the baked one). SimPlant
// (src/firm/platform/host/sim_plant.cpp handleOtosWrite()) captures a real firmware
// write to the chip's REG_SCALAR_LINEAR/REG_SCALAR_ANGULAR registers into
// OtosPlant -- a genuine round trip through App::configureOtos()'s
// scaleToRegister() conversion and Devices::RealOtos's own bus-write
// path, not a shortcut through Configurator's cached copy.
//
// Two scenarios:
//   1. loadPersistedTuning() called AFTER boot() (132-015's fixed
//      order, matching main.cpp) -- the persisted scale reaches the
//      chip-level register.
//   2. loadPersistedTuning() called BEFORE boot() (the pre-132-015
//      order) -- the persisted scale is silently discarded; the register
//      is left at whatever begin() baked (identity, register 0, per
//      SimHarness's own kIdentityOtosConfig override).
//
// Compiled by test_trap1_persisted_otos_ordering.py against the same full
// HOST_BUILD dependency graph every other post-gut sim/unit harness
// compiles (SimHarness composes the real App::RobotLoop graph -- see
// sim_harness.h's own header).
#include <cmath>
#include <cstdio>
#include <string>

#include "config/persisted_tuning.h"
#include "devices/otos.h"
#include "sim_harness.h"

namespace {

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkIntEq(int actual, int expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %d, got %d", what.c_str(), expected, actual);
    fail(buf);
  }
}

// SeededTuningStore -- a trivial, no-flash Config::TuningStore double that
// answers load() with a PRE-POPULATED snapshot (mirrors
// app_robot_loop_harness.cpp's own MockTuningStore, which only ever
// answers load() with "never written" -- this test needs the opposite: a
// store that already holds data before boot, matching a real robot
// rebooting with a prior session's persisted tuning still in flash).
class SeededTuningStore : public Config::TuningStore {
 public:
  explicit SeededTuningStore(const Config::TuningSnapshot& snapshot)
      : blob_(Config::serializeSnapshot(snapshot)) {}

  bool load(uint32_t* outVersion, Config::Blob* outBlob) override {
    *outVersion = Config::kConfigSchemaVersion;
    *outBlob = blob_;
    return true;
  }

  void save(uint32_t version, const Config::Blob& blob) override {
    ++saveCount;
    lastVersion = version;
    lastBlob = blob;
  }

  void wipe() override { ++wipeCount; }

  Config::Blob blob_;
  int saveCount = 0;
  int wipeCount = 0;
  uint32_t lastVersion = 0;
  Config::Blob lastBlob{};
};

// A persisted OTOS snapshot whose linear/angular scale both differ from
// SimHarness's own baked identity default (kIdentityOtosConfig: 1.0/1.0,
// scaleToRegister(1.0) == register 0) -- offset fields are left at 0
// (SimPlant's OtosPlant does not model the OFFSET register at all, see
// otos_plant.h's own header; the register round trip this test needs only
// exists for the linear/angular SCALAR registers, sim_plant.cpp's own
// handleOtosWrite()).
Config::TuningSnapshot persistedOtosSnapshot() {
  Config::TuningSnapshot snapshot;
  snapshot.otosTuned = true;
  snapshot.otosLinearScale = 1.05f;
  snapshot.otosAngularScale = 0.98f;
  return snapshot;
}

}  // namespace

int main() {
  std::printf("=== 132-015 trap 1: persisted OTOS tuning vs. boot() ordering ===\n\n");

  // --- Scenario 1: loadPersistedTuning() AFTER boot() -- the 132-015 fix,
  //     matching main.cpp's own corrected order -- delivers the persisted
  //     scale to the chip-level register. ---
  {
    beginScenario("loadPersistedTuning() AFTER boot() lands the persisted OTOS scale on the "
                  "chip-level register");
    SeededTuningStore store(persistedOtosSnapshot());
    TestSim::SimHarness sim(TestSim::kDefaultTrackWidth, &store);

    sim.boot();  // begin() runs here -- initialized_ becomes true, baked identity scale written
    sim.loadPersistedTuning();  // 132-015 order: runs AFTER boot(), setLinearScalar() now lands

    const int8_t expectedLinearReg = Devices::scaleToRegister(1.05f);
    const int8_t expectedAngularReg = Devices::scaleToRegister(0.98f);
    checkIntEq(sim.plant().otosPlant().linearScalarReg(), expectedLinearReg,
               "chip-level linear scalar register reflects the PERSISTED value, not baked identity");
    checkIntEq(sim.plant().otosPlant().angularScalarReg(), expectedAngularReg,
               "chip-level angular scalar register reflects the PERSISTED value, not baked identity");
  }

  // --- Scenario 2: loadPersistedTuning() BEFORE boot() -- the pre-132-015
  //     order -- silently discards the persisted scale: RealOtos's setter
  //     no-ops (initialized_ still false), then begin() applies the BAKED
  //     identity scale during boot(), and nothing ever re-applies the
  //     persisted value afterward. Demonstrates the ORDERING itself is
  //     what mattered, not merely that loadPersistedTuning() was called
  //     somewhere. ---
  {
    beginScenario("loadPersistedTuning() BEFORE boot() (pre-132-015 order) silently discards "
                  "the persisted OTOS scale");
    SeededTuningStore store(persistedOtosSnapshot());
    TestSim::SimHarness sim(TestSim::kDefaultTrackWidth, &store);

    sim.loadPersistedTuning();  // pre-132-015 order: RealOtos setter no-ops, initialized_ false
    sim.boot();                 // begin() now overwrites with the baked identity scale regardless

    const int8_t bakedIdentityReg = Devices::scaleToRegister(1.0f);  // == 0
    checkIntEq(sim.plant().otosPlant().linearScalarReg(), bakedIdentityReg,
               "chip-level linear scalar register is left at the BAKED default -- the persisted "
               "1.05 scale never reached the chip");
    checkIntEq(sim.plant().otosPlant().angularScalarReg(), bakedIdentityReg,
               "chip-level angular scalar register is left at the BAKED default -- the persisted "
               "0.98 scale never reached the chip");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all trap-1 persisted-OTOS-ordering scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the trap-1 persisted-OTOS-ordering scenarios\n",
              g_failureCount);
  return 1;
}
