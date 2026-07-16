// scripted_i2c_hook.h -- TestSim::ScriptedI2CHook: FIFO write/read scripting
// implemented as a TestSim::SimPlant read/write hook pair.
//
// Ticket 108-009 (clasi/sprints/108-pure-i2cbus-clock-interfaces-and-a-real-
// simplant-simulator-sim-mode-tours/tickets/009-migrate-the-13-register-
// level-unit-tests-to-python-simplant-hook-tests-delete-c-harnesses.md).
// Sprint 108 deleted the concrete Devices::I2CBus's HOST_BUILD scripted-FIFO
// fake (source/devices/i2c_bus_host.cpp, ticket 001) in favor of a pure
// Devices::I2CBus interface plus TestSim::SimPlant -- a REAL, physics-backed
// implementation with a read/write hook seam (tests/_infra/sim/sim_plant.h).
// Several tests/sim/unit/ harnesses ported by this ticket still need EXACT,
// deterministic per-call register-level control (a specific NAK, a specific
// encoder count, an exact transaction-count budget) that SimPlant's live
// physics responses cannot give directly -- SimPlant answers with whatever
// its own WheelPlant/OtosPlant actually computed, not an arbitrary scripted
// byte sequence. This class reproduces the deleted fake's exact
// FIFO-scripting semantics (queueWrite()/queueRead(), txnCount()/
// errCount()/lastErr(), the "an unscripted call returns a distinct mismatch
// status rather than crashing" convention -- see the deleted
// source/devices/i2c_bus_host.cpp's own header comment, git history
//660d7cb2~1) -- but AS a SimPlant read/write hook, following
// architecture-update.md Decision 1's "the hook (middleware, on SimPlant --
// not on I2CBus)" design, not as a second concrete Devices::I2CBus.
//
// Usage: construct one ScriptedI2CHook per TestSim::SimPlant instance --
// installs itself as that plant's read/write hook for its own lifetime.
// Script expected calls with queueWrite()/queueRead(), then pass the SAME
// SimPlant reference (NOT this object) to whatever Devices:: leaf is under
// test -- SimPlant::write()/read() dispatch through this hook automatically
// because SimPlant IS-A Devices::I2CBus. The destructor clears both hooks so
// a SimPlant that outlives this object is left clean.
//
// Source placement: HOST_BUILD-only test infrastructure, alongside the
// harnesses that use it (tests/sim/unit/) -- mirrors sim_plant.h's own
// "this file does NOT live in source/" placement note.
#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include "sim_plant.h"

namespace TestSim {

class ScriptedI2CHook {
 public:
  // Distinct from any real CODAL status -- matches the deleted
  // i2c_bus_host.cpp's own kScriptMismatch convention exactly: an
  // unscripted call, or a wrong-address scripted call, returns this rather
  // than silently returning "OK" or crashing the test process.
  static constexpr int kScriptMismatch = -100;

  explicit ScriptedI2CHook(SimPlant& plant) : plant_(plant) {
    plant_.setWriteHook([this](uint16_t addr, uint8_t* data, int len) {
      return onWrite(addr, data, len);
    });
    plant_.setReadHook([this](uint16_t addr, uint8_t* data, int len) {
      return onRead(addr, data, len);
    });
  }

  ~ScriptedI2CHook() {
    plant_.clearWriteHook();
    plant_.clearReadHook();
  }

  ScriptedI2CHook(const ScriptedI2CHook&) = delete;
  ScriptedI2CHook& operator=(const ScriptedI2CHook&) = delete;

  // Script the next write() call (FIFO order): the wrapped SimPlant returns
  // `status` (0 == OK) for it instead of running its own default protocol
  // handler. A write() whose address doesn't match what was scripted, or
  // one with no script queued, returns kScriptMismatch.
  void queueWrite(uint16_t address, int status = 0) {
    scriptedWrites_.push_back(ScriptedWrite{address, status});
  }

  // Script the next read() call (FIFO order): returns `status` and copies
  // up to `len` bytes of `data` into the caller's buffer.
  void queueRead(uint16_t address, const uint8_t* data, int len, int status = 0) {
    ScriptedRead entry;
    entry.addr = address;
    entry.status = status;
    if (data != nullptr && len > 0) {
      entry.data.assign(data, data + static_cast<size_t>(len));
    }
    scriptedReads_.push_back(entry);
  }

  // Per-device statistics, keyed by the bare 7-bit device address (same
  // convention as the deleted fake's own txnCount()/errCount()/lastErr()).
  uint32_t txnCount(uint16_t addr7) const { return findSlot(addr7).txnCount; }
  uint32_t errCount(uint16_t addr7) const { return findSlot(addr7).errCount; }
  int lastErr(uint16_t addr7) const { return findSlot(addr7).lastErr; }

 private:
  struct ScriptedWrite {
    uint16_t addr;  // expected 8-bit wire address
    int status;
  };
  struct ScriptedRead {
    uint16_t addr;               // expected 8-bit wire address
    std::vector<uint8_t> data;   // canned response bytes
    int status;
  };
  struct DeviceSlot {
    uint16_t addr = 0;  // 7-bit address; 0 == never touched (no real device uses 0)
    uint32_t txnCount = 0;
    uint32_t errCount = 0;
    int lastErr = 0;
  };

  int onWrite(uint16_t address, uint8_t* /*data*/, int /*len*/) {
    int status = kScriptMismatch;
    if (!scriptedWrites_.empty()) {
      ScriptedWrite expected = scriptedWrites_.front();
      scriptedWrites_.pop_front();
      status = (expected.addr == address) ? expected.status : kScriptMismatch;
    }
    record(static_cast<uint16_t>(address >> 1), status);
    return status;
  }

  int onRead(uint16_t address, uint8_t* data, int len) {
    int status = kScriptMismatch;
    if (!scriptedReads_.empty()) {
      ScriptedRead expected = scriptedReads_.front();
      scriptedReads_.pop_front();
      if (expected.addr == address) {
        status = expected.status;
        int copyLen = (len < static_cast<int>(expected.data.size()))
                          ? len
                          : static_cast<int>(expected.data.size());
        for (int i = 0; i < copyLen; ++i) {
          data[i] = expected.data[static_cast<size_t>(i)];
        }
      }
    }
    record(static_cast<uint16_t>(address >> 1), status);
    return status;
  }

  void record(uint16_t addr7, int status) {
    DeviceSlot& slot = mutableSlot(addr7);
    ++slot.txnCount;
    if (status != 0) {  // 0 == OK, matching every other harness's own convention.
      ++slot.errCount;
      slot.lastErr = status;
    }
  }

  DeviceSlot& mutableSlot(uint16_t addr7) {
    for (auto& slot : devices_) {
      if (slot.addr == addr7) return slot;
    }
    devices_.push_back(DeviceSlot{addr7, 0, 0, 0});
    return devices_.back();
  }

  DeviceSlot findSlot(uint16_t addr7) const {
    for (const auto& slot : devices_) {
      if (slot.addr == addr7) return slot;
    }
    return DeviceSlot{};  // never touched -- all-zero, matching the deleted fake's convention.
  }

  SimPlant& plant_;
  std::deque<ScriptedWrite> scriptedWrites_;
  std::deque<ScriptedRead> scriptedReads_;
  std::vector<DeviceSlot> devices_;
};

}  // namespace TestSim
