// persisted_tuning.h -- Config::PersistedTuning (114-004, SUC-003;
// planner slot removed 115-004, gut S1): a version-stamped store for the
// live-pushed CFG patch fields that already survive a wire CFG call --
// MotorConfigPatch's gains/travel_calib, OtosConfigPatch's scale/offset
// fields -- persisted across a POWER CYCLE, WIPED across a FIRMWARE
// VERSION bump. Does not widen the live-tunable set (sprint.md Open
// Question 1; sprint 113's own Decision 1 already rejected that) -- this
// module persists exactly what a CFG patch can already change live, and
// nothing else.
//
// Mirrors com/radio_channel.h's own MicroBitStorage precedent, corrected
// per sprint 114's own architecture self-review: that precedent #includes
// MicroBit.h directly and is not host-testable at all. This module instead
// follows app/comms.h's App::Transport pattern -- a plain virtual
// TuningStore base (never an #ifdef HOST_BUILD fork) plus ONE concrete
// ARM-only adapter, itself guarded -- so persisted_tuning.h/.cpp never drag
// in MicroBit.h under HOST_BUILD, only the concrete adapter's own
// declaration/definition do.
//
// Three layers, in order of how much of this file is actually testable:
//   1. kConfigSchemaVersion, TuningSnapshot, serializeSnapshot()/
//      deserializeSnapshot(), shouldWipe() -- pure, no I/O, no
//      MicroBitStorage dependency at all. Host-testable
//      (src/tests/sim/unit/persisted_tuning_harness.cpp).
//   2. TuningStore -- the abstract persistence seam App::RobotLoop saves
//      through and main.cpp's boot sequence loads/wipes through. Also
//      host-testable via a trivial mock (app_robot_loop_harness.cpp's own
//      write-policy scenario) -- it is a plain interface, no hardware.
//   3. MicroBitTuningStore (guarded, #ifndef HOST_BUILD) -- the real
//      ARM-only MicroBitStorage-backed implementation. NOT exercised by
//      any agent-run test (MicroBitStorage/codal::KeyValueStorage has no
//      HOST_BUILD stand-in anywhere in this tree) -- bench-only, see
//      ticket 006's stakeholder checklist.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "messages/config.h"

// MicroBitStorage (model/MicroBit.h) is `typedef codal::KeyValueStorage
// MicroBitStorage` -- a typedef cannot be forward-declared under its alias
// name (that would declare a second, conflicting type), so this header
// forward-declares the REAL underlying type instead, the same reason
// app/comms.h forward-declares SerialPort/Radio rather than #including
// their real headers: this header stays MicroBit-free even when compiled
// for ARM.
#ifndef HOST_BUILD
namespace codal { class KeyValueStorage; }
#endif

namespace Config {

// kConfigSchemaVersion -- bumped whenever a persisted field's meaning
// changes (rename, unit change, or a curated-field-set change). A version
// mismatch at boot wipes the ENTIRE store rather than reapplying a patch
// whose fields may mean something different now (SUC-003). Bump
// discipline itself (WHEN a future change must bump this) is sprint.md's
// own Open Question 2 -- a documentation follow-up, not resolved here.
//
// Bumped 1 -> 2 by 115-004 (gut S1): the planner slot was dropped from
// TuningSnapshot below, shrinking kBlobSize (110 -> 85 bytes). Without
// this bump, deserializeSnapshot() would read an old 110-byte blob's
// now-shifted trailing bytes into the new 85-byte layout, silently
// corrupting the OTOS-calibration fields with what used to be the
// planner term's floats -- memory-safe but behaviorally wrong. The bump
// instead triggers shouldWipe()'s clean wipe path at boot.
constexpr uint32_t kConfigSchemaVersion = 2;

// TuningSnapshot -- exactly the fields a live CFG patch can already
// change. One MotorConfigPatch slot per side: travel_calib is
// side-selected on the wire (config.proto's own BoundMotorSide), while
// kp/ki/kff/i_max/kaw mirror onto BOTH bound motors (App::RobotLoop::
// handleConfig()'s own existing merge, unchanged by this ticket) -- so
// persisting one shared copy would silently lose a legitimate per-side
// travel_calib divergence the instant one is ever pushed. `side` itself is
// bookkeeping only (LEFT for motorL, RIGHT for motorR) -- deserializeSnapshot()
// always stamps it; it is never read back out of the blob.
//
// Every field starts `Opt<T>{has=false}` -- a fresh snapshot means
// "nothing has been live-tuned since boot," matching a freshly-booted
// device exactly (nothing to persist, nothing to reapply).
//
// `OtosConfigPatch::init` is deliberately never populated by
// deserializeSnapshot() and never read by serializeSnapshot() -- it is a
// one-shot IMU-calibration TRIGGER on the wire, not a value; persisting
// and replaying it at every boot would re-fire chip calibration
// pointlessly. See robot_loop.cpp's own mergeOtosPatch()/applyOtosPatch()
// for where this exclusion is enforced.
struct TuningSnapshot {
  msg::MotorConfigPatch motorL = {};  // side stays LEFT
  msg::MotorConfigPatch motorR = {};  // side stays RIGHT
  msg::OtosConfigPatch otos = {};     // .init never persisted -- see above
};

// --- Byte layout -----------------------------------------------------------
//
// Each Opt<float> field packs as 1 byte (has) + 4 bytes (float, raw bit
// pattern via memcpy) = kOptFloatBytes. Fixed-width, per-field packing --
// NOT a raw struct memcpy (struct padding/alignment is not portably
// deterministic across compilers/optimization levels, unlike the
// generated msg:: structs' own std::is_standard_layout guarantee, which
// covers layout but not padding-byte content).
constexpr size_t kOptFloatBytes = 5;      // 1 (has) + 4 (float)
constexpr size_t kMotorPatchFields = 6;   // travel_calib, kp, ki, kff, i_max, kaw
constexpr size_t kOtosPatchFields = 5;    // linear_scale, angular_scale, offset_x, offset_y, offset_yaw

// kBlobSize -- computed from the field-count constants above (not a magic
// number) so a future curated-field-set change (sprint.md Open Question 2)
// updates this automatically instead of silently truncating. The planner
// term (kPlannerPatchFields * kOptFloatBytes) was DELETED, not zeroed, by
// 115-004 -- msg::PlannerConfigPatch no longer exists after 115-003's proto
// surgery -- so this now computes to 85 (was 110) from the remaining two
// terms.
constexpr size_t kBlobSize = (2 * kMotorPatchFields * kOptFloatBytes) +
                              (kOtosPatchFields * kOptFloatBytes);

using Blob = std::array<uint8_t, kBlobSize>;

// serializeSnapshot()/deserializeSnapshot() -- pure, byte blob in/out, NO
// I/O of any kind, NO MicroBitStorage dependency. Round-trips exactly
// (SUC-003's own acceptance criterion): for any TuningSnapshot s built
// from serializeSnapshot()/deserializeSnapshot()'s own field set,
// deserializeSnapshot(serializeSnapshot(s)) reproduces s's field values.
Blob serializeSnapshot(const TuningSnapshot& snapshot);
TuningSnapshot deserializeSnapshot(const Blob& blob);

// shouldWipe -- the version-compare-and-wipe DECISION. Pure and trivial
// (storedVersion != currentVersion), but named and unit-tested explicitly
// so the wipe decision is one greppable, tested unit rather than an
// inline `!=` wherever it is needed (this ticket's own Approach).
bool shouldWipe(uint32_t storedVersion, uint32_t currentVersion);

// TuningStore -- the persistence seam App::RobotLoop::handleConfig()
// saves through and main.cpp's boot sequence loads/wipes through. Plain
// virtual base (not an #ifdef HOST_BUILD fork) -- mirrors App::Transport
// (app/comms.h): this header/its .cpp never drag in MicroBit.h under
// HOST_BUILD; only the concrete ARM adapter below is guarded.
//
// No sim/host implementation of this interface exists anywhere in this
// tree, by design -- sprint.md's own note: "Config::PersistedTuning/
// MicroBitStorage has no sim counterpart -- the sim has no flash ...
// vacuous by construction, not an oversight." App::RobotLoop treats a
// null TuningStore* as "persistence disabled," which is every sim/test
// composition root's own case (none of the 26 existing SimHarness
// construction sites pass one). A test that DOES want to observe the
// write-policy seam (app_robot_loop_harness.cpp's own debounce scenario)
// supplies a trivial call-counting mock -- this interface, being a plain
// C++ virtual base with no hardware dependency, is itself mockable under
// HOST_BUILD even though its one real implementation is not.
class TuningStore {
 public:
  virtual ~TuningStore() = default;

  // True + fills outVersion/outBlob if a stamped blob was ever written.
  // False if the store is empty (never written, or wiped).
  virtual bool load(uint32_t* outVersion, Blob* outBlob) = 0;

  virtual void save(uint32_t version, const Blob& blob) = 0;

  // Erases the persisted blob entirely. Called by main.cpp's boot
  // sequence only when something WAS stored at a mismatched version
  // (SUC-003: "wipe, proceed on boot-bake alone") -- never called on an
  // already-empty store (nothing to erase).
  virtual void wipe() = 0;
};

#ifndef HOST_BUILD

// MicroBitTuningStore -- the real ARM-only adapter, MicroBitStorage
// (codal::KeyValueStorage) backed, mirroring com/radio_channel.h's own
// load()/save() precedent under NEW, dedicated keys (does not collide
// with radiochan's own key). codal::KeyValueStorage's hard limits
// (KEY_VALUE_STORAGE_VALUE_SIZE=32 bytes/key, KEY_VALUE_STORAGE_MAX_PAIRS=5
// keys total, shared with radiochan's own 1) mean a single kBlobSize+
// version blob must be CHUNKED across multiple keys -- see the .cpp's own
// kNumChunks static_assert, which fails the ARM build loudly if a future
// field addition ever outgrows the available chunk budget, rather than
// silently truncating.
//
// NOT exercised by any agent-run test (no MicroBitStorage/
// codal::KeyValueStorage stand-in exists under HOST_BUILD anywhere in
// this tree) -- covered only by ticket 006's stakeholder bench checklist;
// see this file's own header note above.
class MicroBitTuningStore : public TuningStore {
 public:
  // storage is typically `uBit.storage` (MicroBitStorage ==
  // codal::KeyValueStorage, model/MicroBit.h's own typedef).
  explicit MicroBitTuningStore(codal::KeyValueStorage& storage);

  bool load(uint32_t* outVersion, Blob* outBlob) override;
  void save(uint32_t version, const Blob& blob) override;
  void wipe() override;

 private:
  codal::KeyValueStorage& storage_;
};

#endif  // HOST_BUILD

}  // namespace Config
