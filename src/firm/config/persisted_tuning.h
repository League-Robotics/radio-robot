// persisted_tuning.h -- Config::PersistedTuning: a version-stamped store
// for the live-pushed config fields that already survive a wire push --
// persisted across a POWER CYCLE, WIPED across a FIRMWARE VERSION bump.
// Does not widen the live-tunable set -- this module persists exactly the
// PRECEDENT subset a live config push could already change (see
// TuningSnapshot's own doc comment below for exactly which fields), and
// nothing else.
//
// RESHAPED, 132-013 (patch-surface retirement, the-configuration-
// object.md): TuningSnapshot used to be shaped like config.proto's own
// curated Motor/Otos live-tuning wire messages (config.proto itself now
// deleted) with per-FIELD `Opt<T>` presence -- one bool+float pair per
// field, merged onto a cumulative snapshot by a hand-written accumulator
// (`mergeMotorGainsPatch`/`mergeOtosPatch`, configurator.cpp, also
// deleted). Now it is a flat, plain-data snapshot of the SAME fields'
// Config::Robot homes (config/robot.h), with per-GROUP (not per-field)
// presence -- a group's OWN "tuned" flag, not a flag per field inside it.
// This mirrors applyGroup()'s own "no patch, no presence flags, no merge"
// wire philosophy: since a live push already replaces a whole group's
// worth of fields atomically (or, for applyField(), leaves a group's OTHER
// fields at whatever config_ already held), persistIfEligible()
// (configurator.cpp) simply reads a persist-eligible group's CURRENT full
// state out of config_ after a successful install() -- there is no
// separate merge state to maintain. Per-GROUP presence still exists
// (`wheelControlTuned`/`motorsTravelCalibTuned`/`otosTuned` below) for one
// reason ONLY: reapplyPersistedTuning() (configurator.cpp) must not
// overwrite an UNTOUCHED group's freshly-loaded baked default with a
// zero-initialized snapshot field just because some OTHER group was live
// on this robot at some point -- the three groups' presence is tracked
// independently for exactly this reason.
//
// Mirrors com/radio_channel.h's own MicroBitStorage precedent, except that
// precedent #includes MicroBit.h directly and is not host-testable at
// all. This module instead follows hal/transport.h's Hal::Transport
// pattern -- a plain virtual TuningStore base (never an #ifdef HOST_BUILD fork)
// plus ONE concrete ARM-only adapter, itself guarded -- so
// persisted_tuning.h/.cpp never drag in MicroBit.h under HOST_BUILD, only
// the concrete adapter's own declaration/definition do.
//
// Three layers, in order of how much of this file is actually testable:
//   1. kConfigSchemaVersion, TuningSnapshot, serializeSnapshot()/
//      deserializeSnapshot(), shouldWipe() -- pure, no I/O, no
//      MicroBitStorage dependency at all. Host-testable
//      (src/tests/sim/unit/persisted_tuning_harness.cpp).
//   2. TuningStore -- the abstract persistence seam Core::RobotLoop saves
//      through and main.cpp's boot sequence loads/wipes through. Also
//      host-testable via a trivial mock (app_robot_loop_harness.cpp's own
//      write-policy scenario) -- it is a plain interface, no hardware.
//   3. MicroBitTuningStore (guarded, #ifndef HOST_BUILD) -- the real
//      ARM-only MicroBitStorage-backed implementation. NOT exercised by
//      any agent-run test (MicroBitStorage/codal::KeyValueStorage has no
//      HOST_BUILD stand-in anywhere in this tree) -- bench-only.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// No messages/ include: 132-013's reshape made TuningSnapshot plain data
// (raw float/bool fields, no msg:: type) -- config.proto (the source of
// the old curated Motor/Otos live-tuning wire-message shape) no longer
// exists.

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
// whose fields may mean something different now.
//
// Concrete illustration of why this matters: this version was bumped
// 1 -> 2 when a slot was dropped from TuningSnapshot below, shrinking
// kBlobSize. Without the bump, deserializeSnapshot() would read an old,
// larger blob's now-shifted trailing bytes into the new, smaller layout,
// silently corrupting later fields with what used to be the dropped
// term's floats -- memory-safe but behaviorally wrong. The bump instead
// triggers shouldWipe()'s clean wipe path at boot.
//
// Bumped AGAIN, 2 -> 3, by 132-013's own reshape (curated Motor/Otos
// live-tuning wire-message shape, per-field Opt<T> -> flat
// Config::Robot-groups-shaped, per-group bool) -- a curated-field-set AND
// layout change, the
// same class this constant's own doc comment above already describes.
// (There is a standing, tracked issue that a PREVIOUS field addition
// skipped this bump; this ticket does not repeat that mistake by stacking
// a second unbumped meaning-change on top of it.)
constexpr uint32_t kConfigSchemaVersion = 3;

// TuningSnapshot -- exactly the PRECEDENT subset of Config::Robot a live
// config push can already change AND that already persisted under the old
// curated-message-shaped snapshot (132-013, patch-surface retirement --
// sprint.md Out of Scope's own explicit ticket-013 acceptance criterion:
// document the persisted set, neither expand nor contract it). Flat, plain
// data -- no msg:: type, no per-field Opt<T> presence (that merge-
// accumulator shape is deleted alongside config.proto/ConfigDelta; see
// this file's own top-of-file doc comment for the reshape rationale).
//
//   WHEEL_CONTROL (Drive::ControlGains, Stage B/C) -- IN FULL. Direct
//   successor of the old curated Motor live-tuning message's kp/ki/kff/
//   i_max/kaw, which already persisted (applied to BOTH bound motors via
//   drive_.setControlGains(), never per-side).
//
//   MOTORS -- travel_calib_left/travel_calib_right ONLY, matching the old
//   curated Motor live-tuning message's travel_calib precedent exactly
//   (the one MotorConfig field Core::configureMotor() still live-applies
//   post-construction, boot_calibration.h's own doc comment).
//   fwd_sign/output_deadband/reversal_dwell/vel_* never persisted before
//   and still don't.
//
//   OTOS -- offset_x/offset_y/offset_yaw/linear_scale/angular_scale, IN
//   FULL, mirroring the old curated Otos live-tuning message's own 5
//   scale/offset fields exactly. `init` (that message's 6th field, a
//   fire-and-forget IMU-calibration TRIGGER, never a value) has no
//   Config::Robot-shaped successor at all -- Otos (config/robot.h) has no
//   such field -- and was never persisted before either, so there is
//   nothing to carry forward.
//
//   DRIVE (Stage A per-wheel correction, this sprint's OWN headline new
//   live-wire capability) and ESTIMATOR are deliberately NOT here -- Stage
//   A never had a wire arm before this sprint (no old curated message to
//   set a persistence precedent), and Estimator explicitly never persisted
//   even when it did have one (the old curated Estimator live-tuning
//   message's own doc comment: "a reboot always reverts to the baked JSON
//   default").
//
// Per-GROUP (not per-field) "tuned" flags -- true once THAT group has been
// live-pushed (whole-group OR single-field) at least once since the store
// was last wiped. Exists for ONE reason: reapplyPersistedTuning()
// (configurator.cpp) must not overwrite an untouched group's freshly
// baked default with a zero-initialized snapshot field just because some
// OTHER group was live-tuned on this robot at some point -- see this
// file's own top-of-file doc comment.
struct TuningSnapshot {
  bool wheelControlTuned = false;
  bool motorsTravelCalibTuned = false;
  bool otosTuned = false;

  float wheelControlPidKp = 0.0f;
  float wheelControlPidKi = 0.0f;
  float wheelControlPidIMax = 0.0f;
  float wheelControlPidKaff = 0.0f;
  float wheelControlPidMax = 0.0f;

  float motorsTravelCalibLeft = 0.0f;   // [mm/deg]
  float motorsTravelCalibRight = 0.0f;  // [mm/deg]

  float otosOffsetX = 0.0f;      // [mm]
  float otosOffsetY = 0.0f;      // [mm]
  float otosOffsetYaw = 0.0f;    // [rad]
  float otosLinearScale = 0.0f;
  float otosAngularScale = 0.0f;
};

// --- Byte layout -----------------------------------------------------------
//
// 3 group-presence bools (1 byte each) + 12 raw floats (4 bytes each, raw
// bit-pattern memcpy -- safe because this blob is written and read back by
// the exact same compiled binary, never sent over a wire or read by a
// different build). No per-field presence byte any more (the old
// kOptFloatBytes=5-per-field packing is gone with Opt<T> -- see this
// file's own top-of-file doc comment).
constexpr size_t kGroupPresenceBytes = 3;    // wheelControlTuned/motorsTravelCalibTuned/otosTuned
constexpr size_t kFloatFieldBytes = 4;       // raw float, no presence byte
constexpr size_t kWheelControlFields = 5;    // pid_kp/pid_ki/pid_i_max/pid_kaff/pid_max
constexpr size_t kMotorsTravelCalibFields = 2;  // travel_calib_left/right
constexpr size_t kOtosFields = 5;            // offset_x/offset_y/offset_yaw/linear_scale/angular_scale

// kBlobSize -- computed from the field-count constants above (not a magic
// number), so a future curated-field-set change updates this
// automatically instead of silently truncating.
constexpr size_t kBlobSize =
    kGroupPresenceBytes +
    (kWheelControlFields + kMotorsTravelCalibFields + kOtosFields) * kFloatFieldBytes;

using Blob = std::array<uint8_t, kBlobSize>;

// serializeSnapshot()/deserializeSnapshot() -- pure, byte blob in/out, NO
// I/O of any kind, NO MicroBitStorage dependency. Round-trips exactly:
// for any TuningSnapshot s built from serializeSnapshot()/
// deserializeSnapshot()'s own field set,
// deserializeSnapshot(serializeSnapshot(s)) reproduces s's field values.
Blob serializeSnapshot(const TuningSnapshot& snapshot);
TuningSnapshot deserializeSnapshot(const Blob& blob);

// shouldWipe -- the version-compare-and-wipe DECISION. Pure and trivial
// (storedVersion != currentVersion), but named and unit-tested explicitly
// so the wipe decision is one greppable, tested unit rather than an
// inline `!=` wherever it is needed.
bool shouldWipe(uint32_t storedVersion, uint32_t currentVersion);

// TuningStore -- the persistence seam Core::Configurator::persistIfEligible()
// saves through and main.cpp's boot sequence loads/wipes through. Plain
// virtual base (not an #ifdef HOST_BUILD fork) -- mirrors Hal::Transport
// (hal/transport.h): this header/its .cpp never drag in MicroBit.h under
// HOST_BUILD; only the concrete ARM adapter below is guarded.
//
// No sim/host implementation of this interface exists anywhere in this
// tree, by design -- the sim has no flash to persist to. Core::RobotLoop
// treats a null TuningStore* as "persistence disabled," which is every
// sim/test composition root's own case. A test that DOES want to observe
// the write-policy seam (app_robot_loop_harness.cpp's own debounce
// scenario) supplies a trivial call-counting mock -- this interface,
// being a plain C++ virtual base with no hardware dependency, is itself
// mockable under HOST_BUILD even though its one real implementation is
// not.
class TuningStore {
 public:
  virtual ~TuningStore() = default;

  // True + fills outVersion/outBlob if a stamped blob was ever written.
  // False if the store is empty (never written, or wiped).
  virtual bool load(uint32_t* outVersion, Blob* outBlob) = 0;

  virtual void save(uint32_t version, const Blob& blob) = 0;

  // Erases the persisted blob entirely. Called by main.cpp's boot
  // sequence only when something WAS stored at a mismatched version --
  // never called on an already-empty store (nothing to erase).
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
// this tree) -- covered only by a stakeholder bench checklist; see this
// file's own header note above.
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
