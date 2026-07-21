// persisted_tuning.cpp -- Config::PersistedTuning implementation. See
// persisted_tuning.h's own file header for the module's boundary (pure
// logic vs. the ARM-only MicroBitStorage adapter).
#include "config/persisted_tuning.h"

#include <cstring>

#ifndef HOST_BUILD
#include "MicroBit.h"
#endif

namespace Config {

// ---------------------------------------------------------------------
// Pure serialize/deserialize/shouldWipe -- no MicroBitStorage, no I/O.
// ---------------------------------------------------------------------

namespace {

void putBool(Blob& blob, size_t& offset, bool v) {
  blob[offset] = v ? 1 : 0;
  offset += 1;
}

bool takeBool(const Blob& blob, size_t& offset) {
  bool v = blob[offset] != 0;
  offset += 1;
  return v;
}

// putFloat/takeFloat -- raw bit-pattern memcpy, not a text/varint
// encoding. Safe because this blob is never sent over a wire or read by a
// different build; it is written and read back by the exact same compiled
// binary (same float representation both ends), the same assumption
// serializeSnapshot()'s own round-trip acceptance criterion already makes.
void putFloat(Blob& blob, size_t& offset, float v) {
  std::memcpy(blob.data() + offset, &v, sizeof(float));
  offset += sizeof(float);
}

float takeFloat(const Blob& blob, size_t& offset) {
  float v = 0.0f;
  std::memcpy(&v, blob.data() + offset, sizeof(float));
  offset += sizeof(float);
  return v;
}

void putOptFloat(Blob& blob, size_t& offset, const msg::Opt<float>& v) {
  putBool(blob, offset, v.has);
  putFloat(blob, offset, v.val);
}

msg::Opt<float> takeOptFloat(const Blob& blob, size_t& offset) {
  msg::Opt<float> v;
  v.has = takeBool(blob, offset);
  v.val = takeFloat(blob, offset);
  return v;
}

// putMotorPatch/takeMotorPatch -- travel_calib, kp, ki, kff, i_max, kaw,
// in that fixed order (kMotorPatchFields=6, persisted_tuning.h). `side`
// is NOT part of the blob -- deserializeSnapshot() stamps it directly
// (bookkeeping only, see TuningSnapshot's own doc comment).
void putMotorPatch(Blob& blob, size_t& offset, const msg::MotorConfigPatch& p) {
  putOptFloat(blob, offset, p.travel_calib);
  putOptFloat(blob, offset, p.kp);
  putOptFloat(blob, offset, p.ki);
  putOptFloat(blob, offset, p.kff);
  putOptFloat(blob, offset, p.i_max);
  putOptFloat(blob, offset, p.kaw);
}

msg::MotorConfigPatch takeMotorPatch(const Blob& blob, size_t& offset) {
  msg::MotorConfigPatch p;
  p.travel_calib = takeOptFloat(blob, offset);
  p.kp = takeOptFloat(blob, offset);
  p.ki = takeOptFloat(blob, offset);
  p.kff = takeOptFloat(blob, offset);
  p.i_max = takeOptFloat(blob, offset);
  p.kaw = takeOptFloat(blob, offset);
  return p;
}

}  // namespace

Blob serializeSnapshot(const TuningSnapshot& snapshot) {
  Blob blob{};
  size_t offset = 0;

  putMotorPatch(blob, offset, snapshot.motorL);
  putMotorPatch(blob, offset, snapshot.motorR);

  putOptFloat(blob, offset, snapshot.otos.linear_scale);
  putOptFloat(blob, offset, snapshot.otos.angular_scale);
  putOptFloat(blob, offset, snapshot.otos.offset_x);
  putOptFloat(blob, offset, snapshot.otos.offset_y);
  putOptFloat(blob, offset, snapshot.otos.offset_yaw);

  return blob;
}

TuningSnapshot deserializeSnapshot(const Blob& blob) {
  TuningSnapshot snapshot;
  size_t offset = 0;

  snapshot.motorL = takeMotorPatch(blob, offset);
  snapshot.motorL.side = msg::BoundMotorSide::LEFT;
  snapshot.motorR = takeMotorPatch(blob, offset);
  snapshot.motorR.side = msg::BoundMotorSide::RIGHT;

  snapshot.otos.linear_scale = takeOptFloat(blob, offset);
  snapshot.otos.angular_scale = takeOptFloat(blob, offset);
  snapshot.otos.offset_x = takeOptFloat(blob, offset);
  snapshot.otos.offset_y = takeOptFloat(blob, offset);
  snapshot.otos.offset_yaw = takeOptFloat(blob, offset);
  // .init deliberately left at its default (false) -- see
  // TuningSnapshot's own doc comment.

  return snapshot;
}

bool shouldWipe(uint32_t storedVersion, uint32_t currentVersion) {
  return storedVersion != currentVersion;
}

// ---------------------------------------------------------------------
// MicroBitTuningStore -- ARM-only, guarded. NOT exercised by any
// agent-run test -- see persisted_tuning.h's own file header.
// ---------------------------------------------------------------------

#ifndef HOST_BUILD

namespace {

// codal::KeyValueStorage hard limits (codal-core/inc/drivers/
// KeyValueStorage.h): 32 bytes of value per key, 5 keys total in the
// WHOLE store (shared with com/radio_channel.h's own 1 key). A single
// (version + snapshot blob) payload is CHUNKED across kNumChunks keys of
// kChunkBytes each -- the static_assert below fails the ARM build loudly
// if a future field addition (persisted_tuning.h's kBlobSize) ever
// outgrows the 4 keys this store may use, rather than silently
// truncating the persisted state.
constexpr int kChunkBytes = 32;  // codal's KEY_VALUE_STORAGE_VALUE_SIZE
constexpr size_t kPayloadBytes = sizeof(uint32_t) + kBlobSize;  // version + blob
constexpr int kNumChunks =
    static_cast<int>((kPayloadBytes + kChunkBytes - 1) / kChunkBytes);

static_assert(kNumChunks <= 4,
              "persisted-tuning chunk count must leave room for "
              "com/radio_channel.h's own key within codal::KeyValueStorage's "
              "5-key-total limit (KEY_VALUE_STORAGE_MAX_PAIRS)");

const char* chunkKey(int i) {
  static const char* const kKeys[4] = {"tune0", "tune1", "tune2", "tune3"};
  return kKeys[i];
}

int chunkSize(int i) {
  int consumedBefore = i * kChunkBytes;
  int remaining = static_cast<int>(kPayloadBytes) - consumedBefore;
  return (remaining < kChunkBytes) ? remaining : kChunkBytes;
}

}  // namespace

MicroBitTuningStore::MicroBitTuningStore(codal::KeyValueStorage& storage)
    : storage_(storage) {}

bool MicroBitTuningStore::load(uint32_t* outVersion, Blob* outBlob) {
  uint8_t payload[kPayloadBytes] = {};

  for (int i = 0; i < kNumChunks; ++i) {
    KeyValuePair* pair = storage_.get(chunkKey(i));
    if (pair == nullptr) return false;  // never written (or wiped)
    std::memcpy(payload + (i * kChunkBytes), pair->value, chunkSize(i));
    delete pair;
  }

  uint32_t version = 0;
  std::memcpy(&version, payload, sizeof(uint32_t));
  Blob blob{};
  std::memcpy(blob.data(), payload + sizeof(uint32_t), kBlobSize);

  *outVersion = version;
  *outBlob = blob;
  return true;
}

void MicroBitTuningStore::save(uint32_t version, const Blob& blob) {
  uint8_t payload[kPayloadBytes] = {};
  std::memcpy(payload, &version, sizeof(uint32_t));
  std::memcpy(payload + sizeof(uint32_t), blob.data(), kBlobSize);

  for (int i = 0; i < kNumChunks; ++i) {
    uint8_t chunk[kChunkBytes] = {};
    std::memcpy(chunk, payload + (i * kChunkBytes), chunkSize(i));
    storage_.put(chunkKey(i), chunk, kChunkBytes);
  }
}

void MicroBitTuningStore::wipe() {
  // Whole-store erase (SUC-003: "the entire store is wiped" on a version
  // mismatch) -- codal::KeyValueStorage::wipe() erases EVERY key in the
  // store, including com/radio_channel.h's own persisted radio channel.
  // Accepted, not a bug: sprint.md's own SUC-003 flow states "the entire
  // store is wiped," not "our own keys only," and a version bump is rare
  // (a reflash) -- a radio-channel re-pick after one is a minor, visible
  // inconvenience, not the silent-drift hazard a misapplied stale tuning
  // patch would be.
  storage_.wipe();
}

#endif  // HOST_BUILD

}  // namespace Config
