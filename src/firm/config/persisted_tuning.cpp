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

}  // namespace

// serializeSnapshot()/deserializeSnapshot() -- fixed field order: the 3
// group-presence bools first, then the 12 raw floats grouped
// WHEEL_CONTROL(5)/MOTORS-travel_calib(2)/OTOS(5), matching
// TuningSnapshot's own declaration order (persisted_tuning.h). No Opt<T>,
// no per-field presence byte (132-013's reshape -- see this file's own
// header doc comment): every float is written/read unconditionally, and
// the 3 group-level bools are what a caller checks before trusting a
// group's floats mean anything (reapplyPersistedTuning(), configurator.cpp).
Blob serializeSnapshot(const TuningSnapshot& snapshot) {
  Blob blob{};
  size_t offset = 0;

  putBool(blob, offset, snapshot.wheelControlTuned);
  putBool(blob, offset, snapshot.motorsTravelCalibTuned);
  putBool(blob, offset, snapshot.otosTuned);

  putFloat(blob, offset, snapshot.wheelControlPidKp);
  putFloat(blob, offset, snapshot.wheelControlPidKi);
  putFloat(blob, offset, snapshot.wheelControlPidIMax);
  putFloat(blob, offset, snapshot.wheelControlPidKaff);
  putFloat(blob, offset, snapshot.wheelControlPidMax);

  putFloat(blob, offset, snapshot.motorsTravelCalibLeft);
  putFloat(blob, offset, snapshot.motorsTravelCalibRight);

  putFloat(blob, offset, snapshot.otosOffsetX);
  putFloat(blob, offset, snapshot.otosOffsetY);
  putFloat(blob, offset, snapshot.otosOffsetYaw);
  putFloat(blob, offset, snapshot.otosLinearScale);
  putFloat(blob, offset, snapshot.otosAngularScale);

  return blob;
}

TuningSnapshot deserializeSnapshot(const Blob& blob) {
  TuningSnapshot snapshot;
  size_t offset = 0;

  snapshot.wheelControlTuned = takeBool(blob, offset);
  snapshot.motorsTravelCalibTuned = takeBool(blob, offset);
  snapshot.otosTuned = takeBool(blob, offset);

  snapshot.wheelControlPidKp = takeFloat(blob, offset);
  snapshot.wheelControlPidKi = takeFloat(blob, offset);
  snapshot.wheelControlPidIMax = takeFloat(blob, offset);
  snapshot.wheelControlPidKaff = takeFloat(blob, offset);
  snapshot.wheelControlPidMax = takeFloat(blob, offset);

  snapshot.motorsTravelCalibLeft = takeFloat(blob, offset);
  snapshot.motorsTravelCalibRight = takeFloat(blob, offset);

  snapshot.otosOffsetX = takeFloat(blob, offset);
  snapshot.otosOffsetY = takeFloat(blob, offset);
  snapshot.otosOffsetYaw = takeFloat(blob, offset);
  snapshot.otosLinearScale = takeFloat(blob, offset);
  snapshot.otosAngularScale = takeFloat(blob, offset);

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
//
// 132-013 (patch-surface retirement): kBlobSize dropped from 85 to 51
// bytes (kPayloadBytes 89 -> 55, kNumChunks 3 -> 2) -- the reshape (the
// old curated Motor/Otos live-tuning messages' per-field 5-byte Opt<T>
// packing -> plain 4-byte floats + 3 group-level presence bools) shrank
// the blob even though the persisted FIELD SET is unchanged (see
// TuningSnapshot's own doc comment, persisted_tuning.h). This frees
// budget rather than consuming it: kNumChunks now has 2 full chunks of
// headroom below the 4-chunk ceiling, not 1.
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
  // Whole-store erase: codal::KeyValueStorage::wipe() erases EVERY key in
  // the store, including com/radio_channel.h's own persisted radio
  // channel. Accepted, not a bug -- a version bump is rare (a reflash),
  // and a radio-channel re-pick after one is a minor, visible
  // inconvenience, not the silent-drift hazard a misapplied stale tuning
  // patch would be.
  storage_.wipe();
}

#endif  // HOST_BUILD

}  // namespace Config
