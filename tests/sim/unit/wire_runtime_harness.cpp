// wire_runtime_harness.cpp -- off-hardware acceptance harness for ticket
// 095-004 (SUC-003, architecture-update.md M3 "Wire Runtime"): exercises
// every WireRuntime primitive in source/messages/wire_runtime.{h,cpp}
// against its acceptance criteria -- round-trip (varint/zigzag/fixed32/
// base64), malformed-input rejection (truncated varint, over-claiming
// length-delimited field, bad base64 padding), the length-delimited
// recursion depth bound, the packed-repeated max_count clamp, and
// unknown-field skip across all four wire types.
//
// Mirrors runtime_blackboard_harness.cpp's exact pattern (see that file's
// header for the shape this follows): #includes only
// source/messages/wire_runtime.h (which itself includes only <cstddef>/
// <cstdint> -- no MicroBit.h, no CODAL, no ARM toolchain). Hand-rolled
// assertions, prints PASS/FAIL, exits nonzero on any failure. Run by
// test_wire_runtime.py, which compiles and runs this binary via subprocess
// twice: once with the project's normal host flags (matching
// test_runtime_blackboard.py's shape exactly), and once with
// -fsanitize=address,undefined so the malformed-input scenarios below are
// proven not to read past a buffer's end, not just to return the right
// bool.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "messages/wire_runtime.h"

namespace {

using WireRuntime::WireType;

// --- Hand-rolled assertion plumbing (same tiny shape as
// runtime_blackboard_harness.cpp/runtime_queue_harness.cpp). ---

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

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
}

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkU64Eq(uint64_t actual, uint64_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %llu, got %llu", what.c_str(),
                  static_cast<unsigned long long>(expected), static_cast<unsigned long long>(actual));
    fail(buf);
  }
}

void checkI64Eq(int64_t actual, int64_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %lld, got %lld", what.c_str(),
                  static_cast<long long>(expected), static_cast<long long>(actual));
    fail(buf);
  }
}

void checkSizeEq(size_t actual, size_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %zu, got %zu", what.c_str(), expected, actual);
    fail(buf);
  }
}

void checkFloatEq(float actual, float expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(), static_cast<double>(expected),
                  static_cast<double>(actual));
    fail(buf);
  }
}

// 1. Varint round-trips for the boundary set the acceptance criteria name:
// 0, 1, small positive, UINT32_MAX, and the multi-byte boundary values
// 127/128/16383/16384. A UINT64_MAX case is included beyond the AC set as a
// bonus check on the 10-byte/64-bit path.
void scenarioVarintRoundTrip() {
  beginScenario("Varint: round-trip 0/1/127/128/16383/16384/UINT32_MAX/UINT64_MAX");
  const uint64_t values[] = {0ull, 1ull, 127ull, 128ull, 16383ull, 16384ull,
                              static_cast<uint64_t>(UINT32_MAX), UINT64_MAX};
  for (uint64_t value : values) {
    uint8_t buf[16] = {};
    size_t pos = 0;
    checkTrue(WireRuntime::encodeVarint(value, buf, sizeof(buf), &pos), "encodeVarint succeeds");
    checkTrue(pos >= 1 && pos <= WireRuntime::kMaxVarintBytes, "encoded length within [1, kMaxVarintBytes]");

    size_t decodePos = 0;
    uint64_t decoded = 0;
    checkTrue(WireRuntime::decodeVarint(buf, pos, &decodePos, &decoded), "decodeVarint succeeds");
    checkU64Eq(decoded, value, "decoded value round-trips");
    checkSizeEq(decodePos, pos, "decodeVarint consumes exactly the bytes encodeVarint wrote");
  }

  // Multi-byte boundary: 127 encodes to 1 byte, 128 encodes to 2 -- confirm
  // the byte-length transition lands exactly where the base-128 format
  // requires.
  uint8_t buf127[4] = {};
  size_t pos127 = 0;
  WireRuntime::encodeVarint(127, buf127, sizeof(buf127), &pos127);
  checkSizeEq(pos127, 1, "127 encodes to exactly 1 byte");

  uint8_t buf128[4] = {};
  size_t pos128 = 0;
  WireRuntime::encodeVarint(128, buf128, sizeof(buf128), &pos128);
  checkSizeEq(pos128, 2, "128 encodes to exactly 2 bytes");

  uint8_t buf16383[4] = {};
  size_t pos16383 = 0;
  WireRuntime::encodeVarint(16383, buf16383, sizeof(buf16383), &pos16383);
  checkSizeEq(pos16383, 2, "16383 encodes to exactly 2 bytes");

  uint8_t buf16384[4] = {};
  size_t pos16384 = 0;
  WireRuntime::encodeVarint(16384, buf16384, sizeof(buf16384), &pos16384);
  checkSizeEq(pos16384, 3, "16384 encodes to exactly 3 bytes");
}

// Also proves encodeVarint fails cleanly (no partial write committed to
// *pos) when the destination is too small.
void scenarioVarintEncodeTooSmall() {
  beginScenario("Varint: encodeVarint fails cleanly when the destination is too small");
  uint8_t buf[1] = {0xAA};
  size_t pos = 0;
  checkFalse(WireRuntime::encodeVarint(16384 /* needs 3 bytes */, buf, sizeof(buf), &pos), "too-small destination rejected");
  checkSizeEq(pos, 0, "*pos left unchanged on failure");
}

// 2. Zigzag round-trips for 0, small +/-, INT32_MIN/INT32_MAX (the
// acceptance criteria's exact set), plus the 64-bit variant for completeness
// (this schema has no sint32/sint64 fields today -- see wire_runtime.h's
// item-2 comment -- but the primitive is implemented and tested regardless).
void scenarioZigzagRoundTrip() {
  beginScenario("Zigzag: round-trip 0/+-small/INT32_MIN/INT32_MAX");
  const int32_t values32[] = {0, 1, -1, 2, -2, 1000, -1000, INT32_MIN, INT32_MAX};
  for (int32_t value : values32) {
    uint32_t encoded = WireRuntime::zigzagEncode32(value);
    int32_t decoded = WireRuntime::zigzagDecode32(encoded);
    checkI64Eq(decoded, value, "zigzag32 round-trips");
  }
  // Known protobuf zigzag mapping spot-checks (0->0, -1->1, 1->2, -2->3).
  checkU64Eq(WireRuntime::zigzagEncode32(0), 0, "zigzag32(0) == 0");
  checkU64Eq(WireRuntime::zigzagEncode32(-1), 1, "zigzag32(-1) == 1");
  checkU64Eq(WireRuntime::zigzagEncode32(1), 2, "zigzag32(1) == 2");
  checkU64Eq(WireRuntime::zigzagEncode32(-2), 3, "zigzag32(-2) == 3");

  const int64_t values64[] = {0, 1, -1, INT64_MIN, INT64_MAX};
  for (int64_t value : values64) {
    uint64_t encoded = WireRuntime::zigzagEncode64(value);
    int64_t decoded = WireRuntime::zigzagDecode64(encoded);
    checkI64Eq(decoded, value, "zigzag64 round-trips");
  }
}

// 3. Fixed32/float round-trips for 0.0f, negatives, and this schema's
// actual bounds (+-31.416 rad -- MotionSegment.direction/final_heading;
// +-10000 mm -- MotionSegment.distance).
void scenarioFixedFloatRoundTrip() {
  beginScenario("Fixed32: float round-trip 0.0f/negatives/+-31.416/+-10000");
  const float values[] = {0.0f, -0.0f, 1.5f, -1.5f, 31.416f, -31.416f, 10000.0f, -10000.0f, 3000.0f, -3000.0f};
  for (float value : values) {
    uint8_t buf[4] = {};
    size_t pos = 0;
    checkTrue(WireRuntime::encodeFloat(value, buf, sizeof(buf), &pos), "encodeFloat succeeds");
    checkSizeEq(pos, 4, "fixed32 always encodes to exactly 4 bytes");

    size_t decodePos = 0;
    float decoded = 0.0f;
    checkTrue(WireRuntime::decodeFloat(buf, pos, &decodePos, &decoded), "decodeFloat succeeds");
    checkFloatEq(decoded, value, "decoded float round-trips bit-for-bit");
  }

  // Raw fixed32 (non-float) round-trip too, since item 3 covers both.
  uint8_t rawBuf[4] = {};
  size_t rawPos = 0;
  WireRuntime::encodeFixed32(0xDEADBEEFu, rawBuf, sizeof(rawBuf), &rawPos);
  size_t rawDecodePos = 0;
  uint32_t rawDecoded = 0;
  WireRuntime::decodeFixed32(rawBuf, rawPos, &rawDecodePos, &rawDecoded);
  checkU64Eq(rawDecoded, 0xDEADBEEFu, "raw fixed32 round-trips");
}

// 4. Base64 round-trips for empty input, a single byte, and a full
// 186-byte envelope-sized buffer (architecture-update.md's payload cap).
void scenarioBase64RoundTrip() {
  beginScenario("Base64: round-trip empty/1-byte/186-byte buffers");

  // Empty.
  {
    char encoded[8] = {};
    size_t encodedLen = 999;
    checkTrue(WireRuntime::base64Encode(nullptr, 0, encoded, sizeof(encoded), &encodedLen), "encode empty succeeds");
    checkSizeEq(encodedLen, 0, "empty input encodes to 0 chars");

    uint8_t decoded[8] = {};
    size_t decodedLen = 999;
    checkTrue(WireRuntime::base64Decode(encoded, encodedLen, decoded, sizeof(decoded), &decodedLen),
              "decode empty succeeds");
    checkSizeEq(decodedLen, 0, "empty round-trips to 0 bytes");
  }

  // Single byte.
  {
    const uint8_t data[1] = {0x5A};
    char encoded[8] = {};
    size_t encodedLen = 0;
    checkTrue(WireRuntime::base64Encode(data, sizeof(data), encoded, sizeof(encoded), &encodedLen),
              "encode 1 byte succeeds");
    checkSizeEq(encodedLen, 4, "1 byte encodes to 4 chars (2 padding)");
    checkTrue(encoded[2] == '=' && encoded[3] == '=', "1-byte encoding pads with '=='");

    uint8_t decoded[8] = {};
    size_t decodedLen = 0;
    checkTrue(WireRuntime::base64Decode(encoded, encodedLen, decoded, sizeof(decoded), &decodedLen),
              "decode 1 byte succeeds");
    checkSizeEq(decodedLen, 1, "1-byte round-trips to 1 byte");
    checkTrue(decoded[0] == data[0], "1-byte value round-trips");
  }

  // Full 186-byte envelope-sized buffer (186 is divisible by 3, so this
  // also exercises the zero-padding, exact-multiple-of-3 path).
  {
    uint8_t data[186];
    for (size_t i = 0; i < sizeof(data); ++i) data[i] = static_cast<uint8_t>(i * 37 + 11);

    char encoded[300] = {};  // 186 bytes -> 248 base64 chars (base64EncodedLength(186)), plenty of headroom
    size_t encodedLen = 0;
    checkTrue(WireRuntime::base64Encode(data, sizeof(data), encoded, sizeof(encoded), &encodedLen),
              "encode 186 bytes succeeds");
    checkSizeEq(encodedLen, 248, "186 bytes (divisible by 3) encodes to exactly 248 chars, no padding");

    uint8_t decoded[186] = {};
    size_t decodedLen = 0;
    checkTrue(WireRuntime::base64Decode(encoded, encodedLen, decoded, sizeof(decoded), &decodedLen),
              "decode 186 bytes succeeds");
    checkSizeEq(decodedLen, sizeof(data), "186-byte buffer round-trips to 186 bytes");
    checkTrue(std::memcmp(decoded, data, sizeof(data)) == 0, "186-byte buffer round-trips byte-for-byte");
  }
}

// 5a. Malformed input: a varint whose continuation byte is missing (buffer
// ends while the high bit of the last byte read is still set).
void scenarioMalformedTruncatedVarint() {
  beginScenario("Malformed: truncated varint (continuation byte missing) rejected cleanly");
  const uint8_t buf[1] = {0x80};  // continuation bit set, but no second byte follows
  size_t pos = 0;
  uint64_t value = 0;
  checkFalse(WireRuntime::decodeVarint(buf, sizeof(buf), &pos, &value), "truncated varint rejected");
  checkSizeEq(pos, 0, "*pos left unchanged on truncated-varint failure");

  // A varint that never terminates within kMaxVarintBytes is malformed too
  // (all continuation bits set, buffer otherwise well-supplied).
  uint8_t longBuf[WireRuntime::kMaxVarintBytes + 2];
  for (size_t i = 0; i < sizeof(longBuf); ++i) longBuf[i] = 0x80;  // every byte says "more follow"
  size_t pos2 = 0;
  uint64_t value2 = 0;
  checkFalse(WireRuntime::decodeVarint(longBuf, sizeof(longBuf), &pos2, &value2),
             "varint exceeding kMaxVarintBytes without terminating is rejected");
}

// 5b. Malformed input: a length-delimited field whose varint length prefix
// claims more bytes than remain in the buffer.
void scenarioMalformedLengthDelimitedOverclaim() {
  beginScenario("Malformed: length-delimited field claiming more bytes than remain rejected cleanly");
  uint8_t buf[6] = {};
  size_t encodePos = 0;
  WireRuntime::encodeVarint(100, buf, sizeof(buf), &encodePos);  // claims a 100-byte payload
  // ...but the buffer only has (sizeof(buf) - encodePos) bytes left, far short of 100.

  size_t pos = 0;
  size_t payloadLen = 0;
  checkFalse(WireRuntime::beginLengthDelimited(buf, sizeof(buf), &pos, 0, &payloadLen),
             "over-claiming length-delimited field rejected");
  checkSizeEq(pos, 0, "*pos left unchanged on over-claim failure");
}

// 5c. Malformed input: base64 strings with invalid padding, in several
// distinct ways, all rejected cleanly without touching `out`/`outLen`
// beyond what a caller can safely ignore.
void scenarioMalformedBase64Padding() {
  beginScenario("Malformed: base64 with invalid padding rejected cleanly");
  uint8_t out[8];
  size_t outLen;

  // '=' in an early group (padding may only appear in the FINAL group).
  checkFalse(WireRuntime::base64Decode("AB==CDEF", 8, out, sizeof(out), &outLen), "'=' in a non-final group rejected");

  // '=' at position 0 of the final group.
  checkFalse(WireRuntime::base64Decode("====", 4, out, sizeof(out), &outLen), "'=' at position 0 rejected");

  // Single '=' with a real character after it in the same group ("AB=C").
  checkFalse(WireRuntime::base64Decode("AB=C", 4, out, sizeof(out), &outLen), "'=' followed by a real char rejected");

  // Length not a multiple of 4.
  checkFalse(WireRuntime::base64Decode("ABCDE", 5, out, sizeof(out), &outLen), "length not a multiple of 4 rejected");

  // Character outside the standard alphabet -- in particular, URL-safe '-'/
  // '_' are NOT accepted (proves the alphabet pin in wire_runtime.h's file
  // header is enforced, not just documented).
  checkFalse(WireRuntime::base64Decode("A-B_", 4, out, sizeof(out), &outLen), "url-safe '-'/'_' rejected (wrong alphabet)");

  // Destination buffer too small for the decoded output -- rejected, not a
  // silent truncation.
  uint8_t tinyOut[1];
  checkFalse(WireRuntime::base64Decode("QUJD", 4, tinyOut, sizeof(tinyOut), &outLen),
             "decode fails cleanly when the destination cap is too small");

  // A VALID single-'=' and double-'=' encoding, as a control, still succeeds
  // (confirms the rejections above are about the malformed cases specifically,
  // not padding in general).
  checkTrue(WireRuntime::base64Decode("QQ==", 4, out, sizeof(out), &outLen), "valid double-'=' padding accepted");
  checkSizeEq(outLen, 1, "valid double-'=' padding decodes to 1 byte");
  checkTrue(WireRuntime::base64Decode("QUI=", 4, out, sizeof(out), &outLen), "valid single-'=' padding accepted");
  checkSizeEq(outLen, 2, "valid single-'=' padding decodes to 2 bytes");
}

// 6. Length-delimited recursion depth bound: a synthetic over-nested input
// (more nesting levels than kMaxNestingDepth) is rejected cleanly rather
// than recursing without bound. Simulates the shape a future generated
// decoder (ticket 005) would have -- each level's payload IS the next
// level's whole length-delimited frame -- without needing any msg:: type.
void scenarioDepthBoundRejection() {
  beginScenario("Length-delimited: nesting depth bound (kMaxNestingDepth) rejects over-nested input");

  // Build kMaxNestingDepth + 2 levels of nested length-delimited framing,
  // innermost-first: level N's payload is exactly level (N-1)'s encoded
  // frame (its own varint length prefix + its payload).
  constexpr int kLevels = WireRuntime::kMaxNestingDepth + 2;
  uint8_t frames[kLevels][32] = {};
  size_t frameLens[kLevels] = {};

  // Level 0: a trivial 1-byte payload, framed.
  {
    const uint8_t innerPayload[1] = {0x2A};
    size_t pos = 0;
    checkTrue(WireRuntime::encodeVarint(sizeof(innerPayload), frames[0], sizeof(frames[0]), &pos),
              "level-0 length prefix encodes");
    std::memcpy(frames[0] + pos, innerPayload, sizeof(innerPayload));
    frameLens[0] = pos + sizeof(innerPayload);
  }
  // Level i wraps level (i-1)'s whole encoded frame as its payload.
  for (int i = 1; i < kLevels; ++i) {
    size_t pos = 0;
    checkTrue(WireRuntime::encodeVarint(frameLens[i - 1], frames[i], sizeof(frames[i]), &pos),
              "nested level length prefix encodes");
    std::memcpy(frames[i] + pos, frames[i - 1], frameLens[i - 1]);
    frameLens[i] = pos + frameLens[i - 1];
  }

  // Walk inward from the outermost frame (kLevels-1), incrementing depth
  // once per level -- exactly the shape a recursive generated decoder would
  // use. Must succeed for the first kMaxNestingDepth levels and then fail
  // cleanly (never crash, never read out of bounds) once depth reaches the
  // bound.
  const uint8_t* buf = frames[kLevels - 1];
  size_t len = frameLens[kLevels - 1];
  size_t pos = 0;
  int depth = 0;
  bool sawRejection = false;
  for (; depth < kLevels; ++depth) {
    size_t payloadLen = 0;
    if (!WireRuntime::beginLengthDelimited(buf, len, &pos, depth, &payloadLen)) {
      sawRejection = true;
      break;
    }
    // Descend into the payload for the next level.
    buf = buf + pos;
    len = payloadLen;
    pos = 0;
  }
  checkTrue(sawRejection, "over-nested input is eventually rejected rather than fully decoded");
  checkTrue(depth == WireRuntime::kMaxNestingDepth,
            "rejection happens exactly at kMaxNestingDepth, not earlier or later");
}

// 7. Packed-repeated reader clamps at max_count and never overflows a
// fixed-size output array sized to EXACTLY max_count, even when fed more
// elements than the cap (run under ASan, a real overflow would abort).
void scenarioPackedRepeatedClamp() {
  beginScenario("Packed-repeated: decodePackedVarint/decodePackedFixed32 clamp at max_count, no overflow");

  // Packed varint: 5 elements into a payload, but max_count == 3.
  {
    uint8_t payload[32] = {};
    size_t pos = 0;
    const uint32_t elements[5] = {10, 20, 30, 40, 50};
    for (uint32_t e : elements) {
      checkTrue(WireRuntime::encodeVarint(e, payload, sizeof(payload), &pos), "packed varint element encodes");
    }

    uint32_t out[3] = {};  // sized to EXACTLY max_count -- any overflow write is an ASan stack-buffer-overflow
    size_t outCount = 999;
    checkTrue(WireRuntime::decodePackedVarint(payload, pos, out, 3, &outCount), "decodePackedVarint succeeds");
    checkSizeEq(outCount, 3, "outCount clamps to max_count (3), not the 5 elements present");
    checkU64Eq(out[0], 10, "clamped out[0] retains the first element");
    checkU64Eq(out[1], 20, "clamped out[1] retains the second element");
    checkU64Eq(out[2], 30, "clamped out[2] retains the third element (4th/5th dropped, not overflowed)");
  }

  // Packed fixed32 (float): 6 elements into a payload, max_count == 2.
  {
    uint8_t payload[32] = {};
    size_t pos = 0;
    const float elements[6] = {1.5f, 2.5f, 3.5f, 4.5f, 5.5f, 6.5f};
    for (float e : elements) {
      checkTrue(WireRuntime::encodeFloat(e, payload, sizeof(payload), &pos), "packed fixed32 element encodes");
    }

    float out[2] = {};  // sized to EXACTLY max_count
    size_t outCount = 999;
    checkTrue(WireRuntime::decodePackedFixed32(payload, pos, out, 2, &outCount), "decodePackedFixed32 succeeds");
    checkSizeEq(outCount, 2, "outCount clamps to max_count (2), not the 6 elements present");
    checkFloatEq(out[0], 1.5f, "clamped out[0] retains the first element");
    checkFloatEq(out[1], 2.5f, "clamped out[1] retains the second element");
  }

  // maxCount == 0 is the degenerate clamp: nothing is ever written, even
  // though elements are present and fully parsed.
  {
    uint8_t payload[8] = {};
    size_t pos = 0;
    WireRuntime::encodeVarint(7, payload, sizeof(payload), &pos);
    uint32_t out[1] = {0xDEADBEEF};
    size_t outCount = 999;
    checkTrue(WireRuntime::decodePackedVarint(payload, pos, out, 0, &outCount), "maxCount==0 still succeeds (valid payload)");
    checkSizeEq(outCount, 0, "maxCount==0 writes nothing");
    checkU64Eq(out[0], 0xDEADBEEF, "out[] untouched when maxCount==0");
  }
}

// 8. Unknown-field skip advances past an unrecognized field of each wire
// type without corrupting the read position for the KNOWN field that
// follows it.
void scenarioUnknownFieldSkip() {
  beginScenario("Unknown-field skip: advances correctly for varint/fixed64/fixed32/length-delimited");

  // --- varint ---
  {
    uint8_t buf[32] = {};
    size_t pos = 0;
    WireRuntime::encodeTag(99, WireType::kVarint, buf, sizeof(buf), &pos);  // unknown field 99
    WireRuntime::encodeVarint(123456, buf, sizeof(buf), &pos);
    WireRuntime::encodeTag(1, WireType::kVarint, buf, sizeof(buf), &pos);  // known field 1
    WireRuntime::encodeVarint(42, buf, sizeof(buf), &pos);
    const size_t total = pos;

    size_t readPos = 0;
    uint32_t fieldNum = 0;
    WireType wt = WireType::kVarint;
    checkTrue(WireRuntime::decodeTag(buf, total, &readPos, &fieldNum, &wt), "unknown varint field's tag decodes");
    checkTrue(WireRuntime::skipField(buf, total, &readPos, wt), "skipField advances past the unknown varint value");

    checkTrue(WireRuntime::decodeTag(buf, total, &readPos, &fieldNum, &wt), "following known field's tag decodes");
    checkU64Eq(fieldNum, 1, "known field number is 1, unaffected by the skip");
    uint64_t value = 0;
    checkTrue(WireRuntime::decodeVarint(buf, total, &readPos, &value), "known field's value decodes");
    checkU64Eq(value, 42, "known field's value is correct after skipping the preceding unknown field");
  }

  // --- fixed64 (skip only -- this schema has no fixed64/double fields, but
  // skip must still handle the wire type correctly for forward
  // compatibility) ---
  {
    uint8_t buf[32] = {};
    size_t pos = 0;
    WireRuntime::encodeTag(99, WireType::kFixed64, buf, sizeof(buf), &pos);
    for (int i = 0; i < 8; ++i) buf[pos++] = static_cast<uint8_t>(0xA0 + i);  // 8 raw bytes, any content
    WireRuntime::encodeTag(1, WireType::kFixed32, buf, sizeof(buf), &pos);
    WireRuntime::encodeFixed32(0x11223344u, buf, sizeof(buf), &pos);
    const size_t total = pos;

    size_t readPos = 0;
    uint32_t fieldNum = 0;
    WireType wt = WireType::kVarint;
    checkTrue(WireRuntime::decodeTag(buf, total, &readPos, &fieldNum, &wt), "unknown fixed64 field's tag decodes");
    checkTrue(WireRuntime::skipField(buf, total, &readPos, wt), "skipField advances past the unknown fixed64 value (8 bytes)");

    checkTrue(WireRuntime::decodeTag(buf, total, &readPos, &fieldNum, &wt), "following known field's tag decodes");
    checkU64Eq(fieldNum, 1, "known field number is 1, unaffected by the skip");
    uint32_t value = 0;
    checkTrue(WireRuntime::decodeFixed32(buf, total, &readPos, &value), "known field's value decodes");
    checkU64Eq(value, 0x11223344u, "known field's value is correct after skipping the preceding unknown fixed64 field");
  }

  // --- fixed32 ---
  {
    uint8_t buf[32] = {};
    size_t pos = 0;
    WireRuntime::encodeTag(99, WireType::kFixed32, buf, sizeof(buf), &pos);
    WireRuntime::encodeFixed32(0xCAFEBABEu, buf, sizeof(buf), &pos);
    WireRuntime::encodeTag(1, WireType::kVarint, buf, sizeof(buf), &pos);
    WireRuntime::encodeVarint(7, buf, sizeof(buf), &pos);
    const size_t total = pos;

    size_t readPos = 0;
    uint32_t fieldNum = 0;
    WireType wt = WireType::kVarint;
    checkTrue(WireRuntime::decodeTag(buf, total, &readPos, &fieldNum, &wt), "unknown fixed32 field's tag decodes");
    checkTrue(WireRuntime::skipField(buf, total, &readPos, wt), "skipField advances past the unknown fixed32 value (4 bytes)");

    checkTrue(WireRuntime::decodeTag(buf, total, &readPos, &fieldNum, &wt), "following known field's tag decodes");
    checkU64Eq(fieldNum, 1, "known field number is 1, unaffected by the skip");
    uint64_t value = 0;
    checkTrue(WireRuntime::decodeVarint(buf, total, &readPos, &value), "known field's value decodes");
    checkU64Eq(value, 7, "known field's value is correct after skipping the preceding unknown fixed32 field");
  }

  // --- length-delimited ---
  {
    uint8_t buf[32] = {};
    size_t pos = 0;
    WireRuntime::encodeTag(99, WireType::kLengthDelimited, buf, sizeof(buf), &pos);
    const uint8_t blob[3] = {0x01, 0x02, 0x03};
    WireRuntime::encodeVarint(sizeof(blob), buf, sizeof(buf), &pos);
    std::memcpy(buf + pos, blob, sizeof(blob));
    pos += sizeof(blob);
    WireRuntime::encodeTag(1, WireType::kVarint, buf, sizeof(buf), &pos);
    WireRuntime::encodeVarint(99, buf, sizeof(buf), &pos);
    const size_t total = pos;

    size_t readPos = 0;
    uint32_t fieldNum = 0;
    WireType wt = WireType::kVarint;
    checkTrue(WireRuntime::decodeTag(buf, total, &readPos, &fieldNum, &wt),
              "unknown length-delimited field's tag decodes");
    checkTrue(WireRuntime::skipField(buf, total, &readPos, wt),
              "skipField advances past the unknown length-delimited value (length prefix + payload)");

    checkTrue(WireRuntime::decodeTag(buf, total, &readPos, &fieldNum, &wt), "following known field's tag decodes");
    checkU64Eq(fieldNum, 1, "known field number is 1, unaffected by the skip");
    uint64_t value = 0;
    checkTrue(WireRuntime::decodeVarint(buf, total, &readPos, &value), "known field's value decodes");
    checkU64Eq(value, 99, "known field's value is correct after skipping the preceding unknown length-delimited field");
  }
}

// Tag encode/decode round-trip, plus rejection of an unrecognized wire type
// (proto2 START_GROUP=3), underpinning the skip/length-delimited scenarios
// above.
void scenarioTagRoundTripAndRejectsUnknownWireType() {
  beginScenario("Tag: encode/decode round-trip; unrecognized wire type rejected");
  uint8_t buf[8] = {};
  size_t pos = 0;
  checkTrue(WireRuntime::encodeTag(300, WireType::kLengthDelimited, buf, sizeof(buf), &pos), "encodeTag succeeds");

  size_t readPos = 0;
  uint32_t fieldNum = 0;
  WireType wt = WireType::kVarint;
  checkTrue(WireRuntime::decodeTag(buf, pos, &readPos, &fieldNum, &wt), "decodeTag succeeds");
  checkU64Eq(fieldNum, 300, "field number round-trips");
  checkTrue(wt == WireType::kLengthDelimited, "wire type round-trips");

  // Tag encoding wire type 3 (START_GROUP, deprecated proto2, not used by
  // this proto3 schema) must be rejected by decodeTag, not silently
  // accepted.
  uint8_t groupBuf[8] = {};
  size_t groupPos = 0;
  WireRuntime::encodeVarint((static_cast<uint64_t>(5) << 3) | 3u, groupBuf, sizeof(groupBuf), &groupPos);
  size_t groupReadPos = 0;
  uint32_t groupFieldNum = 0;
  WireType groupWt = WireType::kVarint;
  checkFalse(WireRuntime::decodeTag(groupBuf, groupPos, &groupReadPos, &groupFieldNum, &groupWt),
             "wire type 3 (deprecated START_GROUP) rejected");
  checkSizeEq(groupReadPos, 0, "*pos left unchanged when decodeTag rejects an unknown wire type");
}

}  // namespace

int main() {
  scenarioVarintRoundTrip();
  scenarioVarintEncodeTooSmall();
  scenarioZigzagRoundTrip();
  scenarioFixedFloatRoundTrip();
  scenarioBase64RoundTrip();
  scenarioMalformedTruncatedVarint();
  scenarioMalformedLengthDelimitedOverclaim();
  scenarioMalformedBase64Padding();
  scenarioDepthBoundRejection();
  scenarioPackedRepeatedClamp();
  scenarioUnknownFieldSkip();
  scenarioTagRoundTripAndRejectsUnknownWireType();

  if (g_failureCount == 0) {
    std::printf("OK: all WireRuntime scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the WireRuntime scenarios\n", g_failureCount);
  return 1;
}
