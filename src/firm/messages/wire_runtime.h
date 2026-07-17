// wire_runtime.h -- **BASE64 ALPHABET: STANDARD (RFC 4648 `+/`), NOT
// URL-SAFE (`-_`).** This is pinned ONCE, here, because both sides of the
// `*B<base64>` armor must agree: the host's binary reply path calls Python
// stdlib `base64.b64encode`/`base64.b64decode`, whose DEFAULT alphabet is
// this same standard `+/` one (`base64.urlsafe_b64encode` would require an
// explicit, separate call the host code does not make). Do not "fix" this
// to url-safe on either side without updating both; there is no
// negotiation, no version byte -- whichever alphabet this file
// encodes/decodes with IS the wire format.
//
// WireRuntime: the ONE hand-written, schema-agnostic byte-level codec
// toolkit in this directory -- never regenerated, and never `#include`ing
// `envelope.h`/`motion.h`/any other `messages/*.h` or naming a `msg::*`
// type. The GENERATED `wire.{h,cpp}` is built on top of these primitives to
// walk a specific message's field table. Speaks raw protobuf bytes only:
// varint, zigzag, fixed32, length-delimited framing, packed-repeated
// arrays, unknown-field skip, base64. Knows nothing about field numbers,
// offsets, or bounds belonging to any specific message -- that is
// `wire.{h,cpp}`'s job. See messages/DESIGN.md for the three-layer split.
//
// Every function below operates on a caller-owned buffer passed by
// pointer+size (or a `size_t*` cursor into one); nothing in this file
// allocates (no `new`, no `malloc`, no `std::vector`/`std::string`). Target
// is CODAL's actual compiled standard for this project (`-std=gnu++20`,
// not the vendored target's nominal C++11 pin -- see messages/DESIGN.md
// §3), built `-fno-exceptions -fno-rtti` clean, newlib-nano-safe (no
// `%f`/float `snprintf` -- these are pure binary encode/decode functions
// with no text formatting at all, so that constraint is satisfied by
// construction, not by a workaround).
//
// Encode-side contract: every `encode*` function takes a destination
// `uint8_t* buf`/`size_t cap` and a `size_t* pos` cursor; it writes nothing
// and returns `false`, leaving `*pos` unchanged, if the value would not
// fully fit in `[*pos, cap)` -- never a partial write.
//
// Decode-side contract: every `decode*` function takes a source `const
// uint8_t* buf`/`size_t len` and a `size_t* pos` cursor; it returns `false`
// and leaves `*pos` unchanged on ANY malformed or truncated input --
// never reads at or past `buf[len]`. This is the property the malformed-
// input acceptance criteria (truncated varint, over-claiming
// length-delimited field, bad base64 padding) verify under ASan/UBSan.
#pragma once

#include <cstddef>
#include <cstdint>

namespace WireRuntime {

// Protobuf wire types this codec understands (the low 3 bits of a field
// tag). Wire types 3/4 (deprecated proto2 START_GROUP/END_GROUP) are not
// emitted by proto3 and are rejected by decodeTag()/skipField() as
// unrecognized, not silently mishandled.
enum class WireType : uint8_t {
  kVarint = 0,           // int32/int64/uint32/uint64/sint32/sint64/bool/enum
  kFixed64 = 1,           // fixed64/sfixed64/double
  kLengthDelimited = 2,   // string/bytes/embedded message/packed repeated
  kFixed32 = 5,           // fixed32/sfixed32/float
};

// Length-delimited recursion depth bound. This schema's actual max nesting
// is shallow (CommandEnvelope -> e.g. DrivetrainCommand -> WheelTargets ->
// repeated WheelTarget is the deepest chain today, 3 levels) -- 8 is
// small-constant headroom over that, chosen to reject a
// maliciously/accidentally over-nested input with a clean `false` rather
// than risk unbounded recursion overflowing the stack in the generated
// decoder, which recurses through beginLengthDelimited() once per nested
// message level.
constexpr int kMaxNestingDepth = 8;

// Max bytes a base-128 varint can occupy encoding a full 64-bit value:
// ceil(64 / 7) = 10.
constexpr size_t kMaxVarintBytes = 10;

// --- 1. Varint (protobuf base-128, unsigned, up to 64 bits) -------------
//
// decodeVarint() rejects a value whose 10th continuation byte would carry
// more than bit 63 (i.e. would overflow 64 bits) and rejects a varint that
// is still continuing after kMaxVarintBytes bytes -- both are malformed
// input, not merely large values.
bool encodeVarint(uint64_t value, uint8_t* buf, size_t cap, size_t* pos);
bool decodeVarint(const uint8_t* buf, size_t len, size_t* pos, uint64_t* value);

// --- 2. Zigzag (signed <-> unsigned mapping for sint32/sint64) ----------
//
// This schema has no sint32/sint64 fields today: every signed/bounded
// quantity is a protobuf `float` (fixed32 wire type, see item 3), not a
// zigzag-mapped integer. Implemented anyway as a cheap, standard primitive
// a future schema addition may need -- currently unused; confirm it is
// still unused before assuming it can be deleted.
uint32_t zigzagEncode32(int32_t value);
int32_t zigzagDecode32(uint32_t value);
uint64_t zigzagEncode64(int64_t value);
int64_t zigzagDecode64(uint64_t value);

// --- 3. Fixed32 (protobuf float/fixed32/sfixed32 wire type) -------------
//
// Little-endian on the wire regardless of host/target byte order (both
// arm-none-eabi-g++'s Cortex-M4 target and every host dev machine this
// project builds on are little-endian in practice, but encode/decode below
// assemble/disassemble bytes explicitly rather than assume that, so the
// wire format itself does not silently depend on it). encodeFloat/
// decodeFloat move the IEEE-754 bit pattern via memcpy (never a reinterpret
// cast) -- well-defined, no strict-aliasing UB, and clean under UBSan.
bool encodeFixed32(uint32_t value, uint8_t* buf, size_t cap, size_t* pos);
bool decodeFixed32(const uint8_t* buf, size_t len, size_t* pos, uint32_t* value);
bool encodeFloat(float value, uint8_t* buf, size_t cap, size_t* pos);
bool decodeFloat(const uint8_t* buf, size_t len, size_t* pos, float* value);

// --- Tag (field_number << 3 | wire_type), varint-encoded ----------------
//
// The building block both "length-delimited framing" and "unknown-field
// skip" need to learn a field's wire type before they can act on it --
// exposed as its own pair of functions rather than duplicated inline in
// both.
bool encodeTag(uint32_t fieldNumber, WireType wireType, uint8_t* buf, size_t cap, size_t* pos);
bool decodeTag(const uint8_t* buf, size_t len, size_t* pos, uint32_t* fieldNumber, WireType* wireType);

// --- 4. Length-delimited framing -----------------------------------------
//
// Decodes the varint length prefix at `*pos`, bounds-checks the claimed
// payload length against what actually remains in `[*pos, len)` (rejecting
// the "claims more bytes than remain" malformed case), and enforces the
// nesting-depth bound. On success, `*pos` is advanced PAST the length
// prefix (now pointing at the payload's first byte) and `*payloadLen` is
// set -- the payload bytes themselves are NOT consumed by this call.
//
// Depth contract: `depth` is the nesting level of the message THIS
// length-delimited field is a member of (the outermost/top-level message
// is depth 0). Only recurse with `depth + 1` when the payload is itself
// going to be decoded as a NESTED MESSAGE (each such recursive
// beginLengthDelimited() call one level deeper); a leaf `bytes`/`string`/
// packed-repeated payload is not a further nesting level, so pass the SAME
// `depth` the caller itself was given, not `depth + 1`, for those. Returns
// `false` without reading a length prefix at all if `depth >=
// kMaxNestingDepth`.
bool beginLengthDelimited(const uint8_t* buf, size_t len, size_t* pos, int depth, size_t* payloadLen);

// --- 5. Packed-repeated reader (clamped at caller-supplied max_count) ---
//
// `payload`/`payloadLen` is the byte range already extracted by a prior
// beginLengthDelimited() call (i.e. the packed field's own payload, not the
// enclosing message). Mirrors the generator's `(max_count)` convention:
// every element in the payload is parsed (so a malformed trailing element
// is still caught and rejected even past the cap), but only the first
// `maxCount` are WRITTEN into `out` -- `*outCount` is the number actually
// written (<= maxCount), never more than `out`'s own capacity. Two variants
// cover the only two packable scalar wire shapes this tree's generated
// arrays actually use (see e.g. messages/common.h's `command_modes_[8]`
// (uint32_t) and `args_[4]` (float)):
bool decodePackedVarint(const uint8_t* payload, size_t payloadLen, uint32_t* out, size_t maxCount, size_t* outCount);
bool decodePackedFixed32(const uint8_t* payload, size_t payloadLen, float* out, size_t maxCount, size_t* outCount);

// --- 6. Unknown-field skip ------------------------------------------------
//
// Given a wire type already read via decodeTag() for a field number the
// caller does not recognize, advances `*pos` past that field's VALUE (the
// tag itself must already have been consumed by the caller's decodeTag()
// call) without interpreting its contents. This is what lets a
// declared-only oneof arm or a future schema addition round-trip through
// an older decoder without erroring -- forward compatibility. Length-
// delimited values are skipped as an opaque byte range (never recursed
// into), so this never needs -- and never checks -- the nesting-depth bound.
bool skipField(const uint8_t* buf, size_t len, size_t* pos, WireType wireType);

// --- 7. Base64 -------------------------------------------------------------
//
// See the file-header's first line: standard alphabet (`+/`), `=` padding,
// RFC 4648 section 4. base64EncodedLength()/base64DecodedMaxLength() let a
// caller size a destination buffer before calling encode/decode.
// base64Decode() rejects (returns false, writes nothing) on: input length
// not a multiple of 4, an unrecognized character, or `=` padding in any
// position/count other than the valid trailing 0/1/2 of the FINAL group --
// this is the "base64 string with invalid padding" malformed-input
// acceptance criterion.
size_t base64EncodedLength(size_t rawLen);
size_t base64DecodedMaxLength(size_t encodedLen);
bool base64Encode(const uint8_t* data, size_t len, char* out, size_t cap, size_t* outLen);
bool base64Decode(const char* in, size_t inLen, uint8_t* out, size_t cap, size_t* outLen);

}  // namespace WireRuntime
