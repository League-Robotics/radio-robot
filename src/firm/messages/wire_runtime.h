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
// Sprint 123 added two more schema-agnostic byte primitives to this same
// layer: COBS frame encode/decode (item 8) and CRC-16/CCITT-FALSE
// compute/verify (item 9) -- the wire's new binary framing + integrity
// check, replacing the base64 armor's expansion cost and closing the "no
// integrity check anywhere" gap (see `clasi/sprints/123-.../sprint.md`
// Design Rationale). Ticket 002 cut the actual ARMOR over (comms.cpp/
// telemetry.cpp no longer call base64Encode()/base64Decode() at all) --
// base64 (item 7) itself is RETAINED, not removed, because
// wire_differential_harness.cpp (src/tests/sim/unit/) independently
// depends on it for its own, unrelated debug-CLI wire encoding (comparing
// this codec's encode/decode against a Python protobuf reference), a
// consumer with no connection to the Comms armor scheme. Deleting the
// primitive would be a second, out-of-scope breaking change for zero
// benefit -- ticket 001's own forward-reference to removal assumed no such
// independent consumer existed; ticket 002 found one and kept the
// primitive rather than force that collateral change.
//
// Sprint 124 ticket 003 (protocol v5 Part A, issue §2/§3) extended both
// item 8 and item 9 again, in place, with no new primitive files: COBS is
// now keyed on a caller-supplied delimiter byte (item 8) instead of a
// hardcoded 0x00, and CRC-16/CCITT-FALSE gained an incremental
// crcInit()/crcUpdate() pair (item 9) alongside the existing single-range
// crcCompute() so a caller can hash a command-name prefix and a payload
// together without concatenating them into one scratch buffer first. Both
// changes default to the pre-124 behavior (delimiter 0x00, no prefix) so
// every call site written against the 123 API keeps compiling and
// computing byte-identical results unless it opts in.
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

// --- 8. COBS (Consistent Overhead Byte Stuffing) frame encode/decode ----
//
// Removes every occurrence of a caller-chosen DELIMITER byte from an
// arbitrary payload so the encoded result can be safely delimited on the
// wire by a single instance of that same byte -- the frame delimiter
// itself. This primitive does NOT append that trailing delimiter;
// deciding where the delimiter goes (and demuxing it from any
// differently-terminated content sharing the same byte stream) is
// `Comms`'s job (sprint 123 ticket 002), not this schema-agnostic
// primitive's. `cobsEncode()`'s OUTPUT never contains a byte equal to
// `delimiter` -- that is the whole property that makes the trailing
// delimiter unambiguous, for WHICHEVER byte value `delimiter` is.
//
// `delimiter` defaults to `0x00` -- every call site written before sprint
// 124 ticket 003 added this parameter keeps compiling unchanged and keeps
// computing byte-identical output, because XOR-ing by `0x00` is the
// identity operation (see the mechanism below). Sprint 124 ticket 003
// (protocol v5 Part A, issue §2) needed `0x0A` instead: COBS guarantees
// `0x00`-freedom but nothing about any other byte value, and a payload
// containing a literal `0x0A` corrupted the wire once the frame delimiter
// became `\n` (proven 0/10 on hardware, fixed narrowly in 123-006/febfb450
// before this ticket fixed the general case).
//
// Mechanism: run the standard COBS algorithm (Cheshire & Baker 1999) below
// EXACTLY as if `delimiter` were always `0x00` -- walk the input, split it
// into blocks of at most `kCobsMaxBlockLength` (254) non-zero bytes, each
// block prefixed by a 1-byte code (the distance, inclusive of the code
// byte itself, to the next zero, or `0xFF` for a full 254-byte block that
// hit the cap before finding one) -- but XOR every byte written to `out`
// (both data bytes and code bytes) with `delimiter` at the moment it is
// finalized. This is mathematically identical to computing the whole
// `0x00`-keyed encoding first and XOR-ing every output byte by `delimiter`
// afterward (XOR is position-wise and order-independent), just without a
// second pass. Why it is sound: `cobsEncode()`'s pre-XOR output is
// guaranteed `0x00`-free (code bytes are `>= 0x01`, block data bytes
// non-zero by construction) -- for any byte `b`, `b ^ delimiter ==
// delimiter` iff `b == 0`, which never occurs, so the XOR-ed stream can
// never equal `delimiter`. Decode reverses this by XOR-ing every byte
// AT THE POINT OF READING from `in` (rather than materializing a
// de-XORed copy -- this file allocates nothing), which recovers the
// original `0x00`-keyed bytes the standard algorithm below already knows
// how to walk.
//
// The malformed-input checks below are expressed in terms of the
// `0x00`-keyed algorithm and need no change for a non-zero delimiter --
// they get SHARPER, not weaker: a literal `delimiter` byte arriving inside
// an encoded frame (impossible from a correct encoder, by the property
// above) reads back as a literal `0x00` after the read-time XOR, which
// `cobsDecode()` already rejects as "literal 0x00 code byte" / "literal
// 0x00 inside data block" -- unreachable except under corruption, exactly
// the case it should reject.
//
// Overhead is exactly one code byte per <=254-byte block: at most
// `cobsEncodedMaxLength(len)` bytes total, ~0.4% for the frame sizes this
// wire actually carries (well under 256 B) -- unaffected by `delimiter`,
// which only changes byte VALUES, never how many bytes are written.
//
// `cobsEncodedMaxLength()` is a worst-case bound a caller must size its
// destination buffer to before calling `cobsEncode()` -- exact only when
// the input contains no embedded zero bytes; a zero-dense input can
// encode to fewer bytes than this bound (never more). `cobsDecodedMaxLength()`
// is exact-as-an-upper-bound the other direction: decoding only ever
// REMOVES the code-byte overhead, so the decoded payload can never exceed
// the encoded frame's own length.
//
// Encode/decode never-partial contract (same as every other primitive in
// this file): `cobsEncode()`/`cobsDecode()` write nothing and return
// `false`, leaving `*outLen` unset, on any capacity failure or malformed
// input -- in particular `cobsDecode()` rejects a literal 0x00 byte found
// (post-XOR) INSIDE the encoded frame (an encoder never emits one; seeing
// one means the frame is corrupt) and a code byte whose claimed block
// length runs past the end of the input (truncation).
constexpr size_t kCobsMaxBlockLength = 254;

size_t cobsEncodedMaxLength(size_t rawLen);
size_t cobsDecodedMaxLength(size_t encodedLen);
bool cobsEncode(const uint8_t* data, size_t len, uint8_t* out, size_t cap, size_t* outLen, uint8_t delimiter = 0x00);
bool cobsDecode(const uint8_t* in, size_t inLen, uint8_t* out, size_t cap, size_t* outLen, uint8_t delimiter = 0x00);

// --- 9. CRC-16/CCITT-FALSE (wire integrity check) -----------------------
//
// Decided variant, PINNED exactly so firmware and host (ticket 003) agree
// byte-for-byte -- there is no negotiation, no version byte:
//   poly   = 0x1021
//   init   = 0xFFFF
//   refin  = false  (processed MSB-first, no input reflection)
//   refout = false  (no output reflection)
//   xorout = 0x0000 (no final XOR)
// This is the variant the CRC RevEng catalogue calls "CRC-16/CCITT-FALSE".
// Known-answer vector (from that catalogue): `crcCompute("123456789", 9)
// == 0x29B1` -- exercised as an exact-value test, not merely "some CRC
// changed."
//
// Width decision (sprint 123 Open Question 1): 16 bits, not 32. Frames on
// this wire are small -- well under 256 B even before this sprint's
// COBS+CRC change -- so a 16-bit CRC's 2^-16 miss rate on random
// corruption is strong detection for this size range at HALF the
// per-frame overhead (2 bytes) a CRC-32 would cost (4 bytes), on a link
// whose whole point of this sprint is shedding overhead (base64's 33%
// expansion) rather than re-spending most of it back on the integrity
// check that replaces it.
//
// `crcCompute()` takes a raw pointer/length (not the buf/cap/pos cursor
// shape the encode/decode primitives use) because a CRC is a scalar
// reduction over a byte range, not a byte-buffer transform with its own
// cursor -- there is nothing to advance. `encodeCrc16()`/`decodeCrc16()`
// give the buf/cap/pos-cursor wire-placement pair (little-endian, 2
// bytes) for a caller (ticket 002) that wants to append/read the CRC value
// itself as part of a framed byte sequence, following the exact same
// never-partial contract as `encodeFixed32()`/`decodeFixed32()`.
uint16_t crcCompute(const uint8_t* data, size_t len);
bool crcVerify(const uint8_t* data, size_t len, uint16_t expectedCrc);
bool encodeCrc16(uint16_t crc, uint8_t* buf, size_t cap, size_t* pos);
bool decodeCrc16(const uint8_t* buf, size_t len, size_t* pos, uint16_t* crc);

// crcInit()/crcUpdate() -- the incremental form `crcCompute()` is built on
// (`crcCompute(data, len) == crcUpdate(crcInit(), data, len)`, exactly,
// same loop body). Sprint 124 ticket 003 (issue §3): the wire's CRC input
// extends from "payload alone" to "COMMAND ':' payload" -- the command
// name and the payload are two byte ranges that are NOT adjacent in
// memory (one is a caller's literal/buffer, the other is a freshly
// schema-encoded scratch buffer), so hashing them together needs either a
// concatenation into a THIRD scratch buffer (extra copy, extra size
// bookkeeping) or a way to feed the CRC register more than once. This is
// the latter: `crcInit()` returns the starting register value,
// `crcUpdate()` folds in one more byte range and returns the continued
// register value -- CRC-16/CCITT-FALSE's `xorout = 0x0000`/`refout =
// false` mean the running register IS the CRC value at every point, so
// there is no separate "finalize" step. `Comms::sendReply()`/
// `decodeBinaryFrame()` (comms.cpp) and `wire_codec.py`'s
// `encode_frame()`/`decode_frame()` are the callers that compose
// `crcUpdate(crcUpdate(crcInit(), command, commandLen), payload,
// payloadLen)` (with a `':'` folded in between when a command name is
// present) -- this file itself does not know that a command name or a
// `':'` separator exist; that composition is protocol grammar, which is
// `Comms`'s boundary, not `WireRuntime`'s (see messages/DESIGN.md's
// three-layer split).
uint16_t crcInit();
uint16_t crcUpdate(uint16_t crc, const uint8_t* data, size_t len);

}  // namespace WireRuntime
