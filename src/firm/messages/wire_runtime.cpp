// wire_runtime.cpp -- implementation of the WireRuntime primitives declared
// in wire_runtime.h. See that file's header comment for the module's scope
// and contracts (encode/decode never partially write/read on failure, never
// touch the heap). No `#include` of any `messages/*.h` message header or
// `msg::` type -- this file must stay schema-agnostic.
#include "messages/wire_runtime.h"

#include <array>
#include <cstring>

namespace WireRuntime {

namespace {

// --- Varint byte-length helper (used by encodeVarint to size-check the
// destination BEFORE writing anything, so a too-small buffer never receives
// a partial encode). ---
size_t varintByteLength(uint64_t value) {
  size_t n = 1;
  while (value >= 0x80u) {
    value >>= 7;
    ++n;
  }
  return n;
}

// --- Base64 alphabet (standard, `+/` -- see wire_runtime.h's file-header
// first line) and its compile-time-built decode lookup table. ---
constexpr char kBase64Alphabet[64] = {
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M',
    'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
    'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/',
};

constexpr std::array<int8_t, 256> makeBase64DecodeTable() {
  std::array<int8_t, 256> table{};
  for (auto& entry : table) entry = -1;
  for (int i = 0; i < 64; ++i) {
    table[static_cast<unsigned char>(kBase64Alphabet[i])] = static_cast<int8_t>(i);
  }
  return table;
}

constexpr std::array<int8_t, 256> kBase64DecodeTable = makeBase64DecodeTable();

// --- CRC-16/CCITT-FALSE constants -- see wire_runtime.h item 9 for the
// exact pinned variant (poly/init/refin/refout/xorout) and why. ---
constexpr uint16_t kCrc16CcittFalsePoly = 0x1021;
constexpr uint16_t kCrc16CcittFalseInit = 0xFFFF;

}  // namespace

// --- 1. Varint -----------------------------------------------------------

bool encodeVarint(uint64_t value, uint8_t* buf, size_t cap, size_t* pos) {
  if (buf == nullptr || pos == nullptr || *pos > cap) return false;
  const size_t p = *pos;
  const size_t needed = varintByteLength(value);
  if (cap - p < needed) return false;

  uint64_t v = value;
  for (size_t i = 0; i < needed; ++i) {
    uint8_t byte = static_cast<uint8_t>(v & 0x7Fu);
    v >>= 7;
    if (i + 1 < needed) byte |= 0x80u;
    buf[p + i] = byte;
  }
  *pos = p + needed;
  return true;
}

bool decodeVarint(const uint8_t* buf, size_t len, size_t* pos, uint64_t* value) {
  if (buf == nullptr || pos == nullptr || value == nullptr || *pos > len) return false;
  const size_t p = *pos;

  uint64_t result = 0;
  for (size_t i = 0; i < kMaxVarintBytes; ++i) {
    if (p + i >= len) return false;  // truncated: continuation expected, buffer ended
    const uint8_t byte = buf[p + i];
    const uint64_t payload = static_cast<uint64_t>(byte & 0x7Fu);
    if (i == kMaxVarintBytes - 1 && payload > 1u) {
      // The 10th byte of a 64-bit varint may only supply bit 63 (7*9=63) --
      // anything larger overflows 64 bits. Malformed, not merely large.
      return false;
    }
    result |= payload << (7 * i);
    if ((byte & 0x80u) == 0) {
      *pos = p + i + 1;
      *value = result;
      return true;
    }
  }
  return false;  // continuation bit still set after kMaxVarintBytes bytes
}

// --- 2. Zigzag -------------------------------------------------------------

uint32_t zigzagEncode32(int32_t value) {
  return (static_cast<uint32_t>(value) << 1) ^ static_cast<uint32_t>(value >> 31);
}

int32_t zigzagDecode32(uint32_t value) {
  return static_cast<int32_t>(value >> 1) ^ -static_cast<int32_t>(value & 1u);
}

uint64_t zigzagEncode64(int64_t value) {
  return (static_cast<uint64_t>(value) << 1) ^ static_cast<uint64_t>(value >> 63);
}

int64_t zigzagDecode64(uint64_t value) {
  return static_cast<int64_t>(value >> 1) ^ -static_cast<int64_t>(value & 1u);
}

// --- 3. Fixed32 ------------------------------------------------------------

bool encodeFixed32(uint32_t value, uint8_t* buf, size_t cap, size_t* pos) {
  if (buf == nullptr || pos == nullptr || *pos > cap) return false;
  const size_t p = *pos;
  if (cap - p < 4) return false;
  buf[p + 0] = static_cast<uint8_t>(value & 0xFFu);
  buf[p + 1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  buf[p + 2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  buf[p + 3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
  *pos = p + 4;
  return true;
}

bool decodeFixed32(const uint8_t* buf, size_t len, size_t* pos, uint32_t* value) {
  if (buf == nullptr || pos == nullptr || value == nullptr || *pos > len) return false;
  const size_t p = *pos;
  if (len - p < 4) return false;
  *value = static_cast<uint32_t>(buf[p + 0]) | (static_cast<uint32_t>(buf[p + 1]) << 8) |
           (static_cast<uint32_t>(buf[p + 2]) << 16) | (static_cast<uint32_t>(buf[p + 3]) << 24);
  *pos = p + 4;
  return true;
}

bool encodeFloat(float value, uint8_t* buf, size_t cap, size_t* pos) {
  static_assert(sizeof(float) == 4, "encodeFloat assumes IEEE-754 binary32");
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return encodeFixed32(bits, buf, cap, pos);
}

bool decodeFloat(const uint8_t* buf, size_t len, size_t* pos, float* value) {
  static_assert(sizeof(float) == 4, "decodeFloat assumes IEEE-754 binary32");
  if (value == nullptr) return false;
  uint32_t bits;
  if (!decodeFixed32(buf, len, pos, &bits)) return false;
  std::memcpy(value, &bits, sizeof(bits));
  return true;
}

// --- Tag ---------------------------------------------------------------

bool encodeTag(uint32_t fieldNumber, WireType wireType, uint8_t* buf, size_t cap, size_t* pos) {
  const uint64_t tag = (static_cast<uint64_t>(fieldNumber) << 3) | static_cast<uint64_t>(wireType);
  return encodeVarint(tag, buf, cap, pos);
}

bool decodeTag(const uint8_t* buf, size_t len, size_t* pos, uint32_t* fieldNumber, WireType* wireType) {
  if (fieldNumber == nullptr || wireType == nullptr || pos == nullptr) return false;
  const size_t saved = *pos;
  uint64_t tag;
  if (!decodeVarint(buf, len, pos, &tag)) return false;

  const uint32_t wt = static_cast<uint32_t>(tag & 0x7u);
  if (wt != static_cast<uint32_t>(WireType::kVarint) && wt != static_cast<uint32_t>(WireType::kFixed64) &&
      wt != static_cast<uint32_t>(WireType::kLengthDelimited) && wt != static_cast<uint32_t>(WireType::kFixed32)) {
    *pos = saved;  // unrecognized wire type (e.g. deprecated proto2 groups 3/4) -- reject cleanly
    return false;
  }
  *fieldNumber = static_cast<uint32_t>(tag >> 3);
  *wireType = static_cast<WireType>(wt);
  return true;
}

// --- 4. Length-delimited framing -----------------------------------------

bool beginLengthDelimited(const uint8_t* buf, size_t len, size_t* pos, int depth, size_t* payloadLen) {
  if (buf == nullptr || pos == nullptr || payloadLen == nullptr) return false;
  if (depth < 0 || depth >= kMaxNestingDepth) return false;

  const size_t saved = *pos;
  uint64_t rawLen;
  if (!decodeVarint(buf, len, pos, &rawLen)) return false;

  const size_t p = *pos;
  if (rawLen > static_cast<uint64_t>(len - p)) {
    *pos = saved;  // claims more bytes than remain -- malformed, reject cleanly
    return false;
  }
  *payloadLen = static_cast<size_t>(rawLen);
  return true;
}

// --- 5. Packed-repeated readers ------------------------------------------

bool decodePackedVarint(const uint8_t* payload, size_t payloadLen, uint32_t* out, size_t maxCount, size_t* outCount) {
  if ((payload == nullptr && payloadLen != 0) || out == nullptr || outCount == nullptr) return false;

  size_t pos = 0;
  size_t written = 0;
  while (pos < payloadLen) {
    uint64_t v;
    if (!decodeVarint(payload, payloadLen, &pos, &v)) {
      *outCount = written;
      return false;
    }
    if (written < maxCount) {
      out[written] = static_cast<uint32_t>(v);
      ++written;
    }
    // else: element beyond max_count -- parsed (to keep the rest of the
    // payload byte-aligned and catch trailing malformed data) but clamped,
    // never written -- mirrors gen_messages.py's (max_count) convention.
  }
  *outCount = written;
  return true;
}

bool decodePackedFixed32(const uint8_t* payload, size_t payloadLen, float* out, size_t maxCount, size_t* outCount) {
  if ((payload == nullptr && payloadLen != 0) || out == nullptr || outCount == nullptr) return false;
  if (payloadLen % 4 != 0) return false;  // packed fixed32 payload must be a whole number of 4-byte elements

  size_t pos = 0;
  size_t written = 0;
  while (pos < payloadLen) {
    float v;
    if (!decodeFloat(payload, payloadLen, &pos, &v)) {
      *outCount = written;
      return false;
    }
    if (written < maxCount) {
      out[written] = v;
      ++written;
    }
  }
  *outCount = written;
  return true;
}

// --- 6. Unknown-field skip ------------------------------------------------

bool skipField(const uint8_t* buf, size_t len, size_t* pos, WireType wireType) {
  if (buf == nullptr || pos == nullptr || *pos > len) return false;

  switch (wireType) {
    case WireType::kVarint: {
      uint64_t discard;
      return decodeVarint(buf, len, pos, &discard);
    }
    case WireType::kFixed64: {
      const size_t p = *pos;
      if (len - p < 8) return false;
      *pos = p + 8;
      return true;
    }
    case WireType::kFixed32: {
      const size_t p = *pos;
      if (len - p < 4) return false;
      *pos = p + 4;
      return true;
    }
    case WireType::kLengthDelimited: {
      size_t payloadLen;
      // depth=0: skip never interprets/recurses into the payload's own
      // structure (opaque byte-range skip), so it can never trip the
      // nesting-depth guard regardless of how deep the CALLER is.
      if (!beginLengthDelimited(buf, len, pos, 0, &payloadLen)) return false;
      *pos = *pos + payloadLen;  // beginLengthDelimited already proved payloadLen <= (len - *pos)
      return true;
    }
  }
  return false;  // unrecognized wire type
}

// --- 7. Base64 -------------------------------------------------------------

size_t base64EncodedLength(size_t rawLen) { return ((rawLen + 2) / 3) * 4; }

size_t base64DecodedMaxLength(size_t encodedLen) { return (encodedLen / 4) * 3; }

bool base64Encode(const uint8_t* data, size_t len, char* out, size_t cap, size_t* outLen) {
  if ((data == nullptr && len != 0) || out == nullptr || outLen == nullptr) return false;
  const size_t needed = base64EncodedLength(len);
  if (cap < needed) return false;

  size_t i = 0;
  size_t o = 0;
  while (i + 3 <= len) {
    const uint32_t chunk = (static_cast<uint32_t>(data[i]) << 16) | (static_cast<uint32_t>(data[i + 1]) << 8) |
                            static_cast<uint32_t>(data[i + 2]);
    out[o++] = kBase64Alphabet[(chunk >> 18) & 0x3Fu];
    out[o++] = kBase64Alphabet[(chunk >> 12) & 0x3Fu];
    out[o++] = kBase64Alphabet[(chunk >> 6) & 0x3Fu];
    out[o++] = kBase64Alphabet[chunk & 0x3Fu];
    i += 3;
  }

  const size_t remaining = len - i;
  if (remaining == 1) {
    const uint32_t chunk = static_cast<uint32_t>(data[i]) << 16;
    out[o++] = kBase64Alphabet[(chunk >> 18) & 0x3Fu];
    out[o++] = kBase64Alphabet[(chunk >> 12) & 0x3Fu];
    out[o++] = '=';
    out[o++] = '=';
  } else if (remaining == 2) {
    const uint32_t chunk = (static_cast<uint32_t>(data[i]) << 16) | (static_cast<uint32_t>(data[i + 1]) << 8);
    out[o++] = kBase64Alphabet[(chunk >> 18) & 0x3Fu];
    out[o++] = kBase64Alphabet[(chunk >> 12) & 0x3Fu];
    out[o++] = kBase64Alphabet[(chunk >> 6) & 0x3Fu];
    out[o++] = '=';
  }
  *outLen = o;
  return true;
}

bool base64Decode(const char* in, size_t inLen, uint8_t* out, size_t cap, size_t* outLen) {
  if ((in == nullptr && inLen != 0) || out == nullptr || outLen == nullptr) return false;
  if (inLen == 0) {
    *outLen = 0;
    return true;
  }
  if (inLen % 4 != 0) return false;  // standard padded base64 is always a multiple of 4 chars

  const size_t groups = inLen / 4;
  size_t o = 0;
  for (size_t g = 0; g < groups; ++g) {
    const char* quad = in + g * 4;
    const bool lastGroup = (g + 1 == groups);
    int pad = 0;
    int8_t vals[4] = {0, 0, 0, 0};

    for (int k = 0; k < 4; ++k) {
      const unsigned char c = static_cast<unsigned char>(quad[k]);
      if (c == '=') {
        // '=' is only legal in the FINAL group, and only in position 2 or 3
        // (a base64 group always encodes at least one full byte).
        if (!lastGroup || k < 2) return false;
        if (k == 2) {
          if (static_cast<unsigned char>(quad[3]) != '=') return false;  // "AB=C" -- invalid single-'=' position
          pad = 2;
        } else if (pad != 2) {
          pad = 1;
        }
        continue;
      }
      if (pad != 0) return false;  // a real character after '=' inside the same group
      const int8_t v = kBase64DecodeTable[c];
      if (v < 0) return false;  // character outside the base64 alphabet
      vals[k] = v;
    }

    const uint32_t chunk = (static_cast<uint32_t>(vals[0]) << 18) | (static_cast<uint32_t>(vals[1]) << 12) |
                            (static_cast<uint32_t>(vals[2]) << 6) | static_cast<uint32_t>(vals[3]);
    const int bytesThisGroup = 3 - pad;
    if (o + static_cast<size_t>(bytesThisGroup) > cap) return false;  // destination buffer too small
    out[o++] = static_cast<uint8_t>((chunk >> 16) & 0xFFu);
    if (bytesThisGroup >= 2) out[o++] = static_cast<uint8_t>((chunk >> 8) & 0xFFu);
    if (bytesThisGroup >= 3) out[o++] = static_cast<uint8_t>(chunk & 0xFFu);
  }
  *outLen = o;
  return true;
}

// --- 8. COBS ---------------------------------------------------------------

size_t cobsEncodedMaxLength(size_t rawLen) { return rawLen + rawLen / kCobsMaxBlockLength + 1; }

size_t cobsDecodedMaxLength(size_t encodedLen) { return encodedLen; }

bool cobsEncode(const uint8_t* data, size_t len, uint8_t* out, size_t cap, size_t* outLen) {
  if ((data == nullptr && len != 0) || out == nullptr || outLen == nullptr) return false;
  if (cap < cobsEncodedMaxLength(len)) return false;  // caller must size the destination to the worst case

  size_t readPos = 0;
  size_t writePos = 0;
  size_t codePos = writePos;  // index of the current block's not-yet-known code byte
  if (writePos >= cap) return false;
  out[writePos++] = 0;  // placeholder, overwritten with the real code once the block ends
  uint8_t code = 1;      // distance to the next zero (or block end), inclusive of the code byte itself

  while (readPos < len) {
    const uint8_t byte = data[readPos++];
    if (byte == 0) {
      out[codePos] = code;
      codePos = writePos;
      if (writePos >= cap) return false;
      out[writePos++] = 0;  // placeholder for the next block
      code = 1;
    } else {
      if (writePos >= cap) return false;
      out[writePos++] = byte;
      ++code;
      if (code == 0xFFu) {
        // Block hit the 254-non-zero-byte cap before finding a zero: flush
        // it as a full block (code 0xFF means "254 data bytes, no zero
        // terminator") and start a fresh block, exactly as if a zero had
        // been seen -- this is the "block-size boundary (254 bytes)"
        // behavior the acceptance criteria calls out by name.
        out[codePos] = code;
        codePos = writePos;
        if (writePos >= cap) return false;
        out[writePos++] = 0;
        code = 1;
      }
    }
  }
  out[codePos] = code;
  *outLen = writePos;
  return true;
}

bool cobsDecode(const uint8_t* in, size_t inLen, uint8_t* out, size_t cap, size_t* outLen) {
  if ((in == nullptr && inLen != 0) || out == nullptr || outLen == nullptr) return false;
  // Even the encoding of a zero-byte payload is exactly one code byte
  // (0x01) -- a truly zero-length input here has nothing to decode and is
  // malformed, not "empty payload."
  if (inLen == 0) return false;

  size_t readPos = 0;
  size_t writePos = 0;
  while (readPos < inLen) {
    const uint8_t code = in[readPos];
    if (code == 0) return false;  // a literal 0x00 code byte never appears in a valid COBS frame -- corrupt
    ++readPos;
    const size_t blockLen = static_cast<size_t>(code) - 1;
    if (blockLen > inLen - readPos) return false;  // code claims more data bytes than remain -- truncated frame

    for (size_t i = 0; i < blockLen; ++i) {
      const uint8_t b = in[readPos + i];
      if (b == 0) return false;  // a literal 0x00 inside a data block is corrupt (encode never emits one)
      if (writePos >= cap) return false;
      out[writePos++] = b;
    }
    readPos += blockLen;

    // A block whose code is < 0xFF was terminated by an actual zero byte in
    // the ORIGINAL data, UNLESS this block is the last one in the frame (in
    // which case there was no zero -- the frame just ended). A 0xFF-coded
    // block hit the 254-byte cap with no zero at all, so never emit one for
    // it regardless of position.
    if (code != 0xFFu && readPos < inLen) {
      if (writePos >= cap) return false;
      out[writePos++] = 0;
    }
  }
  *outLen = writePos;
  return true;
}

// --- 9. CRC-16/CCITT-FALSE --------------------------------------------------

uint16_t crcCompute(const uint8_t* data, size_t len) {
  if (data == nullptr) len = 0;  // defensive: never dereference a null pointer, even with a nonzero len argument
  uint16_t crc = kCrc16CcittFalseInit;
  for (size_t i = 0; i < len; ++i) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000u) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ kCrc16CcittFalsePoly);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

bool crcVerify(const uint8_t* data, size_t len, uint16_t expectedCrc) { return crcCompute(data, len) == expectedCrc; }

bool encodeCrc16(uint16_t crc, uint8_t* buf, size_t cap, size_t* pos) {
  if (buf == nullptr || pos == nullptr || *pos > cap) return false;
  const size_t p = *pos;
  if (cap - p < 2) return false;
  buf[p + 0] = static_cast<uint8_t>(crc & 0xFFu);
  buf[p + 1] = static_cast<uint8_t>((crc >> 8) & 0xFFu);
  *pos = p + 2;
  return true;
}

bool decodeCrc16(const uint8_t* buf, size_t len, size_t* pos, uint16_t* crc) {
  if (buf == nullptr || pos == nullptr || crc == nullptr || *pos > len) return false;
  const size_t p = *pos;
  if (len - p < 2) return false;
  *crc = static_cast<uint16_t>(static_cast<uint16_t>(buf[p + 0]) | (static_cast<uint16_t>(buf[p + 1]) << 8));
  *pos = p + 2;
  return true;
}

}  // namespace WireRuntime
