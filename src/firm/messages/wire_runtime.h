#pragma once

#include <cstddef>
#include <cstdint>

namespace WireRuntime {

enum class WireType : uint8_t {
  kVarint = 0,
  kFixed64 = 1,
  kLengthDelimited = 2,
  kFixed32 = 5,
};

constexpr int kMaxNestingDepth = 8;

constexpr size_t kMaxVarintBytes = 10;

bool encodeVarint(uint64_t value, uint8_t* buf, size_t cap, size_t* pos);
bool decodeVarint(const uint8_t* buf, size_t len, size_t* pos, uint64_t* value);

uint32_t zigzagEncode32(int32_t value);
int32_t zigzagDecode32(uint32_t value);
uint64_t zigzagEncode64(int64_t value);
int64_t zigzagDecode64(uint64_t value);

bool encodeFixed32(uint32_t value, uint8_t* buf, size_t cap, size_t* pos);
bool decodeFixed32(const uint8_t* buf, size_t len, size_t* pos, uint32_t* value);
bool encodeFloat(float value, uint8_t* buf, size_t cap, size_t* pos);
bool decodeFloat(const uint8_t* buf, size_t len, size_t* pos, float* value);

bool encodeTag(uint32_t fieldNumber, WireType wireType, uint8_t* buf, size_t cap, size_t* pos);
bool decodeTag(const uint8_t* buf, size_t len, size_t* pos, uint32_t* fieldNumber, WireType* wireType);

bool beginLengthDelimited(const uint8_t* buf, size_t len, size_t* pos, int depth, size_t* payloadLen);

bool decodePackedVarint(const uint8_t* payload, size_t payloadLen, uint32_t* out, size_t maxCount, size_t* outCount);
bool decodePackedFixed32(const uint8_t* payload, size_t payloadLen, float* out, size_t maxCount, size_t* outCount);

bool skipField(const uint8_t* buf, size_t len, size_t* pos, WireType wireType);

size_t base64EncodedLength(size_t rawLen);
size_t base64DecodedMaxLength(size_t encodedLen);
bool base64Encode(const uint8_t* data, size_t len, char* out, size_t cap, size_t* outLen);
bool base64Decode(const char* in, size_t inLen, uint8_t* out, size_t cap, size_t* outLen);

constexpr size_t kCobsMaxBlockLength = 254;

size_t cobsEncodedMaxLength(size_t rawLen);
size_t cobsDecodedMaxLength(size_t encodedLen);
bool cobsEncode(const uint8_t* data, size_t len, uint8_t* out, size_t cap, size_t* outLen, uint8_t delimiter = 0x00);
bool cobsDecode(const uint8_t* in, size_t inLen, uint8_t* out, size_t cap, size_t* outLen, uint8_t delimiter = 0x00);

uint16_t crcCompute(const uint8_t* data, size_t len);
bool crcVerify(const uint8_t* data, size_t len, uint16_t expectedCrc);
bool encodeCrc16(uint16_t crc, uint8_t* buf, size_t cap, size_t* pos);
bool decodeCrc16(const uint8_t* buf, size_t len, size_t* pos, uint16_t* crc);

uint16_t crcInit();
uint16_t crcUpdate(uint16_t crc, const uint8_t* data, size_t len);

}
