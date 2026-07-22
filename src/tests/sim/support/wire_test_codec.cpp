// wire_test_codec.cpp -- see wire_test_codec.h's file header for scope and
// rationale. Every decode loop below is a flat "read tag, dispatch on field
// number, skip anything unrecognized" walk over ONE known message shape --
// deliberately not a re-implementation of wire.cpp's generic FieldDesc/
// MessageTable engine (that engine is generated, internal-linkage, and
// scoped to messages/wire.cpp only -- see this file's own header).
#include "wire_test_codec.h"

#include <cstring>

#include "messages/wire_runtime.h"

namespace TestSupport {

namespace {

using WireRuntime::WireType;

// --- Shared little decode helpers -----------------------------------------

bool readVarintU32(const uint8_t* buf, size_t len, size_t* pos, uint32_t* out) {
  uint64_t v = 0;
  if (!WireRuntime::decodeVarint(buf, len, pos, &v)) return false;
  *out = static_cast<uint32_t>(v);
  return true;
}

bool readBool(const uint8_t* buf, size_t len, size_t* pos, bool* out) {
  uint64_t v = 0;
  if (!WireRuntime::decodeVarint(buf, len, pos, &v)) return false;
  *out = (v != 0);
  return true;
}

bool readFloat(const uint8_t* buf, size_t len, size_t* pos, float* out) {
  return WireRuntime::decodeFloat(buf, len, pos, out);
}

// Decodes a flat {field1: fixed32 float, field2: fixed32 float, field3:
// fixed32 float} message -- the exact shape BOTH Pose2D (x,y,h) and
// BodyTwist3 (v_x,v_y,omega) use. Unknown fields are skipped (forward
// compatible); any recognized field arriving with the wrong wire type is
// treated as a hard decode failure (this shape has no float field that is
// legitimately anything but fixed32).
bool decodeThreeFloats(const uint8_t* buf, size_t len, float* a, float* b, float* c) {
  size_t pos = 0;
  while (pos < len) {
    uint32_t fieldNumber = 0;
    WireType wireType = WireType::kVarint;
    if (!WireRuntime::decodeTag(buf, len, &pos, &fieldNumber, &wireType)) return false;
    if ((fieldNumber == 1 || fieldNumber == 2 || fieldNumber == 3) && wireType == WireType::kFixed32) {
      float v = 0.0f;
      if (!readFloat(buf, len, &pos, &v)) return false;
      if (fieldNumber == 1) *a = v;
      else if (fieldNumber == 2) *b = v;
      else *c = v;
      continue;
    }
    if (!WireRuntime::skipField(buf, len, &pos, wireType)) return false;
  }
  return true;
}

// Decodes a msg::EncoderReading payload (Telemetry.enc_left/enc_right's own
// nested bytes) -- field numbers/wire types mirror src/firm/messages/wire.cpp
// kFields_EncoderReading exactly. Recognized-field/wrong-wire-type is a hard
// failure (same policy as decodeThreeFloats()); unrecognized fields skipped.
bool decodeEncoderReading(const uint8_t* buf, size_t len, msg::EncoderReading* out) {
  size_t pos = 0;
  while (pos < len) {
    uint32_t fieldNumber = 0;
    WireType wireType = WireType::kVarint;
    if (!WireRuntime::decodeTag(buf, len, &pos, &fieldNumber, &wireType)) return false;
    switch (fieldNumber) {
      case 1:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->position)) return false;
        break;
      case 2:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->velocity)) return false;
        break;
      case 3:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->time)) return false;
        break;
      default:
        if (!WireRuntime::skipField(buf, len, &pos, wireType)) return false;
        break;
    }
  }
  return true;
}

// Decodes a msg::OtosReading payload (Telemetry.otos's own nested bytes) --
// field numbers/wire types mirror src/firm/messages/wire.cpp
// kFields_OtosReading exactly.
bool decodeOtosReading(const uint8_t* buf, size_t len, msg::OtosReading* out) {
  size_t pos = 0;
  while (pos < len) {
    uint32_t fieldNumber = 0;
    WireType wireType = WireType::kVarint;
    if (!WireRuntime::decodeTag(buf, len, &pos, &fieldNumber, &wireType)) return false;
    switch (fieldNumber) {
      case 1:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->x)) return false;
        break;
      case 2:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->y)) return false;
        break;
      case 3:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->heading)) return false;
        break;
      case 4:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->v_x)) return false;
        break;
      case 5:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->v_y)) return false;
        break;
      case 6:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->omega)) return false;
        break;
      case 7:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->time)) return false;
        break;
      default:
        if (!WireRuntime::skipField(buf, len, &pos, wireType)) return false;
        break;
    }
  }
  return true;
}

// Decodes a msg::Telemetry payload (the ReplyEnvelope{tlm} oneof arm's own
// nested bytes) -- field numbers/wire types mirror src/firm/messages/wire.cpp
// kFields_Telemetry exactly (FRAME v2, 115-003: telemetry.proto's clean,
// incompatible rewrite -- see that proto's own header for the full "what
// changed from the 103-era frame" list). Recognized-field/wrong-wire-type is
// a hard failure (same policy as decodeThreeFloats()); unrecognized field
// numbers are skipped.
bool decodeTelemetryMessage(const uint8_t* buf, size_t len, msg::Telemetry* out) {
  size_t pos = 0;
  while (pos < len) {
    uint32_t fieldNumber = 0;
    WireType wireType = WireType::kVarint;
    if (!WireRuntime::decodeTag(buf, len, &pos, &fieldNumber, &wireType)) return false;

    switch (fieldNumber) {
      case 1:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->now)) return false;
        break;
      case 2:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->seq)) return false;
        break;
      case 3: {
        if (wireType != WireType::kVarint) return false;
        uint32_t v = 0;
        if (!readVarintU32(buf, len, &pos, &v)) return false;
        out->mode = static_cast<msg::DriveMode>(v);
        break;
      }
      case 4:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->flags)) return false;
        break;
      case 5:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->ack_corr)) return false;
        break;
      case 6:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->ack_err)) return false;
        break;
      case 7: {  // enc_left (EncoderReading)
        if (wireType != WireType::kLengthDelimited) return false;
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, 0, &payloadLen)) return false;
        if (!decodeEncoderReading(buf + pos, payloadLen, &out->enc_left)) return false;
        pos += payloadLen;
        break;
      }
      case 8: {  // enc_right (EncoderReading)
        if (wireType != WireType::kLengthDelimited) return false;
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, 0, &payloadLen)) return false;
        if (!decodeEncoderReading(buf + pos, payloadLen, &out->enc_right)) return false;
        pos += payloadLen;
        break;
      }
      case 9: {  // otos (OtosReading)
        if (wireType != WireType::kLengthDelimited) return false;
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, 0, &payloadLen)) return false;
        if (!decodeOtosReading(buf + pos, payloadLen, &out->otos)) return false;
        pos += payloadLen;
        break;
      }
      case 10: {  // pose (Pose2D)
        if (wireType != WireType::kLengthDelimited) return false;
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, 0, &payloadLen)) return false;
        if (!decodeThreeFloats(buf + pos, payloadLen, &out->pose.x, &out->pose.y, &out->pose.h)) return false;
        pos += payloadLen;
        break;
      }
      case 11: {  // twist (BodyTwist3)
        if (wireType != WireType::kLengthDelimited) return false;
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, 0, &payloadLen)) return false;
        if (!decodeThreeFloats(buf + pos, payloadLen, &out->twist.v_x, &out->twist.v_y, &out->twist.omega)) {
          return false;
        }
        pos += payloadLen;
        break;
      }
      case 12:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->line)) return false;
        break;
      case 13:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->color)) return false;
        break;
      default:
        if (!WireRuntime::skipField(buf, len, &pos, wireType)) return false;
        break;
    }
  }
  return true;
}

// Attempts a ReplyEnvelope{corr_id, body=TLM} decode -- fields/wire types
// mirror kFields_ReplyEnvelope. Any recognized field number arriving with a
// wire type other than what the schema declares is treated as "this is not
// a ReplyEnvelope after all" (returns false) rather than a hard error --
// the caller (decodeOutboundLine()) uses that to fall back to trying
// TelemetrySecondary instead, since the two shapes are otherwise
// undiscriminated on the wire (no message-type tag of their own). Success
// requires the tlm oneof arm (field 4) to have actually been seen --
// telemetry.cpp's emitPrimary() is the only production caller of this
// shape and always sets body_kind=TLM (ACKs ride Telemetry.acks_[], never
// a body_kind=OK/ERR reply -- see this file's own header) -- a ReplyEnvelope
// with only ok/err present is out of this decoder's scope.
bool decodeReplyEnvelopeTlm(const uint8_t* buf, size_t len, uint32_t* corrId, msg::Telemetry* tlm) {
  size_t pos = 0;
  bool sawTlm = false;
  while (pos < len) {
    uint32_t fieldNumber = 0;
    WireType wireType = WireType::kVarint;
    if (!WireRuntime::decodeTag(buf, len, &pos, &fieldNumber, &wireType)) return false;

    if (fieldNumber == 1) {
      if (wireType != WireType::kVarint) return false;
      if (!readVarintU32(buf, len, &pos, corrId)) return false;
    } else if (fieldNumber == 4) {
      if (wireType != WireType::kLengthDelimited) return false;
      size_t payloadLen = 0;
      if (!WireRuntime::beginLengthDelimited(buf, len, &pos, 0, &payloadLen)) return false;
      if (!decodeTelemetryMessage(buf + pos, payloadLen, tlm)) return false;
      pos += payloadLen;
      sawTlm = true;
    } else if (fieldNumber == 2 || fieldNumber == 3) {
      // ok/err oneof arms -- schema-valid but out of this decoder's scope
      // (see the function comment above); reject the wire-type mismatch
      // case exactly like every other field, but a well-formed ok/err still
      // fails this function (sawTlm never set) so the caller correctly
      // reports kUnknown rather than a half-populated kTelemetry.
      if (wireType != WireType::kLengthDelimited) return false;
      size_t payloadLen = 0;
      if (!WireRuntime::beginLengthDelimited(buf, len, &pos, 0, &payloadLen)) return false;
      pos += payloadLen;
    } else {
      if (!WireRuntime::skipField(buf, len, &pos, wireType)) return false;
    }
  }
  return sawTlm;
}

// Attempts a standalone TelemetrySecondary decode -- fields/wire types
// mirror kFields_TelemetrySecondary. Same wire-type-mismatch-means-"not
// this shape" policy as decodeReplyEnvelopeTlm() above.
bool decodeTelemetrySecondaryMessage(const uint8_t* buf, size_t len, msg::TelemetrySecondary* out) {
  size_t pos = 0;
  while (pos < len) {
    uint32_t fieldNumber = 0;
    WireType wireType = WireType::kVarint;
    if (!WireRuntime::decodeTag(buf, len, &pos, &fieldNumber, &wireType)) return false;

    switch (fieldNumber) {
      case 1:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->now)) return false;
        break;
      case 2:
        if (wireType != WireType::kVarint || !readBool(buf, len, &pos, &out->has_cmd_vel)) return false;
        break;
      case 3:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->cmd_vel_left)) return false;
        break;
      case 4:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->cmd_vel_right)) return false;
        break;
      case 5:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->acc_left)) return false;
        break;
      case 6:
        if (wireType != WireType::kFixed32 || !readFloat(buf, len, &pos, &out->acc_right)) return false;
        break;
      case 7:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->glitch_left)) return false;
        break;
      case 8:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->glitch_right)) return false;
        break;
      case 9:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->ts_left)) return false;
        break;
      case 10:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->ts_right)) return false;
        break;
      default:
        if (!WireRuntime::skipField(buf, len, &pos, wireType)) return false;
        break;
    }
  }
  return true;
}

// --- Encode helpers (CommandEnvelope{MOVE|STOP}, host -> firmware) --------

bool encodeVarintField(uint32_t fieldNumber, uint32_t value, uint8_t* buf, size_t cap, size_t* pos) {
  if (value == 0) return true;  // proto3 implicit presence -- matches encodeInto()'s own convention
  if (!WireRuntime::encodeTag(fieldNumber, WireType::kVarint, buf, cap, pos)) return false;
  return WireRuntime::encodeVarint(value, buf, cap, pos);
}

bool encodeFloatField(uint32_t fieldNumber, float value, uint8_t* buf, size_t cap, size_t* pos) {
  if (value == 0.0f) return true;  // proto3 implicit presence
  if (!WireRuntime::encodeTag(fieldNumber, WireType::kFixed32, buf, cap, pos)) return false;
  return WireRuntime::encodeFloat(value, buf, cap, pos);
}

// Encodes a real-oneof SCALAR arm unconditionally -- mirrors wire.cpp's own
// generated kOneofScalar encode rule (encodeInto()'s FieldKind::kOneofScalar
// case): a oneof arm's presence is decided by the caller SELECTING it, never
// by implicit-presence value-skipping, so this writes the tag+value even
// when `value == 0.0f` (unlike encodeFloatField() above, which is only
// correct for a PLAIN scalar field). Used for Move's `stop` oneof
// (time=3/distance=4/angle=5).
bool encodeOneofFloatField(uint32_t fieldNumber, float value, uint8_t* buf, size_t cap, size_t* pos) {
  if (!WireRuntime::encodeTag(fieldNumber, WireType::kFixed32, buf, cap, pos)) return false;
  return WireRuntime::encodeFloat(value, buf, cap, pos);
}

// Encodes a length-delimited field's already-encoded `payload` bytes behind
// its own tag -- used to wrap Move's `velocity` oneof arm (a NESTED MESSAGE
// oneof arm, MoveTwist=1/MoveWheels=2), which per encodeNestedMessage()'s own
// convention is wrapped unconditionally once the caller selects a variant
// (even if every payload float happens to be 0.0 and its own encode left
// `payload` empty).
bool encodeNestedField(uint32_t fieldNumber, const uint8_t* payload, size_t payloadLen,
                        uint8_t* buf, size_t cap, size_t* pos) {
  if (!WireRuntime::encodeTag(fieldNumber, WireType::kLengthDelimited, buf, cap, pos)) return false;
  if (!WireRuntime::encodeVarint(payloadLen, buf, cap, pos)) return false;
  if (cap - *pos < payloadLen) return false;
  std::memcpy(buf + *pos, payload, payloadLen);
  *pos += payloadLen;
  return true;
}

// Encodes CommandEnvelope field 13 (stop, length-delimited oneof arm, empty
// payload -- Stop has no fields at all), then field 1 (corr_id) if nonzero.
size_t encodeStopEnvelope(uint32_t corrId, uint8_t* buf, size_t cap) {
  size_t pos = 0;
  if (!encodeVarintField(1, corrId, buf, cap, &pos)) return 0;
  if (!WireRuntime::encodeTag(13, WireType::kLengthDelimited, buf, cap, &pos)) return 0;
  if (!WireRuntime::encodeVarint(0, buf, cap, &pos)) return 0;  // zero-length payload
  return pos;
}

// Encodes Move's own body (velocity oneof arm already wrapped by the
// caller as `velocityFieldNumber`/`velocityPayload`, the `stop` oneof arm
// selected by `stopKind`/`stopValue`, then the three plain fields
// timeout=6/replace=7/id=8) into `scratch`, wraps it as CommandEnvelope
// field 21 (move, length-delimited oneof arm -- kFields_CommandEnvelope),
// then field 1 (corr_id) if nonzero. Shared by both armorMoveCommand()
// overloads below -- the only difference between a MoveTwist and a
// MoveWheels command is which velocity payload/field number the caller
// already built.
size_t encodeMoveEnvelope(uint32_t velocityFieldNumber, const uint8_t* velocityPayload, size_t velocityPayloadLen,
                           MoveStopKind stopKind, float stopValue, float timeout, bool replace, uint32_t id,
                           uint32_t corrId, uint8_t* buf, size_t cap) {
  size_t pos = 0;
  if (!encodeVarintField(1, corrId, buf, cap, &pos)) return 0;

  uint8_t scratch[64];
  size_t scratchPos = 0;
  if (!encodeNestedField(velocityFieldNumber, velocityPayload, velocityPayloadLen, scratch, sizeof(scratch),
                          &scratchPos)) {
    return 0;
  }

  uint32_t stopFieldNumber = 0;
  switch (stopKind) {
    case MoveStopKind::kTime:     stopFieldNumber = 3; break;
    case MoveStopKind::kDistance: stopFieldNumber = 4; break;
    case MoveStopKind::kAngle:    stopFieldNumber = 5; break;
  }
  if (!encodeOneofFloatField(stopFieldNumber, stopValue, scratch, sizeof(scratch), &scratchPos)) return 0;

  if (!encodeFloatField(6, timeout, scratch, sizeof(scratch), &scratchPos)) return 0;
  if (!encodeVarintField(7, replace ? 1u : 0u, scratch, sizeof(scratch), &scratchPos)) return 0;
  if (!encodeVarintField(8, id, scratch, sizeof(scratch), &scratchPos)) return 0;

  if (!WireRuntime::encodeTag(21, WireType::kLengthDelimited, buf, cap, &pos)) return 0;
  if (!WireRuntime::encodeVarint(scratchPos, buf, cap, &pos)) return 0;
  if (cap - pos < scratchPos) return 0;
  std::memcpy(buf + pos, scratch, scratchPos);
  pos += scratchPos;
  return pos;
}

std::string armor(const uint8_t* raw, size_t rawLen) {
  char b64[512] = {};
  size_t b64Len = 0;
  if (!WireRuntime::base64Encode(raw, rawLen, b64, sizeof(b64), &b64Len)) return std::string();
  std::string out = "*B";
  out.append(b64, b64Len);
  return out;
}

}  // namespace

DecodedLine decodeOutboundLine(const std::string& line) {
  DecodedLine result;
  if (line.size() < 2 || line[0] != '*' || line[1] != 'B') return result;

  const char* b64 = line.c_str() + 2;
  size_t b64Len = line.size() - 2;
  while (b64Len > 0 && (b64[b64Len - 1] == '\r' || b64[b64Len - 1] == '\n' ||
                        b64[b64Len - 1] == ' ' || b64[b64Len - 1] == '\t')) {
    --b64Len;
  }

  uint8_t rawBuf[256];
  size_t rawLen = 0;
  if (!WireRuntime::base64Decode(b64, b64Len, rawBuf, sizeof(rawBuf), &rawLen)) return result;

  uint32_t corrId = 0;
  msg::Telemetry tlm;
  if (decodeReplyEnvelopeTlm(rawBuf, rawLen, &corrId, &tlm)) {
    result.kind = DecodedKind::kTelemetry;
    result.corrId = corrId;
    result.telemetry = tlm;
    return result;
  }

  msg::TelemetrySecondary sec;
  if (decodeTelemetrySecondaryMessage(rawBuf, rawLen, &sec)) {
    result.kind = DecodedKind::kSecondary;
    result.secondary = sec;
    return result;
  }

  return result;  // kUnknown
}

std::string armorMoveCommand(float v_x, float v_y, float omega, MoveStopKind stopKind, float stopValue,
                              float timeout, bool replace, uint32_t id, uint32_t corrId) {
  uint8_t velocityScratch[32];
  size_t velocityLen = 0;
  if (!encodeFloatField(1, v_x, velocityScratch, sizeof(velocityScratch), &velocityLen)) return std::string();
  if (!encodeFloatField(2, v_y, velocityScratch, sizeof(velocityScratch), &velocityLen)) return std::string();
  if (!encodeFloatField(3, omega, velocityScratch, sizeof(velocityScratch), &velocityLen)) return std::string();

  uint8_t rawBuf[128];
  size_t n = encodeMoveEnvelope(/* velocity field = twist */ 1, velocityScratch, velocityLen, stopKind, stopValue,
                                 timeout, replace, id, corrId, rawBuf, sizeof(rawBuf));
  if (n == 0) return std::string();
  return armor(rawBuf, n);
}

std::string armorMoveCommand(float v_left, float v_right, MoveStopKind stopKind, float stopValue, float timeout,
                              bool replace, uint32_t id, uint32_t corrId) {
  uint8_t velocityScratch[32];
  size_t velocityLen = 0;
  if (!encodeFloatField(1, v_left, velocityScratch, sizeof(velocityScratch), &velocityLen)) return std::string();
  if (!encodeFloatField(2, v_right, velocityScratch, sizeof(velocityScratch), &velocityLen)) return std::string();

  uint8_t rawBuf[128];
  size_t n = encodeMoveEnvelope(/* velocity field = wheels */ 2, velocityScratch, velocityLen, stopKind, stopValue,
                                 timeout, replace, id, corrId, rawBuf, sizeof(rawBuf));
  if (n == 0) return std::string();
  return armor(rawBuf, n);
}

std::string armorStopCommand(uint32_t corrId) {
  uint8_t rawBuf[32];
  size_t n = encodeStopEnvelope(corrId, rawBuf, sizeof(rawBuf));
  if (n == 0) return std::string();
  return armor(rawBuf, n);
}

}  // namespace TestSupport
