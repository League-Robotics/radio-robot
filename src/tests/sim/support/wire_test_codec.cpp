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

// readSint32 -- 124-008: reads a zigzag-encoded varint (ScalarType::kSint32,
// wire.cpp) into a raw int32_t. The GENERATED pack*()/unpack*() methods on
// the destination struct (options.proto's (scale) doc comment) turn this
// raw value into/out of a real float -- this helper stops at the raw int,
// same "decode the wire shape, let the generated method do the scale
// arithmetic" split production code (robot_loop.cpp) uses.
bool readSint32(const uint8_t* buf, size_t len, size_t* pos, int32_t* out) {
  uint64_t v = 0;
  if (!WireRuntime::decodeVarint(buf, len, pos, &v)) return false;
  *out = WireRuntime::zigzagDecode32(static_cast<uint32_t>(v));
  return true;
}

// Decodes a msg::EncoderReading payload (Telemetry.enc_left/enc_right's own
// nested bytes) -- field numbers/wire types mirror src/firm/messages/wire.cpp
// kFields_EncoderReading exactly. Recognized-field/wrong-wire-type is a hard
// failure; unrecognized fields skipped. 124-008: position/velocity are now
// zigzag sint32 (issue §B3); time is renamed age (field 3); position_epoch
// is new (field 4).
bool decodeEncoderReading(const uint8_t* buf, size_t len, msg::EncoderReading* out) {
  size_t pos = 0;
  while (pos < len) {
    uint32_t fieldNumber = 0;
    WireType wireType = WireType::kVarint;
    if (!WireRuntime::decodeTag(buf, len, &pos, &fieldNumber, &wireType)) return false;
    switch (fieldNumber) {
      case 1:
        if (wireType != WireType::kVarint || !readSint32(buf, len, &pos, &out->position)) return false;
        break;
      case 2:
        if (wireType != WireType::kVarint || !readSint32(buf, len, &pos, &out->velocity)) return false;
        break;
      case 3:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->age)) return false;
        break;
      case 4:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->position_epoch)) return false;
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
// kFields_OtosReading exactly. 124-008: x/y/heading/v_x/v_y/omega are now
// zigzag sint32 (issue §B3); time is renamed age (field 7).
bool decodeOtosReading(const uint8_t* buf, size_t len, msg::OtosReading* out) {
  size_t pos = 0;
  while (pos < len) {
    uint32_t fieldNumber = 0;
    WireType wireType = WireType::kVarint;
    if (!WireRuntime::decodeTag(buf, len, &pos, &fieldNumber, &wireType)) return false;
    switch (fieldNumber) {
      case 1:
        if (wireType != WireType::kVarint || !readSint32(buf, len, &pos, &out->x)) return false;
        break;
      case 2:
        if (wireType != WireType::kVarint || !readSint32(buf, len, &pos, &out->y)) return false;
        break;
      case 3:
        if (wireType != WireType::kVarint || !readSint32(buf, len, &pos, &out->heading)) return false;
        break;
      case 4:
        if (wireType != WireType::kVarint || !readSint32(buf, len, &pos, &out->v_x)) return false;
        break;
      case 5:
        if (wireType != WireType::kVarint || !readSint32(buf, len, &pos, &out->v_y)) return false;
        break;
      case 6:
        if (wireType != WireType::kVarint || !readSint32(buf, len, &pos, &out->omega)) return false;
        break;
      case 7:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->age)) return false;
        break;
      default:
        if (!WireRuntime::skipField(buf, len, &pos, wireType)) return false;
        break;
    }
  }
  return true;
}

// Decodes a msg::Pose2D payload -- field numbers mirror kFields_Pose2D.
// 124-008: x/y/h are now zigzag sint32 (issue §B3).
bool decodePose2D(const uint8_t* buf, size_t len, msg::Pose2D* out) {
  size_t pos = 0;
  while (pos < len) {
    uint32_t fieldNumber = 0;
    WireType wireType = WireType::kVarint;
    if (!WireRuntime::decodeTag(buf, len, &pos, &fieldNumber, &wireType)) return false;
    if (wireType == WireType::kVarint && (fieldNumber == 1 || fieldNumber == 2 || fieldNumber == 3)) {
      int32_t v = 0;
      if (!readSint32(buf, len, &pos, &v)) return false;
      if (fieldNumber == 1) out->x = v;
      else if (fieldNumber == 2) out->y = v;
      else out->h = v;
      continue;
    }
    if (!WireRuntime::skipField(buf, len, &pos, wireType)) return false;
  }
  return true;
}

// Decodes a msg::BodyTwist3 payload -- field numbers mirror
// kFields_BodyTwist3. 124-008: v_x/v_y/omega are now zigzag sint32 (issue
// §B3).
bool decodeBodyTwist3(const uint8_t* buf, size_t len, msg::BodyTwist3* out) {
  size_t pos = 0;
  while (pos < len) {
    uint32_t fieldNumber = 0;
    WireType wireType = WireType::kVarint;
    if (!WireRuntime::decodeTag(buf, len, &pos, &fieldNumber, &wireType)) return false;
    if (wireType == WireType::kVarint && (fieldNumber == 1 || fieldNumber == 2 || fieldNumber == 3)) {
      int32_t v = 0;
      if (!readSint32(buf, len, &pos, &v)) return false;
      if (fieldNumber == 1) out->v_x = v;
      else if (fieldNumber == 2) out->v_y = v;
      else out->omega = v;
      continue;
    }
    if (!WireRuntime::skipField(buf, len, &pos, wireType)) return false;
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
      // fields 5/6 (ack_corr/ack_err) -- RESERVED, not reused (124-008,
      // issue §B4: the single "freshest ack" scalar slot is deleted, ring
      // membership in acks below already means "really acked"). Any
      // incoming instance of either falls through to skipField() below.
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
        if (!decodePose2D(buf + pos, payloadLen, &out->pose)) return false;
        pos += payloadLen;
        break;
      }
      case 11: {  // twist (BodyTwist3)
        if (wireType != WireType::kLengthDelimited) return false;
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, 0, &payloadLen)) return false;
        if (!decodeBodyTwist3(buf + pos, payloadLen, &out->twist)) return false;
        pos += payloadLen;
        break;
      }
      case 12:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->line)) return false;
        break;
      case 13:
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->color)) return false;
        break;
      case 14: {  // acks (124-008: repeated uint32, PACKED -- corr_id<<4|err)
        if (wireType != WireType::kLengthDelimited) return false;
        size_t payloadLen = 0;
        if (!WireRuntime::beginLengthDelimited(buf, len, &pos, 0, &payloadLen)) return false;
        size_t outCount = 0;
        if (!WireRuntime::decodePackedVarint(buf + pos, payloadLen, out->acks_, /*maxCount=*/4, &outCount)) {
          return false;
        }
        out->acks_count = static_cast<uint8_t>(outCount);
        pos += payloadLen;
        break;
      }
      case 15:  // cycle_busy (123-004, migrated from TelemetrySecondary's field 11)
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->cycle_busy)) return false;
        break;
      case 16:  // cycle_period (123-004, migrated from TelemetrySecondary's field 12)
        if (wireType != WireType::kVarint || !readVarintU32(buf, len, &pos, &out->cycle_period)) return false;
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
      // cycle_busy/cycle_period (formerly fields 11/12, 122-003 interim
      // placement) -- MIGRATED to msg::Telemetry (123-004, see
      // decodeTelemetryMessage()'s cases 15/16 above); fields 11/12 are
      // `reserved` on the wire now (telemetry.proto), so any incoming
      // instance of either falls through to skipField() below.
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

// crcOverScope() -- independent re-implementation of comms.cpp's own
// private helper of the same name (124-003/124-005, issue §3):
// `crc16(COMMAND ':' payload)` when `command` is non-empty, `crc16(payload)`
// alone when it is not. Built the same way (WireRuntime's incremental
// crcInit()/crcUpdate(), never concatenating command+payload into one
// buffer) so this test helper and the production code it exercises are
// tested against each other, not tautologically.
uint16_t crcOverScope(const uint8_t* command, size_t commandLen, const uint8_t* payload, size_t payloadLen) {
  uint16_t crc = WireRuntime::crcInit();
  if (commandLen > 0) {
    crc = WireRuntime::crcUpdate(crc, command, commandLen);
    static constexpr uint8_t kSep = ':';
    crc = WireRuntime::crcUpdate(crc, &kSep, 1);
  }
  return WireRuntime::crcUpdate(crc, payload, payloadLen);
}

// armor() -- 124-005 (protocol v5 Part A, "framing grammar cutover"):
// builds the COMPLETE wire LINE content, `<command>':'<COBS+CRC bytes>`
// (the trailing '\n' terminator itself is a transport-level concern --
// see comms.h's own Transport::send()/readLine() doc comments -- NOT
// included in this function's own return value, mirroring what a real
// Transport::readLine() delivers to Comms::pumpTransport() with the
// delimiter already stripped). Byte-for-byte the same composition as
// App::Comms::sendReply()/decodeBinaryFrame() (comms.cpp): append the
// 2-byte CRC-16/CCITT-FALSE (little-endian, scoped over `command` too)
// to the raw schema-encoded payload, THEN COBS-encode (delimiter 0x0A)
// the combined bytes, THEN prepend `command ':'`. `command` is REQUIRED
// (no default) -- protocol v5 has no unscoped binary frame any more, every
// dispatched command needs a real registry verb name. Callers push the
// result via TestSupport::FakeTransport::enqueueInboundBinary() (an alias
// of enqueueInbound() since 124-005 -- see that class's own doc comment),
// matching how App::Comms::decodeBinaryFrame() reverses this exact
// composition.
std::string armor(const uint8_t* raw, size_t rawLen, const char* command) {
  uint8_t combined[256];
  if (rawLen > sizeof(combined) - 2) return std::string();
  std::memcpy(combined, raw, rawLen);
  size_t combinedLen = rawLen;
  const size_t commandLen = std::strlen(command);
  const uint16_t crc =
      crcOverScope(reinterpret_cast<const uint8_t*>(command), commandLen, raw, rawLen);
  if (!WireRuntime::encodeCrc16(crc, combined, sizeof(combined), &combinedLen)) return std::string();

  uint8_t framed[300];
  size_t framedLen = 0;
  if (!WireRuntime::cobsEncode(combined, combinedLen, framed, sizeof(framed), &framedLen,
                                /*delimiter=*/0x0A)) {
    return std::string();
  }
  std::string line(command, commandLen);
  line += ':';
  line.append(reinterpret_cast<const char*>(framed), framedLen);
  return line;
}

}  // namespace

DecodedLine decodeOutboundLine(const std::string& line) {
  DecodedLine result;

  // Reverse of App::Comms::sendReply()/Telemetry::emitSecondary()'s
  // CRC-then-COBS composition (comms.cpp/telemetry.cpp) -- see this file's
  // own armor() for the exact mirrored encode side. `line` is the raw
  // `<COMMAND>':'<COBS bytes>` content a TestSupport::FakeTransport
  // capture holds (FakeTransport::sent(), 124-005) -- NOT NUL-terminated
  // text, an explicit-length byte buffer that may contain any byte value.
  const size_t colon = line.find(':');
  if (colon == std::string::npos) return result;  // kUnknown -- no command prefix at all
  const std::string command = line.substr(0, colon);
  const std::string cobsPart = line.substr(colon + 1);

  uint8_t combined[256];
  size_t combinedLen = 0;
  if (!WireRuntime::cobsDecode(reinterpret_cast<const uint8_t*>(cobsPart.data()), cobsPart.size(), combined,
                                sizeof(combined), &combinedLen, /*delimiter=*/0x0A)) {
    return result;
  }
  if (combinedLen < 2) return result;

  const size_t payloadLen = combinedLen - 2;
  size_t crcPos = payloadLen;
  uint16_t receivedCrc = 0;
  if (!WireRuntime::decodeCrc16(combined, combinedLen, &crcPos, &receivedCrc)) return result;
  const uint16_t expectedCrc =
      crcOverScope(reinterpret_cast<const uint8_t*>(command.data()), command.size(), combined, payloadLen);
  if (expectedCrc != receivedCrc) return result;

  uint32_t corrId = 0;
  msg::Telemetry tlm;
  if (decodeReplyEnvelopeTlm(combined, payloadLen, &corrId, &tlm)) {
    result.kind = DecodedKind::kTelemetry;
    result.corrId = corrId;
    result.telemetry = tlm;
    return result;
  }

  msg::TelemetrySecondary sec;
  if (decodeTelemetrySecondaryMessage(combined, payloadLen, &sec)) {
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
  return armor(rawBuf, n, "MOVE");
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
  return armor(rawBuf, n, "MOVE");
}

std::string armorStopCommand(uint32_t corrId) {
  uint8_t rawBuf[32];
  size_t n = encodeStopEnvelope(corrId, rawBuf, sizeof(rawBuf));
  if (n == 0) return std::string();
  return armor(rawBuf, n, "STOP");
}

}  // namespace TestSupport
