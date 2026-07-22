// wire_test_codec.h -- TestSupport: a minimal, HAND-WRITTEN wire codec for
// the two directions the generated msg::wire::{encode,decode}() overload set
// does NOT cover, built purely on WireRuntime's public byte-level primitives
// (src/firm/messages/wire_runtime.h) -- the same primitives msg::wire's own
// generated decodeInto()/encodeInto() engine (src/firm/messages/wire.cpp) uses
// internally, never the generated engine itself (that lives in wire.{h,cpp}
// and is not reachable from outside that translation unit -- decodeInto()/
// encodeInto() sit in an anonymous namespace).
//
// The gap this fills (ticket 105-004, SUC-021): envelope.proto's own
// asymmetric API (documented at wire.h's Result decode(CommandEnvelope&,...)
// declaration, and re-confirmed by app_telemetry_harness.cpp's own file
// header -- "No decode(ReplyEnvelope) codec exists (firmware only ever
// ENCODES a ReplyEnvelope)") means the generated codec gives firmware code
// exactly what IT needs (decode an inbound CommandEnvelope, encode an
// outbound ReplyEnvelope/TelemetrySecondary) and nothing else -- there is no
// encode(CommandEnvelope) (a HOST builds commands; production firmware code
// never does) and no decode(ReplyEnvelope)/decode(TelemetrySecondary) (a
// HOST decodes telemetry; production firmware code never does). Every
// existing HOST_BUILD harness that needed to inspect an outbound frame
// worked around this by independently RE-ENCODING the expected value and
// comparing bytes (app_telemetry_harness.cpp's armorReply()) -- sufficient
// when the test already knows the exact expected value, but not when a
// scenario needs to READ a value out of a live telemetry stream (this
// ticket's own twist-ramp scenario: velLeft/velRight are not known in
// advance, they come from the plant). This file is the read side of that
// gap, plus the encode side sim_api's own inject*()/injectStop() need
// (there is no encode(CommandEnvelope) to call).
//
// Scope: exactly the message shapes actually exchanged over sim_api's own
// two directions -- CommandEnvelope{corr_id, cmd=MOVE|STOP} outbound (host
// -> firmware) and ReplyEnvelope{corr_id, body=TLM(Telemetry)} /
// TelemetrySecondary inbound (firmware -> host, the only two shapes
// App::Telemetry::emit() ever actually sends -- see telemetry.cpp's
// emitPrimary()/emitSecondary(): the single ack slot rides inside the
// Telemetry frame itself (ack_corr/ack_err, valid iff flags bit 5), never a
// separate body_kind=OK/ERR ReplyEnvelope). NOT a general schema-walking
// engine -- there is no FieldDesc/MessageTable machinery here, just a flat
// tag-dispatch loop per message shape. A future ticket needing a third
// shape decoded adds a third flat loop, not a generalization of these two.
//
// 115-006 (gut S1 sim lockstep): rewritten for telemetry.proto's FRAME v2
// (115-003) -- decodeTelemetryMessage() now decodes the nested
// EncoderReading/OtosReading messages and the single flags/ack_corr/ack_err
// fields instead of the deleted depth-3 AckEntry ring and the pre-115 flat
// bool/float spread. The THEN-current armorMoveCommand()/encodeMoveEnvelope()
// (the sprint-109 arc-command shape) were deleted at that point -- 115-003
// reserved field 20, not reused (see envelope.proto's own header).
//
// 116-001 (MOVE protocol cutover): `Twist` (arm 19, encodeTwistEnvelope()/
// armorTwistCommand()) is DELETED -- reserved, not reused -- superseded by
// a NEW `Move` message at a FRESH arm number (21, never 20 -- a different
// shape from the deleted 115-003 arc-command `Move` above): a bounded
// motion command (`MoveTwist|MoveWheels` velocity oneof + `time|distance|
// angle` stop oneof + `timeout`/`replace`/`id`). `armorMoveCommand()` is
// REINTRODUCED below under the same name for an unrelated, textually-fresh
// shape -- see its own doc comment. `Stop` encode and the Telemetry/
// TelemetrySecondary decode entry point keep their same field
// numbers/shapes on the CommandEnvelope side (config=6, stop=13 are
// unchanged pre-102 KEPT arms; move=21 is 116-001's own fresh arm).
#pragma once

#include <cstdint>
#include <string>

#include "messages/common.h"
#include "messages/envelope.h"

namespace TestSupport {

// --- Outbound decode (firmware -> host) -----------------------------------

// The two shapes App::Telemetry ever actually emits (telemetry.cpp's
// emitPrimary()/emitSecondary(): never both in the same emit() call) --
// kUnknown covers anything else (malformed armor, unrecognized bytes, a
// HELLO/PING plain-text reply this decoder is not meant to parse).
enum class DecodedKind : uint8_t { kUnknown = 0, kTelemetry = 1, kSecondary = 2 };

struct DecodedLine {
  DecodedKind kind = DecodedKind::kUnknown;
  uint32_t corrId = 0;              // valid when kind == kTelemetry (ReplyEnvelope.corr_id)
  msg::Telemetry telemetry = {};    // valid when kind == kTelemetry
  msg::TelemetrySecondary secondary = {};  // valid when kind == kSecondary
};

// Dearmors ("*B" + base64) and decodes ONE outbound line -- exactly what a
// FakeTransport::sent()/sentReliable() entry holds. Returns kind ==
// kUnknown (all other fields default-constructed) on anything that isn't a
// well-formed instance of one of the two shapes above -- a malformed/
// truncated line, or a plain-text HELLO/PING reply (line[0] != '*').
DecodedLine decodeOutboundLine(const std::string& line);

// --- Inbound encode (host -> firmware) --------------------------------
//
// Builds a complete armored ("*B"+base64) CommandEnvelope line, byte-for-
// byte what a real host would send over serial/radio -- the reverse of
// App::Comms::decodeArmoredLine(). corrId == 0 is proto3's own implicit-
// presence default (omitted from the wire, exactly like every other
// generated encode() path in this codebase) -- pass a nonzero value to
// exercise the single-ack-slot/correlation-id path (Telemetry.ack_corr,
// valid iff flags bit 5).
//
// armorTwistCommand() -- DELETED (116-001, MOVE protocol cutover):
// CommandEnvelope's twist(19) arm and envelope.proto's Twist message are
// deleted, superseded by move(21)/Move below; field 19 is reserved on the
// wire, not reused.
std::string armorStopCommand(uint32_t corrId = 0);

// MOVE stop-condition kind selector for armorMoveCommand() below -- mirrors
// Move's own `stop` oneof arms (time=3/distance=4/angle=5 on the wire).
enum class MoveStopKind : uint8_t { kTime = 0, kDistance = 1, kAngle = 2 };

// armorMoveCommand() -- builds a complete armored CommandEnvelope{move:
// Move{...}} line (116-001, MOVE protocol cutover), byte-for-byte what a
// real host would send, the reverse of App::Comms::decodeArmoredLine() --
// same convention armorTwistCommand()/armorStopCommand() established.
// Two overloads cover Move's two velocity variants (MoveTwist/MoveWheels);
// they never collide under overload resolution because `stopKind`
// (MoveStopKind, a scoped enum with no implicit conversion to/from float)
// sits at a different, type-incompatible parameter position in each
// signature. `stopKind`/`stopValue` select which of Move's three `stop`
// oneof arms is encoded, so these two overloads together cover every
// velocity-variant x stop-kind combination the protocol defines.
// `timeout`/`replace`/`id` are Move's own required/plain fields; `corrId
// == 0` is proto3's own implicit-presence default (omitted from the wire,
// same convention every other builder in this file uses).
std::string armorMoveCommand(float v_x, float v_y, float omega,
                              MoveStopKind stopKind, float stopValue,
                              float timeout, bool replace, uint32_t id,
                              uint32_t corrId = 0);
std::string armorMoveCommand(float v_left, float v_right,
                              MoveStopKind stopKind, float stopValue,
                              float timeout, bool replace, uint32_t id,
                              uint32_t corrId = 0);

}  // namespace TestSupport
