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
// gap, plus the encode side sim_api's own injectTwist()/injectStop() need
// (there is no encode(CommandEnvelope) to call).
//
// Scope: exactly the message shapes actually exchanged over sim_api's own
// two directions -- CommandEnvelope{corr_id, cmd=TWIST|STOP} outbound (host
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
// bool/float spread. armorMoveCommand()/encodeMoveEnvelope() are DELETED --
// envelope.proto's Move message and CommandEnvelope's move(20) arm are gone
// (115-003 reserved field 20, not reused; see envelope.proto's own header).
// Twist/Stop encode and the Telemetry/TelemetrySecondary decode entry point
// keep their same field numbers/shapes on the CommandEnvelope side (config=6,
// stop=13, twist=19 are unchanged pre-102 KEPT arms).
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
// armorMoveCommand() -- DELETED (115-006, gut S1): CommandEnvelope's move(20)
// arm and envelope.proto's Move message were deleted by 115-003 alongside
// the Motion::Executor arc-command queue that consumed it; field 20 is
// reserved on the wire, not reused.
std::string armorTwistCommand(float v_x, float omega, float duration, uint32_t corrId = 0);
std::string armorStopCommand(uint32_t corrId = 0);

}  // namespace TestSupport
