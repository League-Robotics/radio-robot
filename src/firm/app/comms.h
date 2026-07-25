// comms.h -- App::Comms: the COBS+CRC binary-frame armor/dearmor layer
// between the two transports (serial + radio) and decoded
// msg::CommandEnvelope / msg::ReplyEnvelope.
//
// 123-002 (COBS+CRC framer integration): replaces the old "*B"+base64+
// "\r\n" line armor with a binary-clean, 0x00-delimited COBS+CRC frame,
// demuxed from the HELLO/PING text-plane rump (still "\r\n"-terminated)
// on the SAME byte stream. See wire_runtime.h's own file header (123-001)
// for the COBS/CRC primitives this file is built on, and this ticket's
// completion notes for the exact frame layout.
//
// Boundary: inside -- the COBS+CRC armor/dearmor sequence, msg::wire::
// encode()/decode() calls; outside -- deciding what a decoded command DOES
// (that is RobotLoop's own dispatch). Design/rationale: DESIGN.md.
#pragma once

#include <cstdint>

#include "messages/envelope.h"
#include "messages/wire.h"

#ifndef HOST_BUILD
class SerialPort;
class Radio;
#endif

namespace App {

// FrameKind -- which of the two coexisting frame shapes a completed
// Transport::readLine() call delivered. Never ambiguous: a binary frame is
// 0x00-free by COBS construction (0x00 is exclusively the frame
// delimiter); a text-plane line is typed ASCII (HELLO/PING) and never
// contains a 0x00 byte. Whichever terminator the transport's own
// accumulator sees FIRST (0x00 or '\n') decides the kind.
enum class FrameKind : uint8_t { kNone = 0, kText = 1, kBinary = 2 };

// Transport -- the abstract non-blocking line-in/line-out seam Comms is
// built on. Plain virtual base class (not an #ifdef HOST_BUILD fork) so
// comms.h/comms.cpp themselves never drag in MicroBit.h under HOST_BUILD;
// only the two concrete ARM adapters below are guarded.
class Transport {
 public:
  virtual ~Transport() = default;

  // Non-blocking. Demuxes a 0x00-delimited COBS+CRC binary frame from a
  // '\r'?'\n'-terminated text line (HELLO/PING) on the SAME byte stream.
  // Returns kNone when nothing complete is ready (buf/*outLen untouched).
  // kText: buf holds a NUL-terminated ASCII line (trailing '\r'/'\n'
  // stripped), *outLen == strlen(buf). kBinary: buf holds *outLen raw
  // bytes -- the still-COBS+CRC-encoded frame body (the trailing 0x00
  // delimiter itself is consumed by the transport, never included in
  // buf/*outLen). Never partially delivers a frame.
  virtual FrameKind readLine(char* buf, uint16_t cap, uint16_t* outLen) = 0;

  // Async, drop-on-full send -- ALWAYS a binary COBS+CRC frame body
  // (`data`/`len`, 0x00-free by construction); Comms::sendReply() (a
  // high-cadence caller) uses this so a full serial buffer never stalls
  // the loop. The concrete transport appends the trailing 0x00 delimiter
  // itself -- callers never include it in `data`/`len`.
  virtual void send(const uint8_t* data, uint16_t len) = 0;

  // Bounded-wait, must-not-drop send -- ALWAYS a text-plane reply
  // (NUL-terminated ASCII); used for the HELLO/PING text-exception
  // replies (rare, one-off). The concrete transport appends its own
  // text-line terminator ("\r\n" serial / "\n" radio), unchanged from
  // pre-123.
  virtual void sendReliable(const char* msg) = 0;
};

#ifndef HOST_BUILD

// SerialTransport / RadioTransport -- thin ARM-only adapters around the
// project's two real transports (com/serial_port.h, com/radio.h).
// SerialPort/Radio are only forward-declared above (not #included) so
// comms.h itself stays MicroBit-free; the real headers are #included only
// inside comms.cpp's own #ifndef HOST_BUILD block. main.cpp constructs
// these around its own SerialPort/Radio instances and passes them into
// Comms's constructor.
class SerialTransport : public Transport {
 public:
  explicit SerialTransport(SerialPort& serial);
  FrameKind readLine(char* buf, uint16_t cap, uint16_t* outLen) override;
  void send(const uint8_t* data, uint16_t len) override;
  void sendReliable(const char* msg) override;

 private:
  SerialPort& serial_;
};

class RadioTransport : public Transport {
 public:
  explicit RadioTransport(Radio& radio);
  FrameKind readLine(char* buf, uint16_t cap, uint16_t* outLen) override;
  void send(const uint8_t* data, uint16_t len) override;
  void sendReliable(const char* msg) override;

 private:
  Radio& radio_;
};

#endif  // HOST_BUILD

// kMaxEnvelopeBytes -- the larger of the two generated per-direction
// budgets (msg::wire::kCommandEnvelopeMaxEncodedSize (55) /
// kReplyEnvelopeMaxEncodedSize (194, up from 185 pre-123-004 -- ticket
// 004's cycle_busy/cycle_period primary-frame migration)) -- one raw-byte
// scratch buffer, reused sequentially for an incoming decode or an
// outgoing encode (never overlapping within a single call). Computed by
// the constexpr expression itself so a future schema regeneration that
// changes either constant updates this one automatically.
constexpr uint16_t kMaxEnvelopeBytes =
    (msg::wire::kCommandEnvelopeMaxEncodedSize > msg::wire::kReplyEnvelopeMaxEncodedSize)
        ? msg::wire::kCommandEnvelopeMaxEncodedSize
        : msg::wire::kReplyEnvelopeMaxEncodedSize;  // == 194

// kMaxCrcPayloadBytes -- kMaxEnvelopeBytes + 2 (the CRC-16 appended AFTER
// the schema payload, per the CRC-then-COBS composition -- see comms.cpp's
// sendReply()/decodeBinaryFrame() for the exact byte layout). This is the
// buffer the COBS encode/decode step itself operates on.
constexpr uint16_t kMaxCrcPayloadBytes = kMaxEnvelopeBytes + 2;  // == 196

// kFramedMaxBytes -- 123-002 recompute, replacing the old base64
// kArmoredBufSize; re-recomputed by 123-004 (kMaxEnvelopeBytes grew from
// 185 to 194 with the cycle_busy/cycle_period primary-frame migration).
// Worst-case COBS-encoded length of kMaxCrcPayloadBytes (196) zero-free
// bytes: cobsEncodedMaxLength(196) = 196 + 196/254 + 1 = 197
// (WireRuntime::cobsEncodedMaxLength()'s own documented formula, 123-001
// completion notes). This is the size of the buffer Comms builds BEFORE
// handing it to Transport::send() -- the transport appends the trailing
// 0x00 delimiter itself (one more byte on the wire, not counted here).
// Rounded up to 200 (3B headroom), the same rounding-up-for-headroom
// convention the pre-123-004 constant (188 -> 192) and the old base64-era
// kArmoredBufSize (251 -> 256) both used.
constexpr uint16_t kFramedMaxBytes = 200;
static_assert(kFramedMaxBytes >= kMaxCrcPayloadBytes + kMaxCrcPayloadBytes / 254 + 1,
              "kFramedMaxBytes must cover cobsEncodedMaxLength(kMaxCrcPayloadBytes)");

// kArmoredBufSize -- Comms's own inbound scratch-line buffer size, shared
// by pumpTransport()'s single stack buffer for whichever frame kind (text
// or binary) is next on either transport. Sized to the larger of
// kFramedMaxBytes (the binary worst case) and a generous allowance for the
// text-plane rump (HELLO/PING lines are a handful of bytes; the buffer
// only needs to be at least as big as the longest recognized text-plane
// line, which is smaller than kFramedMaxBytes by a wide margin).
constexpr uint16_t kArmoredBufSize = kFramedMaxBytes;

enum class CmdStatus : uint8_t { kNone = 0, kDecoded = 1 };

struct Cmd {
  CmdStatus status = CmdStatus::kNone;
  msg::CommandEnvelope env;
};

class Comms {
 public:
  // banner must outlive the Comms instance (caller-owned, e.g. main.cpp's
  // static buffer) -- Comms does not format or own the banner text itself.
  Comms(Transport& serialLink, Transport& radioLink, const char* banner);

  // Bounded: at most ONE Transport::readLine() call to serialLink_, and
  // (only if serial had nothing) at most one to radioLink_ -- never both
  // acted on in the same call, so "decodes at most one frame per call"
  // holds by construction, not by discarding a second ready line. Resets
  // out.status = kNone at entry; on decode success, decodes into a LOCAL
  // temporary and only assigns it into out on success, so a failed/partial
  // msg::wire::decode() can never leave partial state visible in out.
  //
  // now -- [ms], the caller's own current clock reading (RobotLoop::cycle()
  // passes its already-computed cycleStart) -- formats the PING reply's
  // `t=<ms>` field (117, SUC-056). Comms holds no Devices::Clock&/timing
  // collaborator of its own; every value it ever stamps onto a reply is
  // handed in, not read from an owned clock.
  void pump(Cmd& out, uint32_t now);  // [ms]

  // Encode (msg::wire::encode) + CRC-then-COBS frame (see comms.cpp) +
  // send ONCE on BOTH transports via Transport::send() (async/drop-on-full
  // -- telemetry is always-on and must never stall the loop on
  // backpressure; primary and secondary frames go out on both transports
  // every cadence, not just "back to whoever last spoke"). This is what
  // Telemetry calls. No return value: encode()==0 or a COBS/CRC framing
  // failure means silently send nothing.
  void sendReply(const msg::ReplyEnvelope& reply);

  // Diagnostic counter -- malformed COBS frame, CRC mismatch, malformed
  // protobuf decode, AND unrecognized text-plane lines (not "*", not
  // HELLO, not PING) all increment this. RobotLoop reads it as the
  // App::kFaultCommsMalformed telemetry fault-bit source.
  uint32_t malformedCount() const { return malformedCount_; }

 private:
  // true if a line was consumed (decoded, malformed, or text-plane) --
  // caller stops regardless (bounds pump() to at most one transport
  // acted on per call). now -- [ms], threaded straight from pump()'s own
  // argument -- see pump()'s doc comment above.
  bool pumpTransport(Transport& t, Cmd& out, uint32_t now);  // [ms]

  // NEVER replies -- acks ride Telemetry's ack ring, not per-command; see
  // comms.cpp for the discipline note. `frame`/`frameLen` is the raw
  // COBS+CRC-encoded frame body Transport::readLine() delivered (the
  // trailing 0x00 delimiter already stripped by the transport).
  void decodeBinaryFrame(const uint8_t* frame, uint16_t frameLen, Cmd& out);

  Transport& serialLink_;
  Transport& radioLink_;
  const char* banner_;
  uint32_t malformedCount_ = 0;
};

}  // namespace App
