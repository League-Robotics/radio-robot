// comms.h -- App::Comms: the "*B" armor/dearmor framing layer between the
// two transports (serial + radio) and decoded msg::CommandEnvelope /
// msg::ReplyEnvelope.
//
// Boundary: inside -- the "*B" armor/dearmor sequence, msg::wire::encode()/
// decode() calls; outside -- deciding what a decoded command DOES (that is
// RobotLoop's own dispatch). Design/rationale: DESIGN.md.
#pragma once

#include <cstdint>

#include "messages/envelope.h"
#include "messages/wire.h"

#ifndef HOST_BUILD
class SerialPort;
class Radio;
#endif

namespace App {

// Transport -- the abstract non-blocking line-in/line-out seam Comms is
// built on. Plain virtual base class (not an #ifdef HOST_BUILD fork) so
// comms.h/comms.cpp themselves never drag in MicroBit.h under HOST_BUILD;
// only the two concrete ARM adapters below are guarded.
class Transport {
 public:
  virtual ~Transport() = default;

  // Non-blocking. True + fills buf (NUL-terminated) when one complete line
  // is ready. SerialPort::readLine() already strips the trailing '\n';
  // Radio::poll() does not (dearmor's own trailing-whitespace trim, per
  // the transcription note, handles both). Never sleeps, never blocks.
  virtual bool readLine(char* buf, uint16_t len) = 0;

  // Async, drop-on-full send -- for telemetry; Comms::sendReply() (a
  // high-cadence caller) uses this so a full serial buffer never stalls
  // the loop.
  virtual void send(const char* msg) = 0;

  // Bounded-wait, must-not-drop send -- for replies/EVT; used for the
  // HELLO/PING text-exception replies (rare, one-off).
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
  bool readLine(char* buf, uint16_t len) override;
  void send(const char* msg) override;
  void sendReliable(const char* msg) override;

 private:
  SerialPort& serial_;
};

class RadioTransport : public Transport {
 public:
  explicit RadioTransport(Radio& radio);
  bool readLine(char* buf, uint16_t len) override;
  void send(const char* msg) override;         // both send() and sendReliable()
  void sendReliable(const char* msg) override;  // delegate to radio_.send() --
                                                 // Radio has only one send path
 private:
  Radio& radio_;
};

#endif  // HOST_BUILD

// kMaxEnvelopeBytes -- the larger of the two generated per-direction
// budgets (msg::wire::kCommandEnvelopeMaxEncodedSize (115) /
// kReplyEnvelopeMaxEncodedSize (179)) -- one raw-byte scratch buffer,
// reused sequentially for an incoming decode or an outgoing encode (never
// overlapping within a single call). Computed by the constexpr expression
// itself so a future schema regeneration that changes either constant
// updates this one automatically.
constexpr uint16_t kMaxEnvelopeBytes =
    (msg::wire::kCommandEnvelopeMaxEncodedSize > msg::wire::kReplyEnvelopeMaxEncodedSize)
        ? msg::wire::kCommandEnvelopeMaxEncodedSize
        : msg::wire::kReplyEnvelopeMaxEncodedSize;  // == 179

// kArmoredBufSize -- "*B" (2) + base64(kMaxEnvelopeBytes=179) (ceil(179/3)*4
// = 240) + NUL (1) = 243, rounded up to 256 with headroom -- matches
// SerialPort's own 256-byte _rxBuf and stays under the ~250B outbound-line
// guidance (243 < 250).
constexpr uint16_t kArmoredBufSize = 256;

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
  void pump(Cmd& out);

  // Encode (msg::wire::encode) + armor ("*B" + base64) + send ONCE on BOTH
  // transports via Transport::send() (async/drop-on-full -- telemetry is
  // always-on and must never stall the loop on backpressure; primary and
  // secondary frames go out on both transports every cadence, not just
  // "back to whoever last spoke"). This is what Telemetry calls. No return
  // value: encode()==0 or base64Encode() failure means silently send
  // nothing.
  void sendReply(const msg::ReplyEnvelope& reply);

  // Diagnostic counter -- malformed armor, malformed base64, malformed
  // protobuf decode, AND unrecognized text-plane lines (not "*", not
  // HELLO, not PING) all increment this. RobotLoop reads it as the
  // App::kFaultCommsMalformed telemetry fault-bit source.
  uint32_t malformedCount() const { return malformedCount_; }

 private:
  // true if a line was consumed (decoded, malformed, or text-plane) --
  // caller stops regardless (bounds pump() to at most one transport
  // acted on per call).
  bool pumpTransport(Transport& t, Cmd& out);

  // NEVER replies -- acks ride Telemetry's ack ring, not per-command; see
  // comms.cpp for the discipline note.
  void decodeArmoredLine(const char* line, Cmd& out);

  Transport& serialLink_;
  Transport& radioLink_;
  const char* banner_;
  uint32_t malformedCount_ = 0;
};

}  // namespace App
