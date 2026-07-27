// comms.h -- App::Comms: parses/emits protocol v5's uniform packet grammar
// (`<COMMAND>[':' <data>]'\n'`) and, for binary commands, the COBS+CRC
// binary-frame armor/dearmor layer between the two transports (serial +
// radio) and decoded msg::CommandEnvelope / msg::ReplyEnvelope.
//
// 124-005 (protocol v5 Part A, "framing grammar cutover" -- issue
// protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md §1-§4):
// replaces the old two-heuristic text/binary demux (a transport-level
// guess about which of two incompatible framings a completed line was)
// with ONE uniform grammar in both directions: every line, text or binary,
// is `<COMMAND>[':' <data>]'\n'`. The transport delivers a bare,
// `\n`-terminated line (COBS is now keyed on 0x0A -- see wire_runtime.h
// item 8 -- so a binary frame's own bytes never contain a literal 0x0A,
// making the terminator genuinely unconditional); Comms parses the
// `<COMMAND>` prefix, looks it up in the generated command registry
// (messages/commands.h's kVerbTable[]), and THAT lookup's `binary` flag --
// not anything about the data's own bytes -- is the sole decision of how
// `<data>` is read. Supersedes 123-002's "*B"+base64 armor and 123-006's
// exact-match text-command recognizer alike; both are gone, not
// deprecated. See wire_runtime.h's own file header (123-001/124-003) for
// the COBS/CRC primitives this file is built on.
//
// Boundary: inside -- the grammar parse/emit, CRC-over-command-and-payload
// composition, dispatch of text vs. binary by registry lookup, the
// reply-plane grammar (DEVICE/PONG/ID/VER); outside -- deciding what a
// decoded command DOES (that is RobotLoop's own dispatch), physical
// transport framing (Com::SerialPort/Com::Radio's own job). Design/
// rationale: DESIGN.md.
#pragma once

#include <cstddef>
#include <cstdint>

#include "messages/commands.h"
#include "messages/envelope.h"
#include "messages/wire.h"

#ifndef HOST_BUILD
class SerialPort;
class Radio;
#endif

namespace App {

// kCobsDelimiter -- protocol v5's wire delimiter (issue §2): COBS is now
// keyed on 0x0A ('\n'), not 0x00. Every WireRuntime::cobsEncode()/
// cobsDecode() call in this file passes this explicitly rather than
// relying on that function's own 0x00 default (which stays 0x00 for the
// pre-124 callers WireRuntime.h documents, e.g. wire_differential_harness).
constexpr uint8_t kCobsDelimiter = 0x0A;

// Transport -- the abstract non-blocking line-in/line-out seam Comms is
// built on. Plain virtual base class (not an #ifdef HOST_BUILD fork) so
// comms.h/comms.cpp themselves never drag in MicroBit.h under HOST_BUILD;
// only the two concrete ARM adapters below are guarded.
class Transport {
 public:
  virtual ~Transport() = default;

  // Non-blocking. Delivers ONE complete, unconditionally `\n`-terminated
  // wire line (the delimiter itself consumed, never included in
  // buf/*outLen) -- text or binary, Comms decides which by parsing the
  // `<COMMAND>` prefix, not this call. Returns false when nothing complete
  // is ready yet (buf/*outLen untouched); true with buf holding *outLen
  // raw bytes otherwise. `\r` is NOT stripped here -- under one uniform
  // rule `\r` is legal binary content, so an unconditional strip would be
  // a bug; Comms strips a trailing `\r` only for a line it has already
  // classified as cleartext (124-005, issue §7). Never partially delivers
  // a line.
  virtual bool readLine(char* buf, uint16_t cap, uint16_t* outLen) = 0;

  // Async, drop-on-full send -- `data`/`len` is the COMPLETE wire line
  // content this call must emit, MINUS the trailing `\n` terminator (the
  // concrete transport appends that itself, converged to a single `\n`
  // for both text and binary payloads -- issue §7). For a binary reply
  // this is `<COMMAND>':'<COBS+CRC bytes>`; Comms::sendReply() (a
  // high-cadence caller) uses this so a full serial buffer never stalls
  // the loop.
  virtual void send(const uint8_t* data, uint16_t len) = 0;

  // Bounded-wait, must-not-drop send -- ALWAYS a cleartext reply line
  // (NUL-terminated ASCII, e.g. `banner_`/`"PONG:t=<ms>"`); used for the
  // HELLO/PING/ID/VER text-exception replies (rare, one-off). The
  // concrete transport appends a single trailing `\n` (NOT `\r\n` --
  // issue §7: `sendReliable()`'s old `"\r\n"` is retired along with the
  // rest of the two-terminator split).
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
  bool readLine(char* buf, uint16_t cap, uint16_t* outLen) override;
  void send(const uint8_t* data, uint16_t len) override;
  void sendReliable(const char* msg) override;

 private:
  SerialPort& serial_;
};

class RadioTransport : public Transport {
 public:
  explicit RadioTransport(Radio& radio);
  bool readLine(char* buf, uint16_t cap, uint16_t* outLen) override;
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
// buffer the COBS encode/decode step itself operates on. Unaffected by
// 124-005's command prefix -- the prefix lives OUTSIDE the COBS region (see
// crcOverScope()'s own doc comment, comms.cpp) -- so this is unchanged from
// 123-004.
constexpr uint16_t kMaxCrcPayloadBytes = kMaxEnvelopeBytes + 2;  // == 196

// kFramedMaxBytes -- 123-002 recompute, replacing the old base64
// kArmoredBufSize; re-recomputed by 123-004 (kMaxEnvelopeBytes grew from
// 185 to 194 with the cycle_busy/cycle_period primary-frame migration).
// Worst-case COBS-encoded length of kMaxCrcPayloadBytes (196) zero-free
// bytes: cobsEncodedMaxLength(196) = 196 + 196/254 + 1 = 197
// (WireRuntime::cobsEncodedMaxLength()'s own documented formula, 123-001
// completion notes). This is the size of the COBS-encoded region ALONE --
// still 200 (3B headroom) under 124-005, same as 123-004, because the
// ASCII command prefix is not part of this region (see kMaxLineBytes
// below for the buffer that DOES need to grow for the prefix). The stale
// "kFramedMaxBytes (192)" comment at com/radio.cpp was wrong since
// 123-004 (it is, and remains, 200) -- corrected there by this ticket.
constexpr uint16_t kFramedMaxBytes = 200;
static_assert(kFramedMaxBytes >= kMaxCrcPayloadBytes + kMaxCrcPayloadBytes / 254 + 1,
              "kFramedMaxBytes must cover cobsEncodedMaxLength(kMaxCrcPayloadBytes)");

// maxVerbNameLength() -- compile-time max over messages/commands.h's
// generated kVerbTable[] name lengths (124-001's registry). A constexpr
// function (not a hand-picked literal) so a future verb longer than
// "CONFIG"/"DEVICE" (6 bytes) is picked up automatically the next time
// commands.proto is regenerated, per this sprint's own "re-derive, never
// hand-edit" convention for size constants (issue §6).
constexpr size_t maxVerbNameLength() {
  size_t maxLen = 0;
  for (uint8_t i = 0; i < msg::kVerbCount; ++i) {
    size_t len = 0;
    for (const char* p = msg::kVerbTable[i].name; *p != '\0'; ++p) ++len;
    if (len > maxLen) maxLen = len;
  }
  return maxLen;
}

// kMaxCommandPrefixBytes -- the longest possible `<COMMAND>':'` prefix
// (124-005, issue §6's "kFramedMaxBytes/kMaxCrcPayloadBytes need
// re-derivation against the new worst case (longest command name +
// separator)"): the prefix itself doesn't grow kFramedMaxBytes/
// kMaxCrcPayloadBytes (it is outside the COBS region -- see those
// constants' own comments above), but the WHOLE-LINE buffer below does
// need room for it.
constexpr uint16_t kMaxCommandPrefixBytes = static_cast<uint16_t>(maxVerbNameLength() + 1);  // name + ':'

// kMaxLineBytes -- Comms's own inbound/outbound scratch LINE buffer size:
// the longest possible `<COMMAND>':'<COBS+CRC bytes>` content a single
// Transport::readLine()/Transport::send() call ever needs to hold (the
// transport's own trailing `\n` is one further byte, appended by the
// transport itself, not counted here -- matches kFramedMaxBytes's own
// convention of excluding the delimiter). Replaces 123-002's
// kArmoredBufSize now that a line is prefix+frame, not frame alone.
constexpr uint16_t kMaxLineBytes = kFramedMaxBytes + kMaxCommandPrefixBytes;

enum class CmdStatus : uint8_t { kNone = 0, kDecoded = 1 };

struct Cmd {
  CmdStatus status = CmdStatus::kNone;
  msg::CommandEnvelope env;
};

// kCmdRingDepth -- how many decoded commands Comms buffers between
// ingestion and dispatch (command-ingestion-ring-buffered-comms-subsystem-
// routing-two-stops.md §1). Before this ring, ingestion was rate-limited
// by the control loop: pump() produced at most ONE Cmd per cycle (~21/s at
// the measured 47 ms period) for a single dispatch site, so a burst was
// silently lost in the transports' own buffers -- a SINGLE completed-
// message slot on radio (com/radio.h), a linear accumulator on serial.
//
// Depth 12 is sized against the ack ring it feeds: RobotLoop drains the
// whole ring in ONE cycle, pushing one ack per command into
// App::kAckRingDepth, so a burst larger than the ack ring would execute
// but be unobservable. The two depths are therefore kept EQUAL
// (telemetry.h's kAckRingDepth carries the matching note) -- that
// constraint picks 12, it is not a free choice.
constexpr uint8_t kCmdRingDepth = 6;

// kPumpMaxLines -- hard bound on how many wire lines ONE pump() call
// consumes. pump() runs inside the loop's existing settle/clear/pace
// windows, so it must return in bounded time no matter how hard a host
// floods the link; without this, a transport that always has another
// complete line ready would hold the cycle forever. Two full ring loads:
// large enough never to be the binding limit for a legitimate burst (the
// ring fills first, and the excess is counted as a drop), while still
// capping the worst case at a fixed number of decodes.
constexpr uint8_t kPumpMaxLines = 2 * kCmdRingDepth;

class Comms {
 public:
  // banner/idLine must outlive the Comms instance (caller-owned, e.g.
  // main.cpp's own static buffers) -- Comms does not format or own either
  // string itself, matching its own boundary ("outside: device state" --
  // see this file's header comment). `idLine` is `ID:<fields>`'s full
  // reply content (sprint 124 architecture Decision 4: configured-robot
  // identity -- drivetrain type + calibration-profile name/version --
  // distinct from `banner`'s hardware identity); defaults to a
  // grammar-conformant placeholder so a caller that genuinely has no
  // configured-identity string to report (most host-test fixtures) still
  // gets a well-formed `ID:` reply rather than none at all. `VER:`'s
  // content needs no constructor parameter -- see sendVer(), comms.cpp --
  // it reads the existing generated build-version constant directly
  // (architecture Decision 4: zero new version-tracking infrastructure).
  Comms(Transport& serialLink, Transport& radioLink, const char* banner, const char* idLine = "ID:unknown");

  // Drain BOTH transports into the command ring
  // (command-ingestion-...-two-stops.md §1). Loops -- serial first, radio
  // when serial has nothing -- until neither transport has another complete
  // line or kPumpMaxLines have been consumed. REPLACES the pre-ring
  // "at most one Transport::readLine() to serialLink_, and only if serial
  // had nothing, at most one to radioLink_" contract: that bound made the
  // control loop the ingestion rate limiter, which is exactly the defect
  // this ring exists to remove.
  //
  // Reading is deliberately NOT gated on ring space. Leaving a completed
  // line sitting in a transport only moves the loss somewhere that cannot
  // report it (the radio's completed-message handoff is a single slot --
  // com/radio.h -- and silently discards the next arrival). A line that
  // decodes correctly but finds the ring full is dropped HERE and counted
  // in commandsDroppedCount().
  //
  // Cleartext verbs (HELLO/PING/ID/VER) are answered inline as before and
  // never enter the ring; malformed lines still bump malformedCount().
  //
  // RobotLoop::cycle() calls this from inside EACH of its existing
  // runAndWait bodies, so the transports are freed several times per
  // cycle; dispatch happens once per cycle via takeCommand() below.
  //
  // now -- [ms], the caller's own current clock reading (RobotLoop::cycle()
  // passes its already-computed cycleStart) -- formats the PING reply's
  // `t=<ms>` field (117, SUC-056). Comms holds no Devices::Clock&/timing
  // collaborator of its own; every value it ever stamps onto a reply is
  // handed in, not read from an owned clock.
  void pump(uint32_t now);  // [ms]

  // Pop the OLDEST buffered command, FIFO. Returns false (leaving `out`
  // untouched) when the ring is empty -- the loop's own drain idiom is
  // `while (comms_.takeCommand(cmd)) routeCommand(cmd);`. `out.status` is
  // always kDecoded on a true return: only successfully decoded envelopes
  // ever enter the ring.
  bool takeCommand(Cmd& out);

  // Observability (tests): how many commands are buffered right now.
  uint8_t pendingCommandCount() const { return cmdCount_; }

  // Encode (msg::wire::encode) + CRC-then-COBS frame (see comms.cpp) +
  // send ONCE on BOTH transports via Transport::send() (async/drop-on-full
  // -- telemetry is always-on and must never stall the loop on
  // backpressure; primary and secondary frames go out on both transports
  // every cadence, not just "back to whoever last spoke"). This is what
  // Telemetry calls. No return value: encode()==0 or a COBS/CRC framing
  // failure means silently send nothing.
  //
  // 124-005: the outbound ASCII command name (the CRC's scope-extension
  // per protocol v5's `crc = crc16(COMMAND ':' payload)` composition,
  // issue §3, AND the wire line's own leading `<COMMAND>':'` prefix) is
  // derived INTERNALLY from `reply.body_kind` (TLM/OK/ERR map 1:1 onto
  // messages/commands.h's Verb::TLM/OK/ERR) -- a caller never passes one:
  // `body_kind` already says which verb this is, and passing a second,
  // independently-spelled name string would just be a second place the
  // two could drift apart.
  void sendReply(const msg::ReplyEnvelope& reply);

  // Push the banner unprompted on both transports, cleartext plane.
  // Emitted twice per boot -- main.cpp at power-on, RobotLoop::boot() when
  // the preamble finishes -- byte-identical both times, so a banner parser
  // needs no change. `DEVICE:NEZHA2:robot:<name>:<serial>` already
  // conforms to the v5 grammar unmodified (command `DEVICE`, cleartext
  // data on the first ':') -- formatBanner() needs no edit (issue §8).
  void sendBanner();

  // Diagnostic counter -- malformed COBS frame, CRC mismatch, malformed
  // protobuf decode, an unrecognized `<COMMAND>` (not in messages/
  // commands.h's kVerbTable[]), AND a cleartext command with no inbound
  // handler (e.g. a stray `DEVICE`/`PONG` sent host->robot) all increment
  // this. RobotLoop reads it as the App::kFaultCommsMalformed telemetry
  // fault-bit source.
  //
  // 124-010 (relay-handshake-trips-comms-malformed.md): a line whose
  // first byte is '#'/'!'/'?' -- the radio-relay dongle's own
  // control-plane sigils -- is dropped by dispatchLine() BEFORE reaching
  // this counter (see isRelayControlPlaneLine(), comms.cpp). This is a
  // narrow, symmetry-restoring exception, not a broadening of tolerance:
  // no registered verb name starts with any of those three bytes, so it
  // never hides a genuine malformed command.
  uint32_t malformedCount() const { return malformedCount_; }

  // Diagnostic counter -- a command that decoded cleanly but arrived at a
  // FULL command ring, and was therefore dropped without ever being routed
  // (command-ingestion-...-two-stops.md §1). Distinct from
  // malformedCount() in kind, not just in count: a malformed line is a
  // wire/link problem, a dropped command is a firmware backpressure
  // problem -- the host sent faster than one cycle's drain could route.
  // RobotLoop publishes it in health telemetry beside commsMalformedCount
  // (kFlagFaultCommandsDropped).
  uint32_t commandsDroppedCount() const { return commandsDroppedCount_; }

 private:
  // true if a line was consumed (decoded, malformed, or cleartext); false
  // when the transport had nothing complete ready. pump() uses the return
  // value to decide whether to keep draining, NOT (as before the ring) to
  // stop after the first transport that had anything. now -- [ms], threaded
  // straight from pump()'s own argument -- see pump()'s doc comment above.
  bool pumpTransport(Transport& t, uint32_t now);  // [ms]

  // Append one decoded command to the ring, or -- when the ring is full --
  // drop it and bump commandsDroppedCount_. Drops the NEWEST, not the
  // oldest (the opposite of Telemetry's ack ring, deliberately): an ack
  // ring is an observability record where the freshest entries matter
  // most, but a command ring is a work queue, and evicting the oldest
  // entry would execute a burst OUT OF ORDER -- discarding a command the
  // robot has effectively already accepted while running one that arrived
  // after it. Refusing the newest is honest backpressure.
  void pushCommand(const Cmd& cmd);

  // Parses `<COMMAND>[':' <data>]` out of one already-`\n`-delimited wire
  // line (124-005, issue §1) and dispatches by the registry's `binary`
  // flag -- the SOLE discriminator; nothing about `data`'s own bytes is
  // ever inspected. Unrecognized `<COMMAND>` -> malformedCount_++, EXCEPT
  // a leaked relay control-plane line ('#'/'!'/'?' first byte, 124-010) --
  // dropped uncounted, before the registry lookup even runs.
  void dispatchLine(Transport& t, const char* line, uint16_t lineLen, Cmd& out, uint32_t now);  // [ms]

  // Cleartext command dispatch (HELLO/PING/ID/VER, the only inbound
  // cleartext verbs -- issue §8). Any other cleartext verb arriving
  // inbound (e.g. a stray DEVICE/PONG) -> malformedCount_++, matching a
  // binary verb with no valid frame.
  void dispatchCleartext(msg::Verb verb, Transport& t, uint32_t now);  // [ms]

  // NEVER replies -- acks ride Telemetry's ack ring, not per-command; see
  // comms.cpp for the discipline note. `data`/`dataLen` is the still-COBS
  // (kCobsDelimiter)+CRC-encoded frame body -- the wire line's content
  // AFTER the `<COMMAND>':'` prefix dispatchLine() already parsed off.
  //
  // command/commandLen -- the ASCII command-name bytes dispatchLine()
  // parsed BEFORE this binary frame body was reached -- the CRC's scope
  // extension, mirroring sendReply()'s own composition in the opposite
  // direction (comms.cpp's crcOverScope()).
  void decodeBinaryFrame(const uint8_t* command, size_t commandLen, const uint8_t* data, uint16_t dataLen, Cmd& out);

  Transport& serialLink_;
  Transport& radioLink_;
  const char* banner_;
  const char* idLine_;
  uint32_t malformedCount_ = 0;
  uint32_t commandsDroppedCount_ = 0;

  // The command ring -- same plain circular-buffer idiom as Telemetry's
  // ack ring (telemetry.h): cmdHead_ indexes the OLDEST valid entry,
  // cmdCount_ (0..kCmdRingDepth) is how many slots hold a real command.
  // Only the eviction policy differs -- see pushCommand() above.
  Cmd cmdRing_[kCmdRingDepth]{};
  uint8_t cmdHead_ = 0;
  uint8_t cmdCount_ = 0;
};

}  // namespace App
