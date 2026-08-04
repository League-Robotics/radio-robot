// comms.h -- App::Comms: parses/emits protocol v5's uniform packet grammar
// (`<COMMAND>[':' <data>]'\n'`) and, for binary commands, the COBS+CRC
// binary-frame armor/dearmor layer between the two transports (serial +
// radio) and decoded msg::CommandEnvelope / msg::ReplyEnvelope.
//
// ONE uniform grammar in both directions: every line, text or binary, is
// `<COMMAND>[':' <data>]'\n'`. The transport delivers a bare,
// `\n`-terminated line (COBS is keyed on 0x0A -- see wire_runtime.h item 8
// -- so a binary frame's own bytes never contain a literal 0x0A, making
// the terminator genuinely unconditional); Comms parses the `<COMMAND>`
// prefix, looks it up in the generated command registry
// (messages/commands.h's kVerbTable[]), and THAT lookup's `binary` flag --
// not anything about the data's own bytes -- is the sole decision of how
// `<data>` is read. See wire_runtime.h's own file header for the COBS/CRC
// primitives this file is built on.
//
// Boundary: inside -- the grammar parse/emit, CRC-over-command-and-payload
// composition, dispatch of text vs. binary by registry lookup, the
// reply-plane grammar (DEVICE/PONG/ID/VER); outside -- deciding what a
// decoded command DOES (that is RobotLoop's own dispatch), physical
// transport framing (Com::SerialPort/Com::Radio's own job). Design/
// rationale: DESIGN.md.
#pragma once

#include <cstddef>
#include "app/debug.h"
#include <cstdint>

#include "firm/types/robot_state.h"
#include "messages/commands.h"
#include "messages/envelope.h"
#include "messages/wire.h"

#ifndef HOST_BUILD
class SerialPort;
class Radio;
#endif

namespace App {

// Forward-declared, not #included: telemetry.h already #includes this
// header (its own file header's "Send path" section -- Telemetry holds a
// Comms&), so the reverse edge (comms.h -> telemetry.h) would be a genuine
// include cycle. Comms::updateStatus() below needs only a `const
// Telemetry&` reference type, which a forward declaration supplies; the
// definition (comms.cpp) includes telemetry.h itself, where there is no
// cycle -- see updateStatus()'s own doc comment for why Comms takes this
// as a parameter rather than holding a Telemetry& member.
class Telemetry;

// kCobsDelimiter -- the wire delimiter: COBS is keyed on 0x0A ('\n'), not
// 0x00. Every WireRuntime::cobsEncode()/cobsDecode() call in this file
// passes this explicitly rather than relying on that function's own 0x00
// default (which other callers, e.g. wire_differential_harness, still use).
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
  // classified as cleartext. Never partially delivers a line.
  virtual bool readLine(char* buf, uint16_t cap, uint16_t* outLen) = 0;

  // Async, drop-on-full send -- `data`/`len` is the COMPLETE wire line
  // content this call must emit, MINUS the trailing `\n` terminator (the
  // concrete transport appends that itself -- one `\n` terminator for
  // both text and binary payloads). For a binary reply
  // this is `<COMMAND>':'<COBS+CRC bytes>`; Comms::sendReply() (a
  // high-cadence caller) uses this so a full serial buffer never stalls
  // the loop.
  virtual void send(const uint8_t* data, uint16_t len) = 0;

  // Bounded-wait, must-not-drop send -- ALWAYS a cleartext reply line
  // (NUL-terminated ASCII, e.g. `banner_`/`"PONG:t=<ms>"`); used for the
  // HELLO/PING/ID/VER text-exception replies (rare, one-off). The
  // concrete transport appends a single trailing `\n` (NOT `\r\n`).
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
// budgets (msg::wire::kCommandEnvelopeMaxEncodedSize (234, as of 132-013 --
// see that constant's own generated size-report comment, wire.h) /
// kReplyEnvelopeMaxEncodedSize (232)) -- one raw-byte scratch buffer,
// reused sequentially for an incoming decode or an outgoing encode (never
// overlapping within a single call). Computed by the constexpr expression
// itself so a future schema regeneration that changes either constant
// updates this one automatically.
//
// 132-013 (patch-surface retirement): kCommandEnvelopeMaxEncodedSize jumped
// from 55 to 234 -- not a regression, the expected cost of finally WIRING
// SetConfigGroup (robot_config.proto's own ~220 B `body`, allocated by
// ticket 001, unwired until now) into CommandEnvelope.cmd.config in place
// of the deleted ConfigDelta (whose curated per-target live-tuning
// messages never exceeded ~50 B). This is now the worst-case
// CommandEnvelope arm, ahead
// of ReplyEnvelope's own cfg=228 B -- both directions now push a
// comparable worst case, as expected once a whole-group SET exists
// alongside whole-group GET.
constexpr uint16_t kMaxEnvelopeBytes =
    (msg::wire::kCommandEnvelopeMaxEncodedSize > msg::wire::kReplyEnvelopeMaxEncodedSize)
        ? msg::wire::kCommandEnvelopeMaxEncodedSize
        : msg::wire::kReplyEnvelopeMaxEncodedSize;  // == 234

// kMaxCrcPayloadBytes -- kMaxEnvelopeBytes + 2 (the CRC-16 appended AFTER
// the schema payload, per the CRC-then-COBS composition -- see comms.cpp's
// sendReply()/decodeBinaryFrame() for the exact byte layout). This is the
// buffer the COBS encode/decode step itself operates on. The command
// prefix lives OUTSIDE the COBS region (see crcOverScope()'s own doc
// comment, comms.cpp), so it does not affect this constant.
constexpr uint16_t kMaxCrcPayloadBytes = kMaxEnvelopeBytes + 2;  // == 236

// kFramedMaxBytes -- worst-case COBS-encoded length of kMaxCrcPayloadBytes
// (236, up from 234 as of 132-013 -- CommandEnvelope's own `config` arm,
// now carrying SetConfigGroup's ~220 B `body` in place of the deleted
// ConfigDelta, is the new overall worst case; see kMaxEnvelopeBytes's own
// doc comment above) zero-free bytes: cobsEncodedMaxLength(236) =
// 236 + 236/254 + 1 = 237 (WireRuntime::cobsEncodedMaxLength()'s own
// documented formula). This is the size of the COBS-encoded region
// ALONE, with only 1B of headroom left as of 132-013 (down from 3B pre-
// 132-013, itself already down from 197 computed -> 200 chosen pre-
// 132-011), because the ASCII command prefix is not part of this region
// (see kMaxLineBytes below for the buffer that DOES need room for the
// prefix). The static_assert below is this constant's own safety net
// against exactly this kind of drift -- it fired once already (132-011,
// 200 -> 238) and will fire again the moment either generated envelope
// constant grows past what 238 can cover, rather than silently
// overflowing a buffer. FLAGGED for whichever ticket next grows either
// envelope's worst-case oneof arm: there is now only 1 byte of slack
// before this constant (and, cascading, kMaxLineBytes/kTxBufferCapacity's
// own already-flagged 1-byte margin below) must be revisited.
constexpr uint16_t kFramedMaxBytes = 238;
static_assert(kFramedMaxBytes >= kMaxCrcPayloadBytes + kMaxCrcPayloadBytes / 254 + 1,
              "kFramedMaxBytes must cover cobsEncodedMaxLength(kMaxCrcPayloadBytes)");

// maxVerbNameLength() -- compile-time max over messages/commands.h's
// generated kVerbTable[] name lengths. A constexpr function (not a
// hand-picked literal) so a future verb longer than "CONFIG"/"DEVICE"
// (6 bytes) is picked up automatically the next time commands.proto is
// regenerated -- re-derive, never hand-edit, size constants like this one.
constexpr size_t maxVerbNameLength() {
  size_t maxLen = 0;
  for (uint8_t i = 0; i < msg::kVerbCount; ++i) {
    size_t len = 0;
    for (const char* p = msg::kVerbTable[i].name; *p != '\0'; ++p) ++len;
    if (len > maxLen) maxLen = len;
  }
  return maxLen;
}

// kMaxCommandPrefixBytes -- the longest possible `<COMMAND>':'` prefix:
// the prefix itself doesn't grow kFramedMaxBytes/kMaxCrcPayloadBytes (it
// is outside the COBS region -- see those constants' own comments above),
// but the WHOLE-LINE buffer below does need room for it.
constexpr uint16_t kMaxCommandPrefixBytes = static_cast<uint16_t>(maxVerbNameLength() + 1);  // name + ':'

// kMaxLineBytes -- Comms's own inbound/outbound scratch LINE buffer size:
// the longest possible `<COMMAND>':'<COBS+CRC bytes>` content a single
// Transport::readLine()/Transport::send() call ever needs to hold (the
// transport's own trailing `\n` is one further byte, appended by the
// transport itself, not counted here -- matches kFramedMaxBytes's own
// convention of excluding the delimiter).
//
// FLAGGED, 132-011: this constant is now 249 -- one byte below
// `Com::SerialPort::kTxBufferCapacity` (250, serial_port.h), the CODAL TX
// ring buffer's own conservative ceiling (physical max 254). Pre-132-011
// this margin was 43 bytes (kMaxLineBytes was 207); the `GET_CONFIG`/`CFG`
// verb pair -- the longest registered name (10B, driving
// kMaxCommandPrefixBytes up from 7 to 11) PLUS ConfigSnapshot's own
// ~220 B `body` capacity (driving kFramedMaxBytes up from 200 to 238) --
// consumed nearly all of it at once. STILL 249 as of 132-013 (patch-
// surface retirement, which wired CommandEnvelope's own `config` arm onto
// SetConfigGroup's matching ~220 B `body`) -- kMaxLineBytes itself did not
// grow (kFramedMaxBytes's hand-picked 238 already covered the new
// worst case, see that constant's own doc comment), but its OWN
// static_assert margin shrank from 3 bytes to 1 -- the next ticket that
// grows either envelope's worst-case oneof arm even slightly will need to
// revisit both constants together, not just kFramedMaxBytes's own
// static_assert. This is not a buffer-overflow risk
// (`cobsOut`/`line` in comms.cpp are sized to these constants correctly),
// but it IS a real, now-concrete headroom risk one layer down:
// `SerialPort::send()`'s own backpressure check
// (`kTxBufferCapacity - txBufferedSize() < frameLen`) means a worst-case
// CFG reply competing with ANY other still-draining serial traffic (e.g. a
// telemetry frame not yet fully flushed) can be silently DROPPED under
// `Comms::sendReply()`'s async, drop-on-full send() policy, not merely
// delayed. Not fixed here -- shrinking ConfigSnapshot.body's `(max_count)`
// below the shared kEncodeScratchCap ceiling, or raising
// kTxBufferCapacity, are both decisions outside this ticket's own
// GetConfig/ConfigSnapshot wiring scope (robot_config.proto's own header
// comment already reasons deliberately about the 220 choice) -- flagged
// for whichever ticket next touches either constant.
constexpr uint16_t kMaxLineBytes = kFramedMaxBytes + kMaxCommandPrefixBytes;

enum class CmdStatus : uint8_t { kNone = 0, kDecoded = 1 };

struct Cmd {
  CmdStatus status = CmdStatus::kNone;
  msg::CommandEnvelope env;
};

// kCmdRingDepth -- how many decoded commands Comms buffers between
// ingestion and dispatch. Without this ring, ingestion is rate-limited by
// the control loop: pump() produces at most ONE Cmd per cycle for a
// single dispatch site, so a burst is silently lost in the transports'
// own buffers -- a SINGLE completed-message slot on radio (com/radio.h),
// a linear accumulator on serial.
//
// Depth 12 is sized against the ack ring it feeds: RobotLoop drains the
// whole ring in ONE cycle, pushing one ack per command into
// App::kAckRingDepth, so a burst larger than the ack ring would execute
// but be unobservable. The two depths are therefore kept EQUAL
// (telemetry.h's kAckRingDepth carries the matching note) -- that
// constraint picks 12, it is not a free choice.
//
// ERRATUM -- CHANGING THIS CONSTANT REQUIRES A CLEAN BUILD
// (`just build-clean`). An incremental build that changes only this header
// can silently produce firmware that HARD FAULTS at boot with symptoms
// indistinguishable from memory corruption (garbage vtable dispatch,
// truncated banner): if the translation unit holding cmdRing_ doesn't
// recompile, its compiler-emitted constructor keeps memset-ing the OLD
// depth's byte count over an object the linker sized for the NEW depth,
// zeroing whatever statics happen to sit next to it. If this constant (or
// telemetry.h's matching kAckRingDepth) ever changes, rebuild clean and
// verify before trusting the result.
constexpr uint8_t kCmdRingDepth = 12;

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
  // Status -- what STATUS reports. Deliberately a plain aggregate of
  // already-known booleans: this is a REPORT, not a second source of truth,
  // so every field is copied from the loop's own state rather than derived
  // here. Add fields freely; the wire format is key=value and the v5 grammar
  // ends the verb at the first colon, so new keys need no parser change.
  struct Status {
    bool ready = false;             // boot() finished; Moves are accepted
    bool active = false;            // a Move is running
    bool wheelLeftConnected = false;
    bool wheelRightConnected = false;
    bool otosPresent = false;
    bool wedged = false;            // encoder stuck-position latch
    uint32_t flags = 0;             // the full telemetry flags word

    // tlmMode -- the current App::TlmMode, as its own raw enum value
    // (0=kOff, 1=kAuto, 2=kOn -- telemetry.h). A raw uint8_t, not an
    // App::TlmMode member, because comms.h cannot include telemetry.h
    // (telemetry.h already includes THIS header -- see its own file
    // header): Comms holds no Telemetry& and never parses/derives the
    // mode itself, it only reports whatever RobotLoop copies in here each
    // cycle, exactly like `flags` above.
    // Defaults to 1 (kAuto), matching Telemetry::mode_'s own default.
    uint8_t tlmMode = 1;
  };

  // Refresh the snapshot STATUS answers from. Called once per cycle by
  // RobotLoop; cheap enough to be unconditional.
  void setStatus(const Status& status) { status_ = status; }

  // updateStatus -- the STATUS projection, the same shape of problem
  // Telemetry::update() already solves for its own wire frame. Reads
  // `state` (the loop's own per-cycle blackboard) for every field except
  // the two Telemetry owns (`flags`/`tlmMode`), which are read straight
  // off `tlm` -- an explicit parameter, not a stored Telemetry& member
  // (Comms holds no Telemetry& anywhere else either, matching this file's
  // own boundary note: "outside -- deciding what a decoded command
  // DOES"). This is a deliberate dependency-direction choice: the
  // alternative (publish flags/tlmMode onto Types::RobotState during
  // Telemetry::update() so this method reads one source) is rejected
  // because RobotState is a dependency-free, cross-tree-shared struct
  // (robot_state.h's own file header) and flags/tlmMode are
  // wire-projection artifacts, not per-cycle robot dynamics -- adding them
  // there would blur that boundary for a one-caller convenience.
  // `state.health.ready` is the one genuinely loop-owned fact
  // (RobotLoop::boot()'s own "past this point the loop dispatches
  // commands" bit, published onto the blackboard instead of hard-coded
  // `true` at this projection site -- see robot_state.h's own doc comment
  // on Health::ready).
  //
  // Call exactly once per cycle, UNCONDITIONALLY -- even when the idle gate
  // suppressed this cycle's telemetry frame, since answering STATUS on a
  // parked robot is precisely the case STATUS exists for -- and AFTER
  // Telemetry::applyAction() has applied any same-cycle mode change
  // (`tlm.mode()` is read live here), but BEFORE sendTlmReply(), whose
  // STATUS/HELP reply must report the mode just applied this cycle, not
  // last cycle's. Both ordering constraints are load-bearing; see
  // RobotLoop::cycle()'s own comment at the call site.
  void updateStatus(const Types::RobotState& state, const Telemetry& tlm);

  // TlmAction -- the parsed effect of one `TLM`/`TLM:...` line. Comms
  // parses the argument (case-insensitively) and stages exactly this;
  // Telemetry itself never parses wire text (this file's own boundary
  // note) -- it is RobotLoop::cycle() that turns kSetOff/kSetAuto/kSetOn
  // into a Telemetry::setMode() call and kFrame into a forced emit(). Bare
  // `TLM` and `TLM:NOW` both surface as kFrame, an explicit alias, not two
  // separate code paths.
  enum class TlmAction : uint8_t { kNone, kFrame, kSetOff, kSetAuto, kSetOn, kUnrecognized };

  // takeTlmAction -- consume-on-read: a burst of `TLM:` lines arriving
  // within one cycle collapses to the LAST one's action (RobotLoop drains
  // this once per cycle), never queued or served twice.
  TlmAction takeTlmAction() {
    const TlmAction action = tlmAction_;
    tlmAction_ = TlmAction::kNone;
    return action;
  }

  // DbgAction -- the parsed effect of one inbound `DBG:<subcmd> ...` line
  // (system-test fault injection; ROBOT_DEBUG builds only -- without it,
  // inbound DBG falls through to the malformed count like any other
  // unhandled cleartext verb). Comms parses and STAGES exactly this;
  // RobotLoop::cycle() drains takeDbgAction() and applies it -- the same
  // stage/consume split TlmAction uses, for the same reason (Comms holds
  // no reference to the subsystems a fault targets).
  enum class DbgActionKind : uint8_t { kNone, kMark, kPing, kWedge, kClear,
                                       kUnrecognized };
  struct DbgAction {
    DbgActionKind kind = DbgActionKind::kNone;
    char text[64] = {};   // kMark: the full original data ("mark leg1a")
    uint8_t port = 0;     // kWedge: 1 = left, 2 = right, 3 = both
    uint32_t duration = 0;  // [ms] kWedge auto-clear; 0 = latched
  };

  // takeDbgAction -- consume-on-read from a small FIFO ring. UNLIKE
  // TlmAction's collapse-to-last: a tour legitimately sends `DBG:mark X`
  // and `DBG:wedge ...` back to back within one cycle, and losing the
  // mark silently corrupts the dataset's ordering record. Ring full =
  // drop-newest (the sender is a paced test script, not a firehose).
  DbgAction takeDbgAction() {
    if (dbgCount_ == 0) return DbgAction{};
    const DbgAction action = dbgRing_[dbgHead_];
    dbgHead_ = (dbgHead_ + 1) % kDbgRingDepth;
    --dbgCount_;
    return action;
  }

  // sendTlmReply -- the reply half of a just-consumed TlmAction: the
  // STATUS line for a recognized mode change (kSetOff/kSetAuto/kSetOn), the
  // HELP line for kUnrecognized (`TLM:<garbage>`), nothing for kNone/kFrame
  // (kFrame's reply IS the forced telemetry frame RobotLoop's own emit()
  // call already sends -- no second reply here). Call this AFTER
  // setStatus() has been refreshed with the mode the action just applied,
  // so a mode-change reply's `tlm=` field always reports the NEW mode, not
  // the one from before this cycle. Replies on the SAME transport the
  // `TLM:` line arrived on (remembered from the parse in dispatchLine()).
  void sendTlmReply(TlmAction action);

  // banner/idLine must outlive the Comms instance (caller-owned, e.g.
  // main.cpp's own static buffers) -- Comms does not format or own either
  // string itself, matching its own boundary ("outside: device state" --
  // see this file's header comment). `idLine` is `ID:<fields>`'s full
  // reply content (configured-robot identity -- drivetrain type +
  // calibration-profile name/version -- distinct from `banner`'s hardware
  // identity); defaults to a grammar-conformant placeholder so a caller
  // that genuinely has no configured-identity string to report (most
  // host-test fixtures) still gets a well-formed `ID:` reply rather than
  // none at all. `VER:`'s content needs no constructor parameter -- see
  // sendVer(), comms.cpp -- it reads the existing generated build-version
  // constant directly.
  Comms(Transport& serialLink, Transport& radioLink, const char* banner, const char* idLine = "ID:unknown");

  // Drain BOTH transports into the command ring. Loops -- serial first,
  // radio when serial has nothing -- until neither transport has another
  // complete line or kPumpMaxLines have been consumed. This deliberately
  // does NOT bound itself to "at most one Transport::readLine() to
  // serialLink_, and only if serial had nothing, at most one to
  // radioLink_" -- that bound would make the control loop the ingestion
  // rate limiter, which is exactly the defect the ring exists to remove.
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
  // `t=<ms>` field. Comms holds no Devices::Clock&/timing collaborator of
  // its own; every value it ever stamps onto a reply is handed in, not
  // read from an owned clock.
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
  // The outbound ASCII command name (the CRC's scope-extension per
  // protocol v5's `crc = crc16(COMMAND ':' payload)` composition, AND the
  // wire line's own leading `<COMMAND>':'` prefix) is derived INTERNALLY
  // from `reply.body_kind` (TLM/OK/ERR map 1:1 onto messages/commands.h's
  // Verb::TLM/OK/ERR) -- a caller never passes one: `body_kind` already
  // says which verb this is, and passing a second, independently-spelled
  // name string would just be a second place the two could drift apart.
  void sendReply(const msg::ReplyEnvelope& reply);

  // Push the banner unprompted on both transports, cleartext plane.
  // Emitted twice per boot -- main.cpp at power-on, RobotLoop::boot() when
  // the preamble finishes -- byte-identical both times, so a banner parser
  // needs no change. `DEVICE:NEZHA2:robot:<name>:<serial>` already
  // conforms to the v5 grammar unmodified (command `DEVICE`, cleartext
  // data on the first ':') -- formatBanner() needs no edit.
  void sendBanner();

  // sendReady -- unsolicited "READY" line, once, when the loop will actually
  // accept commands. PING answers from inside boot() and so cannot signal
  // this; see commands.proto's READY row and comms.cpp's own definition.
  void sendReady();

#if defined(ROBOT_DEBUG) || defined(HOST_BUILD)
  // sendDebug -- emit ONE "DBG:<line>" cleartext line on BOTH transports,
  // the same one-off sendReliable() broadcast sendReady()/sendBanner() use
  // (never the async, drop-on-full send() path -- a debug line is rare and
  // must not race telemetry's own high-cadence traffic for buffer room).
  // Guarded out entirely unless ROBOT_DEBUG (or HOST_BUILD, which implies
  // it) -- app/debug.h's App::debugf() is this method's ONLY caller; see
  // that file's own header for the bench/Sim-only compile contract this
  // exists to support. `line` must already be a short, NUL-terminated,
  // single-line message -- app/debug.cpp's own kDebugMsgMaxBytes bounds
  // what debugf() ever passes here.
  void sendDebug(const char* line);
#endif

 private:
  // STATUS/HELP reply formatters -- see their definitions in comms.cpp.
  void sendStatus(Transport& t);
  void sendHelp(Transport& t);

 public:

  // Diagnostic counter -- malformed COBS frame, CRC mismatch, malformed
  // protobuf decode, an unrecognized `<COMMAND>` (not in messages/
  // commands.h's kVerbTable[]), AND a cleartext command with no inbound
  // handler (e.g. a stray `DEVICE`/`PONG` sent host->robot) all increment
  // this. RobotLoop reads it as the App::kFaultCommsMalformed telemetry
  // fault-bit source.
  //
  // A line whose first byte is '#'/'!'/'?' -- the radio-relay dongle's own
  // control-plane sigils -- is dropped by dispatchLine() BEFORE reaching
  // this counter (see isRelayControlPlaneLine(), comms.cpp). This is a
  // narrow, symmetry-restoring exception, not a broadening of tolerance:
  // no registered verb name starts with any of those three bytes, so it
  // never hides a genuine malformed command.
  uint32_t malformedCount() const { return malformedCount_; }

  // Diagnostic counter -- a command that decoded cleanly but arrived at a
  // FULL command ring, and was therefore dropped without ever being
  // routed. Distinct from malformedCount() in kind, not just in count: a
  // malformed line is a wire/link problem, a dropped command is a firmware
  // backpressure problem -- the host sent faster than one cycle's drain
  // could route.
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
  // line and dispatches by the registry's `binary` flag -- the SOLE
  // discriminator; nothing about `data`'s own bytes is ever inspected.
  // Unrecognized `<COMMAND>` -> malformedCount_++, EXCEPT a leaked relay
  // control-plane line ('#'/'!'/'?' first byte) -- dropped uncounted,
  // before the registry lookup even runs.
  void dispatchLine(Transport& t, const char* line, uint16_t lineLen, Cmd& out, uint32_t now);  // [ms]

  // Cleartext command dispatch (HELLO/PING/ID/VER, the only inbound
  // cleartext verbs). Any other cleartext verb arriving
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

  // Live status snapshot, refreshed by RobotLoop every cycle and formatted
  // on demand by the STATUS handler. A snapshot rather than a back-pointer
  // to the loop: Comms must not reach into subsystems to answer a query
  // (it would invert the dependency and make a cleartext reply depend on
  // whatever the loop happens to be mid-update), and the whole thing is a
  // few bytes to copy.
  //
  // Zero-initialised: a STATUS arriving before the first cycle -- i.e.
  // during boot -- correctly answers ready=0.
  Status status_{};

  // Staged by a `TLM`/`TLM:...` line's parse (dispatchLine()), consumed by
  // takeTlmAction()/sendTlmReply() -- see TlmAction's own doc comment above.
  // tlmReplyTransport_ is which Transport (serialLink_ or radioLink_) the
  // triggering line arrived on; a raw pointer is safe here because both
  // ever point at is one of Comms's own long-lived member references (never
  // dangling for the life of this object).
  TlmAction tlmAction_ = TlmAction::kNone;
  static constexpr uint8_t kDbgRingDepth = 4;
  DbgAction dbgRing_[kDbgRingDepth]{};  // staged by dispatchLine(); drained by RobotLoop
  uint8_t dbgHead_ = 0;
  uint8_t dbgCount_ = 0;
  void pushDbgAction(const DbgAction& action) {
    if (dbgCount_ >= kDbgRingDepth) return;  // drop-newest
    dbgRing_[(dbgHead_ + dbgCount_) % kDbgRingDepth] = action;
    ++dbgCount_;
  }
  Transport* tlmReplyTransport_ = nullptr;
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
