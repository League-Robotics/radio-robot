// wire_differential_harness.cpp -- **correctness gate ticket 007
// (BinaryChannel) is built on top of.** A break here is a BLOCKING
// regression, never an xfail/skip (095-006, SUC-005, architecture-update.md
// M8 "Codec Test Harness").
//
// Off-hardware CLI harness for ticket 095-006's differential/fuzz/range
// suite against `google.protobuf` (the host's `pb2/` bindings, ticket 002).
// Unlike wire_runtime_harness.cpp/wire_codec_harness.cpp (095-004/005, fixed
// scenario lists baked into the C++ binary itself), THIS harness is a thin
// one-shot CLI driven by a Python test via `subprocess` per the ticket's own
// instruction ("a Python driver can drive via subprocess") -- Python owns
// generating differential/fuzz/boundary test VALUES (via `pb2`), the harness
// only exposes msg::wire::decode()/encode() through a small argv protocol so
// Python can compare the firmware codec's behavior against google.protobuf's
// byte-for-byte, in both directions, for an arbitrarily large generated
// corpus (fuzz >= 200 cases) without hand-writing a C++ scenario per case.
//
// argv[1] selects the operation; each invocation prints exactly ONE line to
// stdout and exits 0 UNLESS the process itself crashes (ASan/UBSan abort on
// a real memory-safety violation, or an unhandled signal) -- that asymmetry
// is deliberate: a clean `Result{ok=false,...}` is a NORMAL, successful
// decode() outcome for malformed input and must exit 0 with an "ERR" line,
// while a nonzero exit / stderr sanitizer report is what the fuzz suite
// treats as an actual finding.
//
//   decode <base64 CommandEnvelope bytes>
//     -> "OK corr_id=<u32> cmd_kind=<NAME> <arm-specific key=value pairs>"
//     -> "ERR field=<u16> code=<ErrCode NAME>"
//     (a base64 string that fails to decode at the base64 layer itself --
//     should never happen since the Python driver always emits valid
//     padded base64 via stdlib base64.b64encode, see wire_runtime.h's
//     pinned-alphabet note -- is reported the same way decode() itself
//     would report malformed bytes: "ERR field=0 code=ERR_DECODE".)
//
//   encode_ok <corr_id> <q> <rem>
//     Builds ReplyEnvelope{ok=Ack{q,rem}}, calls msg::wire::encode().
//     -> "B64 <base64 bytes>" or "ZERO" (encode() returned 0).
//
//   encode_err <corr_id> <ErrCode NAME> <field>
//     Builds ReplyEnvelope{err=Error{code,field}}.
//     -> "B64 <base64 bytes>" or "ZERO".
//
//   encode_id <corr_id> <model> <name> <serial> <fw_version> <proto_version>
//     Builds ReplyEnvelope{id=DeviceId{...}}. model/name/fw_version must be
//     whitespace-free tokens (argv splitting) -- the differential suite only
//     needs to prove scalar/string ENCODING correctness, not exercise every
//     printable character; DeviceId's string-handling edge cases (empty,
//     max-length, embedded NUL via length-delimited framing) are the
//     wire_codec_harness.cpp (095-005) scenario's job, not this ticket's.
//     -> "B64 <base64 bytes>" or "ZERO".
//
// Float formatting: `%.9g` on both the encode-input parse (strtof) and the
// decode-output print -- 9 significant decimal digits is the proven
// sufficient precision to round-trip any IEEE-754 binary32 value through a
// decimal string with no loss, so the Python side's `float(token)` recovers
// the EXACT float32 value (once canonicalized through the same
// struct.pack/unpack('<f', ...) round-trip on the Python side) with no
// tolerance-based fuzz needed for equality checks.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "messages/envelope.h"
#include "messages/wire.h"
#include "messages/wire_runtime.h"

namespace {

using WireRuntime::WireType;

// --- Small formatting/parsing helpers --------------------------------------

std::string fmtFloat(float v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(v));
  return buf;
}

std::string toHex(const uint8_t* data, size_t len) {
  static const char kHexDigits[] = "0123456789abcdef";
  std::string out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; ++i) {
    out.push_back(kHexDigits[(data[i] >> 4) & 0xF]);
    out.push_back(kHexDigits[data[i] & 0xF]);
  }
  return out;
}

const char* errCodeName(msg::ErrCode c) {
  switch (c) {
    case msg::ErrCode::ERR_NONE: return "ERR_NONE";
    case msg::ErrCode::ERR_UNKNOWN: return "ERR_UNKNOWN";
    case msg::ErrCode::ERR_BADARG: return "ERR_BADARG";
    case msg::ErrCode::ERR_RANGE: return "ERR_RANGE";
    case msg::ErrCode::ERR_FULL: return "ERR_FULL";
    case msg::ErrCode::ERR_DECODE: return "ERR_DECODE";
    case msg::ErrCode::ERR_UNIMPLEMENTED: return "ERR_UNIMPLEMENTED";
    case msg::ErrCode::ERR_OVERSIZE: return "ERR_OVERSIZE";
  }
  return "ERR_UNKNOWN_ENUM_VALUE";
}

msg::ErrCode parseErrCode(const std::string& s) {
  if (s == "ERR_NONE") return msg::ErrCode::ERR_NONE;
  if (s == "ERR_UNKNOWN") return msg::ErrCode::ERR_UNKNOWN;
  if (s == "ERR_BADARG") return msg::ErrCode::ERR_BADARG;
  if (s == "ERR_RANGE") return msg::ErrCode::ERR_RANGE;
  if (s == "ERR_FULL") return msg::ErrCode::ERR_FULL;
  if (s == "ERR_DECODE") return msg::ErrCode::ERR_DECODE;
  if (s == "ERR_UNIMPLEMENTED") return msg::ErrCode::ERR_UNIMPLEMENTED;
  if (s == "ERR_OVERSIZE") return msg::ErrCode::ERR_OVERSIZE;
  return msg::ErrCode::ERR_NONE;
}

const char* cmdKindName(msg::CommandEnvelope::CmdKind k) {
  switch (k) {
    case msg::CommandEnvelope::CmdKind::NONE: return "NONE";
    case msg::CommandEnvelope::CmdKind::DRIVE: return "DRIVE";
    case msg::CommandEnvelope::CmdKind::SEGMENT: return "SEGMENT";
    case msg::CommandEnvelope::CmdKind::REPLACE: return "REPLACE";
    case msg::CommandEnvelope::CmdKind::CONFIG: return "CONFIG";
    case msg::CommandEnvelope::CmdKind::POSE: return "POSE";
    case msg::CommandEnvelope::CmdKind::OTOS: return "OTOS";
    case msg::CommandEnvelope::CmdKind::PING: return "PING";
    case msg::CommandEnvelope::CmdKind::ECHO: return "ECHO";
    case msg::CommandEnvelope::CmdKind::GET: return "GET";
    case msg::CommandEnvelope::CmdKind::STREAM: return "STREAM";
    case msg::CommandEnvelope::CmdKind::STOP: return "STOP";
    case msg::CommandEnvelope::CmdKind::ID: return "ID";
  }
  return "UNKNOWN";
}

const char* controlKindName(msg::DrivetrainCommand::ControlKind k) {
  switch (k) {
    case msg::DrivetrainCommand::ControlKind::NONE: return "NONE";
    case msg::DrivetrainCommand::ControlKind::TWIST: return "TWIST";
    case msg::DrivetrainCommand::ControlKind::WHEELS: return "WHEELS";
    case msg::DrivetrainCommand::ControlKind::NEUTRAL: return "NEUTRAL";
    case msg::DrivetrainCommand::ControlKind::POSE: return "POSE";
  }
  return "UNKNOWN";
}

void printMotionSegment(const msg::MotionSegment& seg) {
  std::printf(
      " distance=%s direction=%s final_heading=%s speed_max=%s accel_max=%s jerk_max=%s"
      " yaw_rate_max=%s yaw_accel_max=%s yaw_jerk_max=%s time=%s v=%s omega=%s stream=%d",
      fmtFloat(seg.distance).c_str(), fmtFloat(seg.direction).c_str(), fmtFloat(seg.final_heading).c_str(),
      fmtFloat(seg.speed_max).c_str(), fmtFloat(seg.accel_max).c_str(), fmtFloat(seg.jerk_max).c_str(),
      fmtFloat(seg.yaw_rate_max).c_str(), fmtFloat(seg.yaw_accel_max).c_str(), fmtFloat(seg.yaw_jerk_max).c_str(),
      fmtFloat(seg.time).c_str(), fmtFloat(seg.v).c_str(), fmtFloat(seg.omega).c_str(), seg.stream ? 1 : 0);
}

// --- decode -----------------------------------------------------------------

int cmdDecode(const std::string& b64) {
  uint8_t raw[600] = {};
  size_t rawLen = 0;
  if (!WireRuntime::base64Decode(b64.c_str(), b64.size(), raw, sizeof(raw), &rawLen)) {
    // Base64-layer failure (should not occur -- the Python driver always
    // emits valid padded standard base64; see the file header note). Report
    // the same shape decode() itself uses for malformed bytes so the Python
    // side has one uniform "ERR" line format to parse.
    std::printf("ERR field=0 code=ERR_DECODE\n");
    return 0;
  }

  msg::CommandEnvelope out;
  const msg::wire::Result r = msg::wire::decode(out, raw, static_cast<uint16_t>(rawLen));
  if (!r.ok) {
    std::printf("ERR field=%u code=%s\n", static_cast<unsigned>(r.field), errCodeName(r.code));
    return 0;
  }

  std::printf("OK corr_id=%u cmd_kind=%s", static_cast<unsigned>(out.corr_id), cmdKindName(out.cmd_kind));
  switch (out.cmd_kind) {
    case msg::CommandEnvelope::CmdKind::DRIVE: {
      const msg::DrivetrainCommand& d = out.cmd.drive;
      std::printf(" control_kind=%s", controlKindName(d.control_kind));
      switch (d.control_kind) {
        case msg::DrivetrainCommand::ControlKind::TWIST:
          std::printf(" v_x=%s v_y=%s omega=%s", fmtFloat(d.control.twist.v_x).c_str(),
                      fmtFloat(d.control.twist.v_y).c_str(), fmtFloat(d.control.twist.omega).c_str());
          break;
        case msg::DrivetrainCommand::ControlKind::WHEELS: {
          std::printf(" w_count=%u", static_cast<unsigned>(d.control.wheels.w_count));
          for (uint8_t i = 0; i < d.control.wheels.w_count; ++i) {
            const msg::WheelTarget& w = d.control.wheels.w_[i];
            std::printf(" w%u_speed_has=%d w%u_speed=%s w%u_position_has=%d w%u_position=%s", i,
                        w.speed.has ? 1 : 0, i, fmtFloat(w.speed.val).c_str(), i, w.position.has ? 1 : 0, i,
                        fmtFloat(w.position.val).c_str());
          }
          break;
        }
        case msg::DrivetrainCommand::ControlKind::NEUTRAL:
          std::printf(" neutral=%s", d.control.neutral == msg::Neutral::BRAKE ? "BRAKE" : "COAST");
          break;
        case msg::DrivetrainCommand::ControlKind::POSE:
          std::printf(" pose_x=%s pose_y=%s pose_h=%s", fmtFloat(d.control.pose.x).c_str(),
                      fmtFloat(d.control.pose.y).c_str(), fmtFloat(d.control.pose.h).c_str());
          break;
        case msg::DrivetrainCommand::ControlKind::NONE:
          break;
      }
      std::printf(" seed_has=%d seed=%d standby_has=%d standby=%d", d.seed.has ? 1 : 0, d.seed.val ? 1 : 0,
                  d.standby.has ? 1 : 0, d.standby.val ? 1 : 0);
      break;
    }
    case msg::CommandEnvelope::CmdKind::SEGMENT:
      printMotionSegment(out.cmd.segment);
      break;
    case msg::CommandEnvelope::CmdKind::REPLACE:
      printMotionSegment(out.cmd.replace);
      break;
    case msg::CommandEnvelope::CmdKind::ECHO:
      std::printf(" payload_count=%u payload_hex=%s", static_cast<unsigned>(out.cmd.echo.payload_count),
                  toHex(out.cmd.echo.payload_, out.cmd.echo.payload_count).c_str());
      break;
    case msg::CommandEnvelope::CmdKind::STOP:
    case msg::CommandEnvelope::CmdKind::PING:
    case msg::CommandEnvelope::CmdKind::ID:
    case msg::CommandEnvelope::CmdKind::CONFIG:
    case msg::CommandEnvelope::CmdKind::POSE:
    case msg::CommandEnvelope::CmdKind::OTOS:
    case msg::CommandEnvelope::CmdKind::GET:
    case msg::CommandEnvelope::CmdKind::STREAM:
    case msg::CommandEnvelope::CmdKind::NONE:
      // No arm-specific fields to print: zero-field arms (stop/ping/id
      // request) and declared-only arms outside this ticket's differential
      // scope (config/pose/otos/get/stream -- ticket 006 scopes coverage to
      // the sprint's IMPLEMENTED arms; decode() itself still handles these
      // generically, exercised by ticket 005's own harness).
      break;
  }
  std::printf("\n");
  return 0;
}

// --- base64-encode-and-print helper for the encode_* commands --------------

void printEncodedOrZero(const msg::ReplyEnvelope& reply) {
  uint8_t buf[256] = {};
  const uint16_t n = msg::wire::encode(reply, buf, sizeof(buf));
  if (n == 0) {
    std::printf("ZERO\n");
    return;
  }
  char b64[400] = {};
  size_t b64Len = 0;
  if (!WireRuntime::base64Encode(buf, n, b64, sizeof(b64) - 1, &b64Len)) {
    std::printf("ZERO\n");
    return;
  }
  b64[b64Len] = '\0';
  std::printf("B64 %s\n", b64);
}

int cmdEncodeOk(int argc, char** argv) {
  if (argc < 5) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  msg::ReplyEnvelope reply;
  reply.corr_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
  reply.body_kind = msg::ReplyEnvelope::BodyKind::OK;
  reply.body.ok.q = static_cast<uint32_t>(std::strtoul(argv[3], nullptr, 10));
  reply.body.ok.rem = std::strtof(argv[4], nullptr);
  printEncodedOrZero(reply);
  return 0;
}

int cmdEncodeErr(int argc, char** argv) {
  if (argc < 5) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  msg::ReplyEnvelope reply;
  reply.corr_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
  reply.body_kind = msg::ReplyEnvelope::BodyKind::ERR;
  reply.body.err.code = parseErrCode(argv[3]);
  reply.body.err.field = static_cast<uint32_t>(std::strtoul(argv[4], nullptr, 10));
  printEncodedOrZero(reply);
  return 0;
}

int cmdEncodeId(int argc, char** argv) {
  if (argc < 8) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  msg::ReplyEnvelope reply;
  reply.corr_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
  reply.body_kind = msg::ReplyEnvelope::BodyKind::ID;
  std::strncpy(reply.body.id.model, argv[3], sizeof(reply.body.id.model) - 1);
  std::strncpy(reply.body.id.name, argv[4], sizeof(reply.body.id.name) - 1);
  reply.body.id.serial = static_cast<uint32_t>(std::strtoul(argv[5], nullptr, 10));
  std::strncpy(reply.body.id.fw_version, argv[6], sizeof(reply.body.id.fw_version) - 1);
  reply.body.id.proto_version = static_cast<uint32_t>(std::strtoul(argv[7], nullptr, 10));
  printEncodedOrZero(reply);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  const std::string op = argv[1];
  if (op == "decode") {
    if (argc < 3) {
      std::printf("USAGE_ERROR\n");
      return 1;
    }
    return cmdDecode(argv[2]);
  }
  if (op == "encode_ok") return cmdEncodeOk(argc, argv);
  if (op == "encode_err") return cmdEncodeErr(argc, argv);
  if (op == "encode_id") return cmdEncodeId(argc, argv);

  std::printf("USAGE_ERROR\n");
  return 1;
}
