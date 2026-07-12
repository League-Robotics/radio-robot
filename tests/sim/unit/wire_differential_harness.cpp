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
//   encode_helptext <corr_id> <text>
//     Stakeholder-directed 6-verb minimal command surface (2026-07-10):
//     builds ReplyEnvelope{helptext=HelpText{text}} -- HELP's reply. `text`
//     must be a whitespace-free token (argv splitting), same constraint
//     encode_id's model/name/fw_version args already have.
//     -> "B64 <base64 bytes>" or "ZERO". `decode`'s CmdKind switch also
//     covers the new hello/ver/help REQUEST arms (zero-field, same
//     "nothing arm-specific to print" bucket as stop/ping/id) -- no new
//     decode verb needed, only new CmdKind enumerators in the existing one.
//
//   -- 096-006 additions below: Telemetry/ConfigSnapshot are reply-only
//      messages (never decoded by firmware -- neither appears in
//      CommandEnvelope.cmd), so their differential coverage is encode-only
//      (firmware-encode -> host-decode); ConfigDelta is command-only
//      (never appears in ReplyEnvelope.body), so its coverage is
//      decode-only (host-encode -> firmware-decode) and needs no new
//      encode_* verb -- `decode`'s CONFIG case above now prints its
//      fields the same way DRIVE/SEGMENT/ECHO already do.
//
//   encode_telemetry <corr_id> <now> <mode> <seq> <has_enc> <enc_left>
//     <enc_right> <has_vel> <vel_left> <vel_right> <has_cmd_vel>
//     <cmd_vel_left> <cmd_vel_right> <has_pose> <pose_x> <pose_y> <pose_h>
//     <has_otos> <otos_x> <otos_y> <otos_h> <otos_connected> <has_twist>
//     <twist_vx> <twist_vy> <twist_omega> <acc_left> <acc_right> <active>
//     <conn_left> <conn_right> <glitch_left> <glitch_right> <ts_left>
//     <ts_right>
//     Builds ReplyEnvelope{tlm=Telemetry{...}}, every field positional in
//     telemetry.proto's own field-number order -- Telemetry is a flat
//     28-field message with no oneof arm of its own, so (unlike encode_ok/
//     encode_id) there is no "shape" choice, just one long positional
//     list. bool-typed args are "0"/"1"; `mode` is DriveMode's numeric
//     value (0=IDLE..5=VELOCITY, planner.proto).
//     -> "B64 <base64 bytes>" or "ZERO".
//
//   encode_cfg_drivetrain <corr_id> <target> <trackwidth> <rotational_slip>
//     <ekf_q_xy> <ekf_q_theta> <ekf_r_otos_xy> <ekf_r_otos_theta>
//   encode_cfg_motor <corr_id> <target> <side> <travel_calib> <kp> <ki>
//     <kff> <i_max> <kaw>
//   encode_cfg_planner <corr_id> <target> <min_speed>
//   encode_cfg_watchdog <corr_id> <target> <watchdog>
//     Builds ReplyEnvelope{cfg=ConfigSnapshot{target, patch=<arm>}} -- one
//     verb per `patch` oneof arm, each populating EVERY field of its own
//     Patch with {has=true, val} (mirrors BinaryChannel's `get` handler,
//     which always populates every field of the selected slice; a
//     ConfigSnapshot reply never carries an absent/has=false field in
//     practice). `target` is ConfigTarget's numeric value
//     (0=CONFIG_DRIVETRAIN..4=CONFIG_WATCHDOG, config.proto); `side` is
//     BoundMotorSide's numeric value (0=LEFT, 1=RIGHT).
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

// fromHex -- inverse of toHex() above (095-007's encode_echo_reply verb):
// decodes a lowercase-hex argv token (Python's bytes.hex()) into raw bytes,
// clamped to `cap`. Malformed/odd-length input yields 0 bytes (this is a
// test-harness convenience, not a wire-facing decoder -- the differential
// suite's own Python side always emits well-formed hex).
size_t fromHex(const std::string& hex, uint8_t* out, size_t cap) {
  auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  size_t n = 0;
  for (size_t i = 0; i + 1 < hex.size(); i += 2) {
    int hi = nibble(hex[i]);
    int lo = nibble(hex[i + 1]);
    if (hi < 0 || lo < 0 || n >= cap) break;
    out[n++] = static_cast<uint8_t>((hi << 4) | lo);
  }
  return n;
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
    // hello/ver/help (stakeholder-directed 6-verb minimal command surface,
    // 2026-07-10).
    case msg::CommandEnvelope::CmdKind::HELLO: return "HELLO";
    case msg::CommandEnvelope::CmdKind::VER: return "VER";
    case msg::CommandEnvelope::CmdKind::HELP: return "HELP";
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

// configDeltaPatchKindName -- 096-006: prints ConfigDelta's own oneof
// discriminant the same way cmdKindName()/controlKindName() print the
// other generated oneofs above.
const char* configDeltaPatchKindName(msg::ConfigDelta::PatchKind k) {
  switch (k) {
    case msg::ConfigDelta::PatchKind::NONE: return "NONE";
    case msg::ConfigDelta::PatchKind::DRIVETRAIN: return "DRIVETRAIN";
    case msg::ConfigDelta::PatchKind::MOTOR: return "MOTOR";
    case msg::ConfigDelta::PatchKind::PLANNER: return "PLANNER";
    case msg::ConfigDelta::PatchKind::WATCHDOG: return "WATCHDOG";
  }
  return "UNKNOWN";
}

// printOpt -- 096-006: one `<name>_has=<0|1> <name>=<val>` pair for an
// `Opt<float>` field, the SAME shape every WHEELS-arm `w%u_speed_has=...`
// pair above already prints (wheel targets are ALSO Opt<float>) -- reused
// here for DrivetrainConfigPatch/MotorConfigPatch/PlannerConfigPatch's own
// Opt<float> fields rather than hand-duplicating the two-printf pattern
// once per field.
void printOpt(const char* name, const msg::Opt<float>& o) {
  std::printf(" %s_has=%d %s=%s", name, o.has ? 1 : 0, name, fmtFloat(o.val).c_str());
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
    case msg::CommandEnvelope::CmdKind::CONFIG: {
      // 096-006: ConfigDelta is command-only (never appears in
      // ReplyEnvelope.body), so this decode-side print IS its whole
      // differential surface (Direction A, host-encode -> firmware-decode
      // -- see test_wire_differential.py's ConfigDelta section). Prints
      // EVERY field of whichever Patch oneof arm decoded, the same
      // has/val-pair shape the WHEELS arm above already uses for its own
      // Opt<float> fields (printOpt()).
      const msg::ConfigDelta& cfg = out.cmd.config;
      std::printf(" patch_kind=%s", configDeltaPatchKindName(cfg.patch_kind));
      switch (cfg.patch_kind) {
        case msg::ConfigDelta::PatchKind::DRIVETRAIN: {
          const msg::DrivetrainConfigPatch& p = cfg.patch.drivetrain;
          printOpt("trackwidth", p.trackwidth);
          printOpt("rotational_slip", p.rotational_slip);
          printOpt("ekf_q_xy", p.ekf_q_xy);
          printOpt("ekf_q_theta", p.ekf_q_theta);
          printOpt("ekf_r_otos_xy", p.ekf_r_otos_xy);
          printOpt("ekf_r_otos_theta", p.ekf_r_otos_theta);
          break;
        }
        case msg::ConfigDelta::PatchKind::MOTOR: {
          const msg::MotorConfigPatch& p = cfg.patch.motor;
          std::printf(" side=%s", p.side == msg::BoundMotorSide::RIGHT ? "RIGHT" : "LEFT");
          printOpt("travel_calib", p.travel_calib);
          printOpt("kp", p.kp);
          printOpt("ki", p.ki);
          printOpt("kff", p.kff);
          printOpt("i_max", p.i_max);
          printOpt("kaw", p.kaw);
          break;
        }
        case msg::ConfigDelta::PatchKind::PLANNER:
          printOpt("min_speed", cfg.patch.planner.min_speed);
          break;
        case msg::ConfigDelta::PatchKind::WATCHDOG:
          std::printf(" watchdog=%u", static_cast<unsigned>(cfg.patch.watchdog));
          break;
        case msg::ConfigDelta::PatchKind::NONE:
        default:
          break;
      }
      break;
    }
    case msg::CommandEnvelope::CmdKind::STOP:
    case msg::CommandEnvelope::CmdKind::PING:
    case msg::CommandEnvelope::CmdKind::ID:
    case msg::CommandEnvelope::CmdKind::POSE:
    case msg::CommandEnvelope::CmdKind::OTOS:
    case msg::CommandEnvelope::CmdKind::GET:
    case msg::CommandEnvelope::CmdKind::STREAM:
    case msg::CommandEnvelope::CmdKind::HELLO:
    case msg::CommandEnvelope::CmdKind::VER:
    case msg::CommandEnvelope::CmdKind::HELP:
    case msg::CommandEnvelope::CmdKind::NONE:
      // No arm-specific fields to print: zero-field arms (stop/ping/id
      // request, and hello/ver/help -- stakeholder-directed 6-verb minimal
      // command surface, 2026-07-10, same "cannot be malformed" shape) and
      // declared-only arms outside this sprint's implemented scope (pose/
      // otos land 098; get/stream have their OWN sim-level behavioral
      // coverage in test_binary_channel.py, ticket 006's own instruction --
      // they carry no (min)/(max)/(abs_max)-validated fields of their own
      // beyond ConfigGet.target's (req), already exercised by 095's
      // wire_codec_harness.cpp).
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
  // t (095-007, Ack schema-gap closure): optional 5th argv, defaulting to 0
  // -- every pre-existing call (argc==5) keeps building the identical
  // Ack{q,rem,t=0} it always has; the differential suite's new t-coverage
  // cases pass argv[5] explicitly.
  reply.body.ok.t = (argc >= 6) ? static_cast<uint32_t>(std::strtoul(argv[5], nullptr, 10)) : 0;
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

// encode_echo_reply <corr_id> <hex payload> (095-007, ReplyEnvelope schema-
// gap closure -- see envelope.proto's own ReplyEnvelope.echo doc comment).
// Builds ReplyEnvelope{echo=Echo{payload}}; payload arrives hex-encoded
// (Python's bytes.hex()) so arbitrary byte values survive argv.
int cmdEncodeEchoReply(int argc, char** argv) {
  if (argc < 4) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  msg::ReplyEnvelope reply;
  reply.corr_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
  reply.body_kind = msg::ReplyEnvelope::BodyKind::ECHO;
  reply.body.echo.payload_count = static_cast<uint8_t>(
      fromHex(argv[3], reply.body.echo.payload_, sizeof(reply.body.echo.payload_)));
  printEncodedOrZero(reply);
  return 0;
}

// encode_helptext -- stakeholder-directed 6-verb minimal command surface
// (2026-07-10): ReplyEnvelope{helptext=HelpText{text}} -- HELP's reply,
// same "one leaf string field" shape encode_id's model/name/fw_version
// args already exercise, just a single positional token this time.
int cmdEncodeHelpText(int argc, char** argv) {
  if (argc < 4) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  msg::ReplyEnvelope reply;
  reply.corr_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
  reply.body_kind = msg::ReplyEnvelope::BodyKind::HELPTEXT;
  std::strncpy(reply.body.helptext.text, argv[3], sizeof(reply.body.helptext.text) - 1);
  printEncodedOrZero(reply);
  return 0;
}

// encode_telemetry -- 096-006: ReplyEnvelope{tlm=Telemetry{...}}, every
// field positional in telemetry.proto's own field-number order (see this
// file's header comment for the full argv list). Telemetry is reply-only
// (never decoded by firmware), so this is its entire differential surface
// (Direction B, firmware-encode -> host-decode).
int cmdEncodeTelemetry(int argc, char** argv) {
  if (argc < 37) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  int i = 2;
  msg::ReplyEnvelope reply;
  reply.corr_id = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  reply.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  msg::Telemetry& t = reply.body.tlm;
  t.now = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.mode = static_cast<msg::DriveMode>(std::strtoul(argv[i++], nullptr, 10));
  t.seq = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.has_enc = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.enc_left = std::strtof(argv[i++], nullptr);
  t.enc_right = std::strtof(argv[i++], nullptr);
  t.has_vel = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.vel_left = std::strtof(argv[i++], nullptr);
  t.vel_right = std::strtof(argv[i++], nullptr);
  t.has_cmd_vel = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.cmd_vel_left = std::strtof(argv[i++], nullptr);
  t.cmd_vel_right = std::strtof(argv[i++], nullptr);
  t.has_pose = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.pose.x = std::strtof(argv[i++], nullptr);
  t.pose.y = std::strtof(argv[i++], nullptr);
  t.pose.h = std::strtof(argv[i++], nullptr);
  t.has_otos = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.otos.x = std::strtof(argv[i++], nullptr);
  t.otos.y = std::strtof(argv[i++], nullptr);
  t.otos.h = std::strtof(argv[i++], nullptr);
  t.otos_connected = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.has_twist = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.twist.v_x = std::strtof(argv[i++], nullptr);
  t.twist.v_y = std::strtof(argv[i++], nullptr);
  t.twist.omega = std::strtof(argv[i++], nullptr);
  t.acc_left = std::strtof(argv[i++], nullptr);
  t.acc_right = std::strtof(argv[i++], nullptr);
  t.active = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.conn_left = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.conn_right = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.glitch_left = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.glitch_right = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.ts_left = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.ts_right = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  printEncodedOrZero(reply);
  return 0;
}

// encode_cfg_drivetrain/motor/planner/watchdog -- 096-006: ReplyEnvelope{
// cfg=ConfigSnapshot{target, patch=<arm>}}, one verb per `patch` oneof arm.
// Every field of the selected Patch is populated {has=true, val} (mirrors
// BinaryChannel's `get` handler -- see this file's header comment).
int cmdEncodeCfgDrivetrain(int argc, char** argv) {
  if (argc < 10) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  msg::ReplyEnvelope reply;
  reply.corr_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
  reply.body_kind = msg::ReplyEnvelope::BodyKind::CFG;
  msg::ConfigSnapshot& cfg = reply.body.cfg;
  cfg.target = static_cast<msg::ConfigTarget>(std::strtoul(argv[3], nullptr, 10));
  cfg.patch_kind = msg::ConfigSnapshot::PatchKind::DRIVETRAIN;
  msg::DrivetrainConfigPatch& p = cfg.patch.drivetrain;
  p.trackwidth = {true, std::strtof(argv[4], nullptr)};
  p.rotational_slip = {true, std::strtof(argv[5], nullptr)};
  p.ekf_q_xy = {true, std::strtof(argv[6], nullptr)};
  p.ekf_q_theta = {true, std::strtof(argv[7], nullptr)};
  p.ekf_r_otos_xy = {true, std::strtof(argv[8], nullptr)};
  p.ekf_r_otos_theta = {true, std::strtof(argv[9], nullptr)};
  printEncodedOrZero(reply);
  return 0;
}

int cmdEncodeCfgMotor(int argc, char** argv) {
  if (argc < 11) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  msg::ReplyEnvelope reply;
  reply.corr_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
  reply.body_kind = msg::ReplyEnvelope::BodyKind::CFG;
  msg::ConfigSnapshot& cfg = reply.body.cfg;
  cfg.target = static_cast<msg::ConfigTarget>(std::strtoul(argv[3], nullptr, 10));
  cfg.patch_kind = msg::ConfigSnapshot::PatchKind::MOTOR;
  msg::MotorConfigPatch& p = cfg.patch.motor;
  p.side = (std::strtoul(argv[4], nullptr, 10) != 0) ? msg::BoundMotorSide::RIGHT : msg::BoundMotorSide::LEFT;
  p.travel_calib = {true, std::strtof(argv[5], nullptr)};
  p.kp = {true, std::strtof(argv[6], nullptr)};
  p.ki = {true, std::strtof(argv[7], nullptr)};
  p.kff = {true, std::strtof(argv[8], nullptr)};
  p.i_max = {true, std::strtof(argv[9], nullptr)};
  p.kaw = {true, std::strtof(argv[10], nullptr)};
  printEncodedOrZero(reply);
  return 0;
}

int cmdEncodeCfgPlanner(int argc, char** argv) {
  if (argc < 5) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  msg::ReplyEnvelope reply;
  reply.corr_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
  reply.body_kind = msg::ReplyEnvelope::BodyKind::CFG;
  msg::ConfigSnapshot& cfg = reply.body.cfg;
  cfg.target = static_cast<msg::ConfigTarget>(std::strtoul(argv[3], nullptr, 10));
  cfg.patch_kind = msg::ConfigSnapshot::PatchKind::PLANNER;
  cfg.patch.planner.min_speed = {true, std::strtof(argv[4], nullptr)};
  printEncodedOrZero(reply);
  return 0;
}

int cmdEncodeCfgWatchdog(int argc, char** argv) {
  if (argc < 5) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  msg::ReplyEnvelope reply;
  reply.corr_id = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
  reply.body_kind = msg::ReplyEnvelope::BodyKind::CFG;
  msg::ConfigSnapshot& cfg = reply.body.cfg;
  cfg.target = static_cast<msg::ConfigTarget>(std::strtoul(argv[3], nullptr, 10));
  cfg.patch_kind = msg::ConfigSnapshot::PatchKind::WATCHDOG;
  cfg.patch.watchdog = static_cast<uint32_t>(std::strtoul(argv[4], nullptr, 10));
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
  if (op == "encode_echo_reply") return cmdEncodeEchoReply(argc, argv);
  if (op == "encode_helptext") return cmdEncodeHelpText(argc, argv);
  if (op == "encode_telemetry") return cmdEncodeTelemetry(argc, argv);
  if (op == "encode_cfg_drivetrain") return cmdEncodeCfgDrivetrain(argc, argv);
  if (op == "encode_cfg_motor") return cmdEncodeCfgMotor(argc, argv);
  if (op == "encode_cfg_planner") return cmdEncodeCfgPlanner(argc, argv);
  if (op == "encode_cfg_watchdog") return cmdEncodeCfgWatchdog(argc, argv);

  std::printf("USAGE_ERROR\n");
  return 1;
}
