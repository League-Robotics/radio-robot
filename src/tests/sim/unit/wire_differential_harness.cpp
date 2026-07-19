// wire_differential_harness.cpp -- **correctness gate the app/ tickets
// (004+) are built on top of.** A break here is a BLOCKING regression,
// never an xfail/skip.
//
// Off-hardware CLI harness for the differential/fuzz/range suite against
// `google.protobuf` (the host's `pb2/` bindings). Rewritten this ticket
// (103-001, SUC-001, architecture-update.md (103) Decisions 2/3) against
// the P4-pruned schema: CommandEnvelope.cmd is exactly {twist, config,
// stop}; ReplyEnvelope.body is exactly {ok, err, tlm}; Telemetry carries
// the depth-3 ack ring + fault_bits/event_bits; TelemetrySecondary is a
// NEW standalone top-level wire message (its own `msg::wire::encode()`
// overload, not a ReplyEnvelope oneof arm).
//
// Unlike wire_runtime_harness.cpp/wire_codec_harness.cpp (fixed scenario
// lists baked into the C++ binary itself), THIS harness is a thin one-shot
// CLI driven by a Python test via `subprocess` -- Python owns generating
// differential/fuzz/boundary test VALUES (via `pb2`), the harness only
// exposes msg::wire::decode()/encode() through a small argv protocol so
// Python can compare the firmware codec's behavior against
// google.protobuf's byte-for-byte, in both directions, for an arbitrarily
// large generated corpus without hand-writing a C++ scenario per case.
//
// argv[1] selects the operation; each invocation prints exactly ONE line to
// stdout and exits 0 UNLESS the process itself crashes (ASan/UBSan abort on
// a real memory-safety violation, or an unhandled signal) -- a clean
// `Result{ok=false,...}` is a NORMAL, successful decode() outcome for
// malformed input and must exit 0 with an "ERR" line; a nonzero exit /
// stderr sanitizer report is what the fuzz suite treats as an actual
// finding.
//
//   decode <base64 CommandEnvelope bytes>
//     -> "OK corr_id=<u32> cmd_kind=<NAME> <arm-specific key=value pairs>"
//     -> "ERR field=<u16> code=<ErrCode NAME>"
//
//   encode_ok <corr_id> <q> <rem> <t>
//     Builds ReplyEnvelope{ok=Ack{q,rem,t}}.
//     -> "B64 <base64 bytes>" or "ZERO" (encode() returned 0).
//
//   encode_err <corr_id> <ErrCode NAME> <field>
//     Builds ReplyEnvelope{err=Error{code,field}}.
//     -> "B64 <base64 bytes>" or "ZERO".
//
//   encode_telemetry <corr_id>
//     <ack0_corr_id> <ack0_status> <ack0_err_code>
//     <ack1_corr_id> <ack1_status> <ack1_err_code>
//     <ack2_corr_id> <ack2_status> <ack2_err_code>
//     <now> <mode> <seq> <has_enc> <enc_left> <enc_right> <has_vel>
//     <vel_left> <vel_right> <has_pose> <pose_x> <pose_y> <pose_h>
//     <has_otos> <otos_x> <otos_y> <otos_h> <otos_connected> <has_twist>
//     <twist_vx> <twist_vy> <twist_omega> <active> <conn_left> <conn_right>
//     <fault_bits> <event_bits>
//     Builds ReplyEnvelope{tlm=Telemetry{...}} with the ack ring ALWAYS
//     populated at its full depth-3 (mirrors app/Telemetry's own designed
//     invariant, ticket 005 -- the wire codec's encode() trusts the
//     caller-supplied count with no re-clamp, so this harness never
//     exercises a malformed count > 3).
//     -> "B64 <base64 bytes>" or "ZERO".
//
//   encode_telemetry_secondary <now> <has_cmd_vel> <cmd_vel_left>
//     <cmd_vel_right> <acc_left> <acc_right> <glitch_left> <glitch_right>
//     <ts_left> <ts_right>
//     Builds a STANDALONE TelemetrySecondary (Decision 3 -- its own
//     independently-armored line, not wrapped in ReplyEnvelope, so no
//     corr_id argument).
//     -> "B64 <base64 bytes>" or "ZERO".
//
// Float formatting: `%.9g` on both the encode-input parse (strtof) and the
// decode-output print -- 9 significant decimal digits is the proven
// sufficient precision to round-trip any IEEE-754 binary32 value through a
// decimal string with no loss.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "messages/envelope.h"
#include "messages/wire.h"
#include "messages/wire_runtime.h"

namespace {

// --- Small formatting helpers ----------------------------------------------

std::string fmtFloat(float v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(v));
  return buf;
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
    case msg::CommandEnvelope::CmdKind::CONFIG: return "CONFIG";
    case msg::CommandEnvelope::CmdKind::STOP: return "STOP";
    case msg::CommandEnvelope::CmdKind::TWIST: return "TWIST";
  }
  return "UNKNOWN";
}

// configDeltaPatchKindName -- prints ConfigDelta's own oneof discriminant
// the same way cmdKindName() prints CommandEnvelope's.
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

// printOpt -- one `<name>_has=<0|1> <name>=<val>` pair for an `Opt<float>`
// field (DrivetrainConfigPatch/MotorConfigPatch/PlannerConfigPatch's own
// Opt<float> fields).
void printOpt(const char* name, const msg::Opt<float>& o) {
  std::printf(" %s_has=%d %s=%s", name, o.has ? 1 : 0, name, fmtFloat(o.val).c_str());
}

// --- decode -----------------------------------------------------------------

int cmdDecode(const std::string& b64) {
  uint8_t raw[600] = {};
  size_t rawLen = 0;
  if (!WireRuntime::base64Decode(b64.c_str(), b64.size(), raw, sizeof(raw), &rawLen)) {
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
    case msg::CommandEnvelope::CmdKind::TWIST:
      std::printf(" v_x=%s omega=%s duration=%s", fmtFloat(out.cmd.twist.v_x).c_str(),
                  fmtFloat(out.cmd.twist.omega).c_str(), fmtFloat(out.cmd.twist.duration).c_str());
      break;
    case msg::CommandEnvelope::CmdKind::CONFIG: {
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
          printOpt("ekf_r_fix_xy", p.ekf_r_fix_xy);
          printOpt("ekf_r_fix_theta", p.ekf_r_fix_theta);
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
          printOpt("heading_kp", cfg.patch.planner.heading_kp);
          printOpt("heading_kd", cfg.patch.planner.heading_kd);
          // arrive_dwell (111-004): the one field kept from the 16-field
          // Drive::Limits/tracker/policy span removed as dead this ticket --
          // see config.proto's own PlannerConfigPatch header comment.
          printOpt("arrive_dwell", cfg.patch.planner.arrive_dwell);
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
    case msg::CommandEnvelope::CmdKind::NONE:
      // No arm-specific fields to print: zero-field arm (stop, "cannot be
      // malformed" shape).
      break;
  }
  std::printf("\n");
  return 0;
}

// --- base64-encode-and-print helper for the encode_* commands --------------

void printEncodedOrZero(const uint8_t* buf, uint16_t n) {
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

void printEncodedOrZero(const msg::ReplyEnvelope& reply) {
  uint8_t buf[256] = {};
  const uint16_t n = msg::wire::encode(reply, buf, sizeof(buf));
  printEncodedOrZero(buf, n);
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

// encode_telemetry -- ReplyEnvelope{tlm=Telemetry{...}}: 3 ack-ring entries
// (9 positional args) followed by every other field positional in
// telemetry.proto's own field-number order (see this file's header comment
// for the full argv list).
int cmdEncodeTelemetry(int argc, char** argv) {
  if (argc < 32) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  int i = 2;
  msg::ReplyEnvelope reply;
  reply.corr_id = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  reply.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  msg::Telemetry& t = reply.body.tlm;

  t.acks_count = 3;
  for (int e = 0; e < 3; ++e) {
    t.acks_[e].corr_id = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
    t.acks_[e].status = static_cast<msg::AckStatus>(std::strtoul(argv[i++], nullptr, 10));
    t.acks_[e].err_code = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  }

  t.now = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.mode = static_cast<msg::DriveMode>(std::strtoul(argv[i++], nullptr, 10));
  t.seq = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.has_enc = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.enc_left = std::strtof(argv[i++], nullptr);
  t.enc_right = std::strtof(argv[i++], nullptr);
  t.has_vel = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.vel_left = std::strtof(argv[i++], nullptr);
  t.vel_right = std::strtof(argv[i++], nullptr);
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
  t.active = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.conn_left = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.conn_right = std::strtoul(argv[i++], nullptr, 10) != 0;
  t.fault_bits = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.event_bits = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));

  printEncodedOrZero(reply);
  return 0;
}

// encode_telemetry_secondary -- STANDALONE TelemetrySecondary (Decision 3 --
// its own independently-armored line, no ReplyEnvelope wrapper, no corr_id).
int cmdEncodeTelemetrySecondary(int argc, char** argv) {
  if (argc < 12) {
    std::printf("USAGE_ERROR\n");
    return 1;
  }
  int i = 2;
  msg::TelemetrySecondary sec;
  sec.now = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  sec.has_cmd_vel = std::strtoul(argv[i++], nullptr, 10) != 0;
  sec.cmd_vel_left = std::strtof(argv[i++], nullptr);
  sec.cmd_vel_right = std::strtof(argv[i++], nullptr);
  sec.acc_left = std::strtof(argv[i++], nullptr);
  sec.acc_right = std::strtof(argv[i++], nullptr);
  sec.glitch_left = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  sec.glitch_right = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  sec.ts_left = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  sec.ts_right = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));

  uint8_t buf[256] = {};
  const uint16_t n = msg::wire::encode(sec, buf, sizeof(buf));
  printEncodedOrZero(buf, n);
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
  if (op == "encode_telemetry") return cmdEncodeTelemetry(argc, argv);
  if (op == "encode_telemetry_secondary") return cmdEncodeTelemetrySecondary(argc, argv);

  std::printf("USAGE_ERROR\n");
  return 1;
}
