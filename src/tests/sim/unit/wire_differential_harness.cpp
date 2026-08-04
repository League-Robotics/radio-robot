// wire_differential_harness.cpp -- **correctness gate the app/ tickets
// (004+) are built on top of.** A break here is a BLOCKING regression,
// never an xfail/skip.
//
// Off-hardware CLI harness for the differential/fuzz/range suite against
// `google.protobuf` (the host's `pb2/` bindings). Rewritten 115-009 (gut
// S1's own test-sweep/green-bar ticket) against the frame-v2 schema
// (115-003, `telemetry-frame-tightening-amendment-to-gut-s1.md`);
// CommandEnvelope.cmd is now exactly {move, config, stop} (116-001, MOVE
// protocol cutover -- `twist`, arm 19, is deleted/reserved, superseded by
// `move`, a fresh arm 21: `MoveTwist|MoveWheels` velocity oneof +
// `time|distance|angle` stop oneof + `timeout`/`replace`/`id`);
// ReplyEnvelope.body is exactly {ok, err, tlm}; Telemetry carries a single
// `flags` bit-string (status+fault+event) and a bounded, PACKED ack ring
// (`acks`/`acks_count` -- the single `ack_corr`/`ack_err` slot is deleted,
// 124-008) plus per-source timestamped `EncoderReading`/`OtosReading`
// objects; ConfigDelta's `patch` oneof is DRIVETRAIN/MOTOR/OTOS (PLANNER
// deleted wholesale, 115-003; WATCHDOG deleted, 116-001 --
// `ConfigTarget.CONFIG_WATCHDOG` stays declared-unused); TelemetrySecondary
// is DELETED outright (124-009, robot-state-blackboard-...md) -- there is
// no longer a second top-level wire message to drive through this harness.
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
//   encode_telemetry <corr_id> <now> <mode> <seq> <flags>
//     <enc_left_position> <enc_left_velocity> <enc_left_age>
//     <enc_left_position_epoch> <enc_right_position> <enc_right_velocity>
//     <enc_right_age> <enc_right_position_epoch> <otos_x> <otos_y>
//     <otos_heading> <otos_v_x> <otos_v_y> <otos_omega> <otos_age>
//     <pose_x> <pose_y> <pose_h> <twist_v_x> <twist_v_y> <twist_omega>
//     <line> <color> <cycle_busy> <cycle_period> <acks_count> <acks_0>
//     <acks_1> <acks_2> <acks_3>
//     Builds ReplyEnvelope{tlm=Telemetry{...}} per the packed-telemetry
//     shape (telemetry.proto, 124-008, issue §B3/B4/B6): one `flags`
//     bit-string, two EncoderReadings, one OtosReading, always-present
//     pose/twist (ALL sint32/zigzag raw wire ints now, not float --
//     position/velocity/x/y/heading/v_x/v_y/omega/h), the packed
//     line/color words, and (123-004, ADDITIVE) the migrated
//     cycle_busy/cycle_period loop-timing fields. The single "freshest
//     ack" slot (ack_corr/ack_err, fields 5/6) is DELETED -- ring
//     membership already means "really acked." The trailing 5 args are
//     the bounded ack ring: `acks_count` (0..App::kAckRingDepth=4) then
//     exactly kAckRingDepth PACKED uint32 slots (`corr_id<<4|err`, not a
//     (corr_id, err) pair) -- slots at or past `acks_count` are still
//     parsed (keeps this verb's own argv shape fixed) but never copied
//     into `t.acks_`/`t.acks_count`.
//     -> "B64 <base64 bytes>" or "ZERO".
//
//   encode_telemetry_secondary -- DELETED (124-009): TelemetrySecondary
//     itself is gone.
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
    case msg::ErrCode::ERR_NOT_CONFIGURED: return "ERR_NOT_CONFIGURED";
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
  if (s == "ERR_NOT_CONFIGURED") return msg::ErrCode::ERR_NOT_CONFIGURED;
  return msg::ErrCode::ERR_NONE;
}

const char* cmdKindName(msg::CommandEnvelope::CmdKind k) {
  switch (k) {
    case msg::CommandEnvelope::CmdKind::NONE: return "NONE";
    case msg::CommandEnvelope::CmdKind::CONFIG: return "CONFIG";
    case msg::CommandEnvelope::CmdKind::STOP: return "STOP";
    case msg::CommandEnvelope::CmdKind::MOVE: return "MOVE";
  }
  return "UNKNOWN";
}

// configGroupTargetName -- prints SetConfigGroup's own `target` enum the
// same way cmdKindName() prints CommandEnvelope's discriminant. 132-013
// (patch-surface retirement): replaces the deleted ConfigDelta's own
// `PatchKind` oneof discriminant -- CommandEnvelope.config now carries
// SetConfigGroup{target, body}, not ConfigDelta.
const char* configGroupTargetName(msg::ConfigGroupTarget t) {
  switch (t) {
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED: return "CONFIG_GROUP_UNSPECIFIED";
    case msg::ConfigGroupTarget::GEOMETRY: return "GEOMETRY";
    case msg::ConfigGroupTarget::MOTORS: return "MOTORS";
    case msg::ConfigGroupTarget::DRIVE: return "DRIVE";
    case msg::ConfigGroupTarget::WHEEL_CONTROL: return "WHEEL_CONTROL";
    case msg::ConfigGroupTarget::PLANNER: return "PLANNER";
    case msg::ConfigGroupTarget::OTOS: return "OTOS";
    case msg::ConfigGroupTarget::ESTIMATOR: return "ESTIMATOR";
  }
  return "UNKNOWN";
}

// moveVelocityKindName/moveStopKindName -- print Move's own two oneof
// discriminants (116-001) the same way cmdKindName()/configDeltaPatchKindName()
// print theirs.
const char* moveVelocityKindName(msg::Move::VelocityKind k) {
  switch (k) {
    case msg::Move::VelocityKind::NONE: return "NONE";
    case msg::Move::VelocityKind::TWIST: return "TWIST";
    case msg::Move::VelocityKind::WHEELS: return "WHEELS";
  }
  return "UNKNOWN";
}

const char* moveStopKindName(msg::Move::StopKind k) {
  switch (k) {
    case msg::Move::StopKind::NONE: return "NONE";
    case msg::Move::StopKind::TIME: return "TIME";
    case msg::Move::StopKind::DISTANCE: return "DISTANCE";
    case msg::Move::StopKind::ANGLE: return "ANGLE";
  }
  return "UNKNOWN";
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
    case msg::CommandEnvelope::CmdKind::MOVE: {
      const msg::Move& mv = out.cmd.move;
      std::printf(" velocity_kind=%s", moveVelocityKindName(mv.velocity_kind));
      switch (mv.velocity_kind) {
        case msg::Move::VelocityKind::TWIST:
          std::printf(" v_x=%s v_y=%s omega=%s", fmtFloat(mv.velocity.twist.v_x).c_str(),
                      fmtFloat(mv.velocity.twist.v_y).c_str(), fmtFloat(mv.velocity.twist.omega).c_str());
          break;
        case msg::Move::VelocityKind::WHEELS:
          std::printf(" v_left=%s v_right=%s", fmtFloat(mv.velocity.wheels.v_left).c_str(),
                      fmtFloat(mv.velocity.wheels.v_right).c_str());
          break;
        case msg::Move::VelocityKind::NONE:
        default:
          break;
      }
      std::printf(" stop_kind=%s", moveStopKindName(mv.stop_kind));
      switch (mv.stop_kind) {
        case msg::Move::StopKind::TIME:
          std::printf(" time=%s", fmtFloat(mv.stop.time).c_str());
          break;
        case msg::Move::StopKind::DISTANCE:
          std::printf(" distance=%s", fmtFloat(mv.stop.distance).c_str());
          break;
        case msg::Move::StopKind::ANGLE:
          std::printf(" angle=%s", fmtFloat(mv.stop.angle).c_str());
          break;
        case msg::Move::StopKind::NONE:
        default:
          break;
      }
      std::printf(" timeout=%s replace=%d id=%u", fmtFloat(mv.timeout).c_str(), mv.replace ? 1 : 0,
                  static_cast<unsigned>(mv.id));
      break;
    }
    case msg::CommandEnvelope::CmdKind::CONFIG: {
      // 132-013 (patch-surface retirement): config now carries
      // SetConfigGroup{target, body}, not the deleted ConfigDelta. `body`
      // is an opaque encoded group (Configurator::applyGroup()'s own job
      // to interpret, not this generic differential harness's) -- print
      // the target and body length only, mirroring
      // wire_codec_harness.cpp's own scenarioRoundTripConfigSetGroup().
      const msg::SetConfigGroup& cfg = out.cmd.config;
      std::printf(" target=%s body_count=%u", configGroupTargetName(cfg.target),
                  static_cast<unsigned>(cfg.body_count));
      break;
    }
    case msg::CommandEnvelope::CmdKind::STOP:
    case msg::CommandEnvelope::CmdKind::NONE:
    case msg::CommandEnvelope::CmdKind::WHEELS:
    case msg::CommandEnvelope::CmdKind::ESTOP:
    case msg::CommandEnvelope::CmdKind::GET_CONFIG:
    case msg::CommandEnvelope::CmdKind::SET_FIELD:
      // No arm-specific fields printed by this harness for these arms --
      // WHEELS/ESTOP/GET_CONFIG/SET_FIELD (132-011/132-012) were never
      // wired into this differential harness's decode printer; only
      // move/config/stop are (pre-existing gap, not introduced by
      // 132-013 -- listed explicitly here so a future arm addition
      // without a matching case fails to compile, -Wswitch/-Werror,
      // rather than silently falling through).
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

// encode_telemetry -- ReplyEnvelope{tlm=Telemetry{...}}: every field
// positional in telemetry.proto's OWN field-number order (frame v2,
// 115-003; the ack ring's 9 trailing args added by 120 -- see this file's
// header comment for the full argv list).
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
  t.flags = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));

  // 124-008 (issue §B3): position/velocity/x/y/heading/v_x/v_y/omega/h are
  // now sint32 (raw wire ints, zigzag-encoded) -- strtol (signed), not
  // strtof. ack_corr/ack_err (fields 5/6) are DELETED (issue §B4).
  t.enc_left.position = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.enc_left.velocity = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.enc_left.age = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.enc_left.position_epoch = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.enc_right.position = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.enc_right.velocity = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.enc_right.age = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.enc_right.position_epoch = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));

  t.otos.x = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.otos.y = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.otos.heading = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.otos.v_x = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.otos.v_y = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.otos.omega = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.otos.age = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));

  t.pose.x = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.pose.y = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.pose.h = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));

  t.twist.v_x = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.twist.v_y = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));
  t.twist.omega = static_cast<int32_t>(std::strtol(argv[i++], nullptr, 10));

  t.line = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.color = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));

  // cycle_busy/cycle_period (123-004, ADDITIVE -- migrated from
  // TelemetrySecondary's fields 11/12, 122-003).
  t.cycle_busy = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  t.cycle_period = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));

  // Bounded ack ring (120, ADDITIVE -- telemetry.proto's Telemetry.acks doc
  // comment). kRingDepth mirrors App::kAckRingDepth (app/telemetry.h) --
  // duplicated as a local literal rather than an #include of app/
  // telemetry.h, which would pull in app/comms.h's Transport interfaces
  // this standalone wire-only harness (wire.cpp + wire_runtime.cpp, no
  // app/ linkage) has no other reason to need. 124-008 (issue §B4): each
  // slot is now ONE packed uint32 (corr_id<<4|err), not a (corr_id, err)
  // pair -- always exactly kRingDepth slots on argv (unused slots past
  // acksCount are still parsed, keeping this verb's own argv shape fixed,
  // but never copied into t.acks_/t.acks_count).
  constexpr uint8_t kRingDepth = 4;
  const uint32_t acksCount = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
  for (uint8_t e = 0; e < kRingDepth; ++e) {
    const uint32_t packed = static_cast<uint32_t>(std::strtoul(argv[i++], nullptr, 10));
    if (e < acksCount) {
      t.acks_[e] = packed;
    }
  }
  t.acks_count = static_cast<uint8_t>(acksCount < kRingDepth ? acksCount : kRingDepth);

  printEncodedOrZero(reply);
  return 0;
}

// cmdEncodeTelemetrySecondary() -- DELETED (124-009): TelemetrySecondary
// itself is gone (robot-state-blackboard-...md, issue's own
// "TelemetrySecondary dies").

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

  std::printf("USAGE_ERROR\n");
  return 1;
}
