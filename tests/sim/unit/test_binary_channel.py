"""095-007: BinaryChannel + `*` discriminator + CommandProcessor/CommandRouter
wiring, exercised end to end over the sim's own command channel
(`sim.command()`/`sim.command_on()`/`sim.route_no_tick()`), mirroring
`test_bare_loop_move_and_tlm.py`'s established pattern (a fixture-provided
`sim`, plain assertions against replies and `sim.vel()`/`sim.tick_for()`)
extended for the BINARY plane: every request here is a `*B<base64>`-armored
`msg::CommandEnvelope`, built with the host's `pb2` bindings (the same
reference codec ticket 006's differential suite already proved correct)
and hand-armored/dearmored in this file (mirrors `send_envelope()`'s own
eventual host-side shape, ticket 002 -- but this file talks to the sim's
raw `sim.command()` line channel directly, not `serial_conn.py`).

Covers every one of ticket 007's acceptance criteria:
  - `drive` posts the exact decoded `msg::DrivetrainCommand`, unmodified,
    to `bb.driveIn` (behavioral: wheels spin).
  - `segment`/`replace` translate all 13 `MotionSegment` fields into a
    `Motion::Segment`, verified INDIVIDUALLY via `sim.peek_segment_in()`/
    `sim.peek_replace_in()` (non-destructive Blackboard reads, `sim_api.cpp`
    095-007 test-support additions) BEFORE any tick drains the queue.
  - `stop` posts `msg::DrivetrainCommand{NEUTRAL=BRAKE}` to `bb.driveIn`.
  - `ping`/`echo`/`id` reply inline (no Blackboard post), matching their
    text counterparts' information content.
  - `pose`/`otos` reply `Error{ERR_UNIMPLEMENTED}` (098 lands these).
  - `config`/`get` (096-004) apply/read `Rt::ConfigDelta`/`bb.*Config`.
  - `stream` (096-005) sets `bb.telemetryPeriod`/`telemetryChannel`/
    `telemetryBinary`, wiring into `tickTelemetry()`'s periodic emission.
    097-008 (Decision 9, pure-binary firmware) deletes text STREAM/SNAP and
    `tickTelemetry()`'s text-emission branch outright -- periodic emission
    is unconditionally binary now, so `bb.telemetryBinary` is still WRITTEN
    by the `stream` arm below but no longer changes observable behavior
    (see `test_binary_stream_binary_false_still_emits_binary_with_shared_seq`).
  - Malformed/out-of-range input yields a typed `Error{code, field}`.
  - A mixed text+binary session in ONE test proves dual-stack coexistence.
"""
from __future__ import annotations

import base64
import pathlib
import sys

import pytest

# tests/sim/unit/test_binary_channel.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_HOST_DIR = _REPO_ROOT / "host"
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))

from robot_radio.robot.pb2 import common_pb2 as pb_common  # noqa: E402
from robot_radio.robot.pb2 import config_pb2 as pb_config  # noqa: E402
from robot_radio.robot.pb2 import drivetrain_pb2 as pb_drivetrain  # noqa: E402
from robot_radio.robot.pb2 import envelope_pb2 as pb_envelope  # noqa: E402
from robot_radio.robot.pb2 import motion_pb2 as pb_motion  # noqa: E402
from robot_radio.robot.pb2 import odometer_pb2 as pb_odometer  # noqa: E402

pytestmark = pytest.mark.filterwarnings("ignore")

# tests/sim/conftest.py's build_lib fixture already inserts this path; guard
# against a double-insert if this module is imported before that fixture runs.
_SIM_INFRA_DIR = _REPO_ROOT / "tests" / "_infra" / "sim"
if str(_SIM_INFRA_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_INFRA_DIR))
from firmware import CHANNEL_RADIO, CHANNEL_SERIAL  # noqa: E402


# ---------------------------------------------------------------------------
# Armor / dearmor helpers -- `*B<base64>`, standard alphabet (wire_runtime.h's
# own pinned-alphabet note).
# ---------------------------------------------------------------------------


def armor(envelope: "pb_envelope.CommandEnvelope") -> str:
    return "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")


def dearmor(line: str) -> "pb_envelope.ReplyEnvelope":
    line = line.strip()
    assert line.startswith("*B"), f"expected an armored binary reply, got: {line!r}"
    raw = base64.b64decode(line[2:])
    reply = pb_envelope.ReplyEnvelope()
    reply.ParseFromString(raw)
    return reply


def send(sim, envelope: "pb_envelope.CommandEnvelope", channel: int = CHANNEL_SERIAL) -> "pb_envelope.ReplyEnvelope":
    """Send one binary command through the sim's dt=0 synchronous channel
    (ticks once, like every text `sim.command()` call already does)."""
    return dearmor(sim.command_on(armor(envelope), channel))


def send_no_tick(sim, envelope: "pb_envelope.CommandEnvelope") -> "pb_envelope.ReplyEnvelope":
    """Route one binary command WITHOUT the trailing tick -- lets the test
    peek bb.segmentIn/bb.replaceIn's raw posted payload before Drivetrain::
    tick() drains it (sim_route_no_tick(), 095-007 test-support addition)."""
    return dearmor(sim.route_no_tick(armor(envelope)))


# (100-007, THE CUTOVER) _PRIMITIVE_SHAPE -- a well-formed, ADMITTABLE
# primitive segment: a modest straight arc (deltaHeading=0, so admit()'s
# curvature-transition checks are moot at zero joint speed regardless), no
# exit speed (a stop segment, always admits from a rest ChainTail).
# Drive::Goal has exactly 3 fields (source/drive/drivetrain.h) -- the retired
# 13-field Motion::Segment shape (distance/direction/final_heading/
# speed_max/.../stream) has no wire-admission-visible equivalent any more
# (primitive=false is REJECTED outright -- see
# test_binary_segment_primitive_false_rejected_err_unimplemented below).
_PRIMITIVE_SHAPE = dict(arc_length=250.0, delta_heading=0.0, exit_speed=0.0, primitive=True)


def _assert_goal_matches_shape(peeked: dict, shape: dict) -> None:
    # peek_segment_in()/peek_replace_in() key their dict by Drive::Goal's
    # own field names (arc_length/delta_heading/exit_speed -- see
    # firmware.py's _GOAL_FIELDS) -- so this compares 1:1 against `shape`
    # (also keyed that way, minus `primitive` which the ring no longer
    # carries -- admission already consumed/verified it).
    for key, expected in shape.items():
        if key == "primitive":
            continue
        assert peeked[key] == pytest.approx(expected, rel=1e-5), \
            f"{key}: got {peeked[key]}, expected {expected}"


# ===========================================================================
# drive -- posts the exact decoded msg::DrivetrainCommand, unmodified.
# ===========================================================================


def test_binary_drive_wheels_spins_the_wheels(sim):
    wheels = pb_drivetrain.WheelTargets(w=[
        pb_common.WheelTarget(speed=150.0), pb_common.WheelTarget(speed=150.0),
    ])
    env = pb_envelope.CommandEnvelope(corr_id=1, drive=pb_drivetrain.DrivetrainCommand(wheels=wheels))
    reply = send(sim, env)
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 1

    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l > 50.0 and vel_r > 50.0, "binary drive{wheels} never reached the plant"


def test_binary_stop_posts_neutral_brake(sim):
    """Byte-identical to handleStop()'s own NEUTRAL{BRAKE} construction --
    behaviorally: a spinning direct-mode drive settles to zero after a
    binary `stop`, matching test_s_and_stop_still_work_unchanged_over_the_wire's
    text-plane assertion shape."""
    wheels = pb_drivetrain.WheelTargets(w=[
        pb_common.WheelTarget(speed=150.0), pb_common.WheelTarget(speed=150.0),
    ])
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=1, drive=pb_drivetrain.DrivetrainCommand(wheels=wheels)))
    assert reply.WhichOneof("body") == "ok"
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l > 50.0 and vel_r > 50.0

    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=2, stop=pb_envelope.Stop()))
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 2
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)


# ===========================================================================
# segment / replace -- 100-007, THE CUTOVER: wire admission over the v2
# primitive shape (arc_length/delta_heading/exit_speed/primitive). Every
# test below supersedes its pre-cutover Motion::Segment-shaped counterpart
# (git history has the old 13-field version if a reference is ever needed).
# ===========================================================================


def test_binary_segment_admits_and_translates_the_3_goal_fields(sim):
    seg = pb_motion.MotionSegment(**_PRIMITIVE_SHAPE)
    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=3, segment=seg))
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 3
    # q = bb.segmentIn.size() (1, just posted) + bb.drivetrain.queue (0,
    # nothing drained yet on a fresh sim) -- same formula the pre-cutover
    # ack used.
    assert reply.ok.q == 1
    assert reply.ok.rem == pytest.approx(0.0)

    peeked = sim.peek_segment_in(0)
    assert peeked is not None, "admitted Goal never reached bb.segmentIn"
    _assert_goal_matches_shape(peeked, _PRIMITIVE_SHAPE)

    tail = sim.chain_tail()
    assert tail["x"] == pytest.approx(250.0), "wire admission must advance bb.chainTail"


def test_binary_replace_translates_the_mover_shape(sim):
    """(100-008) `replace` is MOVER's exclusive wire home now -- decodes
    time/v/omega straight into bb.replaceIn's Rt::MoverRequest, no
    admit()/chainTail involvement (a velocity-mode plan has no pose goal).
    Supersedes the pre-100-008 Goal-shaped
    test_binary_replace_admits_and_translates_the_3_goal_fields (git
    history has that version if a reference is ever needed) -- `replace`
    never had any OTHER producer than MOVER (grep of every
    legacy_verbs.py BINARY_DISPATCH entry), so there is no Goal-shaped
    `replace` behavior left to preserve."""
    seg = pb_motion.MotionSegment(time=800.0, v=250.0, omega=1.0, primitive=True)
    tail_before = sim.chain_tail()
    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=4, replace=seg))
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 4

    peeked = sim.peek_replace_in()
    assert peeked is not None, "MoverRequest never reached bb.replaceIn"
    assert peeked["v"] == pytest.approx(250.0)
    assert peeked["omega"] == pytest.approx(1.0)
    assert peeked["deadman"] == pytest.approx(800.0)

    # No admit()/chainTail check for MOVER -- unlike segment/replace's
    # pre-100-008 Goal path, bb.chainTail is never read or advanced here.
    tail_after = sim.chain_tail()
    assert tail_after == tail_before, "MOVER must never touch bb.chainTail"


def test_binary_replace_primitive_false_rejected_err_unimplemented(sim):
    """The retired (pre-cutover) non-primitive MotionSegment shape is
    REJECTED on `replace` too, the same way handleSegment() rejects it --
    never a silent MOVER interpretation of a malformed/legacy request."""
    seg = pb_motion.MotionSegment(time=400.0, v=250.0, primitive=False)
    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=18, replace=seg))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_UNIMPLEMENTED
    assert sim.peek_replace_in() is None, "a rejected replace must leave bb.replaceIn untouched"


def test_binary_replace_stream_true_is_not_blend_still_admits_as_mover(sim):
    """AC (100-008): BLEND (`stream=true`) on `segment` still ERRs (see
    test_binary_segment_stream_true_rejected_err_unimplemented above,
    UNCHANGED by this ticket) -- but `stream` is not even inspected on
    `replace` (architecture-update.md M8: BLEND is a `segment`-arm-only
    concern), so a MOVER request that happens to also carry `stream=true`
    (matching every pre-100-008 caller's own habit) is NOT accidentally
    rejected -- it admits normally, exactly like any other MOVER."""
    seg = pb_motion.MotionSegment(time=400.0, v=200.0, primitive=True, stream=True)
    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=19, replace=seg))
    assert reply.WhichOneof("body") == "ok"
    peeked = sim.peek_replace_in()
    assert peeked is not None
    assert peeked["v"] == pytest.approx(200.0)


def test_binary_segment_primitive_false_rejected_err_unimplemented(sim):
    """The retired (pre-cutover) non-primitive MotionSegment shape is
    REJECTED outright at the wire, never silently accepted -- ticket
    100-007's own acceptance criteria."""
    seg = pb_motion.MotionSegment(distance=300.0, primitive=False)
    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=15, segment=seg))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_UNIMPLEMENTED
    assert sim.peek_segment_in(0) is None, "a rejected segment must leave the queue untouched"


def test_binary_segment_stream_true_rejected_err_unimplemented(sim):
    """BLEND (the pre-097 `MOVE s=1` streaming merge) has no v2 primitive
    equivalent -- architecture-update.md M8: replies ERR to stream=true."""
    seg = pb_motion.MotionSegment(arc_length=100.0, primitive=True, stream=True)
    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=16, segment=seg))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_UNIMPLEMENTED
    assert sim.peek_segment_in(0) is None


def test_binary_segment_infeasible_admission_typed_err_queue_untouched(sim):
    """A pivot (arc_length=0) with a nonzero exit speed is
    Drive::Verdict::PIVOT_NONZERO_EXIT -- admit() rejects it, the wire
    replies a typed ERR (ERR_RANGE, field=the specific Verdict ordinal),
    and the queue/bb.chainTail are left untouched (SUC-003's own
    postcondition)."""
    tail_before = sim.chain_tail()
    seg = pb_motion.MotionSegment(arc_length=0.0, delta_heading=0.5, exit_speed=100.0,
                                  primitive=True)
    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=17, segment=seg))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_RANGE
    # Verdict::PIVOT_NONZERO_EXIT is enumerator index 4 (OK, EXIT_UNREACHABLE,
    # JOINT_STEP_TOO_LARGE, JOINT_SIGN_REVERSAL, PIVOT_NONZERO_EXIT, ... --
    # source/drive/drivetrain.h).
    assert reply.err.field == 4
    assert sim.peek_segment_in(0) is None, "an admission rejection must leave the queue untouched"
    tail_after = sim.chain_tail()
    assert tail_after == tail_before, "an admission rejection must leave bb.chainTail untouched"


def test_binary_segment_drives_the_plant(sim):
    """Lighter behavioral companion to the field-level proof above -- a
    binary v2 primitive segment actually reaches the adapter, gets planned,
    and drives the wheels, the same end-to-end proof
    test_bare_loop_move_and_tlm.py's text-plane MOVE tests already
    establish for the pre-cutover stack."""
    seg = pb_motion.MotionSegment(arc_length=300.0, delta_heading=0.0, exit_speed=0.0,
                                  primitive=True)
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=5, segment=seg))
    assert reply.WhichOneof("body") == "ok"

    max_v = 0.0
    for _ in range(150):
        sim.tick_for(24)
        vel_l, vel_r = sim.vel()
        max_v = max(max_v, (vel_l + vel_r) / 2.0)
    assert max_v > 50.0, "binary segment never genuinely drove"


def test_binary_segment_full_queue_replies_err_full(sim):
    """bb.segmentIn is an Rt::WorkQueue<Drive::Goal, 8> -- mirrors the
    pre-cutover ERR_FULL behavior once the queue is at capacity.
    route_no_tick() never drains it, so 8 ADMITTED posts fill it exactly --
    each is a short straight run (zero joint speed at admission time, since
    nothing is executing yet, so admit()'s curvature-transition checks
    never fire for this all-straight chain)."""
    seg = pb_motion.MotionSegment(arc_length=10.0, primitive=True)
    for i in range(8):
        reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=100 + i, segment=seg))
        assert reply.WhichOneof("body") == "ok", f"post {i} unexpectedly rejected"

    reply = send_no_tick(sim, pb_envelope.CommandEnvelope(corr_id=109, segment=seg))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_FULL


# ===========================================================================
# ping / echo / id -- inline replies, no Blackboard post.
# ===========================================================================


def test_binary_ping_replies_with_robot_clock_timestamp(sim):
    sim.tick_for(240)   # advance the host fake clock to a known value
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=6, ping=pb_envelope.Ping()))
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 6
    # Ack.t (095-007 schema-gap closure) -- clock-sync parity with text
    # PING's own `OK pong t=<ms>` reply; Types::systemClockNow() at
    # route-time equals the sim's own tracked `now`.
    assert reply.ok.t == 240


@pytest.mark.parametrize("payload", [b"", b"\x00", b"hello binary", bytes(range(64))],
                         ids=["empty", "nul", "ascii", "max64"])
def test_binary_echo_replies_with_payload_verbatim(sim, payload):
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=7, echo=pb_envelope.Echo(payload=payload)))
    assert reply.WhichOneof("body") == "echo"
    assert reply.corr_id == 7
    assert reply.echo.payload == payload


def test_binary_id_replies_with_device_identity(sim):
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=8, id=pb_envelope.DeviceId()))
    assert reply.WhichOneof("body") == "id"
    assert reply.corr_id == 8
    # deviceIdentity() under HOST_BUILD -- system_commands.cpp's own fixed
    # placeholder, the SAME pair handleId()'s text ID reply uses.
    assert reply.id.model == "NEZHA2"
    assert reply.id.name == "HOST-SIM"
    assert reply.id.serial == 0
    assert reply.id.proto_version == 2


# ===========================================================================
# hello / ver / help -- stakeholder-directed 6-verb minimal command surface
# (2026-07-10): the text rump's remaining three verbs (HELLO/VER/HELP) each
# get their own binary arm, completing the set PING/STOP/ID already had.
# hello/ver reuse the id arm's DeviceId reply body (BinaryChannel::handleId()
# is reused firmware-side); help gets a new HelpText reply.
# ===========================================================================


def test_binary_hello_replies_with_device_identity(sim):
    """hello -> the SAME DeviceId reply shape/content id does (handleId()
    reused firmware-side) -- proves the request arm is wired, not just
    that ID itself still works."""
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=12, hello=pb_envelope.Hello()))
    assert reply.WhichOneof("body") == "id"
    assert reply.corr_id == 12
    assert reply.id.model == "NEZHA2"
    assert reply.id.name == "HOST-SIM"
    assert reply.id.serial == 0
    assert reply.id.proto_version == 2


def test_binary_ver_replies_with_device_identity(sim):
    """ver -> the SAME DeviceId reply shape/content id/hello do -- a real
    client reads only fw_version/proto_version off it (VER's own content is
    a strict subset of ID's reply fields)."""
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=13, ver=pb_envelope.Ver()))
    assert reply.WhichOneof("body") == "id"
    assert reply.corr_id == 13
    assert reply.id.model == "NEZHA2"
    assert reply.id.proto_version == 2


def test_binary_help_replies_with_registered_verb_list(sim):
    """help -> HelpText{text}, sourced from the SAME
    Rt::CommandRouter::listVerbs() the text HELP handler reads -- the live
    registered 6-verb rump, not a hardcoded string."""
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=14, help=pb_envelope.Help()))
    assert reply.WhichOneof("body") == "helptext"
    assert reply.corr_id == 14
    verbs = reply.helptext.text.split()
    assert verbs == ["HELP", "HELLO", "PING", "ID", "VER", "STOP"]


# ===========================================================================
# Declared-only arms -- ERR_UNIMPLEMENTED, never a crash, never silent.
# `config`/`get` are no longer declared-only as of 096-004, `stream` is no
# longer declared-only as of 096-005 (see the dedicated sections below), and
# `pose_fix`'s reset/zero_encoders variants are no longer declared-only as
# of 099-004 (see test_pose_fix_reset_zero.py for those two live variants).
# `pose_fix`'s own THIRD variant (neither reset nor zero_encoders set -- a
# genuine delayed camera fix) is ALSO no longer declared-only as of 099-008
# (see test_pose_fix_reset_zero.py's own neither-flag acceptance tests and
# test_pose_fix_end_to_end.py for its live behavior) -- `otos` is the one
# remaining stub.
# ===========================================================================


@pytest.mark.parametrize("kwargs,expected_field", [
    (dict(otos=pb_odometer.OdometerCommand()), 8),
], ids=["otos"])
def test_binary_declared_only_arms_reply_err_unimplemented(sim, kwargs, expected_field):
    env = pb_envelope.CommandEnvelope(corr_id=9, **kwargs)
    reply = send(sim, env)
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_UNIMPLEMENTED
    assert reply.err.field == expected_field


# ===========================================================================
# config / get -- 096-004: BinaryChannel's config/get arms. Every one of
# config_commands.cpp's 15 kAllKeys keys round-trips (`config` then `get` on
# the matching target). Expected values mirror applyConfigKey()'s known
# behavior -- never by running the unregistered text SET handler.
# ===========================================================================


def config_send(sim, corr_id, **patch_kwargs):
    return send(sim, pb_envelope.CommandEnvelope(corr_id=corr_id,
                                                  config=pb_envelope.ConfigDelta(**patch_kwargs)))


def get_send(sim, corr_id, target):
    return send(sim, pb_envelope.CommandEnvelope(corr_id=corr_id, get=pb_envelope.ConfigGet(target=target)))


# (key, DrivetrainConfigPatch field, test value) -- covers 6 of the 15
# kAllKeys keys (tw, rotSlip, ekfQxy, ekfQtheta, ekfROtosXy, ekfROtosTheta).
_DRIVETRAIN_KEYS = [
    ("tw", "trackwidth", 321.0),
    ("rotSlip", "rotational_slip", 0.75),
    ("ekfQxy", "ekf_q_xy", 1.5),
    ("ekfQtheta", "ekf_q_theta", 2.5),
    ("ekfROtosXy", "ekf_r_otos_xy", 3.5),
    ("ekfROtosTheta", "ekf_r_otos_theta", 4.5),
]


@pytest.mark.parametrize("key,field,value", _DRIVETRAIN_KEYS, ids=[k for k, _, _ in _DRIVETRAIN_KEYS])
def test_binary_config_drivetrain_keys_round_trip(sim, key, field, value):
    reply = config_send(sim, 30, drivetrain=pb_config.DrivetrainConfigPatch(**{field: value}))
    assert reply.WhichOneof("body") == "ok", f"{key}: config post rejected"

    reply = get_send(sim, 31, pb_config.CONFIG_DRIVETRAIN)
    assert reply.WhichOneof("body") == "cfg"
    assert reply.cfg.target == pb_config.CONFIG_DRIVETRAIN
    assert reply.cfg.WhichOneof("patch") == "drivetrain"
    assert getattr(reply.cfg.drivetrain, field) == pytest.approx(value)


def test_binary_config_min_speed_round_trips_on_planner_target(sim):
    # 7th of the 15 kAllKeys keys: minSpeed -> PlannerConfigPatch.min_speed.
    reply = config_send(sim, 32, planner=pb_config.PlannerConfigPatch(min_speed=42.0))
    assert reply.WhichOneof("body") == "ok"

    reply = get_send(sim, 33, pb_config.CONFIG_PLANNER)
    assert reply.WhichOneof("body") == "cfg"
    assert reply.cfg.target == pb_config.CONFIG_PLANNER
    assert reply.cfg.WhichOneof("patch") == "planner"
    assert reply.cfg.planner.min_speed == pytest.approx(42.0)


def test_binary_config_heading_kp_kd_round_trip_reaches_live_drivetrain(sim):
    """098-005: heading_kp/heading_kd -- NOT one of config_commands.cpp's 15
    legacy kAllKeys keys (that file, and the text SET/GET path it served,
    were already deleted, 097-007, before these two PlannerConfig fields
    existed -- ticket 098-001) -- added directly to the binary Patch surface
    so a live `SET headingKp=...`/`SET headingKd=...` can reach the running
    Motion::SegmentExecutor without a reflash (098-005/M7).

    Sends a REAL *B-armored CommandEnvelope through sim.command_on() --
    Rt::CommandRouter -> BinaryChannel::handleConfig() ->
    handleConfigPlanner() (source/commands/binary_channel.cpp) -> posts an
    Rt::ConfigDelta to bb.configIn -- the actual firmware wire path, not a
    hand-built internal delta. The sim's own Rt::Configurator (TEST-ONLY,
    096-004 -- sim_api.cpp's SimHandle -- drains bb.configIn to exhaustion
    after every sim_command_on() call) then applies it through the SAME
    Configurator::applyOne() kPlanner case main.cpp's real loop now ticks
    once per pass (098-005) -- including this ticket's fix,
    `drivetrain_.configureMotion(plannerConfig_)`, without which
    bb.plannerConfig would still publish the new value (the fold+publish
    half was already correct) but the LIVE Drivetrain's SegmentExecutor
    would silently keep running on the STALE boot gain. The `get` round trip
    below reads bb.plannerConfig back (handleGet()'s CONFIG_PLANNER arm,
    also extended by this ticket) -- confirming the value that stuck is the
    SAME one Configurator::applyOne() would have handed configureMotion(),
    proving the whole chain: wire decode -> Rt::ConfigDelta mask/fields ->
    Configurator fold -> live Drivetrain re-configure -> bb.plannerConfig
    publish, end to end through the real BinaryChannel code path.
    """
    reply = config_send(sim, 132, planner=pb_config.PlannerConfigPatch(heading_kp=6.0, heading_kd=0.25))
    assert reply.WhichOneof("body") == "ok"

    reply = get_send(sim, 133, pb_config.CONFIG_PLANNER)
    assert reply.WhichOneof("body") == "cfg"
    assert reply.cfg.target == pb_config.CONFIG_PLANNER
    assert reply.cfg.WhichOneof("patch") == "planner"
    assert reply.cfg.planner.heading_kp == pytest.approx(6.0)
    assert reply.cfg.planner.heading_kd == pytest.approx(0.25)

    # A second, DISJOINT-field delta (heading_kd alone) must not clobber the
    # heading_kp value the first delta just set -- the SAME field-masked
    # fold guarantee configurator_harness.cpp's own scenario 7 proves for
    # kDrivetrain, exercised here for kPlanner over the REAL wire path.
    reply = config_send(sim, 134, planner=pb_config.PlannerConfigPatch(heading_kd=0.5))
    assert reply.WhichOneof("body") == "ok"
    reply = get_send(sim, 135, pb_config.CONFIG_PLANNER)
    assert reply.cfg.planner.heading_kp == pytest.approx(6.0), "heading_kp untouched by the heading_kd-only delta"
    assert reply.cfg.planner.heading_kd == pytest.approx(0.5)


def test_binary_config_ml_mr_address_correct_bound_motor_independently(sim):
    """ml (side=LEFT) / mr (side=RIGHT) -- 2 of the 15 kAllKeys keys. Each
    touches ONLY its own bound motor's travel_calib -- config_commands.cpp's
    applyConfigKey() never lets `ml` touch the right motor or vice versa."""
    reply = config_send(sim, 34, motor=pb_config.MotorConfigPatch(side=pb_config.LEFT, travel_calib=1.111))
    assert reply.WhichOneof("body") == "ok"
    reply = config_send(sim, 35, motor=pb_config.MotorConfigPatch(side=pb_config.RIGHT, travel_calib=2.222))
    assert reply.WhichOneof("body") == "ok"

    left = get_send(sim, 36, pb_config.CONFIG_MOTOR_LEFT)
    right = get_send(sim, 37, pb_config.CONFIG_MOTOR_RIGHT)
    assert left.cfg.WhichOneof("patch") == "motor"
    assert right.cfg.WhichOneof("patch") == "motor"
    assert left.cfg.motor.travel_calib == pytest.approx(1.111)
    assert right.cfg.motor.travel_calib == pytest.approx(2.222)
    assert left.cfg.motor.side == pb_config.LEFT
    assert right.cfg.motor.side == pb_config.RIGHT


@pytest.mark.parametrize("field", ["kp", "ki", "kff", "i_max", "kaw"], ids=["kp", "ki", "kff", "iMax", "kaw"])
def test_binary_config_pid_gains_apply_to_both_bound_motors(sim, field):
    """pid.kp/ki/kff/iMax/kaw -- the remaining 5 of the 15 kAllKeys keys.
    Always write BOTH bound motors identically (Decision 5) -- mirrors
    applyConfigKey()'s own hard-coded both-sides behavior exactly; NOT
    disambiguated by `side` (side selects travel_calib ONLY)."""
    reply = config_send(sim, 38, motor=pb_config.MotorConfigPatch(**{field: 9.5}))
    assert reply.WhichOneof("body") == "ok"

    left = get_send(sim, 39, pb_config.CONFIG_MOTOR_LEFT)
    right = get_send(sim, 40, pb_config.CONFIG_MOTOR_RIGHT)
    assert getattr(left.cfg.motor, field) == pytest.approx(9.5)
    assert getattr(right.cfg.motor, field) == pytest.approx(9.5)


def test_binary_config_stimeout_posts_to_watchdog_window_not_a_config_target(sim):
    """sTimeout (Open Question 4, the 15th kAllKeys key) posts straight to
    bb.streamWatchdogWindowIn -- NOT one of the Configurator's four fold
    targets. Verified two ways: get(CONFIG_WATCHDOG) reflects it, AND every
    other target's snapshot is byte-identical to its pre-post baseline
    (proving sTimeout never routed through bb.configIn/the Configurator)."""
    baseline_dt = get_send(sim, 41, pb_config.CONFIG_DRIVETRAIN)
    baseline_planner = get_send(sim, 42, pb_config.CONFIG_PLANNER)
    baseline_left = get_send(sim, 43, pb_config.CONFIG_MOTOR_LEFT)
    baseline_right = get_send(sim, 44, pb_config.CONFIG_MOTOR_RIGHT)

    reply = config_send(sim, 45, watchdog=4242)
    assert reply.WhichOneof("body") == "ok"

    watchdog_reply = get_send(sim, 46, pb_config.CONFIG_WATCHDOG)
    assert watchdog_reply.WhichOneof("body") == "cfg"
    assert watchdog_reply.cfg.target == pb_config.CONFIG_WATCHDOG
    assert watchdog_reply.cfg.WhichOneof("patch") == "watchdog"
    assert watchdog_reply.cfg.watchdog == 4242

    after_dt = get_send(sim, 47, pb_config.CONFIG_DRIVETRAIN)
    after_planner = get_send(sim, 48, pb_config.CONFIG_PLANNER)
    after_left = get_send(sim, 49, pb_config.CONFIG_MOTOR_LEFT)
    after_right = get_send(sim, 50, pb_config.CONFIG_MOTOR_RIGHT)
    assert after_dt.cfg.drivetrain == baseline_dt.cfg.drivetrain
    assert after_planner.cfg.planner == baseline_planner.cfg.planner
    assert after_left.cfg.motor == baseline_left.cfg.motor
    assert after_right.cfg.motor == baseline_right.cfg.motor


def test_binary_config_empty_patch_replies_err_unknown(sim):
    """A well-formed-but-empty ConfigDelta (no oneof `patch` arm set at all)
    decodes cleanly (patch_kind == NONE) -- must still reply, never silently
    drop, never crash."""
    reply = config_send(sim, 53)
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_UNKNOWN
    assert reply.err.field == 6   # CommandEnvelope.cmd.config's own field number


def test_binary_get_missing_target_replies_err_badarg_field_1(sim):
    """ConfigGet.target is `optional` + `(req)=true` (ticket 001) -- an
    envelope that never sets it is caught by the generated decoder's own
    req validation, never a hand-written check in BinaryChannel."""
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=51, get=pb_envelope.ConfigGet()))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_BADARG
    assert reply.err.field == 1


@pytest.mark.parametrize("target", [
    pb_config.CONFIG_DRIVETRAIN, pb_config.CONFIG_MOTOR_LEFT, pb_config.CONFIG_MOTOR_RIGHT,
    pb_config.CONFIG_PLANNER, pb_config.CONFIG_WATCHDOG,
], ids=["drivetrain", "motor_left", "motor_right", "planner", "watchdog"])
def test_binary_get_replies_exactly_one_config_snapshot(sim, target):
    """get{target} replies exactly one ConfigSnapshot for that target -- no
    multi-reply behavior is introduced (Decision 4)."""
    reply = get_send(sim, 52, target)
    assert reply.WhichOneof("body") == "cfg"
    assert reply.cfg.target == target
    # Exactly one `patch` oneof arm is populated -- the acceptance
    # criterion's own wording, asserted explicitly rather than only relying
    # on protobuf's oneof invariant.
    assert reply.cfg.WhichOneof("patch") is not None


# ===========================================================================
# stream -- 096-005: BinaryChannel's `stream` arm, wiring
# msg::StreamControl{binary, period} into tickTelemetry()'s periodic
# emission path. Uses sim.peek_reply_store() (non-destructive), never
# sim.command()/send()/sim.command_on() to OBSERVE periodic output -- both
# reset the target channel's ReplyStore before routing, which would wipe
# out whatever tickTelemetry() had already accumulated across the
# preceding tick_for() calls. 097-008 (Decision 9, pure-binary firmware)
# deletes the text STREAM/SNAP family this section used to have a text-plane
# sibling suite for (test_telemetry_periodic_tick.py, now removed -- its
# three ack/monotonic-seq/period-zero scenarios are fully subsumed by the
# binary-plane tests below, which already covered the identical behavior)
# and deletes tickTelemetry()'s own text-emission branch, so periodic
# emission is unconditionally binary now regardless of `StreamControl.binary`
# -- see test_binary_stream_binary_false_still_emits_binary_with_shared_seq().
# ===========================================================================


def _parse_binary_tlm_frames(text: str) -> list["pb_envelope.ReplyEnvelope"]:
    """Parse zero or more "*B<base64>"-armored ReplyEnvelope lines (newline
    separated, ReplyStore::append()'s own convention) into decoded
    ReplyEnvelope messages, in the order tickTelemetry() appended them."""
    frames = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        frames.append(dearmor(line))
    return frames


def test_binary_stream_ack_reply_carries_no_immediate_frame(sim):
    """Open Question 5 (mirrors text STREAM's own
    test_stream_ack_reply_carries_no_immediate_frame): the binary `stream`
    arm's own ack is exactly one OK reply -- no concatenated first TLM
    frame. send()/command_on() never calls tickTelemetry() at all, so this
    also confirms that boundary still holds for the binary arm."""
    reply = send(sim, pb_envelope.CommandEnvelope(
        corr_id=60, stream=pb_envelope.StreamControl(binary=True, period=50)))
    assert reply.WhichOneof("body") == "ok"
    assert sim.peek_reply_store(CHANNEL_SERIAL) == "", (
        "stream's own ACK should carry no concatenated frame"
    )


def test_binary_stream_periodic_emission_monotonic_seq_over_200ms(sim):
    """binary `stream{binary:true, period:50}` armed, then >= 200ms of
    ticking (tick_for()'s default 24ms step) must yield >= 3 periodic
    ReplyEnvelope{tlm} frames on the SERIAL sync store, corr_id=0
    (unsolicited push -- envelope.proto's own doc comment) and strictly
    increasing seq=."""
    reply = send(sim, pb_envelope.CommandEnvelope(
        corr_id=61, stream=pb_envelope.StreamControl(binary=True, period=50)))
    assert reply.WhichOneof("body") == "ok"
    assert sim.peek_reply_store(CHANNEL_SERIAL) == "", (
        "no periodic frame should exist yet -- tickTelemetry() has not run "
        "a single pass since stream armed the period"
    )

    sim.tick_for(240)   # [ms] >= 200ms of ticking, 10 x 24ms passes

    frames = _parse_binary_tlm_frames(sim.peek_reply_store(CHANNEL_SERIAL))
    assert len(frames) >= 3, f"expected >= 3 periodic binary frames, got {len(frames)}"
    for frame in frames:
        assert frame.WhichOneof("body") == "tlm"
        assert frame.corr_id == 0, "unsolicited push TLM frames must carry corr_id=0"

    seqs = [frame.tlm.seq for frame in frames]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs), (
        f"seq= must strictly increase across periodic binary frames, got {seqs}"
    )


def test_binary_stream_binary_false_still_emits_binary_with_shared_seq(sim):
    """097-008 (Decision 9, pure-binary firmware): `stream{binary:false,
    ...}` used to revert periodic emission to the pre-existing plain-text
    TLM line (tickTelemetry()'s own bb.telemetryBinary branch) -- that text
    branch is DELETED now, so `binary:false` has NO observable wire-framing
    effect any more: emission stays binary regardless, with the SAME shared
    `seq=` counter continuing (not resetting) across the toggle. This is the
    direct replacement for the pre-097-008
    test_binary_stream_toggle_binary_false_reverts_to_text_with_shared_seq
    (deleted -- its own premise, a live text fallback, no longer holds) --
    `StreamControl.binary` is still a real wire field (a legacy-proxy client
    could still set it false) but is now write-only bookkeeping on
    bb.telemetryBinary (blackboard.h), read by nothing."""
    send(sim, pb_envelope.CommandEnvelope(
        corr_id=62, stream=pb_envelope.StreamControl(binary=True, period=50)))
    sim.tick_for(120)   # a couple of binary frames land on the SERIAL store
    first_frames = _parse_binary_tlm_frames(sim.peek_reply_store(CHANNEL_SERIAL))
    assert len(first_frames) >= 2, "sanity: binary periodic emission must be active first"
    last_seq_before_toggle = first_frames[-1].tlm.seq

    # Switching binary off (same period) must NOT disable periodic emission
    # -- only bb.telemetryPeriod == 0 does that (tickTelemetry()'s own
    # guard) -- and, since 097-008, must not change the wire framing either.
    reply = send(sim, pb_envelope.CommandEnvelope(
        corr_id=63, stream=pb_envelope.StreamControl(binary=False, period=50)))
    assert reply.WhichOneof("body") == "ok"
    assert sim.peek_reply_store(CHANNEL_SERIAL) == ""

    sim.tick_for(120)
    after_frames = _parse_binary_tlm_frames(sim.peek_reply_store(CHANNEL_SERIAL))
    assert len(after_frames) >= 2, f"expected periodic BINARY frames after binary=false, got {len(after_frames)}"
    for frame in after_frames:
        assert frame.WhichOneof("body") == "tlm", "binary=false must not revert framing to text"

    after_seqs = [frame.tlm.seq for frame in after_frames]
    assert after_seqs == sorted(after_seqs) and len(set(after_seqs)) == len(after_seqs)
    assert after_seqs[0] > last_seq_before_toggle, (
        "bb.telemetrySeq must be the SAME shared/monotonic counter across "
        "the binary=true -> binary=false toggle, not reset"
    )


def test_binary_stream_period_zero_stops_periodic_emission(sim):
    """`stream{..., period:0}` behaves exactly like text STREAM 0
    (acceptance criterion 2) -- disables periodic emission outright via
    tickTelemetry()'s own `bb.telemetryPeriod == 0` guard, regardless of
    `binary`. bb.telemetryBinary is still recorded (bookkeeping symmetry)
    but has no visible effect once period is 0."""
    send(sim, pb_envelope.CommandEnvelope(
        corr_id=64, stream=pb_envelope.StreamControl(binary=True, period=50)))
    sim.tick_for(240)
    frames_before = _parse_binary_tlm_frames(sim.peek_reply_store(CHANNEL_SERIAL))
    assert len(frames_before) >= 3, (
        "sanity: periodic emission must be active before disabling it"
    )

    reply = send(sim, pb_envelope.CommandEnvelope(
        corr_id=65, stream=pb_envelope.StreamControl(binary=True, period=0)))
    assert reply.WhichOneof("body") == "ok"
    assert sim.peek_reply_store(CHANNEL_SERIAL) == ""

    sim.tick_for(240)
    assert sim.peek_reply_store(CHANNEL_SERIAL) == "", (
        "stream{period:0} must prevent any further periodic frame from being emitted"
    )


def test_binary_stream_binds_periodic_emission_to_the_requesting_channel(sim):
    """096-006 (genuine behavioral gap, 004/005's own test coverage never
    exercised a second channel): `bb.telemetryChannel` is bound to
    `CommandRouter::currentChannel()` -- the channel the `stream` REQUEST
    itself arrived on (binary_channel.cpp's STREAM case) -- mirroring
    `handleStream()`'s own text-plane channel binding. Every other stream
    test in this file only ever sends on CHANNEL_SERIAL (the `send()`
    helper's own default), which never distinguishes "always emits on
    SERIAL" from "emits on whichever channel armed it". Arming on
    CHANNEL_RADIO and observing periodic frames land THERE (and nowhere on
    CHANNEL_SERIAL) proves the binding is real, not a hardcoded default."""
    reply = send(sim, pb_envelope.CommandEnvelope(
        corr_id=70, stream=pb_envelope.StreamControl(binary=True, period=50)), channel=CHANNEL_RADIO)
    assert reply.WhichOneof("body") == "ok"
    assert sim.peek_reply_store(CHANNEL_RADIO) == ""
    assert sim.peek_reply_store(CHANNEL_SERIAL) == ""

    sim.tick_for(240)

    radio_frames = _parse_binary_tlm_frames(sim.peek_reply_store(CHANNEL_RADIO))
    assert len(radio_frames) >= 3, f"expected >= 3 periodic binary frames on CHANNEL_RADIO, got {len(radio_frames)}"
    for frame in radio_frames:
        assert frame.WhichOneof("body") == "tlm"
        assert frame.corr_id == 0

    assert sim.peek_reply_store(CHANNEL_SERIAL) == "", (
        "a stream armed on CHANNEL_RADIO must never emit periodic frames on CHANNEL_SERIAL"
    )


# test_binary_snap_still_works_standalone_while_binary_stream_is_active --
# DELETED (097-008): its own premise was that a SEPARATE one-shot text SNAP
# verb existed alongside the periodic binary stream and proved the two paths
# didn't interfere. Text SNAP is deleted outright by this ticket (Decision
# 9) and there is no binary one-shot TLM/SNAP arm to replace it with (096
# Open Question 2 / 097 Decision 4's own finding) -- the only way to get a
# one-shot binary reading is the SAME `stream` arm this section already
# exercises (arm/read/disarm, `NezhaProtocol.snap()`'s own host-side
# synthesis, host/robot_radio/robot/protocol.py), which collapses into
# single-consumer stream-state semantics already covered by
# test_binary_stream_binds_periodic_emission_to_the_requesting_channel and
# the ack/monotonic-seq/period-zero tests above -- there is no longer a
# distinct "SNAP vs. STREAM non-interference" behavior left to prove.


# ===========================================================================
# Malformed / out-of-range input -- typed Error{code, field}, never a crash.
# ===========================================================================


def test_binary_malformed_base64_replies_err_decode(sim):
    reply = dearmor(sim.command("*B!!!not-valid-base64!!!"))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_DECODE


def test_binary_missing_armor_prefix_replies_err_decode(sim):
    """`*` alone (no `B`) still reaches BinaryChannel::handle() (only
    line[0]=='*' gates the dispatch in CommandProcessor::process()) -- must
    be rejected cleanly, not assumed to always be well-formed."""
    reply = dearmor(sim.command("*XnotArmored"))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_DECODE


def test_binary_out_of_range_segment_field_replies_err_range(sim):
    seg = pb_motion.MotionSegment(distance=99999.0)   # abs_max=10000
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=10, segment=seg))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_RANGE
    assert reply.err.field == 1   # MotionSegment.distance's own field number


def test_binary_empty_envelope_replies_err_unknown(sim):
    """A well-formed-but-empty CommandEnvelope (corr_id only, no oneof arm
    set) decodes cleanly (cmd_kind == NONE) -- BinaryChannel's default case
    must still reply, never silently drop."""
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=11))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.ERR_UNKNOWN


# ===========================================================================
# Dual-stack coexistence -- text and binary in ONE session.
# ===========================================================================


def test_mixed_text_and_binary_session(sim):
    """Proves the text plane and the binary plane share the SAME
    CommandRouter/CommandProcessor/Blackboard instance correctly within one
    session -- not just that each plane works alone (every other test in
    this file only ever sends binary). 097-006: text `S` is deleted, so the
    opening drive below is now ALSO binary -- this test's remaining
    "mixed" proof is the binary-drive/binary-stop/binary-drive sequence
    coexisting correctly with the text `STOP`/`PING` calls further down,
    the SAME CommandRouter instance serving both."""
    # Binary drive starts direct-mode driving.
    wheels = pb_drivetrain.WheelTargets(w=[
        pb_common.WheelTarget(speed=150.0), pb_common.WheelTarget(speed=150.0),
    ])
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=19, drive=pb_drivetrain.DrivetrainCommand(wheels=wheels)))
    assert reply.WhichOneof("body") == "ok"
    sim.tick_for(500)
    vel_l, vel_r = sim.vel()
    assert vel_l > 50.0 and vel_r > 50.0

    # Binary stop brakes the SAME drivetrain instance.
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=20, stop=pb_envelope.Stop()))
    assert reply.WhichOneof("body") == "ok"
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)

    # Binary drive restarts it.
    wheels = pb_drivetrain.WheelTargets(w=[
        pb_common.WheelTarget(speed=150.0), pb_common.WheelTarget(speed=150.0),
    ])
    reply = send(sim, pb_envelope.CommandEnvelope(corr_id=21, drive=pb_drivetrain.DrivetrainCommand(wheels=wheels)))
    assert reply.WhichOneof("body") == "ok"
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l > 50.0 and vel_r > 50.0

    # Text STOP brakes what binary drive started.
    assert sim.command("STOP").strip() == "OK stop"
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)

    # Both liveness verbs work side by side.
    text_ping = sim.command("PING").strip()
    assert text_ping.startswith("OK pong t=")
    bin_ping = send(sim, pb_envelope.CommandEnvelope(corr_id=22, ping=pb_envelope.Ping()))
    assert bin_ping.WhichOneof("body") == "ok"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
