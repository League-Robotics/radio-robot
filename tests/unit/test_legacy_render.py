"""tests/unit/test_legacy_render.py -- 097-004 (M5 rogo Translator Proxy).

Golden-line tests for ``host/robot_radio/robot/legacy_render.py``: every
renderer is checked against the EXACT wire shape the (now-gutted) firmware
text handlers used to produce -- see that module's own docstring for the
pre-gut commit citations each renderer transcribes.

No real serial port/hardware/PTY anywhere here -- pure function calls
against hand-built ``pb2`` messages, the same "no hardware" level
``test_legacy_translate.py``/``test_cli_send_translator.py`` use for the
functions/modules one layer below/beside this one.
"""

from __future__ import annotations

import math

import pytest

from robot_radio.robot import legacy_render as render
from robot_radio.robot.pb2 import common_pb2, envelope_pb2, planner_pb2, telemetry_pb2

_ANGLE_SCALE = 5729.5779513  # [cdeg/rad] -- same constant render.py transcribes


# ---------------------------------------------------------------------------
# render_ok / render_err -- 4 spacing variants each
# (CommandProcessor::replyOK()/replyErr(), command_processor.cpp:284-322)
# ---------------------------------------------------------------------------


def test_render_ok_body_and_corr_id():
    assert render.render_ok("drive", "l=200 r=200", 7) == "OK drive l=200 r=200 #7"


def test_render_ok_body_no_corr_id():
    assert render.render_ok("drive", "l=200 r=200", None) == "OK drive l=200 r=200"


def test_render_ok_no_body_with_corr_id():
    assert render.render_ok("stop", None, 3) == "OK stop #3"


def test_render_ok_no_body_no_corr_id():
    assert render.render_ok("stop", None, None) == "OK stop"


def test_render_err_detail_and_corr_id():
    assert render.render_err("badarg", "missing key", 9) == "ERR badarg missing key #9"


def test_render_err_detail_no_corr_id():
    assert render.render_err("badarg", "missing key", None) == "ERR badarg missing key"


def test_render_err_no_detail_with_corr_id():
    assert render.render_err("unknown", None, 4) == "ERR unknown #4"


def test_render_err_no_detail_no_corr_id():
    assert render.render_err("unknown", None, None) == "ERR unknown"


# ---------------------------------------------------------------------------
# ERR code map + field-name lookup
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("code,expected", [
    (envelope_pb2.ERR_UNKNOWN, "unknown"),
    (envelope_pb2.ERR_BADARG, "badarg"),
    (envelope_pb2.ERR_RANGE, "range"),
    (envelope_pb2.ERR_FULL, "full"),
    (envelope_pb2.ERR_DECODE, "badarg"),
    (envelope_pb2.ERR_UNIMPLEMENTED, "unsupported"),
    (envelope_pb2.ERR_OVERSIZE, "unsupported"),
])
def test_err_code_text_map(code, expected):
    assert render.ERR_CODE_TEXT[code] == expected


def test_field_name_for_error_resolves_via_schema():
    # CommandEnvelope field 2 is `drive` (envelope.proto).
    assert render.field_name_for_error(2) == "drive"


def test_field_name_for_error_zero_is_none():
    assert render.field_name_for_error(0) is None


def test_render_error_full_line():
    err = envelope_pb2.Error(code=envelope_pb2.ERR_RANGE, field=3)  # field 3 = segment
    assert render.render_error(err, 8) == "ERR range segment #8"


def test_render_error_unknown_code_no_field():
    err = envelope_pb2.Error(code=envelope_pb2.ERR_UNKNOWN)
    assert render.render_error(err, None) == "ERR unknown"


# ---------------------------------------------------------------------------
# Liveness/identity (system_commands.cpp pre-097-006, handleId()/handleVer()/
# formatDeviceAnnouncement())
# ---------------------------------------------------------------------------


def _device_id(name="GUTOV", serial=2121102, fw="v0.20260710.1", proto=3):
    return envelope_pb2.DeviceId(model="NEZHA2", name=name, serial=serial,
                                 fw_version=fw, proto_version=proto)


def test_render_id_line_with_corr_id():
    d = _device_id()
    assert render.render_id_line(d, 5) == (
        "ID model=NEZHA2 name=GUTOV serial=2121102 fw=v0.20260710.1 proto=3 #5")


def test_render_id_line_no_corr_id():
    d = _device_id()
    assert render.render_id_line(d, None) == (
        "ID model=NEZHA2 name=GUTOV serial=2121102 fw=v0.20260710.1 proto=3")


def test_render_ver_body():
    d = _device_id()
    assert render.render_ver_body(d) == "fw=v0.20260710.1 proto=3"
    assert render.render_ok("ver", render.render_ver_body(d), None) == "OK ver fw=v0.20260710.1 proto=3"


def test_render_device_banner():
    d = _device_id(name="GUTOV", serial=2121102)
    assert render.render_device_banner(d) == "DEVICE:NEZHA2:robot:GUTOV:2121102"


# ---------------------------------------------------------------------------
# Motion verbs -- render_ok_for_verb() (motion_commands.cpp pre-097-006)
# ---------------------------------------------------------------------------


def test_render_ok_for_verb_s_drive():
    ack = envelope_pb2.Ack()
    line = render.render_ok_for_verb("S", ["200", "200"], {}, ack, "1")
    assert line == "OK drive l=200 r=200 #1"


def test_render_ok_for_verb_t_drive_ms():
    ack = envelope_pb2.Ack()
    line = render.render_ok_for_verb("T", ["200", "200", "1000"], {}, ack, None)
    assert line == "OK drive l=200 r=200 ms=1000"


def test_render_ok_for_verb_d_drive_mm():
    ack = envelope_pb2.Ack()
    line = render.render_ok_for_verb("D", ["200", "200", "300"], {}, ack, None)
    assert line == "OK drive l=200 r=200 mm=300"


def test_render_ok_for_verb_rt():
    ack = envelope_pb2.Ack()
    line = render.render_ok_for_verb("RT", ["9000"], {}, ack, "4")
    assert line == "OK rt rot=9000 #4"


def test_render_ok_for_verb_r_arc():
    """097: R -> {replace: MotionSegment} (segment_for_arc()) -- reply
    mirrors handleR()'s original "OK arc speed=%d radius=%d" text."""
    ack = envelope_pb2.Ack()
    line = render.render_ok_for_verb("R", ["200", "500"], {}, ack, "7")
    assert line == "OK arc speed=200 radius=500 #7"


def test_render_ok_for_verb_turn():
    """097: TURN -> {segment: MotionSegment} (segment_for_turn()) -- reply
    mirrors handleTURN()'s original "OK turn heading=%d eps=%d" text; eps
    comes from the kv dict (default 0 when absent, matching kvFloat())."""
    ack = envelope_pb2.Ack()
    line = render.render_ok_for_verb("TURN", ["9000"], {"eps": "300"}, ack, None)
    assert line == "OK turn heading=9000 eps=300"
    line_no_eps = render.render_ok_for_verb("TURN", ["9000"], {}, ack, None)
    assert line_no_eps == "OK turn heading=9000 eps=0"


def test_render_ok_for_verb_g_goto():
    """097: G -> {segment: MotionSegment} (segment_for_goto_relative()) --
    reply mirrors handleG()'s original "OK goto x=%d y=%d speed=%d" text."""
    ack = envelope_pb2.Ack()
    line = render.render_ok_for_verb("G", ["300", "400", "150"], {}, ack, "1")
    assert line == "OK goto x=300 y=400 speed=150 #1"


def test_render_ok_for_verb_move_uses_ack_q_and_rem():
    ack = envelope_pb2.Ack(q=3, rem=12.7)
    line = render.render_ok_for_verb("MOVE", ["500", "9000", "9000"], {}, ack, None)
    assert line == "OK move dist=500 dir=9000 fh=9000 q=3 rem=12"


def test_render_ok_for_verb_mover_uses_kv_and_ack_q():
    ack = envelope_pb2.Ack(q=2)
    kv = {"t": "400", "v": "-300", "w": "-4500"}
    line = render.render_ok_for_verb("MOVER", ["0", "0", "0"], kv, ack, None)
    assert line == "OK mover t=400 v=-300 w=-4500 q=2"


def test_render_ok_for_verb_ping_uses_ack_t():
    ack = envelope_pb2.Ack(t=99999)
    assert render.render_ok_for_verb("PING", [], {}, ack, None) == "OK pong t=99999"


def test_render_ok_for_verb_stop_no_body():
    ack = envelope_pb2.Ack()
    assert render.render_ok_for_verb("STOP", [], {}, ack, "2") == "OK stop #2"


def test_render_ok_for_verb_unknown_verb_raises():
    with pytest.raises(ValueError):
        render.render_ok_for_verb("ECHO", [], {}, envelope_pb2.Ack(), None)


def test_evt_arming_verbs_exactly_t_d_rt_move():
    assert render.EVT_ARMING_VERBS == frozenset({"T", "D", "RT", "MOVE"})


# ---------------------------------------------------------------------------
# EVT synthesis (CommandProcessor::emitEvent() GOAL_DONE branch)
# ---------------------------------------------------------------------------


def test_render_evt_done_with_corr_id():
    assert render.render_evt_done("D", "42") == "EVT done D #42 reason=idle"


def test_render_evt_done_no_corr_id():
    assert render.render_evt_done("T", None) == "EVT done T reason=idle"


def test_render_evt_done_custom_reason():
    assert render.render_evt_done("RT", None, reason="cap") == "EVT done RT reason=cap"


# ---------------------------------------------------------------------------
# Telemetry -- render_tlm_line() (tlm_frame.cpp's buildTlmFrame() pre-097-008)
# ---------------------------------------------------------------------------


def test_render_tlm_line_minimal_frame_only_mandatory_fields():
    t = telemetry_pb2.Telemetry(now=1000, mode=planner_pb2.IDLE, seq=1)
    assert render.render_tlm_line(t) == "TLM t=1000 mode=I seq=1"


def test_render_tlm_line_full_frame_field_order_and_truncation():
    # final_heading for RT 9000 (see legacy_translate.segment_for_rt/
    # test_send_rt_produces_a_segment_with_final_heading_only) -- round
    # trips back to 9000 cdeg through the SAME _ANGLE_SCALE constant.
    heading = 1.5707963705062866  # [rad] float32-rounded pi/2
    t = telemetry_pb2.Telemetry(
        now=12345, mode=planner_pb2.STREAMING, seq=7,
        has_enc=True, enc_left=100.9, enc_right=-50.1,
        has_vel=True, vel_left=10.0, vel_right=-5.0,
        has_cmd_vel=True, cmd_vel_left=11.0, cmd_vel_right=-6.0,
        has_pose=True, pose=common_pb2.Pose2D(x=1.0, y=2.0, h=heading),
        has_otos=True, otos=common_pb2.Pose2D(x=3.0, y=4.0, h=heading),
        otos_connected=True,
        has_twist=True, twist=common_pb2.BodyTwist3(v_x=42.0, omega=0.5),
    )
    line = render.render_tlm_line(t)
    assert line == (
        "TLM t=12345 mode=S seq=7 enc=100,-50 vel=10,-5 cmd=11,-6 "
        "pose=1,2,9000 otos=3,4,9000 otosconn=1 twist=42,500")


def test_render_tlm_line_mode_char_map():
    for mode, ch in (
        (planner_pb2.IDLE, "I"), (planner_pb2.STREAMING, "S"),
        (planner_pb2.TIMED, "T"), (planner_pb2.DISTANCE, "D"),
        (planner_pb2.GO_TO, "G"),
    ):
        t = telemetry_pb2.Telemetry(now=0, mode=mode, seq=0)
        assert render.render_tlm_line(t) == f"TLM t=0 mode={ch} seq=0"


def test_render_tlm_line_negative_encoder_truncates_toward_zero():
    # int(-3.7) == -3 in Python, matching C++ static_cast<int> truncation.
    t = telemetry_pb2.Telemetry(now=0, mode=planner_pb2.IDLE, seq=0,
                                has_enc=True, enc_left=-3.7, enc_right=3.7)
    assert render.render_tlm_line(t) == "TLM t=0 mode=I seq=0 enc=-3,3"


# ---------------------------------------------------------------------------
# One-shot TLM verb bench body (handleTlm() pre-097-006)
# ---------------------------------------------------------------------------


def test_render_tlm_one_shot_body_tenths_and_ints():
    t = telemetry_pb2.Telemetry(
        now=555, has_enc=True, enc_left=123.45, enc_right=-67.89,
        has_vel=True, vel_left=10.05, vel_right=-2.0,
        has_cmd_vel=True, cmd_vel_left=5.0, cmd_vel_right=-5.0,
        acc_left=1.9, acc_right=-1.1, active=True,
        conn_left=True, conn_right=False, glitch_left=2, glitch_right=0,
        ts_left=100, ts_right=200)
    body = render.render_tlm_one_shot_body(t)
    # 123.45 as a protobuf (float32) field is actually 123.44999... (float32
    # quantization, same representation the firmware's own float uses) --
    # formatTenths() rounds THAT value, not the float64 literal, hence
    # "123.4" not "123.5".
    assert body == (
        "enc=123.4,-67.9 vel=10.1,-2.0 cmd=5,-5 acc=1,-1 active=1 "
        "conn=1,0 glitch=2,0 ts=100,200 now=555")


def test_render_tlm_one_shot_body_inactive_and_disconnected():
    t = telemetry_pb2.Telemetry(now=0, active=False, conn_left=False, conn_right=False)
    body = render.render_tlm_one_shot_body(t)
    assert "active=0" in body
    assert "conn=0,0" in body


def test_lround_rounds_half_away_from_zero_not_banker():
    # Python's round(0.5) == 0 (banker's); lroundf(0.5) == 1.
    assert render._lround(0.5) == 1
    assert render._lround(-0.5) == -1
    assert render._lround(2.5) == 3  # banker's would give 2


# ---------------------------------------------------------------------------
# Config -- render_cfg_line()/format_config_value() (config_commands.cpp
# pre-097-007: formatConfigKeyFromBb()/kAllKeys/formatFixed())
# ---------------------------------------------------------------------------


def test_all_get_keys_matches_kall_keys_order():
    assert render.ALL_GET_KEYS == (
        "tw", "ml", "mr",
        "pid.kp", "pid.ki", "pid.kff", "pid.iMax", "pid.kaw",
        "rotSlip",
        "ekfQxy", "ekfQtheta", "ekfROtosXy", "ekfROtosTheta",
        "minSpeed", "sTimeout",
    )


def test_format_config_value_int_key_truncates():
    assert render.format_config_value("tw", 128.9) == "128"


def test_format_config_value_fixed3_key():
    assert render.format_config_value("rotSlip", 0.92) == "0.920"
    assert render.format_config_value("ml", 0.4871) == "0.487"


def test_format_config_value_uint_key():
    assert render.format_config_value("sTimeout", 1500.0) == "1500"


def test_format_config_value_negative_fixed3():
    assert render.format_config_value("ekfQxy", -0.5) == "-0.500"


def test_render_cfg_line_full_dump_in_kall_keys_order():
    values = {
        "tw": 128.0, "ml": 0.487, "mr": 0.481,
        "pid.kp": 1.0, "pid.ki": 0.1, "pid.kff": 0.5, "pid.iMax": 2.0, "pid.kaw": 0.02,
        "rotSlip": 0.92,
        "ekfQxy": 0.01, "ekfQtheta": 0.02, "ekfROtosXy": 0.05, "ekfROtosTheta": 0.03,
        "minSpeed": 50.0, "sTimeout": 500.0,
    }
    line = render.render_cfg_line(values, "3")
    assert line == (
        "CFG tw=128 ml=0.487 mr=0.481 pid.kp=1.000 pid.ki=0.100 pid.kff=0.500 "
        "pid.iMax=2.000 pid.kaw=0.020 rotSlip=0.920 ekfQxy=0.010 ekfQtheta=0.020 "
        "ekfROtosXy=0.050 ekfROtosTheta=0.030 minSpeed=50 sTimeout=500 #3")


def test_render_cfg_line_targeted_subset_preserves_requested_order():
    values = {"tw": 128.0, "rotSlip": 0.92}
    line = render.render_cfg_line(values, None, keys=("rotSlip", "tw"))
    assert line == "CFG rotSlip=0.920 tw=128"


def test_render_cfg_line_missing_key_omitted():
    values = {"tw": 128.0}
    line = render.render_cfg_line(values, None, keys=("tw", "rotSlip"))
    assert line == "CFG tw=128"


def test_render_cfg_line_no_values_bare_cfg():
    assert render.render_cfg_line({}, None) == "CFG"
    assert render.render_cfg_line({}, "9") == "CFG #9"


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
