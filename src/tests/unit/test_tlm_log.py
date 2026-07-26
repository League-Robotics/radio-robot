"""src/tests/unit/test_tlm_log.py -- 115-008's own testing plan: a short,
synthetic test confirming `src/tests/bench/tlm_log.py`'s CSV row shape and
column count are stable, plus that its I/O wrapper (`stream_to_csv()`)
actually writes that shape to a real CSV file.

`src/tests/bench/` is "HITL CLI tools, not pytest-collected"
(`tests/CLAUDE.md`), so this test loads `tlm_log.py` directly by file path
via `importlib`, mirroring `test_rig_dev.py`'s own established precedent for
testing a `src/tests/bench/` script's pure logic in isolation.

Frames are hand-built `telemetry_pb2.Telemetry` messages (ticket 008's own
testing-plan-sanctioned alternative to ticket 006's C++-only
`wire_test_codec` helpers), adapted via `TLMFrame.from_pb2()` -- the exact
same adaptation a real or simulated connection's decode path performs, so
`frame_to_row()` is exercised against the real `TLMFrame` shape, not a
hand-rolled stand-in.
"""

from __future__ import annotations

import csv
import importlib.util
import math
import pathlib

import pytest

from robot_radio.robot.pb2 import common_pb2, telemetry_pb2
from robot_radio.robot.protocol import TLMFrame

_BENCH_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "bench" / "tlm_log.py"

_ANGLE_SCALE = 18000.0 / math.pi  # [cdeg/rad] -- matches protocol.py's own kAngleScale mirror


def _load_bench_module():
    spec = importlib.util.spec_from_file_location("tlm_log", _BENCH_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def tlm_log():
    return _load_bench_module()


# ---------------------------------------------------------------------------
# Frame builders
# ---------------------------------------------------------------------------

# Every flags bit set that has a dedicated flag_* column, EXCEPT the ones
# deliberately left clear below (fault_wedge_latch, fault_i2c_nak_timeout,
# fault_malformed_frame, event_boot_ready, fault_move_timeout) so the "full"
# frame exercises both a True and a False decode within the same frame.
# 124-008 (issue §B4): bit 5 (formerly ack_fresh) is RESERVED now -- the
# single "freshest ack" scalar slot is deleted, dropped from this mask.
_FULL_FLAGS = (
    (1 << 0)   # otos_present
    | (1 << 1)  # otos_connected
    | (1 << 2)  # active
    | (1 << 3)  # conn_left
    | (1 << 4)  # conn_right
    | (1 << 6)  # fault_i2c_safety_net
    | (1 << 10)  # event_deadman_expired
    | (1 << 12)  # event_config_applied
    | (1 << 13)  # line_present
    | (1 << 14)  # color_present
)


def _pack_ack(corr_id: int, err: int) -> int:
    """Mirrors telemetry.cpp's own pushAckRing() packed-word format EXACTLY
    (124-008, issue §B4): corr_id<<4 | err."""
    return (corr_id << 4) | err


def _full_telemetry() -> "telemetry_pb2.Telemetry":
    """A frame with every gated field present: otos/line/color present, one
    ack in the ring. 124-008 (issue §B3): position/velocity/x/y/heading/
    v_x/v_y/omega/h are sint32+scale raw wire ints now -- values below are
    chosen to be exactly representable at each field's own declared scale
    (options.proto's (scale) doc comment)."""
    return telemetry_pb2.Telemetry(
        now=12345, seq=42, mode=telemetry_pb2.STREAMING,
        flags=_FULL_FLAGS,
        acks=[_pack_ack(99, 0)],
        enc_left=telemetry_pb2.EncoderReading(position=100, velocity=500, age=200, position_epoch=1),
        enc_right=telemetry_pb2.EncoderReading(position=-100, velocity=-500, age=201, position_epoch=2),
        otos=telemetry_pb2.OtosReading(
            x=10, y=20, heading=500, v_x=150, v_y=-30, omega=20, age=45),
        pose=common_pb2.Pose2D(x=100, y=200, h=300),
        twist=common_pb2.BodyTwist3(v_x=2500, v_y=0, omega=150),
        line=10 | (20 << 8) | (30 << 16) | (40 << 24),
        color=1 | (2 << 8) | (3 << 16) | (4 << 24),
    )


def _minimal_telemetry() -> "telemetry_pb2.Telemetry":
    """A frame with every gate clear: no otos, no line, no color, no acks in
    the ring -- only the always-present fields (enc_left/enc_right/pose/
    twist) are populated."""
    return telemetry_pb2.Telemetry(
        now=1, seq=1, mode=telemetry_pb2.IDLE,
        flags=0,
        enc_left=telemetry_pb2.EncoderReading(position=0, velocity=0, age=0),
        enc_right=telemetry_pb2.EncoderReading(position=0, velocity=0, age=0),
        pose=common_pb2.Pose2D(x=0, y=0, h=0),
        twist=common_pb2.BodyTwist3(v_x=0, v_y=0, omega=0),
    )


# ---------------------------------------------------------------------------
# frame_to_row()
# ---------------------------------------------------------------------------

class TestFrameToRow:
    def test_keys_match_csv_fieldnames_in_order(self, tlm_log):
        frame = TLMFrame.from_pb2(_full_telemetry())
        row = tlm_log.frame_to_row(frame)
        assert list(row.keys()) == list(tlm_log.CSV_FIELDNAMES)

    def test_column_count_stable(self, tlm_log):
        frame = TLMFrame.from_pb2(_full_telemetry())
        row = tlm_log.frame_to_row(frame)
        assert len(row) == len(tlm_log.CSV_FIELDNAMES)

    def test_full_frame_transcribes_every_field(self, tlm_log):
        frame = TLMFrame.from_pb2(_full_telemetry())
        row = tlm_log.frame_to_row(frame)

        assert row["now"] == 12345
        assert row["seq"] == 42
        assert row["mode"] == "S"  # STREAMING -> modeChar() 'S' (_DRIVE_MODE_CHAR mirror)
        assert row["flags"] == _FULL_FLAGS

        assert row["flag_otos_present"] is True
        assert row["flag_otos_connected"] is True
        assert row["flag_active"] is True
        assert row["flag_conn_left"] is True
        assert row["flag_conn_right"] is True
        assert row["flag_fault_i2c_safety_net"] is True
        assert row["flag_fault_wedge_latch"] is False
        assert row["flag_fault_i2c_nak_timeout"] is False
        assert row["flag_fault_malformed_frame"] is False
        assert row["flag_fault_move_timeout"] is False
        assert row["flag_event_deadman_expired"] is True
        assert row["flag_event_boot_ready"] is False
        assert row["flag_event_config_applied"] is True
        assert row["flag_line_present"] is True
        assert row["flag_color_present"] is True

        assert row["acks"] == [(99, 0)]

        assert row["enc_left_position"] == pytest.approx(100.0)
        assert row["enc_left_velocity"] == pytest.approx(50.0)
        assert row["enc_left_age"] == 200
        assert row["enc_left_position_epoch"] == 1
        assert row["enc_right_position"] == pytest.approx(-100.0)
        assert row["enc_right_velocity"] == pytest.approx(-50.0)
        assert row["enc_right_age"] == 201
        assert row["enc_right_position_epoch"] == 2

        assert row["otos_x"] == pytest.approx(10.0)
        assert row["otos_y"] == pytest.approx(20.0)
        assert row["otos_heading"] == pytest.approx(0.5)
        assert row["otos_v_x"] == pytest.approx(15.0)
        assert row["otos_v_y"] == pytest.approx(-3.0)
        assert row["otos_omega"] == pytest.approx(0.2)
        assert row["otos_age"] == 45

        assert row["pose_x"] == 100
        assert row["pose_y"] == 200
        assert row["pose_theta"] == int(0.3 * _ANGLE_SCALE)

        assert row["twist_v_x"] == 250
        assert row["twist_omega"] == 1500  # omega * 1000

        assert (row["line_ch1"], row["line_ch2"], row["line_ch3"], row["line_ch4"]) == (
            10, 20, 30, 40)
        assert (row["color_r"], row["color_g"], row["color_b"], row["color_c"]) == (
            1, 2, 3, 4)

    def test_minimal_frame_blanks_gated_fields(self, tlm_log):
        frame = TLMFrame.from_pb2(_minimal_telemetry())
        row = tlm_log.frame_to_row(frame)

        assert row["mode"] == "I"
        assert row["flag_otos_present"] is False
        assert row["flag_line_present"] is False
        assert row["flag_color_present"] is False

        for key in ("otos_x", "otos_y", "otos_heading", "otos_v_x", "otos_v_y",
                    "otos_omega", "otos_age"):
            assert row[key] is None, key
        for key in ("line_ch1", "line_ch2", "line_ch3", "line_ch4",
                    "color_r", "color_g", "color_b", "color_c"):
            assert row[key] is None, key

        # Always-present fields still populate even with every gate clear.
        assert row["enc_left_position"] == pytest.approx(0.0)
        assert row["pose_x"] == 0
        assert row["twist_v_x"] == 0


# ---------------------------------------------------------------------------
# stream_to_csv()
# ---------------------------------------------------------------------------

class _FakeSource:
    """Returns its canned frames on the FIRST call, `[]` on every call after
    -- deterministic regardless of how many times `stream_to_csv()`'s poll
    loop happens to call it (0 or more times) before its own guaranteed
    final drain, so this test needs no real wall-clock wait."""

    def __init__(self, frames):
        self._pending = list(frames)

    def read_pending_binary_tlm_frames(self):
        frames, self._pending = self._pending, []
        return frames


class TestStreamToCsv:
    def test_writes_header_and_one_row_per_frame(self, tlm_log, tmp_path):
        frames = [
            TLMFrame.from_pb2(_full_telemetry()),
            TLMFrame.from_pb2(_minimal_telemetry()),
        ]
        source = _FakeSource(frames)
        csv_path = tmp_path / "capture.csv"

        row_count = tlm_log.stream_to_csv(source, csv_path, duration=0.0)

        assert row_count == len(frames)
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            assert reader.fieldnames == list(tlm_log.CSV_FIELDNAMES)
            rows = list(reader)
        assert len(rows) == len(frames)
        assert rows[0]["now"] == "12345"
        assert rows[0]["seq"] == "42"
        assert rows[1]["now"] == "1"
        # Blank (not the string "None") for a gated field absent on frame 2.
        assert rows[1]["otos_x"] == ""

    def test_creates_parent_directory(self, tlm_log, tmp_path):
        source = _FakeSource([TLMFrame.from_pb2(_minimal_telemetry())])
        csv_path = tmp_path / "nested" / "out" / "capture.csv"

        row_count = tlm_log.stream_to_csv(source, csv_path, duration=0.0)

        assert row_count == 1
        assert csv_path.is_file()
