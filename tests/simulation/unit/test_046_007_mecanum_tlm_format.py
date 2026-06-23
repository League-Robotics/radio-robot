"""test_046_007_mecanum_tlm_format.py — Host-side TLM parser tests for mecanum formats.

Sprint 046, Ticket 007.

Tests verify:
- Differential TLM (vel=2-field, twist=2-field) parses identically to pre-sprint.
- Mecanum TLM twist=3-field (vx, vy, omega_mrad) is parsed correctly.
- Mecanum TLM vel=4-field (FR, FL, BR, BL) is parsed correctly.
- Back-compatibility: the parser does not crash on either robot type.
- The parser returns None for both vel= and twist= on malformed values.
"""

from __future__ import annotations

import pytest

from robot_radio.robot.protocol import TLMFrame, parse_tlm


# ---------------------------------------------------------------------------
# Differential TLM — byte-identical to pre-sprint (regression guard)
# ---------------------------------------------------------------------------

class TestDifferentialTLMUnchanged:
    """Differential TLM format is unchanged by the mecanum sprint."""

    def test_differential_vel_2field(self) -> None:
        """vel=vL,vR (2 values) still parses as (vL, vR) tuple."""
        frame = parse_tlm("TLM t=100 vel=200,195")
        assert frame is not None
        assert frame.vel == (200, 195)

    def test_differential_twist_2field(self) -> None:
        """twist=v,omega (2 values) still parses as (v, omega) tuple."""
        frame = parse_tlm("TLM t=100 twist=250,314")
        assert frame is not None
        assert frame.twist == (250, 314)

    def test_differential_twist_negative_omega(self) -> None:
        """Negative omega in twist= is parsed correctly."""
        frame = parse_tlm("TLM t=200 twist=100,-500")
        assert frame is not None
        assert frame.twist == (100, -500)

    def test_differential_full_frame(self) -> None:
        """Full differential TLM frame with vel=2 and twist=2 parses completely."""
        line = ("TLM t=12345 mode=V seq=3 enc=120,118 pose=300,50,900 "
                "vel=198,202 twist=200,0 ekf_rej=0")
        frame = parse_tlm(line)
        assert frame is not None
        assert frame.t == 12345
        assert frame.mode == "V"
        assert frame.enc == (120, 118)
        assert frame.vel == (198, 202)
        assert frame.twist == (200, 0)
        assert frame.ekf_rej == 0

    def test_differential_vel_3value_still_ignored(self) -> None:
        """3-value vel= (neither differential nor mecanum) is not parsed."""
        frame = parse_tlm("TLM t=100 vel=200,0,15")
        assert frame is not None
        assert frame.vel is None


# ---------------------------------------------------------------------------
# Mecanum TLM — new formats introduced in sprint 046, ticket 007
# ---------------------------------------------------------------------------

class TestMecanumTLMTwist:
    """Mecanum twist= field: 3-tuple (vx_mmps, vy_mmps, omega_mradps)."""

    def test_mecanum_twist_3field_zero_vy(self) -> None:
        """Mecanum twist= with vy=0 (pure forward motion)."""
        frame = parse_tlm("TLM t=100 twist=250,0,0")
        assert frame is not None
        assert frame.twist == (250, 0, 0)

    def test_mecanum_twist_3field_positive_vy(self) -> None:
        """Mecanum twist= with positive vy (lateral motion)."""
        frame = parse_tlm("TLM t=200 twist=0,150,0")
        assert frame is not None
        assert frame.twist == (0, 150, 0)

    def test_mecanum_twist_3field_negative_vy(self) -> None:
        """Mecanum twist= with negative vy (lateral motion opposite direction)."""
        frame = parse_tlm("TLM t=300 twist=100,-75,314")
        assert frame is not None
        assert frame.twist == (100, -75, 314)

    def test_mecanum_twist_3field_all_nonzero(self) -> None:
        """Mecanum twist= with all three components non-zero (strafe + turn)."""
        frame = parse_tlm("TLM t=400 twist=200,-120,500")
        assert frame is not None
        assert frame.twist == (200, -120, 500)

    def test_mecanum_twist_tuple_length(self) -> None:
        """Mecanum twist= produces a 3-element tuple."""
        frame = parse_tlm("TLM t=100 twist=100,50,200")
        assert frame is not None
        assert len(frame.twist) == 3  # type: ignore[arg-type]

    def test_mecanum_twist_vy_index(self) -> None:
        """vy is the second element (index 1) of the mecanum twist tuple."""
        frame = parse_tlm("TLM t=100 twist=300,-42,628")
        assert frame is not None
        assert frame.twist is not None
        vx, vy, omega = frame.twist
        assert vx == 300
        assert vy == -42
        assert omega == 628


class TestMecanumTLMVel:
    """Mecanum vel= field: 4-tuple (vFR, vFL, vBR, vBL) all in mm/s."""

    def test_mecanum_vel_4field_all_same(self) -> None:
        """4-wheel vel= with equal speeds (pure forward)."""
        frame = parse_tlm("TLM t=100 vel=200,200,200,200")
        assert frame is not None
        assert frame.vel == (200, 200, 200, 200)

    def test_mecanum_vel_4field_zero(self) -> None:
        """4-wheel vel= all zero (stopped)."""
        frame = parse_tlm("TLM t=100 vel=0,0,0,0")
        assert frame is not None
        assert frame.vel == (0, 0, 0, 0)

    def test_mecanum_vel_4field_strafe_pattern(self) -> None:
        """4-wheel vel= with mecanum strafe pattern (FR/BL vs FL/BR)."""
        # Pure right strafe: FR=-, FL=+, BR=+, BL=-
        frame = parse_tlm("TLM t=200 vel=-150,150,150,-150")
        assert frame is not None
        assert frame.vel == (-150, 150, 150, -150)

    def test_mecanum_vel_4field_order(self) -> None:
        """vel= 4-field order is FR, FL, BR, BL."""
        frame = parse_tlm("TLM t=100 vel=100,110,105,115")
        assert frame is not None
        assert frame.vel is not None
        vFR, vFL, vBR, vBL = frame.vel
        assert vFR == 100
        assert vFL == 110
        assert vBR == 105
        assert vBL == 115

    def test_mecanum_vel_tuple_length(self) -> None:
        """Mecanum vel= produces a 4-element tuple."""
        frame = parse_tlm("TLM t=100 vel=100,110,105,115")
        assert frame is not None
        assert len(frame.vel) == 4  # type: ignore[arg-type]


class TestMecanumTLMFullFrame:
    """Full mecanum TLM frame with both twist=3 and vel=4."""

    def test_mecanum_full_frame_snap_format(self) -> None:
        """A complete mecanum SNAP reply parses all mecanum fields correctly."""
        line = ("TLM t=5000 mode=V seq=10 enc=250,248 pose=500,20,100 "
                "vel=198,-202,200,-199 twist=250,-45,628 ekf_rej=0")
        frame = parse_tlm(line)
        assert frame is not None
        assert frame.t == 5000
        assert frame.mode == "V"
        assert frame.seq == 10
        assert frame.enc == (250, 248)
        assert frame.pose == (500, 20, 100)
        # 4-field vel: FR, FL, BR, BL
        assert frame.vel == (198, -202, 200, -199)
        # 3-field twist: vx, vy, omega_mrad
        assert frame.twist == (250, -45, 628)
        assert frame.ekf_rej == 0

    def test_mecanum_idle_frame_all_zero(self) -> None:
        """Idle mecanum frame: vel=0,0,0,0 and twist=0,0,0."""
        line = "TLM t=100 mode=I seq=0 enc=0,0 pose=0,0,0 vel=0,0,0,0 twist=0,0,0 ekf_rej=0"
        frame = parse_tlm(line)
        assert frame is not None
        assert frame.vel == (0, 0, 0, 0)
        assert frame.twist == (0, 0, 0)


# ---------------------------------------------------------------------------
# Back-compatibility: parser does not crash on either robot type
# ---------------------------------------------------------------------------

class TestParserBackCompat:
    """The parser handles both differential and mecanum formats without error."""

    def test_differential_vel_not_confused_with_mecanum(self) -> None:
        """Differential 2-field vel= is distinct from mecanum 4-field vel=."""
        diff = parse_tlm("TLM t=100 vel=200,195")
        mec = parse_tlm("TLM t=100 vel=200,195,198,202")
        assert diff is not None and diff.vel == (200, 195)
        assert mec is not None and mec.vel == (200, 195, 198, 202)

    def test_differential_twist_not_confused_with_mecanum(self) -> None:
        """Differential 2-field twist= is distinct from mecanum 3-field twist=."""
        diff = parse_tlm("TLM t=100 twist=250,314")
        mec = parse_tlm("TLM t=100 twist=250,-45,314")
        assert diff is not None and diff.twist == (250, 314)
        assert mec is not None and mec.twist == (250, -45, 314)

    def test_frame_without_vel_or_twist(self) -> None:
        """TLM without vel= or twist= still parses (both stay None)."""
        frame = parse_tlm("TLM t=100 enc=10,10 pose=0,0,0")
        assert frame is not None
        assert frame.vel is None
        assert frame.twist is None

    def test_mecanum_vel_malformed_stays_none(self) -> None:
        """Malformed mecanum vel= (non-integer) leaves vel=None."""
        frame = parse_tlm("TLM t=100 vel=200,195,bad,202")
        assert frame is not None
        assert frame.vel is None

    def test_mecanum_twist_malformed_stays_none(self) -> None:
        """Malformed mecanum twist= (non-integer) leaves twist=None."""
        frame = parse_tlm("TLM t=100 twist=250,bad,314")
        assert frame is not None
        assert frame.twist is None
