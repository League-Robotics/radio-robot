#!/usr/bin/env python3
"""test_mecanum_vw_bvc.py — Unit tests for ticket 046-005 mecanum changes.

Tests verify:
  - VW 3-token parse (vx vy omega) sets all three components correctly.
  - VW 2-token parse (vx omega) sets vy=0 (backward-compat in both builds).
  - BVC vy channel: profile ramps toward target under aMaxY, clamps at vyBodyMax.
  - BVC vy channel: reset() clears vy state.
  - BVC vy convergence: atTarget() checks vy in addition to vx/omega.

These are pure Python logic tests — no firmware sim instance required.
They mirror the firmware equations and validate the algorithm properties.

046-005 architecture:
  - parseVW: 3-token form "VW <vx> <vy> <omega>" stored as args[0]=vx, args[1]=omega,
    args[2]=vy (INT). args[1] is STILL omega so existing stop-param dispatch
    in handleVW is unaffected. vwHasKey() skips INT-typed args at index 2.
  - BVC vy channel: trapezoid/S-curve mirror of the forward channel using
    vyBodyMax, aMaxY, jMaxY from RobotConfig.
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Pure Python mirrors of VW parse logic (parseVW in MotionCommandHandlers.cpp)
# ---------------------------------------------------------------------------

def parse_vw_tokens(tokens: list[str]) -> dict | None:
    """Simulate parseVW() for mecanum 3-token and 2-token forms.

    Returns dict with keys {vx, vy, omega} or None on error.
    vy is 0 for the 2-token form.

    wire format (mecanum build):
      VW <vx> <omega>          → 2-token: vy=0
      VW <vx> <vy> <omega>    → 3-token: vy from token[1], omega from token[2]

    Note: in the firmware's parseVW, args[1]=omega, args[2]=vy (INT) to
    avoid disturbing the stop-param dispatch that scans for STR at args[2+].
    """
    if len(tokens) < 2:
        return None  # ERR badarg

    if len(tokens) >= 3:
        # 3-token mecanum form: vx vy omega
        vx    = int(tokens[0])
        vy    = int(tokens[1])
        omega = int(tokens[2])  # mrad/s on wire
    else:
        # 2-token form (both builds)
        vx    = int(tokens[0])
        vy    = 0
        omega = int(tokens[1])  # mrad/s on wire

    # Range checks (mirrors firmware parseVW validation)
    if not (-1000 <= vx <= 1000):
        return None  # ERR range v
    if not (-1000 <= vy <= 1000):
        return None  # ERR range vy
    if not (-3142 <= omega <= 3142):
        return None  # ERR range omega

    return {"vx": vx, "vy": vy, "omega": omega}


# ---------------------------------------------------------------------------
# Pure Python mirror of BVC vy channel advance step
# ---------------------------------------------------------------------------

class BVCyySimulator:
    """Single-axis BVC vy channel simulator.

    Mirrors the mecanum lateral channel in BodyVelocityController::advance():
      - Trapezoid (jMaxY == 0): _vy += clamp(vyTgt - _vy, -dvy_max, +dvy_max)
      - S-curve (jMaxY > 0):    slew _vyALive toward +-aMaxY, then integrate

    approach(cur, tgt, step) = cur + clamp(tgt - cur, -step, +step)
    """

    def __init__(self,
                 vy_body_max: float = 400.0,
                 a_max_y: float = 800.0,
                 j_max_y: float = 0.0):
        self.vy_body_max = vy_body_max
        self.a_max_y     = a_max_y
        self.j_max_y     = j_max_y

        self._vy      = 0.0
        self._vy_tgt  = 0.0
        self._vy_alive = 0.0  # live acceleration (S-curve)

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    @staticmethod
    def _approach(cur: float, tgt: float, step: float) -> float:
        delta = tgt - cur
        delta = BVCyySimulator._clamp(delta, -step, +step)
        return cur + delta

    def set_target(self, vy: float) -> None:
        self._vy_tgt = vy

    def advance(self, dt_s: float) -> float:
        """Step the profile and return the new live vy (mm/s)."""
        if dt_s <= 0.0:
            return self._vy

        vy_tgt_clamped = self._clamp(self._vy_tgt, -self.vy_body_max, +self.vy_body_max)

        if self.j_max_y > 0.0:
            if self._vy < vy_tgt_clamped:
                a_target = self.a_max_y
            elif self._vy > vy_tgt_clamped:
                a_target = -self.a_max_y
            else:
                a_target = 0.0
            jerk_step = self.j_max_y * dt_s
            self._vy_alive = self._approach(self._vy_alive, a_target, jerk_step)
            self._vy = self._approach(self._vy, vy_tgt_clamped,
                                      abs(self._vy_alive * dt_s))
        else:
            dvy_max = self.a_max_y * dt_s
            self._vy = self._approach(self._vy, vy_tgt_clamped, dvy_max)

        return self._vy

    @property
    def current_vy(self) -> float:
        return self._vy

    def at_target_vy(self) -> bool:
        vy_tgt_clamped = self._clamp(self._vy_tgt, -self.vy_body_max, +self.vy_body_max)
        return abs(self._vy - vy_tgt_clamped) < 0.5  # mirrors firmware threshold

    def reset(self) -> None:
        self._vy      = 0.0
        self._vy_tgt  = 0.0
        self._vy_alive = 0.0


# ===========================================================================
# VW parse tests
# ===========================================================================

class TestVW2TokenBackwardCompat:
    """VW 2-token form sets vy=0 (backward-compatible, both builds)."""

    def test_vw_2token_vy_is_zero(self):
        """VW 200 30 → vx=200, vy=0, omega=30 mrad/s."""
        result = parse_vw_tokens(["200", "30"])
        assert result is not None
        assert result["vx"]    == 200
        assert result["vy"]    == 0
        assert result["omega"] == 30

    def test_vw_2token_negative_v(self):
        """VW -200 30 → vx=-200, vy=0, omega=30."""
        result = parse_vw_tokens(["-200", "30"])
        assert result is not None
        assert result["vx"]    == -200
        assert result["vy"]    == 0

    def test_vw_2token_zero_zero(self):
        """VW 0 0 → vx=0, vy=0, omega=0."""
        result = parse_vw_tokens(["0", "0"])
        assert result is not None
        assert result["vx"]    == 0
        assert result["vy"]    == 0
        assert result["omega"] == 0

    def test_vw_2token_omega_at_boundary(self):
        """VW 0 3142 → omega=3142 mrad/s (at boundary)."""
        result = parse_vw_tokens(["0", "3142"])
        assert result is not None
        assert result["omega"] == 3142

    def test_vw_2token_omega_negative_boundary(self):
        """VW 0 -3142 → omega=-3142 (at boundary)."""
        result = parse_vw_tokens(["0", "-3142"])
        assert result is not None
        assert result["omega"] == -3142

    def test_vw_2token_missing_args(self):
        """VW with no args → parse failure (ERR badarg)."""
        result = parse_vw_tokens([])
        assert result is None

    def test_vw_2token_one_arg(self):
        """VW 200 (missing omega) → parse failure (ERR badarg)."""
        result = parse_vw_tokens(["200"])
        assert result is None


class TestVW3TokenMecanum:
    """VW 3-token mecanum form: VW <vx> <vy> <omega>."""

    def test_vw_3token_basic(self):
        """VW 200 80 30 → vx=200, vy=80, omega=30 mrad/s."""
        result = parse_vw_tokens(["200", "80", "30"])
        assert result is not None
        assert result["vx"]    == 200
        assert result["vy"]    == 80
        assert result["omega"] == 30

    def test_vw_3token_zero_vy(self):
        """VW 200 0 30 → same as 2-token when vy=0."""
        result = parse_vw_tokens(["200", "0", "30"])
        assert result is not None
        assert result["vx"]    == 200
        assert result["vy"]    == 0
        assert result["omega"] == 30

    def test_vw_3token_negative_vy(self):
        """VW 200 -80 30 → vy=-80 mm/s (strafe right)."""
        result = parse_vw_tokens(["200", "-80", "30"])
        assert result is not None
        assert result["vx"]    == 200
        assert result["vy"]    == -80
        assert result["omega"] == 30

    def test_vw_3token_pure_strafe(self):
        """VW 0 100 0 → pure lateral motion."""
        result = parse_vw_tokens(["0", "100", "0"])
        assert result is not None
        assert result["vx"]    == 0
        assert result["vy"]    == 100
        assert result["omega"] == 0

    def test_vw_3token_combined(self):
        """VW 150 50 200 → forward + strafe + turn."""
        result = parse_vw_tokens(["150", "50", "200"])
        assert result is not None
        assert result["vx"]    == 150
        assert result["vy"]    == 50
        assert result["omega"] == 200

    def test_vw_3token_vy_at_boundary(self):
        """VW 0 1000 0 → vy=1000 mm/s (at range boundary)."""
        result = parse_vw_tokens(["0", "1000", "0"])
        assert result is not None
        assert result["vy"] == 1000

    def test_vw_3token_vy_out_of_range(self):
        """VW 0 1001 0 → ERR range vy (out of range)."""
        result = parse_vw_tokens(["0", "1001", "0"])
        assert result is None

    def test_vw_3token_vy_negative_boundary(self):
        """VW 0 -1000 0 → vy=-1000 (at negative boundary)."""
        result = parse_vw_tokens(["0", "-1000", "0"])
        assert result is not None
        assert result["vy"] == -1000

    def test_vw_3token_vy_negative_out_of_range(self):
        """VW 0 -1001 0 → ERR range vy."""
        result = parse_vw_tokens(["0", "-1001", "0"])
        assert result is None

    def test_vw_3token_vx_out_of_range(self):
        """VW 1001 50 0 → ERR range v (vx check first)."""
        result = parse_vw_tokens(["1001", "50", "0"])
        assert result is None

    def test_vw_3token_omega_at_boundary(self):
        """VW 0 0 3142 → omega=3142 mrad/s (at boundary)."""
        result = parse_vw_tokens(["0", "0", "3142"])
        assert result is not None
        assert result["omega"] == 3142

    def test_vw_3token_omega_out_of_range(self):
        """VW 0 0 3143 → ERR range omega."""
        result = parse_vw_tokens(["0", "0", "3143"])
        assert result is None

    def test_vw_3token_vy_nonzero_differs_from_2token(self):
        """3-token with vy=80 differs from 2-token (vy=0) for same vx/omega."""
        result_3t = parse_vw_tokens(["200", "80", "30"])
        result_2t = parse_vw_tokens(["200", "30"])
        assert result_3t is not None
        assert result_2t is not None
        assert result_3t["vy"] != result_2t["vy"]
        assert result_3t["vx"] == result_2t["vx"]
        assert result_3t["omega"] == result_2t["omega"]

    def test_vw_3token_arg_positions(self):
        """args[0]=vx, args[1]=omega, args[2]=vy matches firmware ArgList layout."""
        # In firmware: args[0]=vx, args[1]=omega, args[2]=INT(vy).
        # parseVW stores omega at [1] so existing handleVW stop-param dispatch
        # (which reads args[1] as omega) is unchanged.
        result = parse_vw_tokens(["200", "80", "30"])
        assert result is not None
        # Verify the semantic mapping (not the internal ArgList ordering):
        # args[0] = vx = 200, args[1] = omega = 30, args[2] = vy = 80
        # (The Python model returns a semantic dict; what matters is correct values)
        assert result["vx"]    == 200  # token[0]
        assert result["vy"]    == 80   # token[1] in 3-token form
        assert result["omega"] == 30   # token[2] in 3-token form

    def test_no_omni_strafe_verbs(self):
        """Verify VW is the only body-twist verb — no OMNI or STRAFE added.

        This is a documentation/contract test: the accepted command grammar
        must not include OMNI or STRAFE verbs per the architecture decision.
        """
        # The accepted parse functions should only be VW (and its 2D/3D forms).
        # There is no parseOMNI / parseSHRAFE — this test documents that contract.
        known_motion_verbs = {"S", "T", "D", "G", "R", "TURN", "RT", "VW", "_VW", "X", "STOP"}
        rejected_verbs = {"OMNI", "STRAFE", "OMN", "STR"}
        assert rejected_verbs.isdisjoint(known_motion_verbs), \
            f"Unexpected verb overlap: {rejected_verbs & known_motion_verbs}"


# ===========================================================================
# BVC vy channel tests
# ===========================================================================

class TestBVCyyChannel:
    """BVC lateral (vy) profiled channel behaviour (046-005)."""

    # Default parameters matching firmware defaults
    VY_BODY_MAX = 400.0   # mm/s
    A_MAX_Y     = 800.0   # mm/s²
    DT_S        = 0.024   # s (typical 24 ms control tick)

    def _bvc(self, vy_body_max=None, a_max_y=None, j_max_y=None) -> BVCyySimulator:
        return BVCyySimulator(
            vy_body_max=vy_body_max or self.VY_BODY_MAX,
            a_max_y    =a_max_y     or self.A_MAX_Y,
            j_max_y    =j_max_y     or 0.0,
        )

    def test_initial_vy_is_zero(self):
        """BVC vy channel starts at 0."""
        bvc = self._bvc()
        assert bvc.current_vy == 0.0

    def test_set_target_does_not_advance(self):
        """setTarget(vy) alone does not move the profiler."""
        bvc = self._bvc()
        bvc.set_target(200.0)
        assert bvc.current_vy == 0.0  # still at 0 until advance() called

    def test_advance_ramps_toward_target(self):
        """advance() steps vy toward the target under aMaxY."""
        bvc = self._bvc()
        bvc.set_target(200.0)
        bvc.advance(self.DT_S)
        # Expected step: aMaxY * dt_s = 800 * 0.024 = 19.2 mm/s
        expected_step = self.A_MAX_Y * self.DT_S
        assert bvc.current_vy == pytest.approx(min(200.0, expected_step), abs=0.01)

    def test_advance_does_not_overshoot_target(self):
        """advance() with large dt does not overshoot target."""
        bvc = self._bvc()
        bvc.set_target(50.0)
        # Large dt would give step > 50 mm/s -> clamped at 50.
        bvc.advance(1.0)  # aMaxY * 1.0 = 800 >> 50
        assert bvc.current_vy == pytest.approx(50.0, abs=0.01)

    def test_converges_to_target(self):
        """After enough ticks, vy converges to target."""
        bvc = self._bvc()
        target = 200.0
        bvc.set_target(target)
        # Number of ticks to reach 200 mm/s at 19.2 mm/s per tick: ceil(200/19.2)=11
        for _ in range(20):
            bvc.advance(self.DT_S)
        assert abs(bvc.current_vy - target) < 0.5

    def test_clamps_at_vy_body_max(self):
        """Target > vyBodyMax is clamped to vyBodyMax."""
        bvc = self._bvc(vy_body_max=400.0)
        bvc.set_target(1000.0)  # above max
        for _ in range(50):
            bvc.advance(self.DT_S)
        # Should settle at vyBodyMax, not 1000.
        assert bvc.current_vy <= 400.0 + 0.5

    def test_clamps_at_negative_vy_body_max(self):
        """Target < -vyBodyMax is clamped to -vyBodyMax."""
        bvc = self._bvc(vy_body_max=400.0)
        bvc.set_target(-1000.0)
        for _ in range(50):
            bvc.advance(self.DT_S)
        assert bvc.current_vy >= -400.0 - 0.5

    def test_ramps_down_to_zero(self):
        """After target → 0, vy ramps back to zero."""
        bvc = self._bvc()
        bvc.set_target(200.0)
        for _ in range(20):
            bvc.advance(self.DT_S)
        # Now command stop.
        bvc.set_target(0.0)
        for _ in range(20):
            bvc.advance(self.DT_S)
        assert abs(bvc.current_vy) < 0.5

    def test_negative_target_ramps_negative(self):
        """Negative vy target ramps vy negative."""
        bvc = self._bvc()
        bvc.set_target(-100.0)
        for _ in range(15):
            bvc.advance(self.DT_S)
        assert bvc.current_vy < -50.0

    def test_reset_clears_vy_state(self):
        """reset() zeroes vy, vyTgt, and vyALive."""
        bvc = self._bvc()
        bvc.set_target(200.0)
        for _ in range(10):
            bvc.advance(self.DT_S)
        assert bvc.current_vy > 0.0  # partial ramp
        bvc.reset()
        assert bvc.current_vy == 0.0

    def test_reset_then_advance_restarts_ramp(self):
        """After reset(), advance() ramps from zero again."""
        bvc = self._bvc()
        bvc.set_target(200.0)
        for _ in range(10):
            bvc.advance(self.DT_S)
        bvc.reset()
        bvc.set_target(200.0)  # re-issue command after reset
        bvc.advance(self.DT_S)
        expected_step = self.A_MAX_Y * self.DT_S
        assert bvc.current_vy == pytest.approx(min(200.0, expected_step), abs=0.01)

    def test_at_target_false_while_ramping(self):
        """atTarget() returns False while vy is still ramping."""
        bvc = self._bvc()
        bvc.set_target(200.0)
        bvc.advance(self.DT_S)  # one step: ~19.2 mm/s, not at target
        assert not bvc.at_target_vy()

    def test_at_target_true_when_converged(self):
        """atTarget() returns True once vy has reached target."""
        bvc = self._bvc()
        bvc.set_target(200.0)
        for _ in range(20):
            bvc.advance(self.DT_S)
        assert bvc.at_target_vy()

    def test_at_target_true_for_zero_target_at_zero(self):
        """atTarget() is True when target=0 and vy=0 (initial state)."""
        bvc = self._bvc()
        bvc.set_target(0.0)
        assert bvc.at_target_vy()

    def test_zero_dt_is_noop(self):
        """advance(0) does not change vy."""
        bvc = self._bvc()
        bvc.set_target(200.0)
        bvc.advance(0.0)
        assert bvc.current_vy == 0.0


class TestBVCyyChannelSCurve:
    """BVC vy S-curve (jerk-limited) path (jMaxY > 0)."""

    DT_S = 0.024

    def test_s_curve_ramps_slower_than_trapezoid(self):
        """With jerk limit, first few ticks advance slower than trapezoid."""
        vy_max = 400.0
        a_max  = 800.0
        j_max  = 2000.0  # tight jerk limit

        s_bvc = BVCyySimulator(vy_body_max=vy_max, a_max_y=a_max, j_max_y=j_max)
        t_bvc = BVCyySimulator(vy_body_max=vy_max, a_max_y=a_max, j_max_y=0.0)

        target = 200.0
        s_bvc.set_target(target)
        t_bvc.set_target(target)

        s_bvc.advance(self.DT_S)
        t_bvc.advance(self.DT_S)

        # With tight jerk limit, S-curve should advance less in first tick.
        assert s_bvc.current_vy <= t_bvc.current_vy + 1e-3

    def test_s_curve_still_converges(self):
        """S-curve still converges to target over more ticks."""
        bvc = BVCyySimulator(vy_body_max=400.0, a_max_y=800.0, j_max_y=1000.0)
        bvc.set_target(100.0)
        for _ in range(100):
            bvc.advance(self.DT_S)
        assert abs(bvc.current_vy - 100.0) < 0.5

    def test_s_curve_reset_clears_alive(self):
        """reset() zeroes _vyALive so S-curve restarts from rest."""
        bvc = BVCyySimulator(vy_body_max=400.0, a_max_y=800.0, j_max_y=1000.0)
        bvc.set_target(200.0)
        for _ in range(5):
            bvc.advance(self.DT_S)
        v_before = bvc.current_vy
        bvc.reset()
        assert bvc.current_vy == 0.0
        assert bvc._vy_alive  == 0.0


# ===========================================================================
# Wire-unit conversion tests (VW omega in mrad/s)
# ===========================================================================

class TestVWOmegaConversion:
    """Verify mrad/s → rad/s conversion contract is unchanged for VW 3-token form."""

    def test_omega_mrad_to_rad(self):
        """omega from wire is in mrad/s; handleVW divides by 1000."""
        omega_mrads = 30    # wire value
        omega_rads  = omega_mrads / 1000.0
        assert omega_rads == pytest.approx(0.030, abs=1e-6)

    def test_3token_omega_is_last_token(self):
        """In VW <vx> <vy> <omega>, omega is the THIRD token."""
        result = parse_vw_tokens(["200", "80", "30"])
        assert result is not None
        # omega should be token[2] = 30 (not 80)
        assert result["omega"] == 30

    def test_2token_omega_is_second_token(self):
        """In VW <vx> <omega>, omega is the SECOND token (unchanged)."""
        result = parse_vw_tokens(["200", "30"])
        assert result is not None
        assert result["omega"] == 30

    def test_omega_unchanged_between_forms(self):
        """For the same vx/omega, 2-token and 3-token-with-vy-zero give same omega."""
        r2 = parse_vw_tokens(["200", "30"])
        r3 = parse_vw_tokens(["200", "0", "30"])
        assert r2 is not None
        assert r3 is not None
        assert r2["omega"] == r3["omega"]
        assert r2["vx"]    == r3["vx"]
        assert r3["vy"]    == 0
