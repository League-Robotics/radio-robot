"""test_simset_profile_chunking.py — every Sim Errors knob must actually land.

Regression for the silent SIMSET truncation found 2026-07-02 while chasing
the TestGUI Tour-1 geometry failure: the firmware command tokenizer packs
arguments into a fixed ``ArgList`` of ``MAX_ARGS = 10`` slots
(source/types/CommandTypes.h) and SILENTLY DROPS every key=value pair past
the tenth — while still replying ``OK`` (echoing only what fit).
``SimTransport._apply_profile_to_sim`` used to send the full 15-knob Sim
Errors profile as ONE ``SIMSET`` line, so its last five pairs
(``motorOffsetL/R``, ``trackwidthMm``, ``encNoiseL/R``) never reached the
plant: the GUI's motor-offset, trackwidth, and encoder-noise knobs were dead
on Apply.  Observed live: the operator set trackwidth 127.0 in the panel,
the reply was OK, and ``SIMGET trackwidthMm`` still read 128.000.

The fix chunks the profile into <= ``_SIMSET_MAX_PAIRS_PER_LINE`` (8) pairs
per wire line.  These tests drive the REAL chunking code against the REAL
firmware sim and read every value back via ``SIMGET``.
"""
from __future__ import annotations

import pytest

from robot_radio.testgui import sim_prefs
from robot_radio.testgui.transport import (
    SimTransport,
    _SIMSET_MAX_PAIRS_PER_LINE,
)

# Distinct, non-default value for every profile knob so a dropped pair is
# unambiguous on readback (defaults could false-pass).
_DISTINCT_PROFILE: dict = {
    "encoder_noise_mm": 0.75,
    "slip_turn_extra": 0.13,       # legacy set_field_profile path, no SIMGET
    "otos_linear_noise": 0.031,
    "otos_yaw_noise": 0.032,
    "enc_scale_err_l": 0.011,
    "enc_scale_err_r": 0.012,
    "otos_lin_scale_err": 0.021,
    "otos_ang_scale_err": 0.022,
    "otos_lin_drift_mms": 1.5,
    "otos_yaw_drift_degs": 2.5,
    "body_rot_scrub": 0.91,
    "body_lin_scrub": 0.92,
    "motor_offset_l": 1.05,
    "motor_offset_r": 0.95,
    "trackwidth_mm": 131.0,
}


def _simget(sim, wire_key: str) -> float:
    """Read one SIMGET value back; fail loudly on an unexpected reply."""
    reply = sim.send_command(f"SIMGET {wire_key}").strip()
    # Expected shape: "SIMCFG <key>=<value>"
    assert f"{wire_key}=" in reply, f"SIMGET {wire_key} -> {reply!r}"
    return float(reply.split(f"{wire_key}=")[1].split()[0])


def test_apply_profile_lands_every_simset_knob(sim):
    """_apply_profile_to_sim must apply ALL wire-mapped knobs, including the
    ones past the firmware's 10-arg line cap (motor offsets, trackwidth,
    encoder noise)."""
    transport = SimTransport()  # not connected; used only for the apply logic
    transport._apply_profile_to_sim(sim, dict(_DISTINCT_PROFILE))

    expected_by_wire_key = {
        wire_key: _DISTINCT_PROFILE[key]
        for key, wire_key in sim_prefs.PROFILE_TO_SIMSET_KEY.items()
    }
    expected_by_wire_key["encNoiseL"] = _DISTINCT_PROFILE["encoder_noise_mm"]
    expected_by_wire_key["encNoiseR"] = _DISTINCT_PROFILE["encoder_noise_mm"]

    mismatches = []
    for wire_key, expected in expected_by_wire_key.items():
        actual = _simget(sim, wire_key)
        if actual != pytest.approx(expected, abs=1e-4):
            mismatches.append(f"{wire_key}: sent {expected}, read back {actual}")
    assert not mismatches, (
        "SIMSET knobs did not land (silent truncation regression?):\n  "
        + "\n  ".join(mismatches)
    )


def test_firmware_line_cap_canary(sim):
    """Document the firmware behavior the chunking works around: kv pairs
    past the ArgList MAX_ARGS=10 cap are silently dropped from ONE line
    (the reply is still OK).

    If this canary ever fails, the firmware now accepts more pairs per line
    — revisit ``_SIMSET_MAX_PAIRS_PER_LINE`` (and consider making the
    firmware reply ERR instead of silently truncating).
    """
    # 10 filler pairs (repeating a benign key), then the discriminator.
    filler = " ".join("otosLinNoise=0.0" for _ in range(10))
    reply = sim.send_command(f"SIMSET {filler} trackwidthMm=99.0")
    assert "OK" in reply.upper()  # the firmware does NOT reject the overflow
    assert _simget(sim, "trackwidthMm") != pytest.approx(99.0), (
        "firmware applied the 11th kv pair — MAX_ARGS cap changed; revisit "
        "_SIMSET_MAX_PAIRS_PER_LINE and this canary"
    )


def test_chunk_size_stays_under_firmware_cap():
    """The chunk budget must stay under the firmware ArgList MAX_ARGS=10."""
    assert _SIMSET_MAX_PAIRS_PER_LINE <= 10
