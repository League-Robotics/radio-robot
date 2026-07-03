"""
test_sim_commands_registry.py — ticket 069-003 (SIMSET/SIMGET wire surface),
extended by ticket 069-004 (per-wheel encoder-report-error + OTOS-error rows).

Covers the SimCommands grammar/dispatch contract directly (not the
higher-level RT-90 physical scenario, which lives in
tests/simulation/system/test_069_rt_90deg_body_scrub.py; nor the behavioral
error-isolation scenarios, which live in
tests/simulation/system/test_069_004_encoder_otos_knobs.py):

  - SIMSET/SIMGET round-trip for every ticket-003 AND ticket-004 registry key.
  - Unknown key -> ERR badkey (both SIMSET and SIMGET).
  - SIMSET atomicity: a mixed valid/invalid SIMSET applies NONE of the keys
    (matches SET's existing all-or-nothing semantics -- see
    ConfigRegistry.cpp's handleSet).
  - Bare SIMGET dumps every registered key (tickets 003 + 004 combined) in
    one SIMCFG reply.
  - SIMSET/SIMGET are ERR unknown on a command table built with sim=nullptr
    -- the exact shape the ARM firmware's own command table has (main.cpp
    never passes a SimCommands*), proving the sim-only surface never leaks
    onto real hardware.
"""
from __future__ import annotations

import pytest


# ---------------------------------------------------------------------------
# Round-trip coverage for every ticket-003 kSimRegistry[] row.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "key,value",
    [
        ("bodyRotScrub", 0.85),
        ("bodyLinScrub", 0.90),
        ("trackwidthMm", 175.0),
        ("motorOffsetL", 0.95),
        ("motorOffsetR", 1.05),
        # ---- 069-004: per-wheel encoder-report-error rows -----------------
        ("encScaleErrL", 0.05),
        ("encScaleErrR", -0.05),
        ("encSlipL", 0.03),
        ("encSlipR", 0.04),
        ("encNoiseL", 1.5),
        ("encNoiseR", 2.5),
        # ---- 069-004: OTOS-error rows --------------------------------------
        ("otosLinScaleErr", 0.02),
        ("otosAngScaleErr", -0.02),
        ("otosLinNoise", 0.5),
        ("otosYawNoise", 0.01),
    ],
)
def test_simset_simget_roundtrip(sim, key: str, value: float) -> None:
    """SIMSET <key>=<value> then SIMGET <key> returns the same value."""
    reply = sim.send_command(f"SIMSET {key}={value}")
    assert reply.upper().startswith("OK"), f"SIMSET {key}={value} -> {reply!r}"
    # SIMSET's OK reply echoes the ORIGINAL supplied string (matches SET's
    # handleSet -- see ConfigRegistry.cpp), not a reformatted float.
    assert f"{key}={value}" in reply, f"SIMSET reply should echo applied kv: {reply!r}"

    reply = sim.send_command(f"SIMGET {key}")
    assert reply.startswith("SIMCFG"), f"SIMGET {key} -> {reply!r}"
    # SIMGET always reformats via %.3f (mirrors GET/CFG's CFG_FLOAT format).
    assert f"{key}={value:.3f}" in reply, f"SIMGET {key} -> {reply!r}"


# ---------------------------------------------------------------------------
# otosLinDriftMmS / otosYawDriftDegS: per-second wire value <-> SimOdometer's
# internal per-tick representation.  Unlike the plain pass-through keys
# above, SIMGET's readback is not a string-identical echo of the SIMSET
# input -- it round-trips through RobotConfig::controlPeriodMs (default
# 10 ms) and, for the yaw key, a deg<->rad conversion -- so this asserts
# numeric closeness rather than an exact %.3f string match.
# ---------------------------------------------------------------------------

def _simget_value(sim, key: str) -> float:
    reply = sim.send_command(f"SIMGET {key}")
    assert reply.startswith("SIMCFG"), f"SIMGET {key} -> {reply!r}"
    for tok in reply.split():
        if tok.startswith(f"{key}="):
            return float(tok.split("=", 1)[1])
    raise AssertionError(f"{key} not found in SIMGET reply: {reply!r}")


@pytest.mark.parametrize("drift_mm_s", [5.0, -3.0, 0.0])
def test_otos_lin_drift_mms_roundtrip(sim, drift_mm_s: float) -> None:
    reply = sim.send_command(f"SIMSET otosLinDriftMmS={drift_mm_s}")
    assert reply.upper().startswith("OK"), reply

    got = _simget_value(sim, "otosLinDriftMmS")
    assert abs(got - drift_mm_s) < 0.01, (
        f"otosLinDriftMmS round-trip: set {drift_mm_s}, got {got}"
    )


@pytest.mark.parametrize("drift_deg_s", [3.0, -2.0, 0.0])
def test_otos_yaw_drift_degs_roundtrip(sim, drift_deg_s: float) -> None:
    reply = sim.send_command(f"SIMSET otosYawDriftDegS={drift_deg_s}")
    assert reply.upper().startswith("OK"), reply

    got = _simget_value(sim, "otosYawDriftDegS")
    assert abs(got - drift_deg_s) < 0.01, (
        f"otosYawDriftDegS round-trip: set {drift_deg_s}, got {got}"
    )


def test_simset_multiple_keys_one_command(sim) -> None:
    """A single SIMSET can apply several keys at once, like SET."""
    reply = sim.send_command("SIMSET motorOffsetL=0.8 motorOffsetR=1.2")
    assert reply.upper().startswith("OK"), reply
    assert "motorOffsetL=0.8" in reply
    assert "motorOffsetR=1.2" in reply

    reply = sim.send_command("SIMGET motorOffsetL motorOffsetR")
    assert "motorOffsetL=0.800" in reply
    assert "motorOffsetR=1.200" in reply


# ---------------------------------------------------------------------------
# Unknown key -> ERR badkey
# ---------------------------------------------------------------------------

def test_simset_unknown_key(sim) -> None:
    reply = sim.send_command("SIMSET notARealKey=1.0")
    assert "ERR" in reply.upper()
    assert "badkey" in reply
    assert "notARealKey" in reply


def test_simget_unknown_key(sim) -> None:
    reply = sim.send_command("SIMGET notARealKey")
    assert "ERR" in reply.upper()
    assert "badkey" in reply
    assert "notARealKey" in reply


# ---------------------------------------------------------------------------
# Atomicity: one bad key/value in a SIMSET commits NOTHING.
# ---------------------------------------------------------------------------

def test_simset_atomic_all_or_nothing_bad_key(sim) -> None:
    """A mixed valid/invalid SIMSET (unknown key) applies NEITHER key."""
    # Baseline: read the current (default) value.
    before = sim.send_command("SIMGET bodyRotScrub")
    assert "bodyRotScrub=1.000" in before

    reply = sim.send_command("SIMSET bodyRotScrub=0.5 notARealKey=1.0")
    assert "ERR" in reply.upper()
    assert "badkey" in reply

    after = sim.send_command("SIMGET bodyRotScrub")
    assert "bodyRotScrub=1.000" in after, (
        f"bodyRotScrub should be UNCHANGED after a rejected SIMSET; got {after!r}"
    )


def test_simset_atomic_all_or_nothing_bad_value(sim) -> None:
    """A mixed valid/invalid SIMSET (unparsable value) applies NEITHER key."""
    before = sim.send_command("SIMGET bodyLinScrub")
    assert "bodyLinScrub=1.000" in before

    reply = sim.send_command("SIMSET bodyLinScrub=0.5 trackwidthMm=notanumber")
    assert "ERR" in reply.upper()
    assert "badval" in reply

    after = sim.send_command("SIMGET bodyLinScrub")
    assert "bodyLinScrub=1.000" in after, (
        f"bodyLinScrub should be UNCHANGED after a rejected SIMSET; got {after!r}"
    )


# ---------------------------------------------------------------------------
# Bare SIMGET dumps every registered key.
# ---------------------------------------------------------------------------

def test_simget_bare_dumps_all_keys(sim) -> None:
    """Bare SIMGET dumps ALL registered keys from tickets 003 and 004
    combined -- may span multiple SIMCFG lines once chunked (see
    kSimCfgChunkMax in SimCommands.cpp), so collect every SIMCFG line.
    """
    reply = sim.send_command("SIMGET")
    assert reply.startswith("SIMCFG")
    for key in (
        # ticket 003
        "bodyRotScrub", "bodyLinScrub", "trackwidthMm",
        "motorOffsetL", "motorOffsetR",
        # ticket 004: per-wheel encoder-report-error
        "encScaleErrL", "encScaleErrR", "encSlipL", "encSlipR",
        "encNoiseL", "encNoiseR",
        # ticket 004: OTOS error
        "otosLinScaleErr", "otosAngScaleErr", "otosLinNoise", "otosYawNoise",
        "otosLinDriftMmS", "otosYawDriftDegS",
    ):
        assert key in reply, f"bare SIMGET should include {key}: {reply!r}"


# ---------------------------------------------------------------------------
# sim=nullptr (ARM-equivalent) command table: SIMSET/SIMGET are ERR unknown.
# ---------------------------------------------------------------------------

def test_simset_simget_unknown_verb_without_simcommands(sim) -> None:
    """On a command table built WITHOUT a SimCommands* (sim=nullptr -- the
    real ARM firmware's shape), SIMSET/SIMGET are unrecognised verbs, exactly
    like any other unregistered command.
    """
    reply = sim.send_command_no_simcmds("SIMSET bodyRotScrub=0.5")
    assert "ERR" in reply.upper()
    assert "unknown" in reply

    reply = sim.send_command_no_simcmds("SIMGET")
    assert "ERR" in reply.upper()
    assert "unknown" in reply


def test_normal_command_table_unaffected_by_no_simcmds_table(sim) -> None:
    """Building the throwaway sim=nullptr table must not disturb the main
    command table or robot/sim state (see sim_api.cpp's
    sim_command_no_simcmds() for why this is expected to be side-effect-free).
    """
    sim.send_command("SIMSET bodyRotScrub=0.77")
    # Build + dispatch against the throwaway no-SimCommands table.
    sim.send_command_no_simcmds("PING")
    # The main table's SIMSET-applied value must be untouched.
    reply = sim.send_command("SIMGET bodyRotScrub")
    assert "bodyRotScrub=0.770" in reply, reply
    # And the main table's own SIMSET/SIMGET must still work normally.
    reply = sim.send_command("SIMGET bodyLinScrub")
    assert "bodyLinScrub=1.000" in reply, reply
