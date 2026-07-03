"""
test_sim_commands_registry.py — ticket 069-003 (SIMSET/SIMGET wire surface).

Covers the SimCommands grammar/dispatch contract directly (not the
higher-level RT-90 physical scenario, which lives in
tests/simulation/system/test_069_rt_90deg_body_scrub.py):

  - SIMSET/SIMGET round-trip for every ticket-003 registry key.
  - Unknown key -> ERR badkey (both SIMSET and SIMGET).
  - SIMSET atomicity: a mixed valid/invalid SIMSET applies NONE of the keys
    (matches SET's existing all-or-nothing semantics -- see
    ConfigRegistry.cpp's handleSet).
  - Bare SIMGET dumps every registered key in one SIMCFG reply.
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
    reply = sim.send_command("SIMGET")
    assert reply.startswith("SIMCFG")
    for key in ("bodyRotScrub", "bodyLinScrub", "trackwidthMm",
                "motorOffsetL", "motorOffsetR"):
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
