"""wheel_control_tuning.py -- the one host-side translation from the bench
scripts' historical flat gain keys to the live WHEEL_CONTROL wire group.

Sprint 132 deleted `NezhaProtocol.config(**{"pid.kp": ...})` -- the curated
flat-string-key live-tuning message -- and replaced it with two
schema-addressed calls: `set_config_field(target, field_name, value)` for a
single value and `get_config(target)` for read-back. Five bench scripts were
still calling the deleted API; 133-003 repairs the two a wheel-controller
tuning session actually reaches for.

**Why a shared module rather than three inline dicts.** Two of the five
mappings are not guessable from the flat key, and both are exactly the kind
of silent mis-wiring that produces a confident wrong measurement:

    pid.kff  ->  pid_kaff   the ACCEL FEEDFORWARD [s] (~the plant time
                            constant), not a velocity feedforward
    pid.kaw  ->  pid_max    the TOTAL FAST-LOOP AUTHORITY cap [mm/s]. Not
                            an anti-windup gain: since 133-002 Stage B's I
                            term reads the encoder's POSITION register
                            directly, so there is no accumulator to unwind.

The `pid.kff -> kaff` class of bug is named in `robot_config.proto`'s own
`SetConfigField` doc comment as the reason the wire carries field NUMBERS
instead of strings. Keeping the last surviving string vocabulary in one
tested place is the host-side half of that argument.

**Read-back is not optional.** `.claude/rules/configuration-discipline.md`:
"an ack is not evidence: config that acks OK and lands nowhere is a live
failure mode in this codebase, not a hypothetical." `push_gains()` therefore
reads the group back and compares, and raises rather than returning a run
whose gains are unknown.

Pure except for `push_gains()`/`read_gains()`, which need a connected
`NezhaProtocol`. The mapping itself imports nothing and is unit-tested in
`src/tests/unit/test_wheel_control_tuning.py`.
"""
from __future__ import annotations

from typing import Any

# The historical flat key -> WheelControl protobuf field name. These five
# are also EXACTLY the WHEEL_CONTROL fields the firmware persists to flash
# (`src/firm/config/persisted_tuning.h`'s `kWheelControlFields == 5`), which
# is why a value pushed through this table survives a reboot while a
# `DBG:`-pushed one (v_min / a_steady / pos_err_max / the duty gains) does
# not. See PERSISTENCE below.
FLAT_KEY_TO_FIELD: "dict[str, str]" = {
    "pid.kp": "pid_kp",
    "pid.ki": "pid_ki",
    "pid.kff": "pid_kaff",
    "pid.iMax": "pid_i_max",
    "pid.kaw": "pid_max",
}

# Where each knob a tuning session touches actually lives after the push.
# Printed by every script that asserts tuning, because the asymmetry is
# invisible otherwise and WILL be rediscovered the hard way: an operator
# reconnects, finds five values still in force and four gone, and concludes
# the push did not work.
PERSISTENCE: "dict[str, str]" = {
    "pid_kp": "flash (persisted)",
    "pid_ki": "flash (persisted)",
    "pid_kaff": "flash (persisted)",
    "pid_i_max": "flash (persisted)",
    "pid_max": "flash (persisted)",
    # RAM-only, pushed over the DBG channel (ROBOT_DEBUG builds only).
    # pos_err_max is a real robot-JSON field AND live over the wire -- it is
    # simply not in the persisted set, so a DBG push of it reverts on the
    # next reset like the other three.
    "v_min": "RAM only (DBG)",
    "a_steady": "RAM only (DBG)",
    "pos_err_max": "RAM only (DBG)",
    "duty gain L/R": "RAM only (DBG)",
}

# Tolerance for the read-back compare. The wire carries float32 and the host
# holds float64, so an exact compare would fail on values as ordinary as
# 0.23; 1e-5 is far tighter than any gain resolution a bench session works
# at and far looser than that conversion.
_READBACK_TOLERANCE = 1e-5


class TuningNotConfirmed(RuntimeError):
    """A push was NAKed, timed out, or did not survive read-back.

    Its own exception type so a caller can distinguish "the robot is not
    running the gains I asked for" from any other RuntimeError and abort the
    measurement rather than reporting it. Every caller in this tree treats
    it as fatal -- reporting a distance against unknown gains is worse than
    reporting nothing.
    """


def to_fields(gains: "dict[str, float]") -> "dict[str, float]":
    """Translate a flat-key gain dict to WheelControl field names.

    Raises KeyError on an unknown flat key -- a typo must not silently push
    four of five gains and leave the fifth at whatever the robot had.
    """
    unknown = sorted(set(gains) - set(FLAT_KEY_TO_FIELD))
    if unknown:
        raise KeyError(
            f"unknown gain key(s) {unknown}; known keys are "
            f"{sorted(FLAT_KEY_TO_FIELD)}")
    return {FLAT_KEY_TO_FIELD[key]: float(value) for key, value in gains.items()}


def read_gains(proto: Any) -> "dict[str, float]":
    """Return the robot's CURRENT WheelControl group as a plain
    ``{field_name: float}`` dict.

    Raises TuningNotConfirmed when the group cannot be read -- an
    unreadable robot is an unattributable measurement, not a warning.
    """
    from robot_radio.robot.pb2 import robot_config_pb2

    live = proto.get_config(robot_config_pb2.WHEEL_CONTROL)
    if live is None:
        raise TuningNotConfirmed(
            "get_config(WHEEL_CONTROL) returned nothing -- cannot confirm "
            "what this robot is actually running")
    # `type(live).model_fields`, not the instance attribute: pydantic v2
    # deprecates the instance spelling and warns on every access.
    return {name: float(getattr(live, name)) for name in type(live).model_fields}


def push_gains(proto: Any, gains: "dict[str, float]", *,
               read_timeout: int = 800,  # [ms]
               ) -> "dict[str, float]":
    """Push a flat-key gain dict field by field, then READ THE GROUP BACK and
    verify every pushed value landed. Returns the full read-back.

    Reads the group back even when `gains` is empty: the five pid_* fields
    persist in flash, so a value some EARLIER session pushed is still in
    force, and a script that reported only what it pushed itself would be
    describing a robot that does not exist.

    Raises TuningNotConfirmed on a NAK, a timeout, or a read-back
    disagreement.

    This predates -- and is the model for -- the general
    ``set_config_field(..., verify=True)`` path (133-006,
    ``protocol.ConfigNotVerified``). It is deliberately NOT rewritten onto
    it: this function pushes N fields and reads the group back ONCE, where
    per-field verification would cost N read-backs for the same guarantee.
    Use ``verify=True`` for one-off pushes; use this for a gain set.
    """
    from robot_radio.robot.pb2 import robot_config_pb2

    wanted = to_fields(gains)
    for field_name, value in wanted.items():
        ack = proto.set_config_field(robot_config_pb2.WHEEL_CONTROL, field_name,
                                     value, read_timeout=read_timeout)
        if ack is None:
            raise TuningNotConfirmed(
                f"set_config_field(WHEEL_CONTROL, {field_name}={value:g}) was "
                f"NAKed or timed out")

    live = read_gains(proto)
    mismatched = [
        f"{name}: asked {value:g}, robot reports "
        f"{live.get(name, float('nan')):g}"
        for name, value in wanted.items()
        if abs(live.get(name, float("nan")) - value) > _READBACK_TOLERANCE
    ]
    if mismatched:
        raise TuningNotConfirmed(
            "WHEEL_CONTROL read-back disagrees with what was pushed -- "
            + "; ".join(mismatched)
            + ". An ack is not evidence; this is the acks-OK-lands-nowhere "
              "failure mode.")
    return live


def format_gains(fields: "dict[str, float]") -> str:
    """One compact, sortable line describing a gain set -- for a CSV cell, a
    chart subtitle, or a log header.

    `.claude/rules/configuration-discipline.md` requires a captured dataset
    to record the values it was taken under, so it is self-describing rather
    than dependent on session memory. Zero-valued fields are dropped: an
    all-zero Stage B is the shipped default and naming every zero buries the
    one or two values that were actually set.
    """
    return " ".join(f"{name}={value:g}"
                    for name, value in sorted(fields.items()) if value)


def describe_persistence(fields: "dict[str, float]") -> "list[str]":
    """Render the live gain set as printable ``name value [persistence]``
    lines, so every script that asserts tuning says the same thing about
    what survives a reboot and what does not."""
    lines = [f"    {name:<20} {value:>10.4g}   "
             f"[{PERSISTENCE.get(name, 'RAM only (DBG)')}]"
             for name, value in sorted(fields.items())]
    lines.append(
        "  NOTE: pid_* survive a reboot (flash); v_min / a_steady / "
        "pos_err_max / duty gains do NOT -- they are RAM-only and revert on "
        "any power cycle. Promote a keeper into data/robots/<robot>.json.")
    lines.append(
        "  NOTE: since 133-006 connecting no longer resets the board, so a "
        "RAM-only value now survives a reconnect within one power cycle. "
        "Confirm with get_config_snapshot(...).source_name -- LIVE means a "
        "push is still in force, BAKED means it is gone.")
    return lines
