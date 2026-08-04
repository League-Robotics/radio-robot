"""src/tests/unit/test_wheel_control_tuning.py -- 133-003: the one host-side
translation from the bench scripts' historical flat gain keys to the live
WHEEL_CONTROL wire group (`src/tests/bench/wheel_control_tuning.py`).

Two of the five mappings are not guessable from the flat key (`pid.kff ->
pid_kaff`, `pid.kaw -> pid_max`) and both are exactly the kind of silent
mis-wiring that produces a confident wrong measurement -- push what you
think is an anti-windup gain, actually move the authority cap, and read the
result as a control finding. `robot_config.proto`'s own `SetConfigField`
doc comment names "the `pid.kff -> kaff` class of bug" as the reason the
wire carries field NUMBERS rather than strings; this test is the host-side
half of that argument.

The other property under test is that read-back is REQUIRED
(`.claude/rules/configuration-discipline.md`: "an ack is not evidence:
config that acks OK and lands nowhere is a live failure mode in this
codebase, not a hypothetical").

No hardware: `proto` is a double that records what it was asked to push and
answers `get_config()` from whatever the test wants the robot to claim.
"""
from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

_MODULE_PATH = (pathlib.Path(__file__).resolve().parents[1]
                / "bench" / "wheel_control_tuning.py")


def _load():
    spec = importlib.util.spec_from_file_location("wheel_control_tuning",
                                                  _MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def tuning():
    return _load()


class FakeGroup:
    """Stands in for the generated pydantic `WheelControl` model. Only
    `type(obj).model_fields` and attribute access are used."""

    model_fields = {
        "v_min": None, "bias_max": None, "tau_adapt": None, "a_steady": None,
        "deficit_threshold": None, "deficit_window": None,
        "pid_kp": None, "pid_ki": None, "pid_i_max": None, "pid_kaff": None,
        "pid_max": None, "pos_err_max": None,
    }

    def __init__(self, **values):
        for name in self.model_fields:
            setattr(self, name, float(values.get(name, 0.0)))


class FakeProto:
    """Records every `set_config_field()` and answers `get_config()` from a
    dict the test controls, so "the robot did not actually take the value"
    is expressible."""

    def __init__(self, *, lands: bool = True, naks: "set[str] | None" = None,
                 group: "FakeGroup | None" = None):
        self.pushed: "list[tuple[int, str, float]]" = []
        self._lands = lands
        self._naks = naks or set()
        self._state: "dict[str, float]" = {}
        self._group = group

    def set_config_field(self, target, field_name, value, *, read_timeout=500):
        self.pushed.append((target, field_name, float(value)))
        if field_name in self._naks:
            return None
        if self._lands:
            self._state[field_name] = float(value)
        return object()  # a truthy AckEntry stand-in

    def get_config(self, target, *, read_timeout=500):
        if self._group is not None:
            return self._group
        return FakeGroup(**self._state)


# ---------------------------------------------------------------------------
# The mapping
# ---------------------------------------------------------------------------

def test_the_two_non_obvious_mappings_are_correct(tuning):
    """`pid.kff` is the ACCEL feedforward (`pid_kaff`), and `pid.kaw` is the
    total fast-loop AUTHORITY cap (`pid_max`) -- not an anti-windup gain;
    since 133-002 Stage B's I term reads the position register directly and
    there is no accumulator to unwind."""
    assert tuning.FLAT_KEY_TO_FIELD["pid.kff"] == "pid_kaff"
    assert tuning.FLAT_KEY_TO_FIELD["pid.kaw"] == "pid_max"


def test_every_flat_key_maps_to_a_persisted_field(tuning):
    """All five mapped fields are exactly the set the firmware persists to
    flash (`src/firm/config/persisted_tuning.h`'s `kWheelControlFields ==
    5`). If this ever stops holding, the gate's "which values survive a
    reboot" note becomes a lie."""
    for field_name in tuning.FLAT_KEY_TO_FIELD.values():
        assert tuning.PERSISTENCE[field_name] == "flash (persisted)"
    persisted = {name for name, kind in tuning.PERSISTENCE.items()
                 if kind == "flash (persisted)"}
    assert persisted == set(tuning.FLAT_KEY_TO_FIELD.values())
    assert len(persisted) == 5


def test_the_ram_only_knobs_are_named_as_such(tuning):
    """pos_err_max in particular: it IS a real robot-JSON field and IS live
    over the wire, but it is NOT in the persisted set, so a DBG push of it
    reverts on the next reset. An operator hits this and concludes the push
    did not work."""
    for name in ("v_min", "a_steady", "pos_err_max", "duty gain L/R"):
        assert tuning.PERSISTENCE[name] == "RAM only (DBG)"


def test_to_fields_translates_a_whole_gain_set(tuning):
    assert tuning.to_fields({
        "pid.kp": 0.3, "pid.ki": 0.02, "pid.iMax": 20.0,
        "pid.kff": 0.23, "pid.kaw": 30.0,
    }) == {"pid_kp": 0.3, "pid_ki": 0.02, "pid_i_max": 20.0,
           "pid_kaff": 0.23, "pid_max": 30.0}


def test_to_fields_rejects_an_unknown_key(tuning):
    """A typo must not silently push four of five gains and leave the fifth
    at whatever the robot happened to have."""
    with pytest.raises(KeyError) as excinfo:
        tuning.to_fields({"pid.kp": 1.0, "pid.kd": 0.5})
    assert "pid.kd" in str(excinfo.value)


# ---------------------------------------------------------------------------
# push_gains: schema addressing + mandatory read-back
# ---------------------------------------------------------------------------

def test_push_gains_addresses_the_wheel_control_group_by_field_name(tuning):
    proto = FakeProto()
    live = tuning.push_gains(proto, {"pid.kff": 0.23, "pid.kaw": 30.0})
    assert [(name, value) for _, name, value in proto.pushed] == [
        ("pid_kaff", 0.23), ("pid_max", 30.0)]
    assert live["pid_kaff"] == pytest.approx(0.23)
    assert live["pid_max"] == pytest.approx(30.0)


def test_push_gains_reads_back_even_when_nothing_was_pushed(tuning):
    """The five pid_* fields persist in flash, so a value some EARLIER
    session pushed is still in force. A script that reported only what it
    pushed itself would describe a robot that does not exist."""
    proto = FakeProto(group=FakeGroup(pid_ki=6.0, pid_i_max=100.0))
    live = tuning.push_gains(proto, {})
    assert proto.pushed == []
    assert live["pid_ki"] == pytest.approx(6.0)
    assert live["pid_i_max"] == pytest.approx(100.0)


def test_push_gains_raises_when_a_field_is_naked(tuning):
    proto = FakeProto(naks={"pid_ki"})
    with pytest.raises(tuning.TuningNotConfirmed) as excinfo:
        tuning.push_gains(proto, {"pid.kp": 1.0, "pid.ki": 6.0})
    assert "pid_ki" in str(excinfo.value)


def test_push_gains_raises_when_the_readback_disagrees(tuning):
    """THE property this module exists for: an ack is not evidence. A robot
    that acks OK and lands nothing must abort the measurement, not produce a
    plausible number attributed to gains it never had."""
    proto = FakeProto(lands=False)
    with pytest.raises(tuning.TuningNotConfirmed) as excinfo:
        tuning.push_gains(proto, {"pid.ki": 6.0})
    message = str(excinfo.value)
    assert "read-back disagrees" in message
    assert "asked 6" in message
    assert "acks-OK-lands-nowhere" in message


def test_push_gains_tolerates_float32_round_trip(tuning):
    """The wire carries float32 and the host holds float64, so an exact
    compare would fail on a value as ordinary as 0.23."""
    import struct

    wanted = 0.23
    as_float32 = struct.unpack("f", struct.pack("f", wanted))[0]
    assert as_float32 != wanted  # the round trip really does change it
    proto = FakeProto(group=FakeGroup(pid_kaff=as_float32))
    live = tuning.push_gains(proto, {"pid.kff": wanted})
    assert live["pid_kaff"] == pytest.approx(wanted, abs=1e-6)


def test_read_gains_raises_when_the_group_cannot_be_read(tuning):
    class Unreadable:
        def get_config(self, target, *, read_timeout=500):
            return None

    with pytest.raises(tuning.TuningNotConfirmed):
        tuning.read_gains(Unreadable())


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def test_format_gains_drops_zeros_and_sorts(tuning):
    """An all-zero Stage B is the shipped default; naming every zero buries
    the one or two values that were actually set."""
    assert tuning.format_gains({"pid_ki": 6.0, "pid_kp": 0.0,
                                "pid_i_max": 100.0}) == "pid_i_max=100 pid_ki=6"


def test_format_gains_of_an_untouched_robot_is_empty(tuning):
    assert tuning.format_gains({"pid_kp": 0.0, "pid_ki": 0.0}) == ""


def test_describe_persistence_labels_every_field_and_warns(tuning):
    lines = tuning.describe_persistence({"pid_ki": 6.0, "pos_err_max": 5.0})
    body = "\n".join(lines)
    assert "pid_ki" in body and "flash (persisted)" in body
    assert "pos_err_max" in body and "RAM only (DBG)" in body
    # The operator-facing warning must be present, not merely implied by the
    # per-field labels -- this is the asymmetry that will otherwise be
    # rediscovered mid-session.
    assert "do NOT" in body
    assert "data/robots/" in body
