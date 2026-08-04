"""src/tests/unit/test_velocity_step_response_gain_push.py -- 133-003 Part D:
`src/tests/bench/velocity_step_response.py` used to call the curated
flat-key `NezhaProtocol.config(**{"pid.kp": ...})` API, which sprint 132
deleted. It now builds its gain dict via the extracted `gains_from_args()`
and pushes it through `wheel_control_tuning.push_gains()`
(`src/tests/bench/wheel_control_tuning.py`). This module tests that repair
against a fake `NezhaProtocol` double
(`_fake_wheel_control_proto.FakeWheelControlProto`) -- no hardware, no
`SerialConnection`, nothing opened.

`src/tests/bench/` is "HITL CLI tools, not pytest-collected" (`tests/
CLAUDE.md`), so this loads the bench script directly by file path via
`importlib`, the SAME precedent `test_wheel_controller_ab_bench.py` and
`test_duty_sweep_population.py` already establish.
"""
from __future__ import annotations

import argparse
import importlib.util
import pathlib
import sys

import pytest

from _fake_wheel_control_proto import FakeWheelControlProto

_BENCH_SCRIPT = (pathlib.Path(__file__).resolve().parents[1] / "bench"
                / "velocity_step_response.py")


def _load_bench_module():
    spec = importlib.util.spec_from_file_location("velocity_step_response", _BENCH_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Register in sys.modules BEFORE exec: the bench script's `from
    # __future__ import annotations` (string annotations) resolves some of
    # those strings via `sys.modules[cls.__module__]` at definition time --
    # test_wheel_controller_ab_bench.py's own established fix for the same
    # importlib-by-path pattern.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def bench():
    return _load_bench_module()


def _fake_args(**overrides) -> argparse.Namespace:
    ns = argparse.Namespace(no_config=False, kp=None, ki=None, kff=None, imax=None, kaw=None)
    for key, value in overrides.items():
        setattr(ns, key, value)
    return ns


def test_module_imports_and_wires_the_shared_tuning_helper_without_hardware(bench):
    # Merely loading the module already exercises its own sys.path shim +
    # `from wheel_control_tuning import ...` -- if that wiring were broken
    # the `bench` fixture above would have raised before any test ran.
    assert hasattr(bench, "main")
    assert hasattr(bench, "push_gains")
    assert hasattr(bench, "read_gains")
    assert hasattr(bench, "TuningNotConfirmed")
    assert hasattr(bench, "format_gains")
    assert hasattr(bench, "describe_persistence")


# ---------------------------------------------------------------------------
# gains_from_args() -- the flat-key dict this script builds from
# --kp/--ki/--kff/--imax/--kaw, extracted to a pure function for exactly
# this kind of no-hardware test.
# ---------------------------------------------------------------------------

def test_gains_from_args_builds_the_flat_key_dict(bench):
    args = _fake_args(kp=0.3, ki=0.02, kff=0.23, imax=20.0, kaw=30.0)
    assert bench.gains_from_args(args) == {
        "pid.kp": 0.3, "pid.ki": 0.02, "pid.kff": 0.23,
        "pid.iMax": 20.0, "pid.kaw": 30.0,
    }


def test_gains_from_args_omits_flags_that_were_not_passed(bench):
    args = _fake_args(kp=0.0014)
    assert bench.gains_from_args(args) == {"pid.kp": 0.0014}


def test_gains_from_args_no_config_is_always_empty_even_with_flags_set(bench):
    args = _fake_args(no_config=True, kp=0.3, kaw=30.0)
    assert bench.gains_from_args(args) == {}


# ---------------------------------------------------------------------------
# push_gains() through this script's own gains dict shape
# ---------------------------------------------------------------------------

def test_push_gains_resolves_the_flat_keys_to_wheel_control_fields(bench):
    args = _fake_args(kp=0.3, ki=0.02, kff=0.23, imax=20.0, kaw=30.0)
    gains = bench.gains_from_args(args)
    proto = FakeWheelControlProto()

    readback = bench.push_gains(proto, gains)

    pushed = dict(proto.calls)
    assert pushed["pid_kp"] == pytest.approx(0.3)
    assert pushed["pid_ki"] == pytest.approx(0.02)
    assert pushed["pid_i_max"] == pytest.approx(20.0)
    # The two non-obvious mappings this ticket exists to get right.
    assert pushed["pid_kaff"] == pytest.approx(0.23)   # pid.kff -> pid_kaff, NOT pid_kff
    assert pushed["pid_max"] == pytest.approx(30.0)    # pid.kaw -> pid_max, NOT an anti-windup field
    assert readback["pid_kaff"] == pytest.approx(0.23)
    assert readback["pid_max"] == pytest.approx(30.0)


def test_push_gains_raises_on_nak_rather_than_proceeding(bench):
    args = _fake_args(kp=0.3)
    gains = bench.gains_from_args(args)
    proto = FakeWheelControlProto(nak_field="pid_kp")

    with pytest.raises(bench.TuningNotConfirmed):
        bench.push_gains(proto, gains)


def test_push_gains_raises_on_readback_disagreement_rather_than_proceeding(bench):
    args = _fake_args(kaw=30.0)
    gains = bench.gains_from_args(args)
    # The ack path succeeds (set_config_field() does not return None), but
    # the value that actually lands differs -- "an ack is not evidence"
    # (`.claude/rules/configuration-discipline.md`).
    proto = FakeWheelControlProto(mismatch_field="pid_max")

    with pytest.raises(bench.TuningNotConfirmed):
        bench.push_gains(proto, gains)


def test_no_config_reads_back_whatever_is_already_live(bench):
    # --no-config's own contract: no push, but the script still wants to
    # know (and record) what the robot is actually running.
    proto = FakeWheelControlProto(initial={"pid_kp": 0.0016, "pid_max": 20.0})
    live = bench.read_gains(proto)
    assert live["pid_kp"] == pytest.approx(0.0016)
    assert live["pid_max"] == pytest.approx(20.0)
    assert proto.calls == []  # read-only -- no set_config_field() call at all
