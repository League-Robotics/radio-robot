"""src/tests/unit/test_planner_model.py -- 106-005 (SUC-028/SUC-029).

Covers `robot_radio.planner.model.PlannerParams` -- the sprint's
live-tunable parameter surface. No I/O beyond a scratch JSON file this
module writes itself; no hardware, no sim.

Collected under `src/tests/unit/` per `pyproject.toml`'s `testpaths`.
"""

from __future__ import annotations

import json

import pytest

from robot_radio.planner.model import PlannerParams


def test_defaults_construct_with_no_arguments():
    params = PlannerParams()
    assert params.streaming_interval == pytest.approx(0.15)
    assert params.heading_omega_clamp > 0


def test_heading_gain_defaults_are_107_001s_bench_proven_values():
    """107-001: promotes ticket 106-006's bench-measured gains
    (heading_kp=0.4, heading_omega_clamp=0.2) to PlannerParams' own field
    defaults, replacing the 2.0/0.5 "starting point" that saturated the
    correction trim on the bench rig's high-inertia proxy load. A cheap
    regression guard against an accidental revert."""
    params = PlannerParams()
    assert params.heading_kp == pytest.approx(0.4)
    assert params.heading_omega_clamp == pytest.approx(0.2)


def test_fields_are_directly_mutable_no_redeploy_needed():
    """The core live-tunability mechanism: a plain attribute set."""
    params = PlannerParams()
    params.heading_kp = 9.0
    assert params.heading_kp == 9.0


def test_load_with_no_file_and_no_env_returns_defaults(monkeypatch):
    monkeypatch.delenv("PLANNER_CONFIG", raising=False)
    for f in ("streaming_interval", "heading_kp"):
        monkeypatch.delenv(f"PLANNER_{f.upper()}", raising=False)

    params = PlannerParams.load()
    assert params == PlannerParams()


def test_load_from_explicit_json_path_overrides_fields(tmp_path):
    path = tmp_path / "planner_params.json"
    path.write_text(json.dumps({"heading_kp": 7.5, "v_max": 250.0}))

    params = PlannerParams.load(path=path)

    assert params.heading_kp == 7.5
    assert params.v_max == 250.0
    # Untouched fields keep their defaults.
    assert params.a_max == PlannerParams().a_max


def test_load_from_planner_config_env_var(tmp_path, monkeypatch):
    path = tmp_path / "planner_params.json"
    path.write_text(json.dumps({"omega_max": 3.0}))
    monkeypatch.setenv("PLANNER_CONFIG", str(path))

    params = PlannerParams.load()

    assert params.omega_max == 3.0


def test_load_rejects_unknown_json_key(tmp_path):
    path = tmp_path / "planner_params.json"
    path.write_text(json.dumps({"not_a_real_field": 1.0}))

    with pytest.raises(ValueError, match="unknown key"):
        PlannerParams.load(path=path)


def test_load_env_var_override_wins_over_json(tmp_path, monkeypatch):
    path = tmp_path / "planner_params.json"
    path.write_text(json.dumps({"heading_kp": 7.5}))
    monkeypatch.setenv("PLANNER_HEADING_KP", "11.0")

    params = PlannerParams.load(path=path)

    assert params.heading_kp == 11.0


def test_load_env_var_alone_overrides_a_single_field(monkeypatch):
    monkeypatch.setenv("PLANNER_HEADING_OMEGA_CLAMP", "0.9")

    params = PlannerParams.load()

    assert params.heading_omega_clamp == 0.9
    # Everything else stays default.
    assert params.heading_kp == PlannerParams().heading_kp


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
