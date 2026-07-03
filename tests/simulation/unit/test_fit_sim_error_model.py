"""test_fit_sim_error_model.py — ticket 069-008.

Two independent things are tested here:

  1. The JSONL recording format itself (``save_jsonl``/``load_jsonl``/
     ``split_recording``) round-trips a synthetic ``{t, cmd}``/``{t,
     encpose, otos, pose}`` sequence. Pure Python, no sim, no scipy —
     always runs.

  2. The SIM-TO-SIM validation this ticket's acceptance criteria require:
     inject KNOWN ``SIMSET`` values into Sim A, drive a maneuver, record;
     fit against fresh Sim B replays; assert the fit RECOVERS the injected
     values within a stated tolerance. Then a replay-fidelity check: load
     the fitted params into a fresh Sim C, replay, and confirm its
     trajectory agrees with Sim A's original recording within tolerance.

     Guarded with ``pytest.importorskip("scipy")`` (scoped to the one test
     function that needs it, not the whole module) so the committed suite
     stays GREEN whether or not scipy is installed — scipy is a
     `calibrate` dependency-group extra (see ``pyproject.toml``), not a
     base host dependency.

Real-hardware Tour-1 record->fit->replay is explicitly OUT OF SCOPE for this
sprint (see ``fit_sim_error_model.py``'s module docstring and
architecture-update.md Open Question 1) — not attempted here.
"""
from __future__ import annotations

import json
import math
from unittest.mock import MagicMock

import pytest

from firmware import Sim
from robot_radio.calibration.fit_sim_error_model import (
    WriteLineAdapter,
    default_maneuver,
    fit_sim_error_model,
    load_jsonl,
    load_param_file,
    push_params,
    record_sim_run,
    replay_samples,
    save_jsonl,
    save_param_file,
    split_recording,
)
from robot_radio.robot.protocol import TLMFrame


# ---------------------------------------------------------------------------
# 1. JSONL recording format round-trip (no sim, no scipy)
# ---------------------------------------------------------------------------

def test_jsonl_round_trip_preserves_cmd_and_pose_records(tmp_path):
    records = [
        {"t": 0, "cmd": "T 200 200 2000"},
        {"t": 100, "encpose": [10, 0, 0], "otos": [11, -1, 5], "pose": [10, 0, 2]},
        {"t": 2500, "cmd": "RT 9000"},
        {"t": 2600, "encpose": [400, 5, 8900], "otos": [398, 4, 8850], "pose": [399, 4, 8870]},
    ]
    path = tmp_path / "recording.jsonl"
    save_jsonl(records, path)

    # Written as one JSON object per line.
    lines = path.read_text().strip().splitlines()
    assert len(lines) == len(records)
    for line in lines:
        json.loads(line)  # each line must be valid, standalone JSON

    loaded = load_jsonl(path)
    assert loaded == records


def test_split_recording_separates_commands_and_poses():
    records = [
        {"t": 0, "cmd": "T 200 200 2000"},
        {"t": 100, "encpose": [10, 0, 0], "otos": [11, -1, 5], "pose": [10, 0, 2]},
        {"t": 2500, "cmd": "RT 9000"},
        {"t": 2600, "encpose": [400, 5, 8900], "otos": None, "pose": [399, 4, 8870]},
    ]
    commands, pose_samples = split_recording(records)

    assert commands == [(0, "T 200 200 2000"), (2500, "RT 9000")]

    assert set(pose_samples) == {100, 2600}
    assert isinstance(pose_samples[100], TLMFrame)
    assert pose_samples[100].encpose == (10, 0, 0)
    assert pose_samples[100].otos == (11, -1, 5)
    assert pose_samples[100].pose == (10, 0, 2)

    # A null field stays None (not e.g. coerced to a zero tuple).
    assert pose_samples[2600].otos is None
    assert pose_samples[2600].encpose == (400, 5, 8900)
    assert pose_samples[2600].pose == (399, 4, 8870)


def test_split_recording_empty_input_yields_empty_outputs():
    commands, pose_samples = split_recording([])
    assert commands == []
    assert pose_samples == {}


# ---------------------------------------------------------------------------
# 1b. Parameter file I/O and the push-to-a-live-connection mechanism (no
#     sim, no scipy -- pure Python, always runs).
# ---------------------------------------------------------------------------

def test_param_file_round_trip(tmp_path):
    params = {"bodyRotScrub": 0.8123, "encScaleErrL": 0.06, "otosLinScaleErr": -0.08}
    path = tmp_path / "fitted.json"
    save_param_file(path, params)

    # Emitted as {"<SIMSET key>": <fitted value>, ...} JSON (acceptance criterion).
    on_disk = json.loads(path.read_text())
    assert on_disk == params

    loaded = load_param_file(path)
    assert loaded == params


def test_push_params_prefers_send_command():
    conn = MagicMock(spec=["send_command"])
    conn.send_command.return_value = "OK simset bodyRotScrub=0.8"
    reply = push_params(conn, {"bodyRotScrub": 0.8})
    conn.send_command.assert_called_once_with("SIMSET bodyRotScrub=0.8")
    assert reply == "OK simset bodyRotScrub=0.8"


def test_push_params_falls_back_to_command():
    conn = MagicMock(spec=["command"])
    conn.command.return_value = "OK simset encScaleErrL=0.06"
    reply = push_params(conn, {"encScaleErrL": 0.06})
    conn.command.assert_called_once_with("SIMSET encScaleErrL=0.06")
    assert reply == "OK simset encScaleErrL=0.06"


def test_push_params_batches_multiple_keys_into_one_command():
    conn = MagicMock(spec=["send_command"])
    conn.send_command.return_value = "OK simset bodyRotScrub=0.8 encScaleErrL=0.06"
    push_params(conn, {"bodyRotScrub": 0.8, "encScaleErrL": 0.06})
    # ONE call, not one per key.
    assert conn.send_command.call_count == 1
    (sent_line,), _ = conn.send_command.call_args
    assert sent_line == "SIMSET bodyRotScrub=0.8 encScaleErrL=0.06"


def test_push_params_rejects_conn_with_neither_method():
    conn = MagicMock(spec=[])
    with pytest.raises(TypeError):
        push_params(conn, {"bodyRotScrub": 0.8})


def test_push_params_rejects_empty_params():
    conn = MagicMock(spec=["send_command"])
    with pytest.raises(ValueError):
        push_params(conn, {})


def test_write_line_adapter_bridges_to_send_command():
    ser = MagicMock(spec=["write_line", "read_available"])
    ser.read_available.return_value = ["OK simset bodyRotScrub=0.8"]
    adapter = WriteLineAdapter(ser)
    reply = adapter.send_command("SIMSET bodyRotScrub=0.8")
    ser.write_line.assert_called_once_with("SIMSET bodyRotScrub=0.8")
    assert reply == "OK simset bodyRotScrub=0.8"


# ---------------------------------------------------------------------------
# 2. Sim-to-sim validation (requires scipy; requires the sim build)
# ---------------------------------------------------------------------------

def _otos_sim() -> Sim:
    """A fresh Sim with watchdog extended and OTOS model+fusion enabled --
    required for otos=/pose= to populate in TLM (mirrors the ``_otos_sim()``
    helper in test_069_knob_telemetry_sweep.py)."""
    s = Sim()
    s.send_command("SET sTimeout=60000")
    s.enable_otos_model()
    s.set_otos_fusion(True)
    return s


# Candidate subset for this test: one knob from each of PhysicsWorld's
# independently-owned error surfaces (ground-truth body scrub, per-wheel
# reported-encoder bias, OTOS observation bias) -- exercises all three
# residual-contributing pose channels (pose=/otos= for bodyRotScrub and
# otosLinScaleErr, encpose= for encScaleErrL) while keeping the search small
# (3 unknowns) so the fit runs quickly.
FIT_KEYS = ("bodyRotScrub", "encScaleErrL", "otosLinScaleErr")

# Injected "ground truth" values -- magnitudes chosen at or below the
# non-default values test_069_knob_telemetry_sweep.py already validated as
# clearly observable on this plant (bodyRotScrub 0.5, encScaleErrL 0.05,
# otosLinScaleErr 0.10), so recovery is expected to be well within tolerance.
INJECTED_PARAMS = {
    "bodyRotScrub": 0.80,
    "encScaleErrL": 0.06,
    "otosLinScaleErr": -0.08,
}

# +/-10% relative, or an absolute floor of 0.02 for near-zero true values --
# stated and applied explicitly, per the ticket's acceptance criteria.
TOLERANCE_REL = 0.10
TOLERANCE_ABS = 0.02


def _within_tolerance(true_value: float, fitted_value: float) -> bool:
    tol = max(abs(true_value) * TOLERANCE_REL, TOLERANCE_ABS)
    return abs(fitted_value - true_value) <= tol


def test_sim_to_sim_fit_recovers_injected_params_and_replay_is_faithful():
    pytest.importorskip("scipy")

    commands, total_ms = default_maneuver()
    sample_period_ms = 200

    # --- 1. Inject KNOWN params into Sim A, drive the maneuver, record. ---
    sim_a = _otos_sim()
    sim_a.send_command(
        "SIMSET " + " ".join(f"{k}={v}" for k, v in INJECTED_PARAMS.items())
    )
    records = record_sim_run(sim_a, commands, total_ms, sample_period_ms)

    assert any("cmd" in r for r in records), "recording captured no issued commands"
    assert any(
        ("pose" in r) or ("otos" in r) or ("encpose" in r) for r in records
    ), "recording captured no TLM pose samples"

    # --- 2. Fit against FRESH Sim B instances (one per least_squares call). ---
    result = fit_sim_error_model(
        records,
        sim_factory=_otos_sim,
        candidate_keys=FIT_KEYS,
        total_ms=total_ms,
        sample_period_ms=sample_period_ms,
    )

    assert result.success, (
        f"least_squares did not report success: {result.message} "
        f"(cost={result.cost}, nfev={result.nfev})"
    )

    for key, true_value in INJECTED_PARAMS.items():
        fitted_value = result.params[key]
        tol = max(abs(true_value) * TOLERANCE_REL, TOLERANCE_ABS)
        assert _within_tolerance(true_value, fitted_value), (
            f"{key}: injected={true_value} fitted={fitted_value} "
            f"exceeds tolerance +/-{tol:.4f}"
        )

    # --- 3. Replay-fidelity check: fitted params into a FRESH Sim C, ---
    # replay, and confirm the trajectory agrees with Sim A's ORIGINAL
    # recording within tolerance.
    sim_c = _otos_sim()
    sim_c.send_command(
        "SIMSET " + " ".join(f"{k}={v}" for k, v in result.params.items())
    )
    _, recorded_samples = split_recording(records)
    replayed_samples = replay_samples(sim_c, commands, total_ms, sample_period_ms)

    checked = 0
    for t, rec_frame in recorded_samples.items():
        rep_frame = replayed_samples.get(t)
        if rep_frame is None or rec_frame.pose is None or rep_frame.pose is None:
            continue
        dx = rep_frame.pose[0] - rec_frame.pose[0]
        dy = rep_frame.pose[1] - rec_frame.pose[1]
        dist_mm = math.hypot(dx, dy)
        assert dist_mm < 15.0, (
            f"replay-fidelity: pose= diverges by {dist_mm:.1f}mm at t={t}ms "
            f"(recorded={rec_frame.pose} replayed={rep_frame.pose})"
        )
        checked += 1
    assert checked > 0, "replay-fidelity check compared zero samples"
