"""tests/testgui/test_sim_prefs.py — headless, Qt-free tests for sim_prefs.

Covers:
- DEFAULT_PROFILE keys/values (0.0 / 0.26 / 0.05 / 0.0).
- save_sim_error_profile / load_sim_error_profile round-trip via a
  monkeypatched _PREFS_PATH / _PREFS_DIR pointed at tmp_path (never touches
  the real repo data/ directory).
- Missing file / corrupt JSON -> DEFAULT_PROFILE (copy).
- Partial file merged with defaults for the missing keys.
- Non-numeric value for a known key falls back to that key's default.
- Unknown keys in the persisted file are ignored.
- The module is importable without PySide6.

Run with:
    QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/test_sim_prefs.py -q
"""

from __future__ import annotations

import json


# ---------------------------------------------------------------------------
# DEFAULT_PROFILE
# ---------------------------------------------------------------------------


class TestDefaultProfile:
    def test_default_profile_keys_and_values(self):
        from robot_radio.testgui.sim_prefs import DEFAULT_PROFILE

        assert DEFAULT_PROFILE == {
            # historical four
            "encoder_noise_mm": 0.0,
            "slip_turn_extra": 0.26,
            "otos_linear_noise": 0.05,
            "otos_yaw_noise": 0.0,
            # 069-007: additive/noise terms -- 0.0 is a genuine no-op
            "enc_scale_err_l": 0.0,
            "enc_scale_err_r": 0.0,
            "otos_lin_scale_err": 0.0,
            "otos_ang_scale_err": 0.0,
            "otos_lin_drift_mms": 0.0,
            "otos_yaw_drift_degs": 0.0,
            # 069-007: multiplicative terms -- 1.0 is the genuine no-op
            "body_rot_scrub": 1.0,
            "body_lin_scrub": 1.0,
            "motor_offset_l": 1.0,
            "motor_offset_r": 1.0,
            # 069-007: no safe zero default -- the real plant trackwidth
            "trackwidth_mm": 150.0,
        }

    def test_multiplicative_knobs_default_to_one_not_zero(self):
        """CORRECTNESS-CRITICAL (ticket 069-007): body_rot_scrub,
        body_lin_scrub, motor_offset_l, motor_offset_r are multiplicative --
        their no-op value is 1.0. A 0.0 default would zero out the plant's
        rotation/motion (PhysicsWorld's _bodyRotationalScrub/
        _bodyLinearScrub/_offsetFactorL/_offsetFactorR all default 1.0f)."""
        from robot_radio.testgui.sim_prefs import DEFAULT_PROFILE

        for key in ("body_rot_scrub", "body_lin_scrub", "motor_offset_l", "motor_offset_r"):
            assert DEFAULT_PROFILE[key] == 1.0, f"{key} must default to 1.0, not 0.0"

    def test_trackwidth_defaults_to_real_nonzero_value(self):
        """trackwidth_mm has NO safe zero default -- PhysicsWorld::update()
        divides by it. Must default to a genuine, non-zero, neutral value
        (the plant's real compiled-in trackwidth, 150.0mm)."""
        from robot_radio.testgui.sim_prefs import DEFAULT_PROFILE

        assert DEFAULT_PROFILE["trackwidth_mm"] == 150.0
        assert DEFAULT_PROFILE["trackwidth_mm"] != 0.0

    def test_additive_noise_knobs_default_to_zero(self):
        from robot_radio.testgui.sim_prefs import DEFAULT_PROFILE

        for key in (
            "enc_scale_err_l",
            "enc_scale_err_r",
            "otos_lin_scale_err",
            "otos_ang_scale_err",
            "otos_lin_drift_mms",
            "otos_yaw_drift_degs",
        ):
            assert DEFAULT_PROFILE[key] == 0.0, f"{key} must default to 0.0"

    def test_load_returns_a_copy_not_the_default_object(self, tmp_path, monkeypatch):
        """Mutating the returned dict must not corrupt DEFAULT_PROFILE."""
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "does_not_exist.json"
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)

        profile = sim_prefs.load_sim_error_profile()
        profile["slip_turn_extra"] = 999.0
        assert sim_prefs.DEFAULT_PROFILE["slip_turn_extra"] == 0.26


# ---------------------------------------------------------------------------
# save_sim_error_profile / load_sim_error_profile — persistence round-trip
# ---------------------------------------------------------------------------


class TestPersistence:
    def test_round_trip(self, tmp_path, monkeypatch):
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)
        monkeypatch.setattr(sim_prefs, "_PREFS_DIR", tmp_path)

        profile = {
            "encoder_noise_mm": 2.5,
            "slip_turn_extra": 0.3,
            "otos_linear_noise": 0.1,
            "otos_yaw_noise": 0.02,
            "enc_scale_err_l": 0.01,
            "enc_scale_err_r": -0.01,
            "otos_lin_scale_err": 0.02,
            "otos_ang_scale_err": -0.02,
            "otos_lin_drift_mms": 1.0,
            "otos_yaw_drift_degs": -1.0,
            "body_rot_scrub": 0.9,
            "body_lin_scrub": 0.95,
            "motor_offset_l": 1.02,
            "motor_offset_r": 0.98,
            "trackwidth_mm": 151.0,
        }
        assert set(profile.keys()) == set(sim_prefs.DEFAULT_PROFILE.keys()), (
            "test profile must cover every DEFAULT_PROFILE key for a full round-trip"
        )
        sim_prefs.save_sim_error_profile(profile)
        assert sim_prefs.load_sim_error_profile() == profile

    def test_save_creates_directory(self, tmp_path, monkeypatch):
        from robot_radio.testgui import sim_prefs

        nested_dir = tmp_path / "testgui"
        prefs_path = nested_dir / "sim_error_profile.json"
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)
        monkeypatch.setattr(sim_prefs, "_PREFS_DIR", nested_dir)

        assert not nested_dir.exists()
        sim_prefs.save_sim_error_profile({"encoder_noise_mm": 1.0})
        assert prefs_path.exists()
        assert sim_prefs.load_sim_error_profile()["encoder_noise_mm"] == 1.0

    def test_load_missing_file_returns_defaults(self, tmp_path, monkeypatch):
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "does_not_exist.json"
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)

        assert sim_prefs.load_sim_error_profile() == sim_prefs.DEFAULT_PROFILE

    def test_load_corrupt_json_returns_defaults(self, tmp_path, monkeypatch):
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        prefs_path.write_text("not valid json {{{")
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)

        assert sim_prefs.load_sim_error_profile() == sim_prefs.DEFAULT_PROFILE

    def test_load_non_dict_json_returns_defaults(self, tmp_path, monkeypatch):
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        prefs_path.write_text(json.dumps([1, 2, 3]))
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)

        assert sim_prefs.load_sim_error_profile() == sim_prefs.DEFAULT_PROFILE

    def test_partial_file_merged_with_defaults(self, tmp_path, monkeypatch):
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        prefs_path.write_text(json.dumps({"encoder_noise_mm": 5.0}))
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)

        profile = sim_prefs.load_sim_error_profile()
        assert profile["encoder_noise_mm"] == 5.0
        assert profile["slip_turn_extra"] == 0.26
        assert profile["otos_linear_noise"] == 0.05
        assert profile["otos_yaw_noise"] == 0.0

    def test_non_numeric_value_falls_back_to_default_for_that_key(
        self, tmp_path, monkeypatch
    ):
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        prefs_path.write_text(
            json.dumps({"slip_turn_extra": "not-a-number", "encoder_noise_mm": 3.0})
        )
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)

        profile = sim_prefs.load_sim_error_profile()
        assert profile["slip_turn_extra"] == 0.26
        assert profile["encoder_noise_mm"] == 3.0

    def test_unknown_keys_are_ignored(self, tmp_path, monkeypatch):
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        prefs_path.write_text(json.dumps({"some_future_knob": 42.0}))
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)

        profile = sim_prefs.load_sim_error_profile()
        assert profile == sim_prefs.DEFAULT_PROFILE
        assert "some_future_knob" not in profile

    def test_save_only_writes_known_keys(self, tmp_path, monkeypatch):
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)
        monkeypatch.setattr(sim_prefs, "_PREFS_DIR", tmp_path)

        sim_prefs.save_sim_error_profile(
            {"encoder_noise_mm": 1.0, "bogus_key": "ignored"}
        )
        on_disk = json.loads(prefs_path.read_text())
        assert set(on_disk.keys()) == set(sim_prefs.DEFAULT_PROFILE.keys())

    def test_save_never_raises_on_write_failure(self, tmp_path, monkeypatch):
        """save_sim_error_profile must not raise even if persistence fails."""
        from robot_radio.testgui import sim_prefs

        blocker = tmp_path / "blocker"
        blocker.write_text("I am a file, not a directory")
        bad_dir = blocker / "testgui"
        monkeypatch.setattr(sim_prefs, "_PREFS_DIR", bad_dir)
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", bad_dir / "sim_error_profile.json")

        # Must not raise.
        sim_prefs.save_sim_error_profile({"encoder_noise_mm": 1.0})


# ---------------------------------------------------------------------------
# PROFILE_TO_SIMSET_KEY -- profile key -> SIMSET wire-key map (069-007)
# ---------------------------------------------------------------------------


class TestProfileToSimsetKeyMap:
    def test_map_contents(self):
        from robot_radio.testgui.sim_prefs import PROFILE_TO_SIMSET_KEY

        assert PROFILE_TO_SIMSET_KEY == {
            "enc_scale_err_l": "encScaleErrL",
            "enc_scale_err_r": "encScaleErrR",
            "otos_lin_scale_err": "otosLinScaleErr",
            "otos_ang_scale_err": "otosAngScaleErr",
            "otos_linear_noise": "otosLinNoise",
            "otos_yaw_noise": "otosYawNoise",
            "otos_lin_drift_mms": "otosLinDriftMmS",
            "otos_yaw_drift_degs": "otosYawDriftDegS",
            "body_rot_scrub": "bodyRotScrub",
            "body_lin_scrub": "bodyLinScrub",
            "motor_offset_l": "motorOffsetL",
            "motor_offset_r": "motorOffsetR",
            "trackwidth_mm": "trackwidthMm",
        }

    def test_map_keys_are_all_valid_profile_keys(self):
        """Every key in the map must actually exist in DEFAULT_PROFILE."""
        from robot_radio.testgui.sim_prefs import DEFAULT_PROFILE, PROFILE_TO_SIMSET_KEY

        for key in PROFILE_TO_SIMSET_KEY:
            assert key in DEFAULT_PROFILE, f"{key} is not a DEFAULT_PROFILE key"

    def test_map_excludes_encoder_noise_mm_and_slip_turn_extra(self):
        """encoder_noise_mm fans out to two wire keys (encNoiseL/encNoiseR)
        and slip_turn_extra has no SIMSET key at all -- both are handled
        specially by transport.py, not via this 1:1 map."""
        from robot_radio.testgui.sim_prefs import PROFILE_TO_SIMSET_KEY

        assert "encoder_noise_mm" not in PROFILE_TO_SIMSET_KEY
        assert "slip_turn_extra" not in PROFILE_TO_SIMSET_KEY

    def test_map_is_a_bijection(self):
        """No two profile keys should collide on the same wire key."""
        from robot_radio.testgui.sim_prefs import PROFILE_TO_SIMSET_KEY

        wire_keys = list(PROFILE_TO_SIMSET_KEY.values())
        assert len(wire_keys) == len(set(wire_keys))


# ---------------------------------------------------------------------------
# Importability without PySide6
# ---------------------------------------------------------------------------


class TestImportability:
    def test_importable_without_qt(self):
        """The module must not import PySide6 at module scope."""
        import robot_radio.testgui.sim_prefs as sim_prefs_module

        assert callable(sim_prefs_module.load_sim_error_profile)
        assert callable(sim_prefs_module.save_sim_error_profile)
        assert isinstance(sim_prefs_module.DEFAULT_PROFILE, dict)
