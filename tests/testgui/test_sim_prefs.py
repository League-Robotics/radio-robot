"""tests/testgui/test_sim_prefs.py — headless, Qt-free tests for sim_prefs.

Covers:
- DEFAULT_PROFILE keys/values (0.0 / 0.0 / 0.05 / 0.0 — ticket 073-003
  changed slip_turn_extra from the historical 0.26 to 0.0).
- resolve_calibration_defaults() -- the shared calibration lookup (ticket
  073-003) backing both __main__.py's "From Calibration" button and
  load_sim_error_profile()'s factory-default fallback: found-config and
  missing-config/missing-field fallback paths.
- save_sim_error_profile / load_sim_error_profile round-trip via a
  monkeypatched _PREFS_PATH / _PREFS_DIR pointed at tmp_path (never touches
  the real repo data/ directory).
- Missing file / corrupt JSON -> DEFAULT_PROFILE, except body_rot_scrub,
  which resolves from calibration (ticket 073-003) -- these tests
  monkeypatch get_robot_config to None so the fallback is deterministically
  the neutral 1.0, matching DEFAULT_PROFILE.
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
            # historical four -- slip_turn_extra changed 0.26 -> 0.0 (073-003)
            "encoder_noise_mm": 0.0,
            "slip_turn_extra": 0.0,
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
            # 069-007: no safe zero default -- the firmware config's
            # trackwidth (what the sim seeds the plant with at construction)
            "trackwidth_mm": 128.0,
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
        divides by it. Must default to a genuine, non-zero, neutral value:
        the firmware config's trackwidthMm (DefaultConfig.cpp, 128.0mm),
        which is what sim_api.cpp seeds the plant with at construction.
        NOT PhysicsWorld::kDefaultTrackwidthMm (150.0) -- the plant never
        actually runs at that value, and applying it would mismatch the
        plant against the firmware's kinematic calibration (every
        encoder-arc turn would land off-angle by the ratio)."""
        from robot_radio.testgui.sim_prefs import DEFAULT_PROFILE

        assert DEFAULT_PROFILE["trackwidth_mm"] == 128.0
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
        assert sim_prefs.DEFAULT_PROFILE["slip_turn_extra"] == 0.0  # 073-003: was 0.26


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
        """073-003: body_rot_scrub's fallback is now calibration-resolved,
        not the DEFAULT_PROFILE literal -- pin get_robot_config to None (at
        its SOURCE module -- resolve_calibration_defaults() re-imports it
        per call, exactly like the original "From Calibration" button
        handler did, so patching robot_radio.config.robot_config is the
        real patch point, not sim_prefs) so this test's "==
        DEFAULT_PROFILE" claim is deterministic (the neutral 1.0 fallback)
        regardless of what robot happens to be active."""
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "does_not_exist.json"
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: None)

        assert sim_prefs.load_sim_error_profile() == sim_prefs.DEFAULT_PROFILE

    def test_load_corrupt_json_returns_defaults(self, tmp_path, monkeypatch):
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        prefs_path.write_text("not valid json {{{")
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: None)

        assert sim_prefs.load_sim_error_profile() == sim_prefs.DEFAULT_PROFILE

    def test_load_non_dict_json_returns_defaults(self, tmp_path, monkeypatch):
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        prefs_path.write_text(json.dumps([1, 2, 3]))
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: None)

        assert sim_prefs.load_sim_error_profile() == sim_prefs.DEFAULT_PROFILE

    def test_partial_file_merged_with_defaults(self, tmp_path, monkeypatch):
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        prefs_path.write_text(json.dumps({"encoder_noise_mm": 5.0}))
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)

        profile = sim_prefs.load_sim_error_profile()
        assert profile["encoder_noise_mm"] == 5.0
        assert profile["slip_turn_extra"] == 0.0  # 073-003: was 0.26
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
        assert profile["slip_turn_extra"] == 0.0  # 073-003: was 0.26
        assert profile["encoder_noise_mm"] == 3.0

    def test_unknown_keys_are_ignored(self, tmp_path, monkeypatch):
        """073-003: pin get_robot_config to None, same rationale as
        test_load_missing_file_returns_defaults above."""
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        prefs_path.write_text(json.dumps({"some_future_knob": 42.0}))
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: None)

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
# resolve_calibration_defaults() -- shared calibration lookup (073-003)
# ---------------------------------------------------------------------------


def _fake_robot_config(*, rotational_slip, trackwidth):
    """A real (pydantic) RobotConfig with only the fields this resolver
    cares about overridden -- mirrors test_sim_errors_from_cal_button.py's
    own helper of the same shape."""
    from robot_radio.config.robot_config import (
        CalibrationConfig,
        GeometryConfig,
        IdentityConfig,
        RobotConfig,
    )

    return RobotConfig(
        identity=IdentityConfig(robot_name="fake", uid="fake-uid"),
        geometry=GeometryConfig(trackwidth=trackwidth),
        calibration=CalibrationConfig(rotational_slip=rotational_slip),
    )


class TestResolveCalibrationDefaults:
    def test_resolves_from_active_robot_config(self, monkeypatch):
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        fake_cfg = _fake_robot_config(rotational_slip=0.85, trackwidth=140.0)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

        rot_slip, tw = sim_prefs.resolve_calibration_defaults()
        assert rot_slip == 0.85
        assert tw == 140.0

    def test_falls_back_to_neutral_when_no_config(self, monkeypatch, caplog):
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: None)

        with caplog.at_level("WARNING"):
            rot_slip, tw = sim_prefs.resolve_calibration_defaults()

        assert rot_slip == 1.0
        assert tw == sim_prefs.DEFAULT_PROFILE["trackwidth_mm"]
        assert any(
            "no active robot config found" in r.message for r in caplog.records
        )

    def test_falls_back_when_rotational_slip_missing(self, monkeypatch):
        """geometry.trackwidth present, calibration.rotational_slip missing:
        only body_rot_scrub falls back; trackwidth still comes from config."""
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        fake_cfg = _fake_robot_config(rotational_slip=None, trackwidth=140.0)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

        rot_slip, tw = sim_prefs.resolve_calibration_defaults()
        assert rot_slip == 1.0
        assert tw == 140.0

    def test_falls_back_when_trackwidth_missing(self, monkeypatch):
        """calibration.rotational_slip present, geometry.trackwidth missing:
        only trackwidth falls back; body_rot_scrub still comes from config."""
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        fake_cfg = _fake_robot_config(rotational_slip=0.85, trackwidth=None)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

        rot_slip, tw = sim_prefs.resolve_calibration_defaults()
        assert rot_slip == 0.85
        assert tw == sim_prefs.DEFAULT_PROFILE["trackwidth_mm"]

    def test_log_callback_receives_warn_prefixed_message_on_fallback(
        self, monkeypatch
    ):
        """The optional ``log`` callback (used by __main__.py's "From
        Calibration" button to keep its GUI log-pane behavior byte-identical
        after the 073-003 refactor) receives the same "[WARN] ..." text the
        module logger gets, just prefixed for a plain-text log widget."""
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: None)

        messages: list[str] = []
        rot_slip, tw = sim_prefs.resolve_calibration_defaults(log=messages.append)

        assert len(messages) == 1
        assert messages[0].startswith("[WARN]")
        assert "no active robot config found" in messages[0]

    def test_log_callback_not_called_when_no_fallback_needed(self, monkeypatch):
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        fake_cfg = _fake_robot_config(rotational_slip=0.85, trackwidth=140.0)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

        messages: list[str] = []
        sim_prefs.resolve_calibration_defaults(log=messages.append)
        assert messages == []


# ---------------------------------------------------------------------------
# load_sim_error_profile()'s calibration-resolved body_rot_scrub fallback
# (073-003)
# ---------------------------------------------------------------------------


class TestLoadFallbackResolvesBodyRotScrubFromCalibration:
    def test_load_missing_file_resolves_body_rot_scrub_from_calibration(
        self, tmp_path, monkeypatch
    ):
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "does_not_exist.json"
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)

        fake_cfg = _fake_robot_config(rotational_slip=0.92, trackwidth=140.0)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

        profile = sim_prefs.load_sim_error_profile()
        assert profile["body_rot_scrub"] == 0.92
        assert profile["slip_turn_extra"] == 0.0
        # trackwidth_mm's own fallback is unaffected by this ticket -- stays
        # DEFAULT_PROFILE's static value, not resolve_calibration_defaults()'s.
        assert profile["trackwidth_mm"] == sim_prefs.DEFAULT_PROFILE["trackwidth_mm"]

    def test_load_missing_file_falls_back_to_neutral_when_no_config(
        self, tmp_path, monkeypatch
    ):
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "does_not_exist.json"
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: None)

        profile = sim_prefs.load_sim_error_profile()
        assert profile["body_rot_scrub"] == 1.0

    def test_persisted_body_rot_scrub_wins_over_calibration_resolution(
        self, tmp_path, monkeypatch
    ):
        """An operator's EXISTING persisted profile is not silently
        overridden by the calibration-resolved default (Open Questions item
        4 -- no migration of existing persisted files)."""
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        prefs_path.write_text(json.dumps({"body_rot_scrub": 0.5}))
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)

        fake_cfg = _fake_robot_config(rotational_slip=0.92, trackwidth=140.0)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

        profile = sim_prefs.load_sim_error_profile()
        assert profile["body_rot_scrub"] == 0.5


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
