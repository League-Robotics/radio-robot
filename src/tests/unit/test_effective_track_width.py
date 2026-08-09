"""src/tests/unit/test_effective_track_width.py -- pin the host-side effective
track derivation against the firmware's.

``robot_radio.testgui.transport.effective_track_width()`` must produce the
identical number ``Core::effectiveTrackWidth()``
(``src/firm/core/boot_calibration.cpp``) derives at boot::

    effective = trackwidth / rotational_slip   (slip > 0, else trackwidth)

WHY THIS TEST EXISTS. The host encoder dead-reckoner (orange playfield trace)
and the firmware's own pose (magenta trace) integrate the SAME encoder counts
through ``dtheta = (dR - dL) / trackwidth``. If the host uses the raw caliper
trackwidth (tovez: 128.0) while the firmware uses the effective one (140.4),
the host over-rotates by 1.097x on EVERY turn and the two traces fan apart in
proportion to accumulated rotation -- roughly 50 deg of skew over a tour with
~540 deg of commanded turning. It looks exactly like a broken estimator or a
dead sensor, which is what makes it expensive to chase.

The regression these cases pin is not the formula but the LOOKUP: the slip
moved to ``geometry.rotational_slip`` (132-014) while this function still read
``calibration.rotational_slip``, so it silently returned the raw width for
every robot. A wrong-but-plausible fallback is invisible; hence the explicit
"reads the current schema" case below.

Qt-free and daemon-free: builds plain stand-in config objects.

Run with::

    uv run python -m pytest src/tests/unit/test_effective_track_width.py -q
"""

from __future__ import annotations

import pytest

from robot_radio.testgui.transport import effective_track_width


class _Geometry:
    def __init__(self, trackwidth=None, rotational_slip=None):  # [mm], ratio
        if trackwidth is not None:
            self.trackwidth = trackwidth
        if rotational_slip is not None:
            self.rotational_slip = rotational_slip


class _Calibration:
    def __init__(self, rotational_slip=None):
        if rotational_slip is not None:
            self.rotational_slip = rotational_slip


class _Config:
    def __init__(self, geometry=None, calibration=None):
        self.geometry = geometry
        self.calibration = calibration


# ---------------------------------------------------------------------------
# The tovez numbers -- the ones the playfield traces actually diverge over
# ---------------------------------------------------------------------------


class TestTovezDerivation:
    def test_matches_firmware_effective_track(self):
        """128 mm caliper / 0.9117 slip -> 140.4 mm, per data/robots/tovez.json."""
        cfg = _Config(geometry=_Geometry(trackwidth=128.0, rotational_slip=0.9117))
        assert effective_track_width(cfg) == pytest.approx(140.397, abs=1e-3)

    def test_correction_is_the_ten_percent_that_skews_the_trace(self):
        """The raw-vs-effective ratio is the per-turn over-rotation factor."""
        cfg = _Config(geometry=_Geometry(trackwidth=128.0, rotational_slip=0.9117))
        assert effective_track_width(cfg) / 128.0 == pytest.approx(1.097, abs=1e-3)

    def test_reads_the_current_schema_not_the_retired_one(self):
        """Slip lives under `geometry` (132-014). Reading only `calibration`
        returned the RAW width for every robot -- silently, which is why it
        survived. A config with NO `calibration` at all (today's tovez.json)
        must still get the correction."""
        cfg = _Config(
            geometry=_Geometry(trackwidth=128.0, rotational_slip=0.9117),
            calibration=None,
        )
        assert effective_track_width(cfg) == pytest.approx(140.397, abs=1e-3)
        assert effective_track_width(cfg) != 128.0


# ---------------------------------------------------------------------------
# The guard -- mirrors `(kScrub > 0.0f)` in boot_calibration.cpp exactly
# ---------------------------------------------------------------------------


class TestUncalibratedFallback:
    def test_zero_slip_is_the_uncalibrated_sentinel(self):
        """slip == 0 means "apply no correction", NOT divide-by-zero."""
        cfg = _Config(geometry=_Geometry(trackwidth=128.0, rotational_slip=0.0))
        assert effective_track_width(cfg) == 128.0

    def test_missing_slip_applies_no_correction(self):
        cfg = _Config(geometry=_Geometry(trackwidth=128.0))
        assert effective_track_width(cfg) == 128.0

    def test_negative_slip_applies_no_correction(self):
        """Outside the documented {0} u [0.5, 1.0] domain: refuse to use it."""
        cfg = _Config(geometry=_Geometry(trackwidth=128.0, rotational_slip=-0.5))
        assert effective_track_width(cfg) == 128.0


# ---------------------------------------------------------------------------
# Legacy schema + "decline to integrate" cases
# ---------------------------------------------------------------------------


class TestLegacyAndUnusable:
    def test_legacy_calibration_slip_still_honoured(self):
        """Configs predating 132-014 kept slip under `calibration`."""
        cfg = _Config(
            geometry=_Geometry(trackwidth=128.0),
            calibration=_Calibration(rotational_slip=0.9117),
        )
        assert effective_track_width(cfg) == pytest.approx(140.397, abs=1e-3)

    def test_geometry_slip_wins_over_legacy_calibration_slip(self):
        cfg = _Config(
            geometry=_Geometry(trackwidth=128.0, rotational_slip=0.8),
            calibration=_Calibration(rotational_slip=0.5),
        )
        assert effective_track_width(cfg) == pytest.approx(160.0, abs=1e-6)

    def test_no_config_returns_none(self):
        """None, not a plausible default -- callers must be able to decline."""
        assert effective_track_width(None) is None

    def test_no_trackwidth_returns_none(self):
        assert effective_track_width(_Config(geometry=_Geometry())) is None
        assert effective_track_width(_Config(geometry=None)) is None

    def test_zero_trackwidth_returns_none(self):
        cfg = _Config(geometry=_Geometry(trackwidth=0.0, rotational_slip=0.9117))
        assert effective_track_width(cfg) is None


# ---------------------------------------------------------------------------
# The real robot file -- proves the wiring, not just the arithmetic
# ---------------------------------------------------------------------------


def test_active_robot_config_gets_the_correction():
    """End-to-end against the on-disk robot config, if one is resolvable."""
    try:
        from robot_radio.config.robot_config import get_robot_config

        cfg = get_robot_config()
    except Exception:  # noqa: BLE001 — no active robot on this checkout
        pytest.skip("no active robot config resolvable")
    if cfg is None or getattr(cfg, "geometry", None) is None:
        pytest.skip("active robot config carries no geometry")

    raw = getattr(cfg.geometry, "trackwidth", None)  # [mm]
    slip = getattr(cfg.geometry, "rotational_slip", None)
    if not raw or not slip or slip <= 0.0:
        pytest.skip("active robot is uncalibrated (no usable rotational_slip)")

    assert effective_track_width(cfg) == pytest.approx(float(raw) / float(slip))
