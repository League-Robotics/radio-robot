"""src/tests/testgui/test_canvas.py — headless tests for canvas.py's playfield
asset paths and calibration loading (ticket 083-003).

Sprint 077's greenfield rebuild parked the pre-rebuild tree at
``tests_old/old/playfield_tour/``, not ``tests/old/playfield_tour/`` --
``canvas.py``'s three asset-path constants pointed at the stale location,
so ``_load_calibration()``/``_build_playfield_calibration()`` always hit the
``except`` branch and silently fell back to the hardcoded default field
dimensions (``_FIELD_WIDTH_CM_DEFAULT``/``_FIELD_HEIGHT_CM_DEFAULT``).

107-004: a later reorg (commit ``ea9b3e28``) moved the whole parked tree from
``tests_old/`` to ``archive/tests_old/`` and never updated ``canvas.py``'s
three constants to match -- the exact same stale-path failure mode this
file's own ticket (083-003) originally fixed, recurring one level deeper.
Nobody caught it because this file was dropped from ``testpaths`` at sprint
102 ticket 005 (``tests/testgui`` parked wholesale) before the reorg landed.
Re-adding ``src/tests/testgui/`` to ``testpaths`` (this ticket) surfaced it;
``canvas.py``'s constants now point at ``archive/tests_old/old/
playfield_tour/`` -- this file's own assertions (``"tests_old" in parts``,
etc.) still hold unchanged since ``archive/tests_old/...`` still contains a
``tests_old`` path component, just one directory deeper.

No ``QApplication`` is needed here: importing ``robot_radio.testgui.canvas``
and calling its module-level path constants / ``_load_calibration()`` /
``_build_playfield_calibration()`` touches no PySide6 (deferred inside
``build_canvas()`` and its helpers). Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_canvas.py -q

Collected under ``src/tests/testgui/`` per ``pyproject.toml``'s ``testpaths``
(107-004 re-added the directory -- dropped at sprint 102 ticket 005).
"""
from __future__ import annotations

import json

import pytest

from robot_radio.testgui import canvas

# 136-002: the three tests below all read fixture files under
# ``src/archive/tests_old/old/playfield_tour/`` that no longer exist on
# disk -- an unrelated commit (``e4bffd8e``, 2026-08-07) deleted the entire
# ``src/archive/`` tree (198420 lines, no archive path named in its own
# commit message) and nobody has yet decided whether that was intentional.
# See the tracked issue for the full trace (confirmed via git log, not
# assumed) and what a fix requires (a stakeholder call, not a triage
# judgment).
_XFAIL_ARCHIVE_DELETED = (
    "136-002: fixture missing -- src/archive/tests_old/old/playfield_tour/ "
    "was deleted by commit e4bffd8e (2026-08-07, an unrelated MicroPython-"
    "feasibility-docs commit whose own message names none of the ~200K "
    "deleted lines this test's fixtures were part of) and nobody has yet "
    "decided whether that deletion was intentional. Tracked: "
    "clasi/issues/later/test-canvas-playfield-tour-fixtures-deleted-by-e4bffd8e.md."
)


# ---------------------------------------------------------------------------
# (a) Asset-path constants resolve under tests_old/old/playfield_tour/
# ---------------------------------------------------------------------------

@pytest.mark.xfail(reason=_XFAIL_ARCHIVE_DELETED, strict=True)
def test_asset_path_constants_resolve_under_tests_old() -> None:
    """The three asset-path constants point at real files under
    ``tests_old/old/playfield_tour/`` -- not the stale ``tests/old/`` path."""
    for const in (
        canvas._PLAYFIELD_IMAGE,
        canvas._PLAYFIELD_DESKEWED,
        canvas._PLAYFIELD_CALIB,
    ):
        parts = const.parts
        assert "tests_old" in parts, f"{const} does not resolve under tests_old/"
        assert "old" in parts, f"{const} does not resolve under .../old/"
        assert "playfield_tour" in parts, f"{const} does not resolve under .../playfield_tour/"
        assert const.exists(), f"{const} does not exist on disk"


# ---------------------------------------------------------------------------
# (b) _load_calibration() reads the real JSON, not the hardcoded defaults
# ---------------------------------------------------------------------------

@pytest.mark.xfail(reason=_XFAIL_ARCHIVE_DELETED, strict=True)
def test_load_calibration_reads_real_json_dimensions() -> None:
    """``_load_calibration()`` returns the calibration JSON's actual
    ``width``/``height`` -- verified two ways:

    1. Its return value matches an independent ``json.loads()`` read of the
       same file (proves it is not returning stale/cached data).
    2. Pointing it at a *different* calibration file with different
       dimensions changes the result -- this is the discriminating check,
       since (coincidentally) the real fixture's width/height happen to
       equal the hardcoded defaults (134.0, 89.3), so a bare equality
       assertion against the defaults would pass even from the broken
       fallback path.
    """
    data = json.loads(canvas._PLAYFIELD_CALIB.read_text())
    expected_w = float(data["playfield"]["width"])
    expected_h = float(data["playfield"]["height"])

    w, h = canvas._load_calibration()
    assert (w, h) == (expected_w, expected_h)


def test_load_calibration_does_not_silently_fall_back_to_defaults(
    tmp_path, monkeypatch
) -> None:
    """Pointing ``_PLAYFIELD_CALIB`` at a fixture with dimensions distinct
    from the hardcoded defaults proves ``_load_calibration()`` actually
    reads the file's ``width``/``height`` rather than defaulting."""
    fake_calib = tmp_path / "fake_playfield_calibration.json"
    fake_calib.write_text(json.dumps({"playfield": {"width": 999.0, "height": 555.0}}))
    monkeypatch.setattr(canvas, "_PLAYFIELD_CALIB", fake_calib)

    w, h = canvas._load_calibration()

    assert (w, h) == (999.0, 555.0)
    assert (w, h) != (canvas._FIELD_WIDTH_CM_DEFAULT, canvas._FIELD_HEIGHT_CM_DEFAULT)


def test_load_calibration_falls_back_to_defaults_when_file_missing(
    tmp_path, monkeypatch
) -> None:
    """Confirms the graceful-degradation path still works for a genuinely
    missing/malformed file (unchanged behaviour -- not this ticket's fix,
    but worth pinning so the two code paths are distinguishable)."""
    missing = tmp_path / "does_not_exist.json"
    monkeypatch.setattr(canvas, "_PLAYFIELD_CALIB", missing)

    w, h = canvas._load_calibration()

    assert (w, h) == (canvas._FIELD_WIDTH_CM_DEFAULT, canvas._FIELD_HEIGHT_CM_DEFAULT)


# ---------------------------------------------------------------------------
# (c) _build_playfield_calibration() reads the real JSON (deskew path)
# ---------------------------------------------------------------------------

@pytest.mark.xfail(reason=_XFAIL_ARCHIVE_DELETED, strict=True)
def test_build_playfield_calibration_reads_real_json_dimensions() -> None:
    """``_build_playfield_calibration()`` (the deskew-path calibration
    builder) also reads the real field dimensions and homography, not
    the module defaults -- skips if numpy/movie are unavailable, matching
    the function's own graceful-degradation contract."""
    data = json.loads(canvas._PLAYFIELD_CALIB.read_text())
    expected_w = float(data["playfield"]["width"])
    expected_h = float(data["playfield"]["height"])

    calib = canvas._build_playfield_calibration()
    if calib is None:
        import pytest

        pytest.skip("numpy/movie unavailable -- deskew calibration build skipped")

    assert calib.field_width_cm == expected_w
    assert calib.field_height_cm == expected_h
