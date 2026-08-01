"""src/tests/unit/test_geofence_capture_fix_yaw_wrap.py — ticket 127-003.

``Geofence.captureFix()`` used to average yaw with a plain linear median of
the raw radian readings (`square_tour.py:289-293`, pre-move). That is
wrap-unsafe: a robot heading that straddles +-pi (e.g. samples alternating
just above +pi and just below -pi) has a true mean heading near +-pi, but a
linear median of the raw values can land near 0 -- roughly 180 degrees wrong.

127-003 moved `Geofence` into `robot_radio.field.geofence` and fixed this in
transit to use the circular mean `testgui/transport.read_camera_pose` already
implements: `atan2(mean(sin(yaw)), mean(cos(yaw)))`. x/y keep their original
per-axis median (unchanged).

This module is Qt-free, camera-daemon-free (the AprilCam client is never
imported here) -- `Geofence.captureFix()` only touches `self._dc`/`self._cam`,
which this test fakes out directly, bypassing `Geofence.__init__` (which
requires a live `aprilcam` daemon connection). Collected under
`src/tests/unit/` per `pyproject.toml`'s `testpaths`.

Covers:
  1. A synthetic set of tag readings straddling +-pi that a linear median
     would get wrong (landing near 0) but the circular mean gets right
     (landing near +-pi).
  2. x/y still use a per-axis median (unchanged behavior) on a simple,
     non-wrapping set of samples.
  3. A tag never seen returns None (unchanged "fail soft" behavior).
"""
from __future__ import annotations

import math

from robot_radio.field.geofence import Geofence

TAG_ID = 100


class _FakeTag:
    def __init__(self, x: float, y: float, yaw: float) -> None:
        self.id = TAG_ID
        self.world_xy = (x, y)
        self.yaw = yaw


class _FakeTagsResult:
    def __init__(self, tags: "list[_FakeTag]") -> None:
        self.tags = tags


class _FakeDaemonControl:
    """Cycles through a fixed sequence of per-poll tag readings, repeating
    the last one once exhausted -- `captureFix()` polls `samples` times."""

    def __init__(self, readings: "list[list[_FakeTag]]") -> None:
        self._readings = readings
        self._i = 0

    def get_tags(self, _cam) -> _FakeTagsResult:
        tags = self._readings[min(self._i, len(self._readings) - 1)]
        self._i += 1
        return _FakeTagsResult(tags)


def _makeGeofence(readings: "list[list[_FakeTag]]") -> Geofence:
    """Build a Geofence without running __init__ (which requires a live
    aprilcam daemon) -- captureFix() only touches self._dc/self._cam."""
    geofence = object.__new__(Geofence)
    geofence._dc = _FakeDaemonControl(readings)
    geofence._cam = None
    return geofence


# ---------------------------------------------------------------------------
# 1. Circular-mean yaw averaging, wrap-safe near +-pi.
# ---------------------------------------------------------------------------


#: A symmetric, evenly-split set of yaw readings straddling +-pi -- the true
#: (circular-mean) heading is exactly pi. VERIFIED (see ticket 127-003 work):
#: the OLD per-axis linear median of these same 8 values lands at ~2.60 rad
#: (~149 deg, ~31 deg off), while the circular mean lands exactly on pi.
_WRAP_STRADDLING_YAWS = [2.6, 2.8, 3.0, 3.10, -3.10, -3.0, -2.8, -2.6]


def test_capture_fix_yaw_circular_mean_handles_wrap_around_pi() -> None:
    readings = [[_FakeTag(0.0, 0.0, yaw)] for yaw in _WRAP_STRADDLING_YAWS]
    geofence = _makeGeofence(readings)

    fix = geofence.captureFix("wrap-test", samples=len(readings))

    assert fix is not None
    _, _, yaw = fix
    # The circular mean must land essentially exactly on +-pi.
    wrappedDelta = abs(((yaw - math.pi) + math.pi) % (2 * math.pi) - math.pi)
    assert wrappedDelta < 1e-6, f"expected yaw ~= pi, got {yaw:.4f} rad"


def test_capture_fix_yaw_linear_median_would_have_been_wrong() -> None:
    """Documents the bug this ticket fixes: on the SAME wrap-straddling
    samples `captureFix()` now handles correctly, the OLD per-axis linear
    median (`sorted(yaws)[n // 2]`) lands ~31 degrees off the true pi
    heading -- i.e. the fix is not a no-op on this input."""
    oldLinearMedian = sorted(_WRAP_STRADDLING_YAWS)[len(_WRAP_STRADDLING_YAWS) // 2]
    wrappedDelta = abs(((oldLinearMedian - math.pi) + math.pi) % (2 * math.pi) - math.pi)
    assert wrappedDelta > 0.4, (
        "sanity check: the old linear-median formula should land well off "
        f"pi on this wrap-straddling input (got {math.degrees(wrappedDelta):.1f} "
        "deg off), demonstrating the bug the circular mean fixes"
    )


# ---------------------------------------------------------------------------
# 2. x/y per-axis median is unchanged.
# ---------------------------------------------------------------------------


def test_capture_fix_x_y_still_use_per_axis_median() -> None:
    readings = [
        [_FakeTag(1.0, 10.0, 0.0)],
        [_FakeTag(3.0, 30.0, 0.0)],
        [_FakeTag(2.0, 20.0, 0.0)],
        [_FakeTag(5.0, 50.0, 0.0)],
        [_FakeTag(4.0, 40.0, 0.0)],
    ]
    geofence = _makeGeofence(readings)

    fix = geofence.captureFix("median-test", samples=5)

    assert fix is not None
    x, y, yaw = fix
    assert x == 3.0  # median of 1,2,3,4,5
    assert y == 30.0  # median of 10,20,30,40,50
    assert abs(yaw) < 1e-9


# ---------------------------------------------------------------------------
# 3. Tag never seen returns None (fail-soft, unchanged).
# ---------------------------------------------------------------------------


def test_capture_fix_returns_none_when_tag_never_seen() -> None:
    geofence = _makeGeofence([[]])

    fix = geofence.captureFix("no-tag-test", samples=3)

    assert fix is None
