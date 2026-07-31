"""robot_radio.field.geofence — the hard playfield geofence and camera-fix
helpers.

Promoted (127-003) out of `src/tests/bench/square_tour.py`, where
`Geofence`/`GeofenceViolation`/`checkPlayfieldLights`/`captureFix` used to
be the only definition, reached by `otos_calibration_bench.py` through a
`sys.path` hack -- and out of `otos_calibration_bench.py` itself, which had
grown its own `captureFixWithRetry` wrapper around `Geofence.captureFix`.
Moved verbatim (behavior unchanged) except one deliberate fix: `captureFix`'s
yaw averaging used to be a linear median of raw yaw, which is wrap-unsafe
near +-pi; it now uses the circular mean `testkit/camera.read_camera_pose`
already implements (`atan2(mean(sin), mean(cos))`). The per-axis median for
x/y is unchanged.

This is the one canonical geofence/camera-fix implementation every playfield
script depends on -- see `.claude/rules/playfield-testing.md` for the
obligations it implements (hard halt via `estop()`, fail-closed on lost tag,
lights preflight, per-segment camera fix).
"""

from __future__ import annotations

import math
import time

PLAYFIELD_LIGHTS_URL = "http://192.168.1.122/rpc/Switch.GetStatus?id=0"


class GeofenceViolation(RuntimeError):
    """The robot left the safe box. Motors are already stopped."""


class Geofence:
    """A HARD geofence: it stops the robot, it does not merely warn.

    2026-07-29: an earlier version of this run logged "RAIL WARNING" lines to
    a file and had no authority to do anything about them. The robot drove
    off the table and the stakeholder caught it by hand. A camera that only
    observes is not a safety device -- it has to be in the loop, with the
    power to halt, and it has to be checked on the same timebase the robot is
    moving on (inside advance(), not between segments).

    Fails CLOSED on losing the tag: if the robot cannot be seen, it is not
    known to be safe, so it is stopped. Missing the tag is exactly what
    happens when the robot leaves the field.
    """

    HALF_W = 67.15   # [cm] field
    HALF_H = 44.65

    # MEASURED 2026-07-29: the fence fires at the line but the robot COASTS
    # past it -- 6.2 cm at 120 mm/s (halted at y=-24.9, came to rest at
    # -31.1). Cruise is 150 mm/s, so budget ~8 cm. The margin therefore has
    # to cover coast, not just robot half-extent, or the halt still ends up
    # off the table.
    #
    # This is genuinely tight on this field: a 500 mm square needs 50 cm of
    # CENTRE travel in y, and the field is only 89.3 cm, leaving 19.6 cm
    # total -- under 10 cm per side for margin + coast + tour error. 12 cm
    # keeps ~5 cm of real clearance after coast while still permitting a
    # correctly-driven 500 mm square.
    def __init__(self, proto, margin: float = 12.0, lost_grace: float = 1.5):
        from aprilcam.client.control import DaemonControl
        from aprilcam.config import Config

        self._dc = DaemonControl.connect_default(Config.load())
        self._cam = self._dc.list_cameras()[0]
        self._proto = proto
        self.margin = margin           # [cm] robot half-extent plus slack
        self._lost_grace = lost_grace  # [s] tolerate brief detection dropouts
        self._last_seen = time.monotonic()
        self.last = None

    def _halt(self, why: str):
        # estop(), not stop(): a geofence breach is a "halt now" event --
        # stop() is a PLANNED stop that would wait behind whatever is
        # already queued and coast the robot the rest of the way off the
        # field (measured on hardware 2026-07-29: 39.8cm of travel on a
        # 40cm leg before a stop() sent mid-leg took effect).
        halt_error: Exception | None = None
        for _ in range(3):
            try:
                self._proto.estop()
                halt_error = None
                break
            except Exception as exc:
                halt_error = exc
            time.sleep(0.05)
        if halt_error is not None:
            # A halt that silently failed is indistinguishable from one
            # that worked -- exactly what let the robot drive off the
            # table before this was caught by hand. Surface it.
            raise GeofenceViolation(
                f"{why} -- AND estop() failed on all 3 attempts, last "
                f"error: {halt_error!r} -- ROBOT MAY STILL BE MOVING"
            ) from halt_error
        raise GeofenceViolation(why)

    def check(self) -> None:
        for t in self._dc.get_tags(self._cam).tags:
            if t.id == 100 and t.world_xy:
                x, y = t.world_xy
                self.last = (x, y)
                self._last_seen = time.monotonic()
                if (abs(x) > self.HALF_W - self.margin
                        or abs(y) > self.HALF_H - self.margin):
                    self._halt(f"geofence: robot at ({x:.1f},{y:.1f}) cm is "
                               f"within {self.margin} cm of the field edge")
                return
        if time.monotonic() - self._last_seen > self._lost_grace:
            self._halt(f"geofence: tag 100 not seen for "
                       f"{time.monotonic()-self._last_seen:.1f}s -- "
                       f"position unknown, last {self.last}")

    def captureFix(self, label: str, samples: int = 7
                   ) -> "tuple[float, float, float] | None":
        """Median-of-`samples` camera pose fix for tag 100: (x_cm, y_cm,
        yaw_rad). Caller is responsible for having settled to REST first --
        this does not wait. Returns None (never raises) if the tag was not
        seen on ANY sample -- a missing fix degrades the tour's report, it
        must not abort the tour (stakeholder mandate, 2026-07-29: a camera
        fix at every segment boundary, per `.claude/rules/
        playfield-testing.md`).

        x/y use a per-axis median (unchanged). yaw uses the circular mean
        (atan2(mean(sin), mean(cos))), matching
        `testkit/camera.read_camera_pose` -- a linear median of raw yaw is
        wrap-unsafe near +-pi (127-003 fix)."""
        xs: "list[float]" = []
        ys: "list[float]" = []
        yaws: "list[float]" = []
        for _ in range(samples):
            for t in self._dc.get_tags(self._cam).tags:
                if t.id == 100 and t.world_xy is not None and t.yaw is not None:
                    xs.append(t.world_xy[0])
                    ys.append(t.world_xy[1])
                    yaws.append(t.yaw)
                    break
            time.sleep(0.03)
        if not xs:
            print(f"  camera fix '{label}': tag 100 not seen")
            return None
        xs.sort()
        ys.sort()
        n = len(xs)
        meanSin = sum(math.sin(yaw) for yaw in yaws) / len(yaws)
        meanCos = sum(math.cos(yaw) for yaw in yaws) / len(yaws)
        fix = (xs[n // 2], ys[n // 2], math.atan2(meanSin, meanCos))
        print(f"  camera fix '{label}': x={fix[0]:+.1f}cm y={fix[1]:+.1f}cm "
              f"yaw={math.degrees(fix[2]):+.1f}deg ({n}/{samples} samples)")
        return fix

    def close(self):
        try:
            self._dc.close()
        except Exception:
            pass


def checkPlayfieldLights() -> None:
    """Preflight: FAIL loudly if the playfield room lights (Shelly Plus 1,
    192.168.1.122 -- same relay as the bench room, `.claude/rules/
    playfield-testing.md`) are off. A dark field makes every AprilTag
    vanish and reads exactly like a broken camera or a lost robot -- this
    check exists so that failure mode is named immediately instead of sent
    hunting the camera (2026-07-29: a tour aborted with "tag 100 not seen"
    for exactly this reason).

    Non-fatal if the relay itself is unreachable (a different network
    problem, not this script's to diagnose) -- warns and continues.
    """
    import json
    import urllib.error
    import urllib.request

    try:
        with urllib.request.urlopen(PLAYFIELD_LIGHTS_URL, timeout=2.0) as resp:
            status = json.loads(resp.read())
    except (urllib.error.URLError, OSError, ValueError) as exc:
        print(f"WARNING: could not reach the playfield lights relay "
              f"({PLAYFIELD_LIGHTS_URL}): {exc!r} -- continuing without "
              f"checking. If tags vanish, suspect the lights first.")
        return

    if not status.get("output"):
        raise SystemExit(
            "FAIL: the playfield room LIGHTS are OFF (Shelly relay "
            "192.168.1.122) -- a dark field makes every AprilTag vanish and "
            "looks exactly like a broken camera. Turn them on: curl -s "
            "'http://192.168.1.122/rpc/Switch.Set?id=0&on=true'")


def _playfieldLightsOn() -> "bool | None":  # None if the relay could not be reached
    """Query the playfield Shelly relay's light-switch state directly
    (`checkPlayfieldLights()`'s own URL) -- returns True/False, or None if
    the relay is unreachable (a different network problem, not this
    module's to diagnose)."""
    import json
    import urllib.error
    import urllib.request

    try:
        with urllib.request.urlopen(PLAYFIELD_LIGHTS_URL, timeout=2.0) as resp:
            status = json.loads(resp.read())
        return bool(status.get("output"))
    except (urllib.error.URLError, OSError, ValueError) as exc:
        print(f"  WARNING: could not reach the playfield lights relay: {exc!r}")
        return None


def _turnPlayfieldLightsOn() -> None:
    """Best-effort: switch the playfield Shelly relay's light channel on.
    Never raises -- a failed light-on attempt just means the subsequent
    captureFix() retries are less likely to recover, which is already
    handled by giving up honestly after the retry window."""
    import urllib.error
    import urllib.request

    url = "http://192.168.1.122/rpc/Switch.Set?id=0&on=true"
    try:
        urllib.request.urlopen(url, timeout=2.0)
        print("  turned playfield lights ON (Shelly 192.168.1.122)")
    except (urllib.error.URLError, OSError) as exc:
        print(f"  WARNING: failed to turn playfield lights on: {exc!r}")


def captureFixWithRetry(geofence: "Geofence", label: str,
                        retrySeconds: float = 5.0) -> "tuple[float, float, float] | None":
    # [s]
    """Like Geofence.captureFix(), but on losing tag 100 -- a dropout that
    has repeatedly happened and self-recovered during bench sessions --
    checks whether the playfield Shelly lights have switched themselves off
    (a known, observed cause) and turns them back on, then RETRIES the fix
    for up to retrySeconds before giving up. Never masks a genuine, lasting
    loss: if the tag still is not seen after the retry window, returns None
    exactly like captureFix() does, and the caller must treat that as a real
    failure, not silently skip past it."""
    fix = geofence.captureFix(label)
    if fix is not None:
        return fix

    print(f"  '{label}': tag 100 lost -- checking playfield lights before retrying")
    lightsOn = _playfieldLightsOn()
    if lightsOn is False:
        print("  playfield lights are OFF -- turning them on (known dropout cause)")
        _turnPlayfieldLightsOn()
        time.sleep(1.0)  # let the light + camera exposure settle

    deadline = time.monotonic() + retrySeconds
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        time.sleep(0.5)
        fix = geofence.captureFix(f"{label} (retry {attempt})")
        if fix is not None:
            print(f"  '{label}': tag 100 recovered on retry {attempt}")
            return fix
    print(f"  '{label}': tag 100 still not seen after {retrySeconds:.0f}s of retrying -- "
          "giving up")
    return None
