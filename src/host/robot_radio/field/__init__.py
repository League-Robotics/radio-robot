"""robot_radio.field — playfield abstraction over the AprilCam daemon.

Exports:
  Playfield          — camera-backed playfield access (tags, objects, pixel-world, paths).
  Tag                — a detected AprilTag with world-frame position and yaw.
  Feature             — a static playfield feature (rectangle, dot, april_tag, etc.).
  Geofence            — the hard playfield geofence (halts the robot via estop()).
  GeofenceViolation   — raised when the geofence halts the robot.
  checkPlayfieldLights — preflight: fail loudly if the playfield lights are off.
  captureFixWithRetry  — Geofence.captureFix() with a lights-aware retry.
"""

from robot_radio.field.geofence import (
    Geofence,
    GeofenceViolation,
    captureFixWithRetry,
    checkPlayfieldLights,
)
from robot_radio.field.playfield import Feature, Playfield, Tag

__all__ = [
    "Playfield",
    "Tag",
    "Feature",
    "Geofence",
    "GeofenceViolation",
    "checkPlayfieldLights",
    "captureFixWithRetry",
]
