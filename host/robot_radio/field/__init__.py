"""robot_radio.field — playfield abstraction over the AprilCam daemon.

Exports:
  Playfield — camera-backed playfield access (tags, objects, pixel-world, paths).
  Tag       — a detected AprilTag with world-frame position and yaw.
  Feature   — a static playfield feature (rectangle, dot, april_tag, etc.).
"""

from robot_radio.field.playfield import Feature, Playfield, Tag

__all__ = ["Playfield", "Tag", "Feature"]
