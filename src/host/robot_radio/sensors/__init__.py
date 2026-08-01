"""robot_radio.sensors — sensor abstraction subpackage.

Re-exports all public names so callers can use either::

    from robot_radio.sensors import Odometry
    from robot_radio.sensors import Otos     # lazy — nav.pose not loaded until here

Modules that pull in large optional dependencies (``otos`` → nav.pose)
are loaded lazily via ``__getattr__`` so that a bare
``import robot_radio.sensors`` does NOT import those trees.

The fully-qualified import form also works without triggering this
module at all::

    from robot_radio.sensors.otos import Otos  # direct, still lazy

Eagerly-available names (no heavy deps):
    Odometry,
    ColorClassifier, nezha_classifier, calibrate_white,
    ThrashMonitor

Lazily-available names (loaded on first access):
    Otos          — requires robot_radio.nav.pose (acceptable; isolated)
"""

from __future__ import annotations

from robot_radio.sensors.odometry import Odometry
from robot_radio.sensors.color import ColorClassifier, nezha_classifier, calibrate_white
from robot_radio.sensors.motion_monitor import ThrashMonitor

# Names that are always available
__all__ = [
    "Odometry",
    "ColorClassifier",
    "nezha_classifier",
    "calibrate_white",
    "ThrashMonitor",
    # Lazy names — included in __all__ for documentation; loaded via __getattr__
    "Otos",
]

_LAZY = {
    "Otos": ("robot_radio.sensors.otos", "Otos"),
}


def __getattr__(name: str):
    """Lazy import for heavy-dependency submodules."""
    if name in _LAZY:
        module_path, attr = _LAZY[name]
        import importlib
        mod = importlib.import_module(module_path)
        obj = getattr(mod, attr)
        # Cache in module globals to avoid repeated __getattr__ calls
        globals()[name] = obj
        return obj
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
