"""robot_radio.testgui — interactive PySide6 test cockpit for robot control.

This package provides a desktop GUI for interactive robot control and
pose-trace visualization.  PySide6 is an *optional* dependency declared in
the ``[dependency-groups] gui`` group of ``host/pyproject.toml``.

Importing this package does **not** require PySide6 to be installed.  PySide6
imports are deferred to the functions and classes that actually use them,
mirroring the lazy-matplotlib pattern in ``testkit/dash.py``.

Launch with::

    python -m robot_radio.testgui

or, after ``uv sync --group gui``::

    uv run python -m robot_radio.testgui
"""

from __future__ import annotations

__version__ = "0.1.0"
