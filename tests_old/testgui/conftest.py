"""tests/testgui/conftest.py — pytest configuration for headless GUI tests.

Sets QT_QPA_PLATFORM=offscreen before any PySide6 import so that widgets
render without a display server.  Tests in this directory require PySide6
(``uv sync --group gui``).
"""
from __future__ import annotations

import os
import pathlib
import sys

# Set the offscreen platform BEFORE PySide6 is imported.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# Ensure the host package is on sys.path.
_HOST_DIR = pathlib.Path(__file__).parent.parent.parent / "host"
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))
