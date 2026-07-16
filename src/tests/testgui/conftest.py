"""src/tests/testgui/conftest.py — headless GUI test configuration (ticket 083-004).

Sets ``QT_QPA_PLATFORM=offscreen`` before any PySide6 import a collected test
module might trigger, so this directory's tests never need a real display
server. ``tests/conftest.py`` (the tree root) already puts ``host/`` on
``sys.path`` for every domain — this conftest only adds the one thing that
is specific to this domain: the offscreen Qt platform.

Uses ``setdefault`` rather than an unconditional assignment so an operator
who already exported ``QT_QPA_PLATFORM`` on the command line (the documented
invocation in every test module's docstring here) is not overridden.

Every test module in this directory that needs PySide6 requires
``uv sync --group gui`` to have been run — plain ``robot_radio.testgui`` and
its non-Qt submodules (``transport``, ``drive``, ``traces``, ``sim_prefs``,
``commands``) remain importable without it (see each module's own docstring
and ``tests/unit``'s importability guard).
"""
from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
