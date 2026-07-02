"""SessionRecorder — Qt-free TX/RX session logger for the Test GUI.

State machine
-------------
idle → recording ↔ paused → idle

- ``start(filename=None)`` transitions idle → recording.
- ``pause()`` transitions recording → paused.
- ``resume()`` transitions paused → recording.
- ``stop()`` transitions any active state → idle, flushes and closes the file.

JSONL schema
------------
Each line written to the output file is a standalone JSON object::

    {"t_mono": <float>, "t_wall": "<ISO-8601 UTC>", "dir": "TX"|"RX", "line": "<str>"}

Fields:
    t_mono  float    ``time.monotonic()`` at append time, seconds.
    t_wall  str      Wall-clock UTC in ISO-8601 with millisecond precision,
                     e.g. ``"2026-07-01T14:23:00.123+00:00"``.
    dir     str      Direction tag: ``"TX"`` for commands sent to the robot,
                     ``"RX"`` for responses and telemetry received from the robot.
    line    str      The formatted log string, stripped of trailing ``\\r\\n``.
                     Transmitted commands are marked ``> ...`` and received
                     replies/telemetry ``< ...`` (matching ``dir``).

Threading assumption
--------------------
All ``append()`` calls **must** occur on the Qt main thread.  This is
guaranteed by the existing architecture: TX commands are dispatched from
GUI slots (main thread); RX/telemetry lines arrive via ``_TelemetryBridge``
which marshals them to the main thread before calling ``_append_log``.
No internal locking is required.
"""
from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal

Direction = Literal["TX", "RX"]


def direction_from_marker(text: str) -> Direction | None:
    """Infer the TX/RX direction of a formatted transport log line.

    Transport log lines are formatted ``[HH:MM:SS] > <wire>`` for commands
    transmitted to the robot and ``[HH:MM:SS] < <wire>`` for replies/telemetry
    received from it.  Internal GUI status lines (``[INFO]``, ``[WARN]``,
    ``[ERROR]``, ``[REC]`` …) carry neither marker.

    Returns ``"TX"`` for ``>``, ``"RX"`` for ``<``, and ``None`` for anything
    else — the latter so status lines are routed to the log pane but *not*
    written to the recording (which is a pure wire-traffic log).
    """
    # Strip the leading ``[HH:MM:SS] `` timestamp, if present, then inspect the
    # first character of the remaining body.
    body = text.split("] ", 1)[1] if "] " in text else text
    marker = body[:1]
    if marker == ">":
        return "TX"
    if marker == "<":
        return "RX"
    return None


class SessionRecorder:
    """Record GUI TX/RX lines to a timestamped JSONL file.

    Parameters
    ----------
    recordings_dir:
        Directory where recording files are written.  Created automatically
        if it does not exist.  Defaults to ``"recordings"`` relative to the
        current working directory.
    """

    def __init__(self, recordings_dir: str | Path = "recordings") -> None:
        self._dir = Path(recordings_dir)
        self._file = None
        self._state: str = "idle"  # "idle" | "recording" | "paused"
        self._path: Path | None = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def state(self) -> str:
        """Current recorder state: ``"idle"``, ``"recording"``, or ``"paused"``."""
        return self._state

    @property
    def current_path(self) -> Path | None:
        """Path of the current (or most-recently-started) session file, or ``None`` if idle."""
        return self._path

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self, filename: str | None = None) -> Path:
        """Open a new recording session.

        Parameters
        ----------
        filename:
            Optional explicit filename within ``recordings_dir``.  If omitted,
            a timestamped name is generated: ``recording_<YYYYMMDD_HHMMSS>.jsonl``.

        Returns
        -------
        Path
            The absolute path of the newly created recording file.

        Raises
        ------
        RuntimeError
            If the recorder is not in the idle state.
        """
        if self._state != "idle":
            raise RuntimeError(f"Cannot start: recorder is {self._state!r}")
        self._dir.mkdir(parents=True, exist_ok=True)
        if filename is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{ts}.jsonl"
        self._path = self._dir / filename
        self._file = open(self._path, "w", encoding="utf-8")  # noqa: WPS515
        self._state = "recording"
        return self._path

    def pause(self) -> None:
        """Suspend appending.  No-op if not currently recording."""
        if self._state != "recording":
            return
        self._state = "paused"

    def resume(self) -> None:
        """Resume appending after a pause.  No-op if not currently paused."""
        if self._state != "paused":
            return
        self._state = "recording"

    def stop(self) -> Path | None:
        """Finalize and close the session file.

        Flushes any buffered data and closes the file handle.  The recorder
        returns to the idle state; further ``append()`` calls are no-ops
        until ``start()`` is called again.

        Returns
        -------
        Path | None
            The path of the saved file, or ``None`` if the recorder was
            already idle.
        """
        if self._state == "idle":
            return None
        path = self._path
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None
        self._state = "idle"
        self._path = None
        return path

    def append(self, direction: Direction, line: str) -> None:
        """Append one TX/RX entry to the recording.

        No-op if the recorder is idle or paused.

        Parameters
        ----------
        direction:
            ``"TX"`` for a command sent to the robot; ``"RX"`` for a
            response or telemetry line received from the robot.
        line:
            The raw wire string.  Trailing ``\\r`` and ``\\n`` characters
            are stripped before writing.
        """
        if self._state != "recording":
            return
        entry = {
            "t_mono": time.monotonic(),
            "t_wall": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "dir": direction,
            "line": line.rstrip("\r\n"),
        }
        self._file.write(json.dumps(entry) + "\n")
