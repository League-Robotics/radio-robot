"""runner -- the system-test executor: one backend, one destructive
telemetry drain, one record stream.

Executes a parsed Tour (tourfile.py) against a backend (sim today;
hardware backend follows the same surface), recording every wire line both
directions through the Recorder (recorder.py). EXPECT assertions evaluate
against the same in-process record stream the JSONL writer consumes.

Key invariants (from the bench-verified lineage this replaces):
- Exactly ONE destructive drain of the telemetry queue (the executor's
  ack-matching loop). The recorder observes frames via the backend's
  parallel per-frame callback, never by draining.
- Move completion is the ack RING scanned for Move.id (121-002's fix),
  never a timed settle.
- One-leg lookahead inside a motion segment: leg N+1 enqueues while leg N
  is active (first leg replace=True, rest replace=False).
- The safety path is estop(), never the planned stop() (2026-07-29
  measurement: a mid-leg stop() drives the whole leg first).
"""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import jq

sys.path.insert(0, str(Path(__file__).resolve().parent))
from recorder import Recorder  # noqa: E402
from tourfile import (  # noqa: E402
    CamfixStep, DbgStep, DwellStep, ExpectStep, MoveStep, SendStep, StopStep,
    Tour,
)

_MOVE_ID_BASE = 1 << 20  # keep Move.id clear of auto-assigned corr_ids
_ACTIVE_BIT = 1 << 2  # telemetry flags: kFlagActive
_POLL = 0.02  # [s] ack-poll interval
_STOP_SETTLE_FRAMES = 2  # consecutive inactive frames = at rest
_STOP_TIMEOUT = 15.0  # [s] bounded wait for a planned stop to land


class TourFailure(Exception):
    """A tour step failed; the run stops (after estop) and reports."""


@dataclass
class StepResult:
    line_no: int
    kind: str
    ok: bool
    detail: str = ""


@dataclass
class TourRunResult:
    tour: str
    ok: bool
    steps: list[StepResult] = field(default_factory=list)

    def summary(self) -> str:
        n_ok = sum(1 for s in self.steps if s.ok)
        status = "PASS" if self.ok else "FAIL"
        lines = [f"{status} {self.tour}: {n_ok}/{len(self.steps)} steps"]
        for s in self.steps:
            mark = "ok " if s.ok else "FAIL"
            lines.append(f"  [{mark}] line {s.line_no:3d} {s.kind}"
                         + (f" -- {s.detail}" if s.detail else ""))
        return "\n".join(lines)


class SimBackend:
    """Drives the real firmware through SimLoop. Satisfies the executor's
    backend surface: move/stop/estop/read_pending_frames/send_line/
    true_pose/firmware_version/close."""

    name = "sim"

    def __init__(self, robot_config: str, *, recorder: Recorder,
                 speed_factor: int = 1):
        from robot_radio.config.robot_config import load_robot_config
        from robot_radio.io.sim_loop import SimLoop

        self._loop = SimLoop()
        self._loop.on_telemetry = recorder.rx_tlm
        self._loop.on_debug = recorder.rx_line
        self._loop.on_cleartext = recorder.rx_line
        self._loop.connect(start_tick_thread=True)
        self._loop.configure_from_robot(load_robot_config(robot_config))
        if speed_factor > 1:
            self._loop.set_speed_factor(speed_factor)

    def move(self, **kwargs: Any) -> int:
        return self._loop.move(**kwargs)

    def stop(self) -> int:
        return self._loop.stop()

    def estop(self) -> int:
        return self._loop.estop()

    def read_pending_frames(self) -> list:
        return self._loop.read_pending_binary_tlm_frames()

    def send_line(self, line: str) -> None:
        """Inject one cleartext wire line (terminator appended)."""
        self._loop.inject_command(line.encode("ascii") + b"\n")

    def true_pose(self) -> dict | None:
        return self._loop.get_true_pose()

    def firmware_version(self) -> str:
        try:
            return self._loop.firmware_version()
        except Exception:
            return "?"

    def enable_telemetry(self) -> None:
        pass  # sim pushes telemetry while active; kAuto suffices

    def close(self) -> None:
        try:
            self._loop.estop()
        except Exception:
            pass
        self._loop.disconnect()


def _move_kwargs(step: MoveStep, move_id: int, *, replace: bool) -> dict:
    kw: dict[str, Any] = {"timeout": step.timeout * 1000.0,  # [ms]
                          "replace": replace, "id": move_id}
    if step.variant == "wheels":
        kw["v_left"] = step.v_left
        kw["v_right"] = step.v_right
    else:
        kw["v_x"] = step.v_x
        kw["v_y"] = step.v_y
        kw["omega"] = step.omega
    if step.stop_kind == "time":
        kw["stop_time"] = step.stop_value * 1000.0  # [ms]
    elif step.stop_kind == "dist":
        kw["stop_distance"] = step.stop_value  # [mm]
    else:
        kw["stop_angle"] = step.stop_value  # [rad]
    return kw


class TourRunner:
    """Executes one Tour against a backend, recording as it goes."""

    def __init__(self, backend: Any, recorder: Recorder, *,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep):
        self._b = backend
        self._r = recorder
        self._clock = clock
        self._sleep = sleep
        self._move_seq = 0
        # All records, appended by bus subscription -- EXPECT's haystack.
        self._records: list[dict] = []
        recorder.subscribe(self._records.append)
        # seq_file of the previous directive's own tx record -- EXPECT's
        # window anchor ("between the previous command and the timeout").
        self._anchor_seq = 0

    # -- helpers --------------------------------------------------------

    def _next_move_id(self) -> int:
        self._move_seq += 1
        return _MOVE_ID_BASE + self._move_seq

    def _drain(self) -> list:
        """THE single destructive telemetry drain."""
        return self._b.read_pending_frames()

    def _wait_for_ack(self, move_id: int, timeout: float) -> bool:  # [s]
        deadline = self._clock() + timeout
        while self._clock() < deadline:
            for frame in self._drain():
                for ack in (frame.acks or []):
                    if ack.corr_id == move_id:
                        return True
            self._sleep(_POLL)
        return False

    def _wait_at_rest(self, timeout: float = _STOP_TIMEOUT) -> bool:  # [s]
        quiet = 0
        deadline = self._clock() + timeout
        while self._clock() < deadline:
            for frame in self._drain():
                if frame.flags is not None and not (frame.flags & _ACTIVE_BIT):
                    quiet += 1
                    if quiet >= _STOP_SETTLE_FRAMES:
                        return True
                else:
                    quiet = 0
            self._sleep(_POLL)
        return False

    # -- step executors -------------------------------------------------

    def _run_segment(self, moves: list[MoveStep]) -> list[StepResult]:
        """Run consecutive MoveSteps with one-leg lookahead."""
        results: list[StepResult] = []
        ids: list[tuple[MoveStep, int]] = []
        for i, step in enumerate(moves):
            move_id = self._next_move_id()
            kw = _move_kwargs(step, move_id, replace=(i == 0))
            corr_id = self._b.move(**kw)
            self._r.tx_cmd("MOVE", {**kw, "corr_id": corr_id,
                                    "line_no": step.line_no})
            self._set_anchor()
            ids.append((step, move_id))
            # Lookahead depth 1: wait for the PREVIOUS move before sending
            # the one after next.
            if i >= 1:
                prev_step, prev_id = ids[i - 1]
                ok = self._wait_for_ack(prev_id, prev_step.timeout + 5.0)
                results.append(StepResult(prev_step.line_no, "move", ok,
                                          "" if ok else "no completion ack"))
                if not ok:
                    raise TourFailure(f"line {prev_step.line_no}: move never "
                                      f"completed (id {prev_id})")
        last_step, last_id = ids[-1]
        ok = self._wait_for_ack(last_id, last_step.timeout + 5.0)
        results.append(StepResult(last_step.line_no, "move", ok,
                                  "" if ok else "no completion ack"))
        if not ok:
            raise TourFailure(
                f"line {last_step.line_no}: move never completed (id {last_id})")
        return results

    def _run_stop(self, step: StopStep) -> StepResult:
        corr_id = self._b.stop()
        self._r.tx_cmd("STOP", {"corr_id": corr_id, "line_no": step.line_no})
        self._set_anchor()
        ok = self._wait_at_rest()
        if ok and step.dwell > 0.0:
            self._sleep(step.dwell)
        return StepResult(step.line_no, "stop", ok,
                          "" if ok else "never came to rest")

    def _run_dbg(self, step: DbgStep) -> StepResult:
        line = "DBG:" + step.text
        self._b.send_line(line)
        self._r.tx_cmd("DBG", {"text": step.text, "line_no": step.line_no},
                       plane="cleartext")
        self._set_anchor()
        return StepResult(step.line_no, "dbg", True, step.text)

    def _run_send(self, step: SendStep) -> StepResult:
        line = step.verb + (":" + step.data if step.data else "")
        self._b.send_line(line)
        self._r.tx_cmd(step.verb, {"data": step.data,
                                   "line_no": step.line_no},
                       plane="cleartext")
        self._set_anchor()
        return StepResult(step.line_no, "send", True, step.verb)

    def _run_expect(self, step: ExpectStep) -> StepResult:
        try:
            program = jq.compile(step.query)
        except Exception as exc:
            return StepResult(step.line_no, "expect", False,
                              f"bad query: {exc}")

        def matches(rec: dict) -> bool:
            try:
                return bool(program.input_value(rec).first())
            except Exception:
                return False

        anchor = self._anchor_seq
        start = self._clock()
        deadline = start + step.timeout
        scanned = 0
        matched: int | None = None
        while self._clock() < deadline and matched is None:
            snapshot = self._records  # append-only; len() is safe
            while scanned < len(snapshot):
                rec = snapshot[scanned]
                scanned += 1
                if rec.get("seq_file", 0) >= anchor and matches(rec):
                    matched = rec["seq_file"]
                    break
            if matched is None:
                self._sleep(_POLL)
        waited = self._clock() - start
        ok = matched is not None
        self._r.expect_result(step.query, ok, matched, waited, step.line_no)
        return StepResult(step.line_no, "expect", ok,
                          step.query if ok else f"no match in {step.timeout}s: "
                          + step.query)

    def _run_camfix(self, step: CamfixStep) -> StepResult:
        pose = self._b.true_pose()
        if pose is None:
            self._r.camera_fix({"ok": False, "error": "no truth source",
                                "line_no": step.line_no})
            return StepResult(step.line_no, "camfix", False, "no truth source")
        dx = pose["x"] - step.x
        dy = pose["y"] - step.y
        err = math.hypot(dx, dy)
        ok = err <= step.radius
        detail = f"pos err {err:.1f} mm (radius {step.radius:.0f})"
        heading_err = None
        if step.heading is not None:
            heading_err = _norm_angle(pose["h"] - step.heading)
            h_ok = abs(heading_err) <= step.tol
            ok = ok and h_ok
            detail += f", yaw err {math.degrees(heading_err):.1f} deg"
        self._r.camera_fix({
            "source": self._b.name, "x": pose["x"], "y": pose["y"],
            "heading": pose["h"],
            "target": {"x": step.x, "y": step.y, "radius": step.radius,
                       "heading": step.heading, "tol": step.tol},
            "error": err, "heading_error": heading_err, "ok": ok,
            "line_no": step.line_no})
        self._set_anchor()
        return StepResult(step.line_no, "camfix", ok, detail)

    def _set_anchor(self) -> None:
        # Next EXPECT's window opens at the record just written (the
        # previous directive's own tx record).
        if self._records:
            self._anchor_seq = self._records[-1]["seq_file"]

    # -- the run --------------------------------------------------------

    def run(self, tour: Tour) -> TourRunResult:
        result = TourRunResult(tour=tour.name, ok=True)
        steps = list(tour.steps)
        i = 0
        try:
            while i < len(steps):
                step = steps[i]
                if isinstance(step, MoveStep):
                    segment = [step]
                    while i + 1 < len(steps) and isinstance(steps[i + 1],
                                                            MoveStep):
                        i += 1
                        segment.append(steps[i])
                    self._r.tx_step("segment", segment[0].line_no,
                                    {"moves": len(segment)})
                    result.steps.extend(self._run_segment(segment))
                elif isinstance(step, StopStep):
                    r = self._run_stop(step)
                    result.steps.append(r)
                    if not r.ok:
                        raise TourFailure(f"line {step.line_no}: stop failed")
                elif isinstance(step, DwellStep):
                    self._r.tx_step("dwell", step.line_no,
                                    {"seconds": step.seconds})
                    self._sleep(step.seconds)
                    result.steps.append(StepResult(step.line_no, "dwell", True))
                elif isinstance(step, DbgStep):
                    result.steps.append(self._run_dbg(step))
                elif isinstance(step, SendStep):
                    result.steps.append(self._run_send(step))
                elif isinstance(step, ExpectStep):
                    r = self._run_expect(step)
                    result.steps.append(r)
                    if not r.ok:
                        raise TourFailure(
                            f"line {step.line_no}: EXPECT failed: {r.detail}")
                elif isinstance(step, CamfixStep):
                    r = self._run_camfix(step)
                    result.steps.append(r)
                    if not r.ok:
                        raise TourFailure(
                            f"line {step.line_no}: CAMFIX failed: {r.detail}")
                i += 1
        except TourFailure as exc:
            self._b.estop()
            self._r.note("tour failed; estop sent", reason=str(exc))
            result.ok = False
        except BaseException:
            self._b.estop()
            raise
        return result


def _norm_angle(a: float) -> float:  # [rad] -> (-pi, pi]
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:16]


def _git_sha(repo_root: Path) -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                              cwd=repo_root, capture_output=True, text=True,
                              timeout=5).stdout.strip()
    except Exception:
        return "?"


def build_run_meta(tour: Tour, *, tier: str, robot_config: str,
                   argv: list[str]) -> dict:
    root = Path(__file__).resolve().parents[3]
    cfg = Path(robot_config)
    return {
        "schema_version": 1,
        "tier": tier,
        "tour_file": tour.source,
        "tour_sha256": hashlib.sha256(tour.text.encode()).hexdigest()[:16],
        "tour_text": tour.text,
        "robot_config": str(cfg),
        "robot_config_sha256": _sha256(cfg) if cfg.exists() else "?",
        "host_git_sha": _git_sha(root),
        "started_wall": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "argv": argv,
    }
