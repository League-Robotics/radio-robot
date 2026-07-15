"""robot_radio.planner.heading -- outer heading-correction loop.

`HeadingCorrector` wraps a reused `controllers.pid.PID` instance
(architecture-update.md Decision 7 -- no new PID implementation) and turns
a commanded heading + the latest drained telemetry frame into a CLAMPED
omega trim. `StreamingExecutor` adds this trim onto a profile setpoint's
own `omega` every tick -- holding heading on a straight (commanded heading
stays at the run's initial heading, since a straight profile's own omega
is 0 throughout) and tracking the profile's own planned heading trajectory
on a turn (commanded heading advances by the profile's own omega each
tick) -- the SAME mechanism serves both, with no special-casing in this
module (the executor owns advancing "commanded heading"; this module only
ever compares one commanded value against one measured value).

Pose source selection (binding requirement carried by SUC-028/SUC-029)
------------------------------------------------------------------------
Reads `robot_config.geometry.otos_untrusted` ONCE, at construction, to fix
the measured-heading source for the corrector's entire lifetime:

- `otos_untrusted=True`  -> `TLMFrame.pose` (encoder-derived dead-reckoned
  heading, `App::Odometry::integrate()` on the firmware side -- see that
  method's own header for the midpoint-arc integration it performs). This
  is the bench-rig case ("rig = encoder heading") -- the rig's OTOS sensor
  sits on a mechanically decoupled 360-degree servo mount, so its reported
  pose is structurally invalid (`GeometryConfig.otos_untrusted`'s own
  docstring).
- `otos_untrusted=False` (default) -> `TLMFrame.otos` (raw OTOS pose).

Output clamp (binding requirement carried forward from the deleted
on-robot heading loop's own lesson, `heading-loop-output-clamp-and-
velocity-resonance.md` Part 1): an unclamped correction over-drove the
wheels into the ~140mm/s resonance band ticket 002 tames. `update()`
re-syncs the wrapped `PID`'s gains AND `out_min`/`out_max` from
`PlannerParams` on every call (not just at construction) so a live
mutation of `params.heading_kp`/`params.heading_omega_clamp` takes effect
on the corrector's very next tick -- binding requirement #9 ("everything
tunable live"), applied here as well as in `executor.py`.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING, Any

from robot_radio.controllers.pid import PID, normalize_angle

if TYPE_CHECKING:
    from robot_radio.planner.model import PlannerParams
    from robot_radio.robot.protocol import TLMFrame

logger = logging.getLogger(__name__)

# TLMFrame.pose/otos headings are integer centidegrees (matches
# NezhaProtocol's own _ANGLE_SCALE convention -- see protocol.py's file
# header, and note this module's own name deliberately mirrors that
# convention rather than embedding a unit pair into the identifier).
_HEADING_SCALE = math.pi / 18000.0  # [rad/cdeg]


class HeadingCorrector:
    """Encoder-or-OTOS heading feedback -> a clamped omega trim."""

    def __init__(self, params: "PlannerParams", robot_config: Any = None) -> None:
        """`robot_config`: anything exposing `.geometry.otos_untrusted`
        (duck-typed -- a real `config.robot_config.RobotConfig`, or a
        lightweight test double). `None` (no active robot config resolved)
        defaults to `otos_untrusted=False`, matching `GeometryConfig`'s own
        field default.
        """
        self._params = params
        geometry = getattr(robot_config, "geometry", None)
        otos_untrusted = bool(getattr(geometry, "otos_untrusted", False))
        self._source = "pose" if otos_untrusted else "otos"
        self._pid = PID(
            params.heading_kp, params.heading_ki, params.heading_kd,
            out_min=-params.heading_omega_clamp, out_max=params.heading_omega_clamp)

    @property
    def source(self) -> str:
        """Which `TLMFrame` field this corrector reads: `"pose"` (encoder)
        or `"otos"` (raw OTOS)."""
        return self._source

    def reset(self) -> None:
        """Clear the wrapped PID's integral/derivative history -- called by
        `StreamingExecutor.begin()` at the start of every fresh run
        (including a preemption's replan) so a new run never inherits the
        interrupted run's accumulated error state."""
        self._pid.reset()

    def measured_heading(self, frame: "TLMFrame | None") -> float | None:  # [rad]
        """This corrector's selected source, read off `frame`, in radians.
        `None` if `frame` is `None` or the selected field is absent (e.g. a
        pre-fault frame with no `has_pose`/`has_otos`) -- the executor logs
        this as a degraded-feedback condition, never crashes on it."""
        if frame is None:
            return None
        raw = frame.pose if self._source == "pose" else frame.otos
        if raw is None:
            return None
        return raw[2] * _HEADING_SCALE

    def update(self, commanded_heading: float, frame: "TLMFrame | None",
              now: float) -> float:  # [rad] [s] -> [rad/s]
        """Return the clamped omega trim to ADD onto a profile setpoint's
        own `omega` this tick.

        Re-syncs the wrapped PID's gains/clamp from `self._params` first
        (live-tunable, see this module's own docstring), then computes
        `error = normalize_angle(commanded_heading - measured_heading)` and
        runs it through the PID. Returns exactly `0.0` -- logged loudly,
        never silent -- if no measured heading is available this tick
        (binding requirement #2).
        """
        self._pid.kp = self._params.heading_kp
        self._pid.ki = self._params.heading_ki
        self._pid.kd = self._params.heading_kd
        self._pid.out_min = -self._params.heading_omega_clamp
        self._pid.out_max = self._params.heading_omega_clamp

        measured = self.measured_heading(frame)
        if measured is None:
            logger.warning(
                "HeadingCorrector.update(): no measured heading available "
                "(source=%s) -- returning zero trim this tick", self._source)
            return 0.0

        error = normalize_angle(commanded_heading - measured)
        trim = self._pid.update(error, now)
        # PID.update() already clamps to out_min/out_max -- re-clamp
        # defensively here too (never trust a single clamp site for a
        # value that ultimately commands motor hardware; the ceiling this
        # tick's own re-sync above just applied is the authoritative one).
        clamp = self._params.heading_omega_clamp
        return max(-clamp, min(clamp, trim))
