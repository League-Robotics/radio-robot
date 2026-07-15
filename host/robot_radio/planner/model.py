"""robot_radio.planner.model -- the sprint's live-tunable parameter surface.

`PlannerParams` is the ONE place `planner/executor.py` and `planner/
heading.py` read their numeric knobs from -- streaming cadence, velocity/
acceleration ceilings, heading-loop gains/clamp, and the actuation-latency
constant. It exists so every other planner module reads from one place
instead of scattering constants (binding requirement #9 of
`host-planner-design-lessons-from-drive-v2-review.md`: "everything
tunable live").

Live-tunability, concretely
----------------------------
`PlannerParams` is a plain, MUTABLE dataclass -- not frozen. A caller (a
REPL, a bench script, a test) mutates a field directly

    params.heading_kp = 8.0

and `HeadingCorrector`/`StreamingExecutor` pick up the new value on their
VERY NEXT read -- neither module ever caches a field into a local variable
at construction time; every tick re-reads `self._params.<field>` fresh.
This is what "no code redeploy" means for a host-side Python parameter (as
opposed to a wire-pushed firmware `SET`, which is a different, already-
solved live-tunability story -- `MotorConfigPatch`/`PlannerConfigPatch`
apply, ticket 002).

`PlannerParams.load()` is a SEPARATE, optional convenience on top of that:
it builds a `PlannerParams` from field defaults, then layers a JSON file
(mirroring `data/robots/*.json`'s convention -- a flat object whose keys
are this dataclass's own field names) and/or `PLANNER_<FIELD>` environment
variables on top. Neither layer is required -- `PlannerParams()` alone is
a fully valid, usable instance -- `load()` exists for reproducible/
scripted starting points, and can be called again at any time (no process
restart, no code change) to pick up new values.
"""

from __future__ import annotations

import dataclasses
import json
import os
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any


@dataclass
class PlannerParams:
    """Every live-tunable numeric knob `executor.py`/`heading.py` use.

    Grouped by the binding requirement each serves (see this sprint's
    architecture-update.md Step 6 disposition table, reproduced in ticket
    005's own description).
    """

    # -- Streaming cadence + latency (binding requirements #8, #9) --------
    streaming_interval: float = 0.15  # [s] twist() pacing interval -- the
    # ONLY empirically soak-tested paced rate (ack-ring-intermittent-
    # delivery-gap.md finding 2); NOT the ~25Hz telemetry cadence (ticket
    # 001), which is unrelated and unvalidated for commands (architecture-
    # update.md Decision 6).
    link_latency_margin: float = 0.15  # [s] extra deadman margin added on
    # top of streaming_interval when computing each twist()'s duration, so
    # the firmware deadman never expires between two on-time sends. Default
    # chosen so the resulting duration (streaming_interval +
    # link_latency_margin) lands at ~2x streaming_interval -- inside the
    # "duration ~= 2-3x cadence" safety-net band a died host still trips
    # the deadman within, at most, a couple of missed ticks.
    latency_tau: float = 0.13  # [s] ~130ms actuation-lag time constant
    # (binding requirement #8) -- a first-class, live-tunable parameter,
    # consumed by the heading loop's own correction timing.

    # -- Velocity/acceleration ceilings ------------------------------------
    v_max: float = 200.0  # [mm/s] hard ceiling on |v_x| -- re-validated by
    # the executor immediately before every twist() send (binding
    # requirement #5), independent of profile.py's own boundary validation.
    a_max: float = 500.0  # [mm/s^2] straight accel/decel ceiling, consumed
    # by whatever caller builds a profile.py ProfileLimits for a straight
    # leg (not read by the executor itself -- profile.py owns shape).
    omega_max: float = 2.0  # [rad/s] hard ceiling on |omega| (turn rate +
    # heading trim combined) -- re-validated the same way as v_max.
    alpha_max: float = 6.0  # [rad/s^2] turn accel/decel ceiling, consumed
    # by whatever caller builds a profile.py ProfileLimits for a turn leg.

    # -- Heading-correction loop (binding requirements #9, #10) ------------
    heading_kp: float = 0.4  # 107-001: bench-proven default (was 2.0's
    # "starting point" -- ticket 106-006's real bench session found the
    # shipped 2.0/0.5 pair saturated the correction trim on this rig's
    # high-inertia proxy load, landing a 60 deg turn at ~79 deg (+19 deg,
    # ~+32% overshoot)). 0.4 measured much better across 4 clean-gain bench
    # runs: -4.09, -1.18, +2.10, +15.75 deg landing error (one outlier --
    # see ticket 107-001's own Completion Notes; this is a real improvement,
    # NOT a fully solved gain -- a tight (+-3 deg) tolerance across
    # repeated runs is explicitly deferred to a later, dedicated
    # gain-tuning session, per both issues' own "Recommended follow-up").
    heading_ki: float = 0.0
    heading_kd: float = 0.0
    heading_omega_clamp: float = 0.2  # [rad/s] symmetric PID output clamp,
    # 107-001 bench-proven default (was 0.5) -- carries forward the deleted
    # on-robot heading loop's own lesson
    # (heading-loop-output-clamp-and-velocity-resonance.md Part 1): an
    # unclamped correction over-drove the wheels into the ~140mm/s
    # resonance band ticket 002 tames. Lowering the clamp to 0.2 bounds the
    # maximum "catch-up" contribution that was driving the 106-006
    # saturation/overshoot failure mode; see heading_kp's own comment above
    # for the bench evidence and the carried-forward outlier risk.

    # -- Completion / bounded-overshoot (binding requirements #1, #6) ------
    completion_tolerance_linear: float = 5.0  # [mm] how close to the
    # signed target distance counts as "reached" at run end.
    completion_tolerance_angular: float = 0.02  # [rad] same, for turns.
    overshoot_bound_linear: float = 30.0  # [mm] outer bound (BOTH
    # directions of the signed target) -- exceeding it mid-run is a
    # logged failure, not silently accepted (binding requirement #6).
    overshoot_bound_angular: float = 0.1  # [rad] same, for turns.

    # ------------------------------------------------------------------
    # Live-editable load/override plumbing
    # ------------------------------------------------------------------

    @classmethod
    def load(cls, path: str | Path | None = None,
             env_prefix: str = "PLANNER_") -> "PlannerParams":
        """Build a `PlannerParams` from field defaults, then layer a JSON
        file and/or `<env_prefix><FIELD_NAME>` environment variables on
        top -- both layers optional, applied in that order (JSON first,
        env last, so an environment variable always wins for a session).

        `path`: an explicit JSON file path, OR (if None) the value of the
        `PLANNER_CONFIG` environment variable, OR (if that is also unset)
        no file at all -- field defaults only. The JSON file is a flat
        object whose keys must be a subset of this dataclass's own field
        names (mirroring `data/robots/*.json`'s per-key convention); an
        unknown key raises `ValueError` immediately -- this project's
        "flag rather than silently ignore" discipline (mirrors
        `NezhaProtocol.set_config()`'s own unknown-key handling), not a
        best-effort partial apply.

        Every `<env_prefix><FIELD_NAME>` variable (e.g. `PLANNER_HEADING_KP`)
        present in the environment overrides the corresponding field,
        parsed as `float`. This is the "no code redeploy, no process
        restart even" tuning path -- call `load()` again at any time (a
        bench REPL, a test) to pick up newly-exported values.
        """
        overrides: dict[str, Any] = {}

        json_path = Path(path) if path else None
        if json_path is None:
            env_path = os.environ.get("PLANNER_CONFIG")
            json_path = Path(env_path) if env_path else None

        field_names = {f.name for f in fields(cls)}

        if json_path is not None:
            data = json.loads(json_path.read_text())
            unknown = sorted(set(data) - field_names)
            if unknown:
                raise ValueError(
                    f"PlannerParams.load(): unknown key(s) {unknown!r} in "
                    f"{json_path}")
            overrides.update(data)

        for f in fields(cls):
            env_name = f"{env_prefix}{f.name.upper()}"
            if env_name in os.environ:
                overrides[f.name] = float(os.environ[env_name])

        return dataclasses.replace(cls(), **overrides)
