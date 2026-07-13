"""Tier-0 fuzz test (ticket 100-006 AC): >=1000 generated StepInput/config
combinations fed to step() assert zero NaN/Inf anywhere in StepOutput.

Deterministic seed (reproducible failures); combos are spread across three
plan kinds (arc, pivot, velocity-mode) and sweep StepInput's full field
range -- including deliberately out-of-band values (huge poses, negative
time, saturating poseStep) that a real caller should never produce but a
defensive ABI/control-law must not blow up on.
"""
from __future__ import annotations

import math
import random

import pytest
from _common import make_limits, TRACKWIDTH
from drive import BodyState, Drive, Goal, PlanRequest, Pose, StepInput, StepState, Twist, Verdict, WheelState

_COMBOS = 1200
_SEED = 20260712


def _finite(x: float) -> bool:
    return math.isfinite(x)


def _assert_step_output_finite(out, label: str) -> None:
    assert _finite(out.command.left), f"{label}: command.left not finite: {out.command.left}"
    assert _finite(out.command.right), f"{label}: command.right not finite: {out.command.right}"
    r = out.record
    for name in ("e_along", "e_cross", "e_theta", "v_trim", "omega_trim", "v_cmd", "omega_cmd",
                 "wheel_left", "wheel_right"):
        value = getattr(r, name)
        assert _finite(value), f"{label}: record.{name} not finite: {value}"


def _random_step_input(rng: random.Random) -> StepInput:
    return StepInput(
        t=rng.uniform(-2.0, 15.0),
        measured=BodyState(
            pose=Pose(x=rng.uniform(-1.0e4, 1.0e4), y=rng.uniform(-1.0e4, 1.0e4),
                      h=rng.uniform(-10.0, 10.0)),
            twist=Twist(v_x=rng.uniform(-2000.0, 2000.0), v_y=0.0, omega=rng.uniform(-50.0, 50.0)),
        ),
        left=WheelState(position=rng.uniform(-1.0e4, 1.0e4), velocity=rng.uniform(-2000.0, 2000.0),
                         position_valid=True, velocity_valid=True),
        right=WheelState(position=rng.uniform(-1.0e4, 1.0e4), velocity=rng.uniform(-2000.0, 2000.0),
                          position_valid=True, velocity_valid=True),
        pose_step=rng.uniform(0.0, 200.0),
        pose_step_theta=rng.uniform(-1.0, 1.0),
    )


def _random_step_state(rng: random.Random) -> StepState:
    def maybe_hold() -> float:
        return -1.0 if rng.random() < 0.5 else rng.uniform(0.0, 10.0)

    return StepState(
        dwell_start=maybe_hold(),
        sustain_start=maybe_hold(),
        last_replan=maybe_hold(),
        replan_count=rng.randint(0, 3),
        settling=rng.choice([True, False]),
    )


def test_fuzz_zero_nan_or_inf_in_step_output(build_drive_lib):
    limits = make_limits()
    rng = random.Random(_SEED)

    with Drive(limits, TRACKWIDTH) as drive:
        arc = drive.plan(PlanRequest(goal=Goal(arc_length=500.0, delta_heading=0.3), start=Pose()))
        pivot = drive.plan(PlanRequest(goal=Goal(arc_length=0.0, delta_heading=1.2), start=Pose()))
        velocity = drive.plan_velocity(Twist(v_x=200.0, omega=0.2), deadman=3000.0,
                                        current=BodyState())
        assert arc.verdict == Verdict.OK, arc.verdict
        assert pivot.verdict == Verdict.OK, pivot.verdict
        assert velocity.verdict == Verdict.OK, velocity.verdict

        plans = [("arc", arc.plan), ("pivot", pivot.plan), ("velocity", velocity.plan)]
        try:
            checked = 0
            for i in range(_COMBOS):
                label, plan = plans[i % len(plans)]
                step_input = _random_step_input(rng)
                state = _random_step_state(rng)

                out, new_state = plan.step(step_input, state)
                checked += 1

                _assert_step_output_finite(out, f"combo {i} ({label})")
                assert _finite(new_state.dwell_start), f"combo {i} ({label}): dwell_start not finite"
                assert _finite(new_state.sustain_start), f"combo {i} ({label}): sustain_start not finite"
                assert _finite(new_state.last_replan), f"combo {i} ({label}): last_replan not finite"

            assert checked >= 1000, checked
        finally:
            for _, plan in plans:
                plan.close()


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
