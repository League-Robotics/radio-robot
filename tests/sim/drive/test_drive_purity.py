"""Tier-0 purity/property tests (ticket 100-006 AC, SUC-002):

  - determinism: the same (plan, StepInput, StepState) fed to step() twice
    produces byte-identical StepOutput, across arc/pivot/velocity-mode plans.
  - StepState round-trips through the ctypes struct boundary UNCHANGED when
    the tick would not alter it (a fresh RUNNING-phase tick, perfectly on
    reference, triggers no replan/dwell/settling bookkeeping at all).
"""
from __future__ import annotations

import pytest
from _common import make_limits, TRACKWIDTH
from drive import (BodyState, Drive, Goal, PlanRequest, Pose, Status, StepInput, StepState,
                    Twist, Verdict, WheelState)


def test_step_is_deterministic_for_arc_pivot_and_velocity_mode(build_drive_lib):
    limits = make_limits()
    with Drive(limits, TRACKWIDTH) as drive:
        cases = [
            ("arc", PlanRequest(goal=Goal(arc_length=500.0, delta_heading=0.3), start=Pose())),
            ("pivot", PlanRequest(goal=Goal(arc_length=0.0, delta_heading=1.2), start=Pose())),
        ]
        for label, request in cases:
            result = drive.plan(request)
            assert result.verdict == Verdict.OK, (label, result.verdict)
            with result.plan as plan:
                t = plan.duration() * 0.4
                ref = plan.reference_at(t)
                step_input = StepInput(
                    t=t,
                    measured=BodyState(pose=Pose(ref.x + 8.0, ref.y - 3.0, ref.theta + 0.01),
                                        twist=Twist(v_x=ref.v - 5.0, omega=ref.omega)),
                    left=WheelState(position=12.3, velocity=ref.v, position_valid=True,
                                     velocity_valid=True),
                    right=WheelState(position=15.1, velocity=ref.v, position_valid=True,
                                      velocity_valid=True),
                    pose_step=5.0,
                    pose_step_theta=0.001,
                )
                state = StepState(sustain_start=0.2, replan_count=1)

                out_a, state_a = plan.step(step_input, state)
                out_b, state_b = plan.step(step_input, state)

                assert out_a == out_b, f"{label}: same (plan, input, state) -> byte-identical StepOutput"
                assert state_a == state_b, f"{label}: same (plan, input, state) -> byte-identical StepState"

        vel_result = drive.plan_velocity(Twist(v_x=180.0, omega=0.15), deadman=2500.0,
                                          current=BodyState())
        assert vel_result.verdict == Verdict.OK, vel_result.verdict
        with vel_result.plan as plan:
            step_input = StepInput(t=1.0, measured=BodyState(twist=Twist(v_x=100.0, omega=0.05)))
            state = StepState()

            out_a, state_a = plan.step(step_input, state)
            out_b, state_b = plan.step(step_input, state)

            assert out_a == out_b, "velocity-mode: same (plan, input, state) -> byte-identical StepOutput"
            assert state_a == state_b, "velocity-mode: same (plan, input, state) -> byte-identical StepState"


def test_step_state_round_trips_unchanged_when_tick_does_not_alter_it(build_drive_lib):
    limits = make_limits()
    with Drive(limits, TRACKWIDTH) as drive:
        result = drive.plan(PlanRequest(goal=Goal(arc_length=500.0, delta_heading=0.0), start=Pose()))
        assert result.verdict == Verdict.OK, result.verdict
        with result.plan as plan:
            # Perfectly on-reference, mid-segment: no trim saturation, no
            # envelope violation, no pose-fix step, not exhausted -- nothing
            # in evaluate()'s RUNNING branch touches dwellStart/lastReplan/
            # replanCount/settling, and sustainStart is re-assigned its OWN
            # -1.0 (the "trigger never activated" branch), so a bit-exact
            # StepState() in should produce a bit-exact StepState() out.
            t = plan.duration() * 0.5
            ref = plan.reference_at(t)
            step_input = StepInput(
                t=t,
                measured=BodyState(pose=Pose(ref.x, ref.y, ref.theta),
                                    twist=Twist(v_x=ref.v, omega=ref.omega)),
                left=WheelState(velocity=ref.v, position_valid=True, velocity_valid=True),
                right=WheelState(velocity=ref.v, position_valid=True, velocity_valid=True),
            )
            state_in = StepState()

            out, state_out = plan.step(step_input, state_in)

            assert out.status == Status.RUNNING, out.status
            assert state_out == state_in, (
                f"StepState must round-trip UNCHANGED for a tick that alters nothing: "
                f"in={state_in!r} out={state_out!r}"
            )


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
