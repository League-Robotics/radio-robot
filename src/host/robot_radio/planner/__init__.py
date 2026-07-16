"""robot_radio.planner -- host-side trajectory planning package.

Fresh package (``architecture-update.md`` Decision 3 of sprint 106) -- does
NOT reuse ``robot_radio.nav``, whose own fate is a separate, future
stakeholder call.

    from robot_radio.planner import ProfileLimits, ProfileSetpoint
    from robot_radio.planner import profile_for_distance, profile_for_turn

This ticket (106-004) ships only ``profile.py`` -- the pure trapezoidal
profile generator. ``executor.py``/``heading.py``/``model.py`` (the
streaming executor, heading-correction loop, and live-tunable parameter
surface) are ticket 106-005.
"""

from robot_radio.planner.profile import (
    ProfileLimits,
    ProfileSetpoint,
    profile_for_distance,
    profile_for_turn,
)

__all__ = [
    "ProfileLimits",
    "ProfileSetpoint",
    "profile_for_distance",
    "profile_for_turn",
]
