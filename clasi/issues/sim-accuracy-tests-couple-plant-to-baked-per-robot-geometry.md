# sim accuracy tests couple the plant to BAKED per-robot geometry

`src/tests/sim/test_motor_primitive.py`'s two zero-error accuracy checks
(`test_distance_encoder_and_otos_match_truth`,
`test_heading_encoder_and_otos_match_truth`) fail since 2026-08-14 by
7.4 mm / 6.1 deg. Nothing in the kinematics broke; the REAL ROBOT changed.

Mechanism: the sim binary compiles the baked boot config, which is generated
from `tovez.json` and committed. GEOMETRY (`trackwidth`, `rotational_slip`)
is deliberately BOOT-ONLY -- `configure_from_robot()`'s Tier 1 explicitly
excludes it ("a live push must never reach trackWidth"), and the OTOS scale
group is likewise taken from the bake. So the firmware under test runs
tovez's CURRENT bench-measured geometry (trackwidth 115, linear_scale 1.0,
angular_scale 0.9908, offset_yaw -0.077 -- all re-measured after the O-ring
wheel swap), while the test configures the PLANT and its expectations from
`tovez_nocal.json`-era constants (trackwidth 128, module constants
TRACK_WIDTH/TICKS_PER_MM). The test therefore measures the drift between two
robots: the one measured yesterday and the one measured today.

Changing the module constants alone does NOT fix it (tried: error identical
at 7.45 mm) -- the plant's own track width and the firmware's baked values
have more than one coupling point, and at least one is not reachable from the
test's parameters.

Proper fix (not attempted at 03:30): derive BOTH sides from ONE source --
either compile the sim harness against `tovez_nocal.json`'s bake instead of
`tovez.json`'s, or make the plant/test constants load from the same JSON that
generated the bake the binary carries. Configuration-discipline's own rule
("every value the robot uses comes from the file") wants the second.

Same family, third member: `src/tests/sim/system/
test_straight_leg_crab_regression.py` -- the firmware now bakes tovez's
bench-fitted ASYMMETRIC Stage-A wheel gains (left 0.8 / right 0.9567,
re-fitted 2026-08-14 to cancel the real drivetrain's veer), but the sim
plant is symmetric, so the correction has nothing to correct and CAUSES the
crab the test guards against. Baked bench-fit values and an idealized plant
cannot both be "the robot".

Until then all three tests are strict xfails referencing this issue.
