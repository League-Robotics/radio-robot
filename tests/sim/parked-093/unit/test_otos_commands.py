"""Off-hardware acceptance proof for ticket 084-008 (SUC-007): registers the
seven OTOS verbs (``OI``/``OZ``/``OR``/``OP``/``OV``/``OL``/``OA``) —
already fully specified in ``docs/protocol-v2.md`` §11 — resolving them
against ``hardware.odometer()`` (the sprint-082 seam), against
``Subsystems::SimHardware``/``Hal::SimOdometer``.

Drives ``libfirmware_host`` through the full wire dispatch (``Sim.command()``)
-- ``CommandProcessor`` -> ``source/commands/otos_commands.cpp`` ->
``Hal::Odometer``/``Hal::SimOdometer`` -- mirroring this directory's existing
``test_pose_commands.py``/``test_motion_commands_*`` pattern.

The companion "ERR nodev against Subsystems::NezhaHardware" acceptance
criterion (all seven verbs, explicitly, one test each) is proven separately
in ``test_otos_commands_nodev.py``/``otos_commands_harness.cpp`` -- this
tree's ``libfirmware_host`` never compiles ``subsystems/nezha_hardware.cpp``
(see that file's own doc comment), so NezhaHardware cannot be reached
through the ``Sim`` wrapper at all.
"""

import math

import pytest

_CDEG_TO_RAD = math.pi / 18000.0


# ---------------------------------------------------------------------------
# OI -- re-initialise OTOS signal processing / tracking. No dedicated
# real-vs-sim register model exists to assert beyond "acks OK" and "does not
# move the accumulator" (mirrors OR's own no-position-effect, below).
# ---------------------------------------------------------------------------


def test_oi_acks_ok_against_the_sim(sim):
    assert sim.command("OI").strip() == "OK oi"


def test_oi_does_not_move_the_accumulator(sim):
    sim.command("OV 250 -125 900")
    before = sim.otos_pose()

    assert sim.command("OI").strip() == "OK oi"

    after = sim.otos_pose()
    assert after == pytest.approx(before, abs=1e-6), (
        "OI re-initialises tracking, but must not itself move the reported "
        "position (matches OtosSensor::init()'s real register writes, which "
        "never touch POSITION_XL)"
    )


# ---------------------------------------------------------------------------
# OZ -- zero the OTOS world-frame position to the current location
# (setPositionRaw(0, 0, 0)).
# ---------------------------------------------------------------------------


def test_oz_acks_ok_and_zeroes_the_accumulator(sim):
    sim.command("OV 500 -300 450")
    x, y, h = sim.otos_pose()
    assert (x, y, h) != (0.0, 0.0, 0.0), "expected a nonzero accumulator before OZ"

    assert sim.command("OZ").strip() == "OK oz"

    x, y, h = sim.otos_pose()
    assert abs(x) < 1e-3
    assert abs(y) < 1e-3
    assert abs(h) < 1e-3


# ---------------------------------------------------------------------------
# OR -- reset OTOS Kalman/tracking state (resetTracking()). This class has no
# separate Kalman filter to inspect directly, so its effect is proven
# observably: it rebaselines the ground-truth sampling accumulator, so a
# `sim.set_true_pose()` teleport that happens WHILE the odometer has not
# ticked does not fabricate a phantom jump on the next tick (the exact hazard
# a genuine hardware tracking reset also guards against). Without OR, the
# next tick's delta would be computed against the STALE pre-teleport
# baseline, producing a large spurious jump.
# ---------------------------------------------------------------------------


def test_or_acks_ok(sim):
    assert sim.command("OR").strip() == "OK or"


def test_or_resets_tracking_no_phantom_jump_after_a_true_pose_teleport(sim):
    # Establish a genuine baseline: at least one real tick so the ground-
    # truth sampling accumulator is baselined at a KNOWN starting pose.
    sim.tick_for(24)
    x0, y0, _h0 = sim.otos_pose()

    # Teleport the plant's true pose far away WITHOUT letting the odometer
    # observe an intervening tick -- exactly the situation OR's ground-truth
    # rebaseline exists to guard against.
    sim.set_true_pose(2000.0, 2000.0, 0.0)

    assert sim.command("OR").strip() == "OK or"

    # The first tick after OR only re-establishes the ground-truth baseline
    # at the NEW (teleported) true pose -- no delta is integrated. Without
    # OR, this same tick would diff the teleported pose against the STALE
    # pre-teleport baseline, fabricating a ~2000mm jump in the accumulator.
    sim.tick_for(24)
    x1, y1, _h1 = sim.otos_pose()

    assert abs(x1 - x0) < 5.0, f"OR should have suppressed the phantom jump, got dx={x1 - x0}"
    assert abs(y1 - y0) < 5.0, f"OR should have suppressed the phantom jump, got dy={y1 - y0}"


# ---------------------------------------------------------------------------
# OV -- set the OTOS world-frame position (setPositionRaw(x, y, h)). x, y:
# mm; h: cdeg (docs/protocol-v2.md §11).
# ---------------------------------------------------------------------------


def test_ov_replies_setpos_with_the_supplied_values(sim):
    assert sim.command("OV 500 -300 450").strip() == "OK setpos x=500 y=-300 h=450"


def test_ov_visibly_moves_the_accumulator(sim):
    assert sim.command("OV 500 -300 450").strip() == "OK setpos x=500 y=-300 h=450"

    x, y, h = sim.otos_pose()
    assert abs(x - 500.0) < 1.0
    assert abs(y - (-300.0)) < 1.0
    assert abs(h - 450 * _CDEG_TO_RAD) < 1e-3


def test_ov_too_few_args_rejected_with_badarg(sim):
    assert sim.command("OV 500 -300").strip() == "ERR badarg"
    assert sim.command("OV 500").strip() == "ERR badarg"
    assert sim.command("OV").strip() == "ERR badarg"


# ---------------------------------------------------------------------------
# OP -- read the current OTOS world-frame position. x, y: mm; h: cdeg.
# ---------------------------------------------------------------------------


def test_op_reads_back_zero_at_boot(sim):
    assert sim.command("OP").strip() == "OK pos x=0 y=0 h=0"


def test_op_reads_back_exactly_what_ov_set(sim):
    sim.command("OV 500 -300 450")
    assert sim.command("OP").strip() == "OK pos x=500 y=-300 h=450"


def test_op_reads_back_zero_after_oz(sim):
    sim.command("OV 500 -300 450")
    sim.command("OZ")
    assert sim.command("OP").strip() == "OK pos x=0 y=0 h=0"


# ---------------------------------------------------------------------------
# OL/OA -- get or set the OTOS linear/angular scalar calibration registers
# (int8_t). Deliberate store-and-echo against Hal::SimOdometer this sprint --
# explicitly asserted to have NO physical effect on the accumulator
# (architecture-update.md (084) Decision 5's Consequences).
# ---------------------------------------------------------------------------


def test_ol_reads_back_zero_at_boot(sim):
    assert sim.command("OL").strip() == "OK linear scalar=0"


def test_oa_reads_back_zero_at_boot(sim):
    assert sim.command("OA").strip() == "OK angular scalar=0"


def test_ol_sets_and_echoes_with_no_physical_effect(sim):
    sim.command("OV 100 200 300")
    before = sim.otos_pose()

    assert sim.command("OL 42").strip() == "OK linear scalar=42"
    # Read-back (no argument) echoes the same stored value.
    assert sim.command("OL").strip() == "OK linear scalar=42"

    sim.tick_for(200)
    after = sim.otos_pose()
    assert after == pytest.approx(before, abs=1e-6), (
        "OL must have no physical effect on the accumulator (Decision 5's "
        "store-and-echo Consequences)"
    )


def test_oa_sets_and_echoes_with_no_physical_effect(sim):
    sim.command("OV 100 200 300")
    before = sim.otos_pose()

    assert sim.command("OA -17").strip() == "OK angular scalar=-17"
    assert sim.command("OA").strip() == "OK angular scalar=-17"

    sim.tick_for(200)
    after = sim.otos_pose()
    assert after == pytest.approx(before, abs=1e-6), (
        "OA must have no physical effect on the accumulator (Decision 5's "
        "store-and-echo Consequences)"
    )


def test_ol_and_oa_shadows_are_independent(sim):
    """Setting OL must not disturb OA's own shadowed value, and vice versa
    -- OtosCommandState::configShadow holds both fields together, so a
    read-modify-write bug here would show up as one clobbering the other."""
    sim.command("OL 10")
    sim.command("OA 20")
    assert sim.command("OL").strip() == "OK linear scalar=10"
    assert sim.command("OA").strip() == "OK angular scalar=20"

    sim.command("OL 99")
    assert sim.command("OL").strip() == "OK linear scalar=99"
    assert sim.command("OA").strip() == "OK angular scalar=20"
