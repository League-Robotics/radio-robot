"""src/tests/unit/test_world_pose.py -- ticket 127-004.

Pure, synthetic-input unit tests for ``robot_radio.pathplan.world_pose`` --
``Transform2``/``solveTransform`` (SE(2) math) and ``WorldPose`` (the
host-side ``T_world_from_odom`` tracker, design issue T3). No hardware, no
sim binary, no camera daemon, no serial port -- every ``TLMFrame`` here is
hand-built directly (never ``from_pb2()``), matching the dataclass's own
"never decoded leaves fields at their None default" convention.

Covers (ticket acceptance criteria):
  1. Transform composition and its inverse (``Transform2.apply()``/
     ``inverse()``), pure SE(2) math.
  2. ``solveTransform()`` -- the re-anchor step in isolation.
  3. ``WorldPose.reanchor()`` updates ``T_world_from_odom`` (both the
     encoder- and OTOS-anchored transforms) correctly from a camera fix.
  4. Age-based (frame-age, ``t - age`` via ``TLMFrame.recvTime``)
     extrapolation -- a reading's reported body-frame velocity carries its
     pose forward from its own capture instant to a queried host time.
  5. Encoder-vs-OTOS divergence (``WorldPose.encoderOtosDivergence()``) on
     synthetic diverging inputs, including a heading residual that
     straddles +-pi (the project's own known wrap-unsafe-subtraction trap).
"""

from __future__ import annotations

import math

from robot_radio.nav.pose import Pose
from robot_radio.pathplan.world_pose import (
    PoseDivergence,
    Transform2,
    WorldPose,
    solveTransform,
)
from robot_radio.robot.protocol import EncoderReading, OtosReading, TLMFrame

_FLAG_OTOS_PRESENT = 1 << 0


def _encoderReading(age: int = 0) -> EncoderReading:
    return EncoderReading(position=0.0, velocity=0.0, age=age, position_epoch=0)


def _tlmFrame(*, recvTime: float, pose: "tuple[int, int, int]",
              twist: "tuple[int, int]" = (0, 0), encAge: int = 0,
              otos: "OtosReading | None" = None) -> TLMFrame:
    """Build a synthetic, hand-decoded TLMFrame -- `t` is a dummy robot-clock
    value (WorldPose.ingest() only requires it to be non-None), `recvTime`
    is the host-clock value under direct test control (never real wall
    time, so extrapolation math is deterministic)."""
    frame = TLMFrame(
        t=1000, pose=pose, twist=twist, recvTime=recvTime,
        enc_left=_encoderReading(encAge), enc_right=_encoderReading(encAge),
    )
    if otos is not None:
        frame.flags = _FLAG_OTOS_PRESENT
        frame.otos_reading = otos
    return frame


# ---------------------------------------------------------------------------
# 1. Transform2: composition and inverse.
# ---------------------------------------------------------------------------


def test_transform_apply_translates_and_rotates() -> None:
    t = Transform2(x=10.0, y=5.0, rotation=math.pi / 2)  # +90 deg

    world = t.apply(Pose(x=1.0, y=0.0, heading=0.0))

    # rotate (1, 0) by +90 deg -> (0, 1), then translate by (10, 5).
    assert math.isclose(world.x, 10.0, abs_tol=1e-9)
    assert math.isclose(world.y, 6.0, abs_tol=1e-9)
    assert math.isclose(world.heading, math.pi / 2, abs_tol=1e-9)


def test_transform_inverse_round_trips_an_arbitrary_pose() -> None:
    t = Transform2(x=-13.7, y=42.1, rotation=math.radians(37.0))
    original = Pose(x=3.2, y=-8.4, heading=math.radians(12.0))

    roundTripped = t.inverse().apply(t.apply(original))

    assert math.isclose(roundTripped.x, original.x, abs_tol=1e-9)
    assert math.isclose(roundTripped.y, original.y, abs_tol=1e-9)
    assert math.isclose(roundTripped.heading, original.heading, abs_tol=1e-9)


def test_transform_identity_is_a_no_op() -> None:
    identity = Transform2(0.0, 0.0, 0.0)
    pose = Pose(x=7.0, y=-3.0, heading=1.2)

    assert identity.apply(pose) == pose


# ---------------------------------------------------------------------------
# 2. solveTransform(): the re-anchor step, in isolation.
# ---------------------------------------------------------------------------


def test_solve_transform_maps_local_pose_exactly_onto_world_pose() -> None:
    local = Pose(x=2.0, y=1.0, heading=math.radians(15.0))
    world = Pose(x=50.0, y=-20.0, heading=math.radians(100.0))

    t = solveTransform(local, world)
    mapped = t.apply(local)

    assert math.isclose(mapped.x, world.x, abs_tol=1e-9)
    assert math.isclose(mapped.y, world.y, abs_tol=1e-9)
    assert math.isclose(mapped.heading, world.heading, abs_tol=1e-9)


def test_solve_transform_is_identity_when_local_already_equals_world() -> None:
    pose = Pose(x=5.0, y=5.0, heading=math.radians(45.0))

    t = solveTransform(pose, pose)

    assert math.isclose(t.x, 0.0, abs_tol=1e-9)
    assert math.isclose(t.y, 0.0, abs_tol=1e-9)
    assert math.isclose(t.rotation, 0.0, abs_tol=1e-9)


# ---------------------------------------------------------------------------
# 3. WorldPose.reanchor(): updates T_world_from_odom from a camera fix.
# ---------------------------------------------------------------------------


def test_reanchor_updates_encoder_transform_from_camera_fix() -> None:
    world = WorldPose()
    # 500mm east, 0mm north, heading 0 -- x_mm, y_mm, heading_cdeg.
    world.ingest(_tlmFrame(recvTime=100.0, pose=(500, 0, 0)))

    world.reanchor((60.0, 20.0, math.radians(90.0)))  # x_cm, y_cm, yaw_rad

    # Local pose is (50cm, 0, 0); the fix says that same instant is really
    # (60, 20, +90deg) in world frame -- solved directly for comparison.
    expected = solveTransform(Pose(x=50.0, y=0.0, heading=0.0),
                              Pose(x=60.0, y=20.0, heading=math.radians(90.0)))
    assert math.isclose(world.worldFromOdom.x, expected.x, abs_tol=1e-9)
    assert math.isclose(world.worldFromOdom.y, expected.y, abs_tol=1e-9)
    assert math.isclose(world.worldFromOdom.rotation, expected.rotation, abs_tol=1e-9)

    pose = world.worldPose(atHostTime=100.0)
    assert math.isclose(pose.x, 60.0, abs_tol=1e-6)
    assert math.isclose(pose.y, 20.0, abs_tol=1e-6)
    assert math.isclose(pose.heading, math.radians(90.0), abs_tol=1e-6)


def test_reanchor_leaves_a_never_ingested_source_at_identity() -> None:
    world = WorldPose()
    world.ingest(_tlmFrame(recvTime=100.0, pose=(0, 0, 0)))  # no OTOS

    world.reanchor((10.0, 10.0, 0.0))

    assert world.worldPoseOtos() is None
    assert world.worldFromOdomOtos == Transform2(0.0, 0.0, 0.0)


def test_reanchor_before_any_ingest_is_a_harmless_no_op() -> None:
    world = WorldPose()

    world.reanchor((10.0, 10.0, 0.0))  # nothing ingested yet

    assert world.worldFromOdom == Transform2(0.0, 0.0, 0.0)
    assert world.worldPose() is None


# ---------------------------------------------------------------------------
# 4. Age-based (t - age) extrapolation.
# ---------------------------------------------------------------------------


def test_world_pose_extrapolates_forward_using_reported_velocity() -> None:
    world = WorldPose()
    # v = 100 mm/s along heading 0; encoder age 0 -> sample hostTime == recvTime.
    world.ingest(_tlmFrame(recvTime=100.0, pose=(0, 0, 0), twist=(100, 0)))

    pose = world.worldPose(atHostTime=102.0)  # 2s later

    # 100 mm/s == 10 cm/s; 2s of travel == 20cm.
    assert math.isclose(pose.x, 20.0, abs_tol=1e-6)
    assert math.isclose(pose.y, 0.0, abs_tol=1e-9)


def test_world_pose_extrapolation_accounts_for_sample_age_not_just_recv_time() -> None:
    world = WorldPose()
    # Same velocity, but this sample was collected 200ms BEHIND recvTime --
    # its true (host-clock) capture instant is recvTime - 0.2s, per
    # TLMFrame.recvTime's own docstring ("recvTime - age / 1000.0").
    world.ingest(_tlmFrame(recvTime=100.0, pose=(0, 0, 0), twist=(100, 0), encAge=200))

    # Querying exactly at recvTime means 0.2s have ALREADY elapsed since the
    # sample's true capture instant, not 0s.
    pose = world.worldPose(atHostTime=100.0)

    assert math.isclose(pose.x, 2.0, abs_tol=1e-6)  # 10 cm/s * 0.2s


def test_world_pose_extrapolation_falls_back_to_ingest_time_when_recv_time_unset() -> None:
    # A frame built any other way than read_pending_binary_tlm_frames()
    # leaves recvTime at its None default (TLMFrame's own documented
    # convention) -- WorldPose must stay usable, not crash.
    frame = TLMFrame(t=1000, pose=(0, 0, 0), twist=(0, 0),
                      enc_left=_encoderReading(), enc_right=_encoderReading())
    assert frame.recvTime is None

    world = WorldPose()
    world.ingest(frame)  # must not raise

    assert world.worldPose() is not None


# ---------------------------------------------------------------------------
# 5. Encoder-vs-OTOS divergence.
# ---------------------------------------------------------------------------


def test_divergence_is_none_until_both_sources_ingested() -> None:
    world = WorldPose()
    world.ingest(_tlmFrame(recvTime=100.0, pose=(0, 0, 0)))  # encoder only

    assert world.encoderOtosDivergence(atHostTime=100.0) is None


def test_divergence_reports_zero_right_after_a_synchronized_reanchor() -> None:
    world = WorldPose()
    otos = OtosReading(x=0.0, y=0.0, heading=0.0, v_x=0.0, v_y=0.0, omega=0.0, age=0)
    world.ingest(_tlmFrame(recvTime=100.0, pose=(0, 0, 0), otos=otos))

    world.reanchor((30.0, -15.0, math.radians(20.0)))

    divergence = world.encoderOtosDivergence(atHostTime=100.0)
    assert divergence is not None
    assert math.isclose(divergence.distance, 0.0, abs_tol=1e-6)
    assert math.isclose(divergence.heading, 0.0, abs_tol=1e-6)


def test_divergence_measures_disagreement_between_encoder_and_otos_motion() -> None:
    world = WorldPose()
    otos0 = OtosReading(x=0.0, y=0.0, heading=0.0, v_x=0.0, v_y=0.0, omega=0.0, age=0)
    world.ingest(_tlmFrame(recvTime=100.0, pose=(0, 0, 0), otos=otos0))
    world.reanchor((0.0, 0.0, 0.0))  # identity transforms -- world == local

    # Encoder says the robot moved 10cm; OTOS (same instant) says 8cm.
    otos1 = OtosReading(x=80.0, y=0.0, heading=0.0, v_x=0.0, v_y=0.0, omega=0.0, age=0)
    world.ingest(_tlmFrame(recvTime=101.0, pose=(100, 0, 0), otos=otos1))

    divergence = world.encoderOtosDivergence(atHostTime=101.0)
    assert divergence is not None
    assert math.isclose(divergence.distance, 2.0, abs_tol=1e-6)  # 10cm - 8cm
    assert math.isclose(divergence.heading, 0.0, abs_tol=1e-6)


def test_divergence_heading_residual_is_wrap_safe_across_pi() -> None:
    """The project's own known trap: OTOS heading wraps to (-pi, pi], the
    firmware's encoder-pose heading does not. Two headings that are
    numerically close to +-pi but on OPPOSITE sides of the branch cut are
    actually close together -- a raw subtraction would report them as
    ~6.2 rad (~355 deg) apart; the wrap-safe residual must report them as
    only a few degrees apart."""
    world = WorldPose()
    encHeadingRad = 3.10   # just under +pi
    otosHeadingRad = -3.10  # just under -pi -- straddles the branch cut
    headingCdeg = round(math.degrees(encHeadingRad) * 100)
    otos = OtosReading(x=0.0, y=0.0, heading=otosHeadingRad,
                       v_x=0.0, v_y=0.0, omega=0.0, age=0)

    world.ingest(_tlmFrame(recvTime=100.0, pose=(0, 0, headingCdeg), otos=otos))
    # No reanchor -- both transforms stay identity, so world heading ==
    # local heading directly, isolating the wrap math under test.

    divergence = world.encoderOtosDivergence(atHostTime=100.0)

    assert divergence is not None
    rawUnwrappedDiff = encHeadingRad - otosHeadingRad  # ~6.2 rad -- WRONG if used directly
    assert abs(rawUnwrappedDiff) > 6.0, "sanity check: raw subtraction is the wrong, large value"
    assert abs(divergence.heading) < 0.1, (
        f"expected a small wrap-safe residual, got {divergence.heading:.4f} rad "
        f"(raw unwrapped diff was {rawUnwrappedDiff:.4f} rad)"
    )
