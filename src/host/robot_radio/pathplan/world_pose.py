"""robot_radio.pathplan.world_pose — WorldPose, the host-side SE(2) frame
tracker (127-004, design issue T3).

`WorldPose` owns `T_world_from_odom`, an SE(2) transform (`Transform2`)
mapping the firmware's own reported pose into world/camera-frame
coordinates: ``world_pose = T ∘ firmware_pose``. The firmware already
integrates its own pose every cycle (``TLMFrame.pose``, the encoder-derived
pose; ``TLMFrame.otos_reading``, the OTOS burst) -- this class never
re-integrates either one, it only RE-ANCHORS ``T`` at each camera fix. That
is exactly why this design needs no `set-pose` wire command at all (design
issue Finding 2, "A host-side path planner needs no set-pose command"): the
host maintains its own transform and re-estimates it from camera
observations without ever telling the firmware anything.

Two transforms, not one
------------------------
`WorldPose` tracks a transform for the ENCODER pose (`worldFromOdom`,
anchored off `TLMFrame.pose`) and, separately, one for the OTOS pose
(`worldFromOdomOtos`, anchored off `TLMFrame.otos_reading`). This is
deliberate, not an accident of implementation: the divergence between the
two world-pose estimates, over time between re-anchors, is free mid-motion
OTOS characterization the project has never had -- see
`encoderOtosDivergence()`, which is what ticket 007 uses to recommend a
camera re-anchor cadence from DATA rather than the "OTOS holds for minutes"
assumption the design issue records from demos (not yet measured there).

Time alignment: frame-age extrapolation only
---------------------------------------------
Neither `TLMFrame.pose` nor `TLMFrame.otos_reading` is timestamped on the
HOST's own clock -- only `TLMFrame.t` (the ROBOT's clock) and each reading's
own `age` (`# [ms] behind TLMFrame.t`) are on the wire. `TLMFrame.recvTime`
(127-004, this ticket) is the host's `time.monotonic()` at the instant the
frame was drained, giving each reading an approximate HOST-clock capture
instant: ``recvTime - age / 1000.0``. This module extrapolates a reading
forward from that instant using the reading's own reported body-frame
velocity (`TLMFrame.twist` for the encoder pose, `OtosReading.v_x/v_y/omega`
for OTOS) -- the same `t - age` pattern
`src/motion/planner/bench/hil_drive.py`'s `ingestTelemetry()` already uses,
just anchored onto the host's clock instead of staying purely in the
robot's. `ClockSync` (`robot/clock_sync.py`) is NOT activated here -- it has
no live caller and is blocked on a separate, pre-existing `serial_conn.py`
corr-id bug (out of scope for this ticket).

Angular wrap discipline
------------------------
OTOS heading (`OtosReading.heading`) is wrapped to (-pi, pi]; the firmware's
own encoder-pose heading (`TLMFrame.pose[2]`) is UNWRAPPED and accumulates
across a whole tour. Any residual computed between the two -- or between
either one and a camera fix's own yaw -- goes through `nav.pose.heading_error`
(reused, not reforked), never a raw subtraction. This is a known project
trap (see this ticket's own instructions and `.claude/rules/
playfield-testing.md`'s heading-convention note).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from robot_radio.field import Geofence, captureFixWithRetry
from robot_radio.nav.pose import Pose, heading_error
from robot_radio.robot.protocol import TLMFrame

_POSITION_SCALE = 10.0  # [mm/cm] TLMFrame/OtosReading positions are in mm; Pose (nav/pose.py) is in cm.


def _rotate(x: float, y: float, angle: float) -> "tuple[float, float]":
    """Rotate the vector (x, y) by `angle` radians (CCW-positive, matching
    this project's own world-frame convention)."""
    cosA = math.cos(angle)
    sinA = math.sin(angle)
    return x * cosA - y * sinA, x * sinA + y * cosA


@dataclass(frozen=True)
class Transform2:
    """An SE(2) transform: rotate then translate. `x`/`y` in cm (matching
    `nav.pose.Pose`'s own convention), `rotation` in radians.

    `apply(pose)` maps a pose expressed in this transform's SOURCE frame
    (e.g. the firmware's own odom-integrated pose) into its TARGET frame
    (world/camera frame): ``target = T.apply(source)``. `inverse()` is the
    transform in the opposite direction, satisfying
    ``T.inverse().apply(T.apply(p)) == p`` for any pose `p` (up to floating
    point and the heading's own 2*pi periodicity -- see `apply()`'s own
    note)."""

    x: float
    y: float
    rotation: float

    def apply(self, pose: Pose) -> Pose:
        """Map `pose` (this transform's source/local frame) into its target
        frame. Heading is a plain sum (`pose.heading + self.rotation`), NOT
        wrapped here -- `Transform2` is pure SE(2) math with no opinion on
        whether its caller's heading convention is wrapped (OTOS) or
        unwrapped/accumulating (the firmware's own encoder pose); wrapping,
        where wanted, is the caller's job (see `WorldPose`'s own wrap
        discipline)."""
        dx, dy = _rotate(pose.x, pose.y, self.rotation)
        return Pose(x=self.x + dx, y=self.y + dy, heading=pose.heading + self.rotation)

    def inverse(self) -> "Transform2":
        """The transform mapping this transform's TARGET frame back into
        its SOURCE frame -- standard SE(2) inverse, ``(R^T, -R^T t)``."""
        dx, dy = _rotate(self.x, self.y, -self.rotation)
        return Transform2(x=-dx, y=-dy, rotation=-self.rotation)


def solveTransform(localPose: Pose, worldPose: Pose) -> Transform2:
    """The transform `T` such that ``T.apply(localPose) == worldPose``
    (mod 2*pi on heading -- see below), i.e. the re-anchor step: `localPose`
    is a firmware-reported pose (encoder or OTOS) at the SAME instant
    `worldPose` (a camera fix) was captured.

    The rotation is the WRAP-SAFE shortest-way residual
    (`nav.pose.heading_error`, reused not reforked) between the two
    headings, never a raw subtraction -- required because the firmware's
    encoder-pose heading is unwrapped/accumulating while a camera fix's yaw
    and OTOS heading are both wrapped to (-pi, pi]. Consequence: the
    resulting transform satisfies ``T.apply(localPose).heading ==
    worldPose.heading`` only MODULO 2*pi when `localPose.heading` has
    already wound past a full turn (i.e. compare via `heading_error(...,
    ...) ~= 0`, not `==`, when `localPose` is the unwrapped encoder pose)."""
    rotation = heading_error(localPose.heading, worldPose.heading)
    dx, dy = _rotate(localPose.x, localPose.y, rotation)
    return Transform2(x=worldPose.x - dx, y=worldPose.y - dy, rotation=rotation)


@dataclass(frozen=True)
class PoseDivergence:
    """Encoder-vs-OTOS world-pose divergence -- see
    `WorldPose.encoderOtosDivergence()`'s own docstring for what this
    measures and why it exists."""

    distance: float  # [cm] planar distance between the two world-pose estimates
    heading: float   # [rad] wrapped shortest-way residual, encoder heading minus OTOS heading


@dataclass(frozen=True)
class _OdomSample:
    """One firmware-reported local (odom-frame) pose, its own body-frame
    velocity, and the HOST-clock instant it was captured -- everything
    `WorldPose` needs to extrapolate a reading forward to "now" (or to a
    camera fix's own capture instant) via frame-age extrapolation."""

    pose: Pose                          # [cm, cm, rad] local/odom-frame pose at capture
    velocity: "tuple[float, float, float]"  # [cm/s, cm/s, rad/s] body-frame v_x, v_y, omega at capture
    hostTime: float                     # [s] host monotonic time of capture (recvTime - age/1000)
    wrapHeading: bool                   # True for OTOS (heading wraps to (-pi, pi]); False for the unwrapped encoder pose


def _extrapolate(sample: _OdomSample, atHostTime: float) -> Pose:
    """Dead-reckon `sample`'s pose forward from its own capture instant to
    `atHostTime` using its own reported body-frame velocity -- the frame-age
    extrapolation step (`t - age`, via `sample.hostTime`, already computed
    at ingest time)."""
    elapsed = atHostTime - sample.hostTime  # [s]
    v_x, v_y, omega = sample.velocity
    dx, dy = _rotate(v_x, v_y, sample.pose.heading)
    heading = sample.pose.heading + omega * elapsed
    if sample.wrapHeading:
        heading = heading_error(0.0, heading)
    return Pose(x=sample.pose.x + dx * elapsed, y=sample.pose.y + dy * elapsed, heading=heading)


class WorldPose:
    """Host-side SE(2) frame tracker -- see this module's own docstring for
    the full design rationale. Owns two independent `T_world_from_odom`
    transforms (encoder-anchored, OTOS-anchored); reads `TLMFrame` fields
    via `ingest()` and camera fixes via `reanchor()`/`reanchorFromCamera()`;
    does not talk to the wire directly and does not compute moves (that is
    the planner's job, ticket 006)."""

    def __init__(self) -> None:
        self._transformEnc = Transform2(0.0, 0.0, 0.0)   # T_world_from_odom, encoder-anchored
        self._transformOtos = Transform2(0.0, 0.0, 0.0)  # T_world_from_odom, OTOS-anchored
        self._latestEnc: "_OdomSample | None" = None
        self._latestOtos: "_OdomSample | None" = None

    @property
    def worldFromOdom(self) -> Transform2:
        """The encoder-anchored `T_world_from_odom` -- the PRIMARY
        transform (``world_pose = T ∘ firmware_pose``, design issue Finding
        2). Identity until the first `reanchor()`."""
        return self._transformEnc

    @property
    def worldFromOdomOtos(self) -> Transform2:
        """The OTOS-anchored counterpart, tracked SEPARATELY so its
        divergence from `worldFromOdom` is measurable -- see
        `encoderOtosDivergence()`. Identity until the first `reanchor()`."""
        return self._transformOtos

    def ingest(self, frame: TLMFrame) -> None:
        """Fold one already-decoded `TLMFrame` into the tracker's
        latest-known local (odom) pose for both sources it can supply -- the
        encoder pose (`frame.pose`, always present on the wire) and, when
        fresh, the OTOS reading (`frame.otos_reading`, valid iff
        `frame.otos_present`). Does not talk to the wire itself -- the
        caller (a planner loop, a bench script) owns draining
        `NezhaProtocol`.

        If `frame.recvTime` is unset (e.g. a frame built any other way than
        `NezhaProtocol.read_pending_binary_tlm_frames()` -- see
        `TLMFrame.recvTime`'s own docstring), this falls back to
        `time.monotonic()` at ingest time so `WorldPose` stays usable
        against any frame source; only frames drained through the one
        recvTime-populating call site get sub-cycle extrapolation
        precision."""
        if frame.t is None or frame.pose is None:
            return
        recvTime = frame.recvTime if frame.recvTime is not None else time.monotonic()

        # TLMFrame.pose has no `age` field of its own -- it is derived from
        # the SAME per-cycle encoder integration as enc_left/enc_right, so
        # this uses the (max of the) two wheels' own `age` as its effective
        # age. Production firmware emits age=0 for both wheels as of this
        # ticket (EncoderReading's own docstring), so this is presently a
        # zero-cost no-op; kept general for when per-sample skew ships.
        encAge = 0
        if frame.enc_left is not None or frame.enc_right is not None:
            ages = [r.age for r in (frame.enc_left, frame.enc_right) if r is not None]
            encAge = max(ages) if ages else 0

        x, y, heading = frame.pose  # [mm] [mm] [cdeg]
        encPose = Pose(x=x / _POSITION_SCALE, y=y / _POSITION_SCALE,
                        heading=math.radians(heading / 100.0))
        # frame.twist is (v [mm/s], omega [mrad/s]) fused body-frame
        # velocity; v_y is always zero on the wire for this differential
        # build (TLMFrame's own docstring).
        velocity, omega = frame.twist if frame.twist is not None else (0, 0)  # [mm/s] [mrad/s]
        encVelocity = (velocity / _POSITION_SCALE, 0.0, omega / 1000.0)
        self._latestEnc = _OdomSample(pose=encPose, velocity=encVelocity,
                                       hostTime=recvTime - encAge / 1000.0,
                                       wrapHeading=False)

        if frame.otos_present and frame.otos_reading is not None:
            o = frame.otos_reading
            otosPose = Pose(x=o.x / _POSITION_SCALE, y=o.y / _POSITION_SCALE, heading=o.heading)
            otosVelocity = (o.v_x / _POSITION_SCALE, o.v_y / _POSITION_SCALE, o.omega)
            self._latestOtos = _OdomSample(pose=otosPose, velocity=otosVelocity,
                                            hostTime=recvTime - o.age / 1000.0,
                                            wrapHeading=True)

    def worldPose(self, atHostTime: "float | None" = None) -> "Pose | None":
        """Current world-frame pose from the ENCODER-anchored transform,
        extrapolated to `atHostTime` (default: `time.monotonic()`, i.e.
        "now") via frame-age extrapolation. `None` until at least one frame
        has been `ingest()`-ed."""
        if self._latestEnc is None:
            return None
        when = time.monotonic() if atHostTime is None else atHostTime
        return self._transformEnc.apply(_extrapolate(self._latestEnc, when))

    def worldPoseOtos(self, atHostTime: "float | None" = None) -> "Pose | None":
        """Current world-frame pose from the OTOS-anchored transform,
        extrapolated to `atHostTime` the same way `worldPose()` is. `None`
        until at least one frame with `otos_present` has been
        `ingest()`-ed."""
        if self._latestOtos is None:
            return None
        when = time.monotonic() if atHostTime is None else atHostTime
        return self._transformOtos.apply(_extrapolate(self._latestOtos, when))

    def reanchor(self, fix: "tuple[float, float, float]") -> None:
        """Re-anchor BOTH tracked transforms from one camera fix
        `(x_cm, y_cm, yaw_rad)` -- `field.Geofence.captureFix()`'s own
        return shape. A source with no `ingest()`-ed sample yet is left
        un-anchored (identity transform) until the next `ingest()` +
        `reanchor()` pair."""
        worldTarget = Pose(x=fix[0], y=fix[1], heading=fix[2])
        if self._latestEnc is not None:
            self._transformEnc = solveTransform(self._latestEnc.pose, worldTarget)
        if self._latestOtos is not None:
            self._transformOtos = solveTransform(self._latestOtos.pose, worldTarget)

    def reanchorFromCamera(self, geofence: Geofence, label: str,
                           retrySeconds: float = 5.0) -> bool:
        # [s]
        """Live convenience wrapper: captures a fix via `field`'s promoted
        `captureFixWithRetry` (ticket 003) and re-anchors from it -- the
        re-anchor path importing camera-fix access from `robot_radio.field`
        rather than reimplementing it (this ticket's own acceptance
        criterion). Returns True on success, False if the tag was not seen
        after retrying (matches `captureFixWithRetry`'s own None-on-failure
        contract) -- never raises for a lost tag."""
        fix = captureFixWithRetry(geofence, label, retrySeconds)
        if fix is None:
            return False
        self.reanchor(fix)
        return True

    def encoderOtosDivergence(self, atHostTime: "float | None" = None) -> "PoseDivergence | None":
        """How far the two independently re-anchored world-pose estimates
        have drifted apart since their last (possibly different) re-anchor
        -- the deliberate by-product this ticket calls out: tracking a
        transform for the encoder pose AND one for the OTOS pose yields
        this divergence for free, and it is what ticket 007 uses to
        recommend a camera re-anchor cadence from DATA instead of the "OTOS
        holds for minutes" assumption the design issue records from demos.
        `None` until BOTH sources have been `ingest()`-ed at least once."""
        when = time.monotonic() if atHostTime is None else atHostTime
        encPose = self.worldPose(when)
        otosPose = self.worldPoseOtos(when)
        if encPose is None or otosPose is None:
            return None
        dx = encPose.x - otosPose.x
        dy = encPose.y - otosPose.y
        distance = math.hypot(dx, dy)
        heading = heading_error(otosPose.heading, encPose.heading)
        return PoseDivergence(distance=distance, heading=heading)
