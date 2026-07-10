"""legacy_translate.py -- M4 Legacy Verb Translator (097-002).

Pure, stateless functions that turn a legacy (v2 text-plane) verb's
wire-shaped arguments into the matching binary-plane ``pb2`` message. No
``SerialConnection``/socket/I/O reference anywhere in this module -- every
function here takes plain values and returns a plain ``pb2`` message; the
caller owns the envelope wrapping and the wire round trip.

File-location choice (architecture-update.md (097) Step 7 Open Question 5,
left to this ticket's own judgment): a standalone module, not module-level
functions inside ``protocol.py``. ``NezhaProtocol`` (``protocol.py``) is not
this module's only caller -- ticket 004 (``rogo`` REPL Translator, M5) needs
the SAME verb -> envelope mapping to let a human type v2 text at ``rogo
send`` while the wire carries binary, and a standalone module lets it import
this file without pulling in ``NezhaProtocol``/``SerialConnection``'s own
import surface (``robot_radio.io.serial_conn``, pyserial, etc.).

Scope ticket 097-002: S (drive), T (timed), D (distance) -- the three
legacy verbs ``NezhaProtocol.drive()``/``.timed()``/``.distance()`` need
(ticket 002's own ten-method conversion list). Extended by ticket 097-004
(M5 rogo REPL Translator) with RT/MOVE/MOVER -- ``rogo send`` speaks the
full v2 text grammar and needs the SAME verb -> envelope mapping M2's
``NezhaProtocol`` used for S/T/D, translated here, never reimplemented at
the call site (``host/robot_radio/io/cli.py``'s own ``cmd_send``). This
file's own functions are additive, so this second extension is not a
breaking change to ticket 002's original three.

Transcription source (never re-derived -- 095 Decision 5's "transcribe,
don't re-derive" discipline, reapplied host-side; every function below cites
the firmware function/file it ports):

  - ``handleS()`` (source/commands/motion_commands.cpp): ``S <l> <r>`` ->
    ``msg::WheelTargets{speed=l, speed=r}``, posted as
    ``DrivetrainCommand{wheels}`` to ``bb.driveIn``. No kinematics -- a
    direct per-wheel-speed passthrough.

  - ``handleT()`` (source/commands/motion_commands.cpp): ``T <l> <r> <ms>``
    -> ``BodyKinematics::forward(l, r, trackwidth)`` for ``v`` (``omega`` is
    explicitly discarded, ``(void)omega;`` -- T is straight-line only);
    ``Motion::Segment.distance = v * (ms / 1000.0)``. Every other
    ``Motion::Segment`` field is left at its 0 default (falls back to the
    ``SegmentExecutor``'s configured profile -- see that handler's own
    comment on why a low per-segment ``speedMax`` induces a small terminal
    decel overshoot).

  - ``handleD()`` (source/commands/motion_commands.cpp): ``D <l> <r> <mm>``
    -> ``BodyKinematics::forward(l, r, trackwidth)`` for ``v`` (``omega``
    discarded, same reasoning as T); ``sign = -1 if v < 0 else 1``;
    ``Motion::Segment.distance = sign * mm``. Every other field 0, same
    reasoning as T.

  - ``BodyKinematics::forward()`` (source/kinematics/body_kinematics.cpp):
    ``v = (vR + vL) * 0.5``; ``omega = (vR - vL) / b`` (``b`` = trackwidth,
    mm). Transcribed verbatim below as ``forward()``, including the
    undefined-for-``b == 0`` behavior -- the firmware itself does not guard
    against ``b == 0`` either (trackwidth is a device config value, never 0
    in practice). T/D's own translators (``segment_for_timed``/
    ``segment_for_distance`` below) use only the ``v`` output and therefore
    take NO ``trackwidth`` parameter of their own -- ``omega``, the only
    trackwidth-dependent output, is discarded by both firmware handlers
    they transcribe, so this module never needs a trackwidth value at all
    for T/D (nothing to fetch host-side, and no new parameter on
    ``NezhaProtocol.timed()``/``.distance()``, whose signatures ticket 002
    must hold unchanged).

  - ``handleRT()`` (source/commands/motion_commands.cpp): ``RT <relAngle>``
    -> one pure in-place-turn ``Motion::Segment``: ``distance = 0``,
    ``finalHeading = relAngle`` (relative, CCW+), converted wire centi-
    degrees -> radians via the shared ``kCdegToRad`` constant
    (``motion_commands.cpp``, transcribed below as ``_CDEG_TO_RAD``). Every
    other field 0 default -- posted to ``bb.segmentIn`` like MOVE (the
    Planner path is parked), so ``segment_for_rt()`` below builds the SAME
    ``MotionSegment`` shape ``segment_for_timed()``/``segment_for_distance()``
    do.

  - ``handleMove()`` (source/commands/motion_commands.cpp): ``MOVE
    <distance_mm> <direction_cdeg> <finalHeading_cdeg> [v=][a=][j=][w=]
    [wa=][wj=][s=]`` -> ``Motion::Segment``, a 1:1 field copy off
    ``parseMove``'s packed args; ``direction``/``finalHeading``/``w``/
    ``wa``/``wj`` convert wire centidegrees -> radians via
    ``_CDEG_TO_RAD``; ``v``/``a``/``j`` (mm/s, mm/s^2, mm/s^3) pass through
    unconverted; ``s=1`` -> ``stream=True`` (merge-chain into the live
    plan). An absent ``kv`` key defaults to 0.0, matching ``kvFloat()``'s
    own 0.0 default and ``Motion::Segment``'s "0 => executor's configured
    default" convention (parseMove's own doc comment).

  - ``handleMover()`` (source/commands/motion_commands.cpp): ``MOVER
    <distance_mm> <direction_cdeg> <finalHeading_cdeg> [t=][v=][w=][a=][j=]
    [wa=][wj=]`` -> ``Motion::Segment``, REPLACE semantics (posted to
    ``bb.replaceIn``, not ``bb.segmentIn``): ``stream`` is always ``True``;
    ``v``/``w`` are SIGNED (carry direction in time mode, unlike MOVE's
    unsigned ceilings) and converted the same way (``w`` wire centidegrees/s
    -> rad/s via ``_CDEG_TO_RAD``); ``speedMax = |v|``, ``yawRateMax =
    |w|`` (converted) -- the ceiling handleMover() itself derives from the
    signed target, transcribed verbatim, not re-derived.

  - ``kCdegToRad`` (source/commands/motion_commands.cpp): ``constexpr float
    kCdegToRad = 3.14159265f / 18000.0f;`` -- centidegrees -> radians,
    shared by ``handleTURN``/``handleRT``/``handleMove``/``handleMover``.
    Transcribed below as ``_CDEG_TO_RAD`` (module-level, not a function --
    RT/MOVE/MOVER all multiply by it directly, mirroring the firmware's own
    "one shared constant, applied inline at each call site" shape rather
    than introducing a wrapper function no firmware code has).
"""

from __future__ import annotations

from robot_radio.robot.pb2 import common_pb2, drivetrain_pb2, motion_pb2

# kCdegToRad mirror (source/commands/motion_commands.cpp):
#   constexpr float kCdegToRad = 3.14159265f / 18000.0f;
# centidegrees -> radians, shared by handleRT()/handleMove()/handleMover()
# below -- transcribed as the literal float32 constant, not math.radians(),
# so the conversion factor matches the firmware's own float32 rounding
# exactly rather than double-precision math.pi.
_CDEG_TO_RAD = 3.14159265 / 18000.0  # [rad/cdeg]


def forward(v_left: float, v_right: float,  # [mm/s]
           trackwidth: float) -> tuple[float, float]:  # [mm]
    """``BodyKinematics::forward()`` (source/kinematics/body_kinematics.cpp),
    transcribed verbatim -- both outputs, for completeness/testability
    against the cited source:

      v     = (vR + vL) * 0.5    [mm/s]
      omega = (vR - vL) / b      [rad/s], b = trackwidth [mm]

    T/D's own translators below use only the ``v`` output (see this
    module's file header) and do not call this function with a real
    trackwidth -- they compute ``v`` inline instead, since ``omega`` (the
    only trackwidth-dependent output) is discarded by both handlers they
    transcribe.
    """
    v = (v_right + v_left) * 0.5
    omega = (v_right - v_left) / trackwidth
    return v, omega


def wheel_targets_for_drive(v_left: float, v_right: float,  # [mm/s]
                            ) -> common_pb2.WheelTargets:
    """``handleS()`` (motion_commands.cpp): ``S <l> <r>`` -> per-wheel SPEED
    targets only (position left uncommanded) -- the SAME wire shape
    ``cli.py``'s ``cmd_binary_drive()`` already builds by hand
    (``env.drive.wheels.w.add(speed=...)`` twice)."""
    wt = drivetrain_pb2.WheelTargets()
    wt.w.add(speed=float(v_left))
    wt.w.add(speed=float(v_right))
    return wt


def segment_for_timed(v_left: float, v_right: float,  # [mm/s]
                      duration: int,  # [ms]
                      ) -> motion_pb2.MotionSegment:
    """``handleT()`` (motion_commands.cpp): ``T <l> <r> <ms>`` -> one
    straight ``Motion::Segment``, ``distance = v * (ms / 1000)`` where ``v``
    is ``BodyKinematics::forward()``'s ``v`` output (``(vR + vL) * 0.5``;
    ``omega`` is discarded by ``handleT()``, so it is never computed here).
    Every other ``MotionSegment`` field is left at its proto3 zero default,
    matching ``Motion::Segment``'s own "0 => executor's configured default"
    convention (``handleT()`` leaves ``speedMax`` at 0 for the same reason
    -- see that handler's own comment)."""
    v = (v_right + v_left) * 0.5
    seg = motion_pb2.MotionSegment()
    seg.distance = v * (float(duration) / 1000.0)
    return seg


def segment_for_distance(v_left: float, v_right: float,  # [mm/s]
                         travel: int,  # [mm]
                         ) -> motion_pb2.MotionSegment:
    """``handleD()`` (motion_commands.cpp): ``D <l> <r> <mm>`` -> one
    straight ``Motion::Segment``, ``distance = sign(v) * mm`` where ``v`` is
    ``BodyKinematics::forward()``'s ``v`` output (``omega`` discarded, same
    reasoning as ``segment_for_timed()`` above). ``sign`` matches
    ``handleD()``'s own ``(v < 0.0f) ? -1.0f : 1.0f`` exactly -- ``v == 0``
    yields ``+1``, not 0. Every other field 0, same reasoning as
    ``segment_for_timed()``."""
    v = (v_right + v_left) * 0.5
    sign = -1.0 if v < 0.0 else 1.0
    seg = motion_pb2.MotionSegment()
    seg.distance = sign * float(travel)
    return seg


def segment_for_rt(rel_angle: float,  # [cdeg]
                   ) -> motion_pb2.MotionSegment:
    """``handleRT()`` (motion_commands.cpp): ``RT <relAngle>`` -> one pure
    in-place-turn ``Motion::Segment``: ``distance = 0``, ``finalHeading =
    relAngle`` (relative, CCW+), wire centidegrees converted to radians via
    ``_CDEG_TO_RAD`` (``kCdegToRad`` transcription, see this module's file
    header). Every other field stays at its proto3 zero default -- RT posts
    to ``bb.segmentIn`` exactly like MOVE (the Planner path is parked), so
    this builds the SAME ``MotionSegment`` shape ``segment_for_timed()``/
    ``segment_for_distance()`` do, just with ``finalHeading`` set instead of
    ``distance``."""
    seg = motion_pb2.MotionSegment()
    seg.final_heading = float(rel_angle) * _CDEG_TO_RAD
    return seg


def segment_for_move(distance: float,  # [mm]
                     direction: float,  # [cdeg]
                     final_heading: float,  # [cdeg]
                     speed_max: float = 0.0,  # [mm/s]
                     accel_max: float = 0.0,  # [mm/s^2]
                     jerk_max: float = 0.0,  # [mm/s^3]
                     yaw_rate_max: float = 0.0,  # [cdeg/s]
                     yaw_accel_max: float = 0.0,  # [cdeg/s^2]
                     yaw_jerk_max: float = 0.0,  # [cdeg/s^3]
                     stream: bool = False,
                     ) -> motion_pb2.MotionSegment:
    """``handleMove()`` (motion_commands.cpp): ``MOVE <distance_mm>
    <direction_cdeg> <finalHeading_cdeg> [v=][a=][j=][w=][wa=][wj=][s=]`` ->
    ``Motion::Segment``, a 1:1 field copy off ``parseMove``'s packed args.
    ``direction``/``final_heading``/``yaw_rate_max``/``yaw_accel_max``/
    ``yaw_jerk_max`` are wire centidegrees (per second, where applicable),
    converted to radians via ``_CDEG_TO_RAD`` -- the SAME conversion
    ``handleMove()`` itself applies. ``speed_max``/``accel_max``/
    ``jerk_max`` (mm/s, mm/s^2, mm/s^3) pass through unconverted. An absent
    caller kwarg already defaults to 0.0 (this function's own default
    values), matching ``kvFloat()``'s 0.0 default and ``Motion::Segment``'s
    "0 => executor's configured default" convention (parseMove's own doc
    comment). ``stream`` mirrors ``s=1`` -> ``stream=True`` (merge-chain
    into the live plan)."""
    seg = motion_pb2.MotionSegment()
    seg.distance = float(distance)
    seg.direction = float(direction) * _CDEG_TO_RAD
    seg.final_heading = float(final_heading) * _CDEG_TO_RAD
    seg.speed_max = float(speed_max)
    seg.accel_max = float(accel_max)
    seg.jerk_max = float(jerk_max)
    seg.yaw_rate_max = float(yaw_rate_max) * _CDEG_TO_RAD
    seg.yaw_accel_max = float(yaw_accel_max) * _CDEG_TO_RAD
    seg.yaw_jerk_max = float(yaw_jerk_max) * _CDEG_TO_RAD
    seg.stream = bool(stream)
    return seg


def segment_for_mover(distance: float,  # [mm]
                      direction: float,  # [cdeg]
                      final_heading: float,  # [cdeg]
                      time: float = 0.0,  # [ms]
                      v: float = 0.0,  # [mm/s] signed
                      accel_max: float = 0.0,  # [mm/s^2]
                      jerk_max: float = 0.0,  # [mm/s^3]
                      omega: float = 0.0,  # [cdeg/s] signed
                      yaw_accel_max: float = 0.0,  # [cdeg/s^2]
                      yaw_jerk_max: float = 0.0,  # [cdeg/s^3]
                      ) -> motion_pb2.MotionSegment:
    """``handleMover()`` (motion_commands.cpp): ``MOVER <distance_mm>
    <direction_cdeg> <finalHeading_cdeg> [t=][v=][w=][a=][j=][wa=][wj=]`` ->
    ``Motion::Segment``, REPLACE semantics (posted to ``bb.replaceIn``, not
    ``bb.segmentIn`` -- the deadman-velocity teleop shape). ``stream`` is
    ALWAYS ``True`` (``handleMover()``'s own unconditional
    ``seg.stream = true;``). ``v``/``omega`` are SIGNED (they carry
    direction in time mode, unlike MOVE's unsigned ceilings); ``omega``'s
    wire centidegrees/s convert to rad/s via ``_CDEG_TO_RAD``, same as
    MOVE. ``speed_max = |v|``, ``yaw_rate_max = |omega|`` (converted) --
    the ceiling ``handleMover()`` itself derives from the signed target,
    transcribed verbatim (``seg.speedMax = fabsf(v); seg.yawRateMax =
    fabsf(w) * kCdegToRad;``), not re-derived. ``accel_max``/``jerk_max``/
    ``yaw_accel_max``/``yaw_jerk_max`` pass through the same way MOVE's do."""
    seg = motion_pb2.MotionSegment()
    seg.stream = True
    seg.time = float(time)
    seg.distance = float(distance)
    seg.direction = float(direction) * _CDEG_TO_RAD
    seg.final_heading = float(final_heading) * _CDEG_TO_RAD
    seg.v = float(v)
    seg.omega = float(omega) * _CDEG_TO_RAD
    seg.speed_max = abs(float(v))
    seg.yaw_rate_max = abs(float(omega)) * _CDEG_TO_RAD
    seg.accel_max = float(accel_max)
    seg.jerk_max = float(jerk_max)
    seg.yaw_accel_max = float(yaw_accel_max) * _CDEG_TO_RAD
    seg.yaw_jerk_max = float(yaw_jerk_max) * _CDEG_TO_RAD
    return seg
