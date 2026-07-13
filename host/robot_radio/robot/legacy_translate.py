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

import math

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


def segment_for_seg(arc_length: float = 0.0,      # [mm] signed; 0 = pivot
                    delta_heading: float = 0.0,   # [rad] signed, CCW+
                    exit_speed: float = 0.0,       # [mm/s] boundary velocity
                    ) -> motion_pb2.MotionSegment:
    """The v2-native arc/pivot PRIMITIVE builder (100-007, THE CUTOVER) --
    a direct 1:1 mapping onto ``Drive::Goal``'s own three fields
    (``arcLength``/``deltaHeading``/``exitSpeed``, source/drive/
    drivetrain.h), ``primitive=True``. This is the ``SEG`` proxy verb's own
    builder (the issue's "a segment_for_seg()-style builder for real arcs");
    every OTHER builder in this module that now produces a v2 primitive
    segment (``segment_for_timed``/``_distance``/``_rt``/``_turn``/``_arc``,
    ``primitives_for_move``) is a thin, verb-specific wrapper over this same
    shape -- transcribed once here, reused everywhere, never duplicated."""
    seg = motion_pb2.MotionSegment()
    seg.arc_length = float(arc_length)
    seg.delta_heading = float(delta_heading)
    seg.exit_speed = float(exit_speed)
    seg.primitive = True
    return seg


def segment_for_timed(v_left: float, v_right: float,  # [mm/s]
                      duration: int,  # [ms]
                      ) -> motion_pb2.MotionSegment:
    """``handleT()`` (motion_commands.cpp, pre-097-006 gut): ``T <l> <r>
    <ms>`` -> one straight v2 primitive segment, ``arc_length = v * (ms /
    1000)`` where ``v`` is ``BodyKinematics::forward()``'s ``v`` output
    (``(vR + vL) * 0.5``; ``omega`` is discarded by the original
    ``handleT()``, so it is never computed here).

    DEVIATION (100-007, THE CUTOVER): the pre-cutover ``distance``/
    ``speed_max`` MotionSegment shape this function used to build is
    REJECTED at the wire post-cutover (``primitive=false``) -- this now
    builds a ``primitive=true`` segment via ``segment_for_seg()`` instead.
    The per-segment ``speedMax`` fallback-to-executor-default the original
    ``handleT()`` relied on has no v2 equivalent (``Drive::Goal`` carries no
    speed override at all -- see ``primitives_for_move()``'s own docstring
    for the full rationale, shared verbatim here)."""
    v = (v_right + v_left) * 0.5
    return segment_for_seg(arc_length=v * (float(duration) / 1000.0))


def segment_for_distance(v_left: float, v_right: float,  # [mm/s]
                         travel: int,  # [mm]
                         ) -> motion_pb2.MotionSegment:
    """``handleD()`` (motion_commands.cpp, pre-097-006 gut): ``D <l> <r>
    <mm>`` -> one straight v2 primitive segment, ``arc_length = sign(v) *
    mm`` where ``v`` is ``BodyKinematics::forward()``'s ``v`` output
    (``omega`` discarded, same reasoning as ``segment_for_timed()`` above).
    ``sign`` matches ``handleD()``'s own ``(v < 0.0f) ? -1.0f : 1.0f``
    exactly -- ``v == 0`` yields ``+1``, not 0.

    DEVIATION (100-007, THE CUTOVER): same as ``segment_for_timed()`` above
    -- builds ``primitive=true`` via ``segment_for_seg()`` now, no
    per-segment speed override."""
    v = (v_right + v_left) * 0.5
    sign = -1.0 if v < 0.0 else 1.0
    return segment_for_seg(arc_length=sign * float(travel))


def segment_for_rt(rel_angle: float,  # [cdeg]
                   ) -> motion_pb2.MotionSegment:
    """``handleRT()`` (motion_commands.cpp, pre-097-006 gut): ``RT
    <relAngle>`` -> one pure in-place-turn v2 primitive segment:
    ``arc_length = 0`` (a pivot), ``delta_heading = relAngle`` (relative,
    CCW+), wire centidegrees converted to radians via ``_CDEG_TO_RAD``
    (``kCdegToRad`` transcription, see this module's file header).

    DEVIATION (100-007, THE CUTOVER): builds ``primitive=true`` via
    ``segment_for_seg()`` now (was ``final_heading`` on the retired
    non-primitive shape) -- same reasoning as ``segment_for_timed()``
    above."""
    return segment_for_seg(delta_heading=float(rel_angle) * _CDEG_TO_RAD)


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


def primitives_for_move(distance: float,  # [mm]
                        direction: float,  # [cdeg]
                        final_heading: float,  # [cdeg]
                        ) -> list[motion_pb2.MotionSegment]:
    """(100-007, THE CUTOVER) decomposes a legacy ``MOVE <distance>
    <direction> <finalHeading>`` into <=3 v2 PRIMITIVE ``MotionSegment``s
    (``primitive=True`` each, via ``segment_for_seg()``): a leading pivot to
    ``direction`` (if nonzero), a straight run of ``distance`` (if
    nonzero), and a trailing pivot from ``direction`` to ``final_heading``
    (if the two differ) -- the SAME three-phase shape the pre-cutover
    ``handleMove()``'s single ``Motion::Segment`` drove through the retired
    ``Motion::SegmentExecutor`` (PRE_PIVOT/TRANSLATE/TERMINAL_PIVOT,
    source/motion/segment_executor.h -- parked, not deleted). The v2 wire
    has no per-message multi-phase encoding (``Drive::Goal`` is ONE
    constant-curvature arc primitive -- source/drive/drivetrain.h), so this
    function performs the decomposition HOST-SIDE; the caller sends each
    result as its OWN ``segment`` ``CommandEnvelope``, in order (see
    ``legacy_verbs.py``'s own multi-envelope send, ``envelope_for_move()``).

    Decomposition strategy, exactly: phase 1 (pivot) fires iff
    ``direction != 0``, with ``delta_heading = direction`` (converted to
    radians); phase 2 (straight) fires iff ``distance != 0``, with
    ``arc_length = distance``; phase 3 (trailing pivot) fires iff
    ``final_heading != direction``, with ``delta_heading = final_heading -
    direction`` (both converted to radians first, so the trailing delta is
    computed in the SAME frame ``segment_for_seg()``'s pivots already use).
    A phase whose own delta is exactly 0.0 is OMITTED entirely (never sent
    as a degenerate zero-motion segment) -- e.g. ``MOVE 500 0 0`` (straight
    only) sends exactly ONE segment.

    DEVIATION from the pre-cutover single-segment translation (095 Decision
    5's "transcribe, don't re-derive... document deviations" discipline,
    reapplied here): ``v=``/``a=``/``j=``/``w=``/``wa=``/``wj=`` per-segment
    speed/accel/jerk overrides are NOT supported and are silently dropped --
    ``Drive::Goal``/``PlanRequest`` (source/drive/drivetrain.h) carry no
    such field at all; ``Drive::Drivetrain::plan()`` always solves against
    the ONE construction-time ``Drive::Limits``, never a per-call override
    (a structural consequence of the v2 primitive shape, not an oversight
    of this function). ``s=1`` (BLEND, the streaming merge) has no v2
    primitive equivalent either -- the firmware rejects
    ``primitive=true`` combined with ``stream=true`` outright (typed ERR,
    architecture-update.md M8); this function never sets ``stream``, and a
    caller must not either."""
    direction_rad = float(direction) * _CDEG_TO_RAD
    final_heading_rad = float(final_heading) * _CDEG_TO_RAD

    segments: list[motion_pb2.MotionSegment] = []
    if direction_rad != 0.0:
        segments.append(segment_for_seg(delta_heading=direction_rad))
    if float(distance) != 0.0:
        segments.append(segment_for_seg(arc_length=float(distance)))
    trailing = final_heading_rad - direction_rad
    if trailing != 0.0:
        segments.append(segment_for_seg(delta_heading=trailing))
    return segments


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
    ``yaw_accel_max``/``yaw_jerk_max`` pass through the same way MOVE's do.

    KNOWN BROKEN post-100-007 (THE CUTOVER), left UNCHANGED and documented
    rather than silently patched: this builds the RETIRED non-primitive
    (``primitive=false``) ``MotionSegment`` shape, which the firmware now
    REJECTS outright at the wire (typed ``ERR_UNIMPLEMENTED``) -- MOVER's
    real v2 home is ``Drive::Drivetrain::planVelocity()``, wired into the
    adapter's ``replaceIn`` path by ticket 100-008 (M8), explicitly out of
    ticket 100-007's own scope. Do not "fix" this function ahead of that
    ticket; it stays a faithful transcription of the pre-cutover
    ``handleMover()`` until 100-008 replaces it with a real primitive
    builder."""
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


# ---------------------------------------------------------------------------
# Open-loop segment approximations for R/TURN/G (097, this ticket) -- the
# `segment`/`replace` binary arms have no Planner/fused-pose-closed-loop
# equivalent (Subsystems::Planner is parked; Telemetry.pose is pinned at
# (0,0,0) until sprint 098 wires PoseEstimator::tick()). Each function below
# is transcribed from its verb's pre-097-006 firmware handler (motion_
# commands.cpp, ``git show 18ba84d8^:source/commands/motion_commands.cpp``),
# then documents EXACTLY how the open-loop segment/replace translation below
# deviates from that closed-loop original -- never silently reconciled, per
# 095 Decision 5's "transcribe, don't re-derive" discipline.
# ---------------------------------------------------------------------------


def segment_for_arc(speed: float,  # [mm/s]
                    radius: float,  # [mm]
                    duration: float = 1000.0,  # [ms]
                    ) -> motion_pb2.MotionSegment:
    """``handleR()`` (motion_commands.cpp, pre-097-006 gut): ``R <speed>
    <radius>`` -> an open-loop constant-curvature arc -- ``omega =
    speed/radius`` (0 when ``radius == 0``, transcribed verbatim; positive
    radius -> positive omega -> CCW/left arc) -- posted as a CONTINUOUS
    ``msg::VelocityGoal`` that ran until an explicit ``STOP``/``stop=``
    clause fired.

    DEVIATION (100-007, THE CUTOVER -- supersedes the 097 open-loop
    velocity-pulse approximation this function used to build): the v2
    stack's own ``Drive::Goal`` (source/drive/drivetrain.h) IS a constant-
    curvature arc primitive, so R's translation no longer needs a
    replace-arm velocity-pulse approximation at all -- this now builds a
    REAL, single, ``primitive=true`` arc segment via ``segment_for_seg()``:
    ``arc_length = speed * (duration / 1000)`` [mm]; ``delta_heading =
    arc_length / radius`` [rad] (0 when ``radius == 0``, the SAME zero-guard
    the original ``handleR()``'s own ``omega = speed/radius`` used,
    transcribed verbatim); ``exit_speed = 0.0`` (a stop segment -- R's own
    pre-097 continuous-until-STOP shape still has no v2 primitive
    equivalent; this ends gracefully at rest after ``duration`` instead of
    running forever, same practical effect as the 097 pulse approximation
    it replaces, just via a real segment/plan instead of a replace-arm
    velocity target). Sent as ``segment`` (not ``replace``) by
    ``envelope_for_r()`` (legacy_verbs.py) -- it is now a genuine finite
    primitive, not a replace-semantics velocity pulse.
    """
    arc_length = float(speed) * (float(duration) / 1000.0)
    delta_heading = (arc_length / radius) if radius != 0.0 else 0.0
    return segment_for_seg(arc_length=arc_length, delta_heading=delta_heading, exit_speed=0.0)


def segment_for_turn(heading: float,  # [cdeg] absolute target heading
                     ) -> motion_pb2.MotionSegment:
    """``handleTURN()`` (motion_commands.cpp, pre-097-006 gut): ``TURN
    <heading> [eps]`` is CLOSED-LOOP against ``bb.fusedPose.pose.h`` -- it
    reads the robot's CURRENT fused heading, computes the shortest-path
    signed delta to the absolute target, and closes a ``HEADING`` stop
    condition around that delta.

    DEVIATION (097, carried forward unchanged by 100-007): fused pose is
    not readable host-side (``Telemetry.pose`` is not on the wire), so
    "current heading" cannot be subtracted here either. This approximates
    the closed-loop turn AS IF the robot always starts at heading 0: it
    builds the SAME pure in-place-turn v2 primitive segment
    ``segment_for_rt()`` does (``arc_length=0``, ``delta_heading=heading``,
    ``primitive=True`` -- via ``segment_for_seg()``, 100-007's own
    conversion of this function to the primitive shape), i.e. it treats the
    absolute target as a from-zero relative turn. Correct immediately after
    a fresh pose reset; drifts from the true absolute-heading meaning
    across repeated turns until a live fused heading this function can
    subtract is wired host-side (unrelated to this sprint). ``eps`` (the
    closed-loop tolerance gate) has no open-loop analogue -- ``Drive::``'s
    own terminal settle machine self-terminates on measured state
    regardless -- so callers accept it for the OK reply body only; it is
    not a parameter here.
    """
    seg = segment_for_seg(delta_heading=float(heading) * _CDEG_TO_RAD)
    return seg


def segment_for_goto_relative(x: float,  # [mm]
                              y: float,  # [mm]
                              speed: float = 0.0,  # [mm/s]
                              ) -> list[motion_pb2.MotionSegment]:
    """``handleG()`` (motion_commands.cpp, pre-097-006 gut): ``G <x> <y>
    <speed>`` posts a ``msg::GotoGoal`` -- ``Subsystems::Planner`` owns an
    internal PRE_ROTATE/PURSUE state machine that closes the loop on fused
    pose the entire way to the target.

    DEVIATION (097, carried forward as a LIST by 100-007): Planner is
    parked and fused pose is not live host-side, so this builds the
    open-loop segment approximation ``MOVE`` itself would use for a single
    pre-pivot-then-straight leg: pivot to face the relative target
    (``direction = atan2(y, x)``), drive the straight-line distance to it
    (``distance = hypot(x, y)``), and finish facing the direction of travel
    (``final_heading = direction``, a deliberate choice -- NOT 0 -- so the
    robot does not waste motion pivoting back to its start heading once it
    arrives). No mid-course correction: open-loop primitives, not a pursuit
    loop.

    100-007, THE CUTOVER: reuses ``primitives_for_move()`` verbatim
    (``final_heading == direction``, so its own trailing-pivot phase is
    ALWAYS omitted here by construction -- at most 2 primitives: a leading
    pivot then a straight run) -- returns a LIST, not a single
    ``MotionSegment``, mirroring MOVE's own multi-envelope shape. ``speed``
    (mm/s, the old ``speed_max`` ceiling) has no v2 primitive equivalent --
    see ``primitives_for_move()``'s own docstring on why per-segment speed/
    accel/jerk overrides are dropped, not an oversight of this function.
    """
    distance = math.hypot(x, y)
    direction_rad = math.atan2(y, x)
    direction_cdeg = direction_rad / _CDEG_TO_RAD
    return primitives_for_move(distance, direction_cdeg, direction_cdeg)
