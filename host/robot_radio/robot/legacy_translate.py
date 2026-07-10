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

Scope this ticket (097-002): S (drive), T (timed), D (distance) -- the three
legacy verbs ``NezhaProtocol.drive()``/``.timed()``/``.distance()`` need
(ticket 002's own ten-method conversion list). architecture-update.md's M4
module description also names RT/MOVE/MOVER as computations this translator
eventually covers (a shared dependency of ticket 004, which speaks the full
v2 text grammar), but ``NezhaProtocol`` has no ``rt()``/``move()``/
``mover()`` method today -- nothing in this ticket's own acceptance criteria
needs them. Left for ticket 004 (or a follow-up) to extend this module when
it builds those verbs' translation; this file's own functions are additive,
so extending it later is not a breaking change to what's here now.

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
"""

from __future__ import annotations

from robot_radio.robot.pb2 import common_pb2, drivetrain_pb2, motion_pb2


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
