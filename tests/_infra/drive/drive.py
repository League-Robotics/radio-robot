"""drive.py -- ctypes loader for the tier-0 host Drive:: library (ticket
100-006), wrapping drive_api.cpp's extern "C" ABI
(tests/_infra/drive/drive_api.cpp) with Python-friendly dataclasses --
mirrors tests/_infra/sim/firmware.py's ``Sim`` class shape as closely as the
two subsystems' different surfaces allow.

Loads ``libdrive_host.dylib`` (macOS) / ``.so`` (Linux) -- built by
``just build-drive`` (tests/_infra/drive/CMakeLists.txt) -- and exposes the
ABI through two classes:

  ``Drive``  -- one immutable ``Drive::Drivetrain`` config (limits +
                trackwidth), a context manager mirroring ``Sim``'s own
                ``with Sim() as sim:`` shape. ``admit()``/``plan()``/
                ``replan()``/``plan_velocity()``.
  ``Plan``   -- one solved ``Drive::MotionPlan``, ALSO a context manager
                (``Drive.plan()``/``replan()``/``plan_velocity()`` return a
                ``PlanResult`` carrying one of these, or ``None`` on a
                non-OK verdict). ``duration()``/``reference_at()``/
                ``step()`` and the other pure plan queries.

Usage::

    from drive import Drive, Limits, ProfileLimits, Goal, PlanRequest, Pose

    limits = Limits(linear=ProfileLimits(velocity=400.0, accel=800.0, decel=800.0),
                     rotational=ProfileLimits(velocity=3.0, accel=15.0, decel=15.0),
                     v_wheel_max=600.0, trim_v_max=120.0, trim_omega_max=1.0,
                     track_k_s=2.0, track_k_theta=6.0, track_k_cross=1.5e-5,
                     min_speed=20.0)
    with Drive(limits, trackwidth=128.0) as drive:
        result = drive.plan(PlanRequest(goal=Goal(arc_length=500.0), start=Pose()))
        assert result.verdict == Verdict.OK
        with result.plan as plan:
            ref = plan.reference_at(0.5)

See tests/sim/drive/conftest.py's ``build_drive_lib`` fixture for the pytest
integration.
"""
from __future__ import annotations

import ctypes
import enum
import pathlib
import sys
from dataclasses import dataclass, field

_HERE = pathlib.Path(__file__).resolve().parent


def _lib_name() -> str:
    return "libdrive_host.dylib" if sys.platform == "darwin" else "libdrive_host.so"


_DEFAULT_LIB_PATH = _HERE / "build" / _lib_name()


# ---------------------------------------------------------------------------
# Public value types -- dataclasses mirroring source/drive/types.h /
# motion_plan.h / drivetrain.h field-for-field, in Python-idiomatic
# snake_case (the ctypes.Structure field spelling below is independent --
# what matters for ABI compatibility is field ORDER and TYPE, not name; see
# drive_api.cpp's own "Struct-passing convention" file header).
# ---------------------------------------------------------------------------


@dataclass
class ProfileLimits:
    velocity: float = 0.0  # [mm/s] or [rad/s] outer ceiling
    accel: float = 0.0  # [mm/s^2] or [rad/s^2]
    decel: float = 0.0  # [mm/s^2] or [rad/s^2]
    jerk: float = 0.0  # [mm/s^3] or [rad/s^3]; 0 = Ruckig's own +infinity sentinel


@dataclass
class Limits:
    linear: ProfileLimits = field(default_factory=ProfileLimits)
    rotational: ProfileLimits = field(default_factory=ProfileLimits)
    v_wheel_max: float = 0.0  # [mm/s]
    trim_v_max: float = 0.0  # [mm/s]
    trim_omega_max: float = 0.0  # [rad/s]
    wheel_step_max: float = 0.0  # [mm/s]
    track_k_s: float = 0.0  # [1/s]
    track_k_theta: float = 0.0  # [1/s]
    track_k_cross: float = 0.0  # [rad/mm^2]
    min_speed: float = 0.0  # [mm/s] pivot-mode threshold


@dataclass
class Pose:
    x: float = 0.0  # [mm]
    y: float = 0.0  # [mm]
    h: float = 0.0  # [rad] heading, CCW+, wrapped to (-pi, pi]


@dataclass
class Twist:
    v_x: float = 0.0  # [mm/s] body-forward
    v_y: float = 0.0  # [mm/s] body-lateral; 0 for a differential drivetrain
    omega: float = 0.0  # [rad/s] yaw rate, CCW+


@dataclass
class Goal:
    arc_length: float = 0.0  # [mm] signed path length; 0 = pivot in place
    delta_heading: float = 0.0  # [rad] total heading change, CCW+
    exit_speed: float = 0.0  # [mm/s] boundary velocity at segment end; 0 = stop


@dataclass
class PlanRequest:
    goal: Goal = field(default_factory=Goal)
    start: Pose = field(default_factory=Pose)
    entry_speed: float = 0.0  # [mm/s]
    entry_accel: float = 0.0  # [mm/s^2]


@dataclass
class WheelVelocities:
    left: float = 0.0  # [mm/s]
    right: float = 0.0  # [mm/s]


@dataclass
class WheelState:
    position: float = 0.0  # [mm]
    velocity: float = 0.0  # [mm/s]
    position_valid: bool = False
    velocity_valid: bool = False


@dataclass
class BodyState:
    pose: Pose = field(default_factory=Pose)
    twist: Twist = field(default_factory=Twist)


@dataclass
class RefState:
    s: float = 0.0  # [mm] or [rad] master-DOF position
    v: float = 0.0  # [mm/s] body speed along path (0 during pivot)
    a: float = 0.0  # [mm/s^2]
    theta: float = 0.0  # [rad] reference heading (world)
    omega: float = 0.0  # [rad/s]
    alpha: float = 0.0  # [rad/s^2]
    x: float = 0.0  # [mm] reference world position
    y: float = 0.0  # [mm]


@dataclass
class StepState:
    dwell_start: float = -1.0  # [s] terminal tolerance first held (<0 = not held)
    sustain_start: float = -1.0  # [s] replan-envelope first exceeded (<0 = inside)
    last_replan: float = -1.0  # [s] rate-limit anchor
    replan_count: int = 0  # toward the N-max abort
    settling: bool = False  # terminal state machine entered


@dataclass
class StepInput:
    t: float = 0.0  # [s] elapsed since plan start (caller's clock)
    measured: BodyState = field(default_factory=BodyState)
    left: WheelState = field(default_factory=WheelState)
    right: WheelState = field(default_factory=WheelState)
    pose_step: float = 0.0  # [mm] magnitude of an external pose-fix step
    pose_step_theta: float = 0.0  # [rad] applied since last step (0 = none)


class Verdict(enum.IntEnum):
    OK = 0
    EXIT_UNREACHABLE = 1
    JOINT_STEP_TOO_LARGE = 2
    JOINT_SIGN_REVERSAL = 3
    PIVOT_NONZERO_EXIT = 4
    RADIUS_TOO_TIGHT = 5
    CEILING_INFEASIBLE = 6
    SOLVE_FAILED = 7


class Status(enum.IntEnum):
    RUNNING = 0
    SETTLING = 1
    REPLAN_DUE = 2
    DONE_STOP = 3
    DONE_HANDOFF = 4
    ABORT_TIMEOUT = 5
    ABORT_REPLAN_LIMIT = 6


@dataclass
class TrackRecord:
    in_: StepInput = field(default_factory=StepInput)
    ref: RefState = field(default_factory=RefState)
    e_along: float = 0.0  # [mm]
    e_cross: float = 0.0  # [mm]
    e_theta: float = 0.0  # [rad]
    v_trim: float = 0.0  # [mm/s]
    omega_trim: float = 0.0  # [rad/s]
    v_cmd: float = 0.0  # [mm/s]
    omega_cmd: float = 0.0  # [rad/s]
    wheel_left: float = 0.0  # [mm/s]
    wheel_right: float = 0.0  # [mm/s]
    trim_saturated: bool = False
    status: Status = Status.RUNNING


@dataclass
class StepOutput:
    command: WheelVelocities = field(default_factory=WheelVelocities)
    status: Status = Status.RUNNING
    record: TrackRecord = field(default_factory=TrackRecord)


@dataclass
class ChainTail:
    pose: Pose = field(default_factory=Pose)
    exit_speed: float = 0.0  # [mm/s]
    kappa: float = 0.0  # [1/mm]


# ---------------------------------------------------------------------------
# ctypes.Structure mirrors -- field ORDER and TYPE must match drive_api.cpp's
# Drv* structs exactly (field NAMES are Python-side only, see module
# docstring). Every field is c_float or c_int32 (drive_api.cpp's own
# "Struct-passing convention").
# ---------------------------------------------------------------------------


class _CProfileLimits(ctypes.Structure):
    _fields_ = [
        ("velocity", ctypes.c_float),
        ("accel", ctypes.c_float),
        ("decel", ctypes.c_float),
        ("jerk", ctypes.c_float),
    ]


class _CLimits(ctypes.Structure):
    _fields_ = [
        ("linear", _CProfileLimits),
        ("rotational", _CProfileLimits),
        ("v_wheel_max", ctypes.c_float),
        ("trim_v_max", ctypes.c_float),
        ("trim_omega_max", ctypes.c_float),
        ("wheel_step_max", ctypes.c_float),
        ("track_k_s", ctypes.c_float),
        ("track_k_theta", ctypes.c_float),
        ("track_k_cross", ctypes.c_float),
        ("min_speed", ctypes.c_float),
    ]


class _CPose(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("h", ctypes.c_float)]


class _CTwist(ctypes.Structure):
    _fields_ = [("v_x", ctypes.c_float), ("v_y", ctypes.c_float), ("omega", ctypes.c_float)]


class _CGoal(ctypes.Structure):
    _fields_ = [
        ("arc_length", ctypes.c_float),
        ("delta_heading", ctypes.c_float),
        ("exit_speed", ctypes.c_float),
    ]


class _CPlanRequest(ctypes.Structure):
    _fields_ = [
        ("goal", _CGoal),
        ("start", _CPose),
        ("entry_speed", ctypes.c_float),
        ("entry_accel", ctypes.c_float),
    ]


class _CWheelVelocities(ctypes.Structure):
    _fields_ = [("left", ctypes.c_float), ("right", ctypes.c_float)]


class _CWheelState(ctypes.Structure):
    _fields_ = [
        ("position", ctypes.c_float),
        ("velocity", ctypes.c_float),
        ("position_valid", ctypes.c_int32),
        ("velocity_valid", ctypes.c_int32),
    ]


class _CBodyState(ctypes.Structure):
    _fields_ = [("pose", _CPose), ("twist", _CTwist)]


class _CRefState(ctypes.Structure):
    _fields_ = [
        ("s", ctypes.c_float),
        ("v", ctypes.c_float),
        ("a", ctypes.c_float),
        ("theta", ctypes.c_float),
        ("omega", ctypes.c_float),
        ("alpha", ctypes.c_float),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
    ]


class _CStepState(ctypes.Structure):
    _fields_ = [
        ("dwell_start", ctypes.c_float),
        ("sustain_start", ctypes.c_float),
        ("last_replan", ctypes.c_float),
        ("replan_count", ctypes.c_int32),
        ("settling", ctypes.c_int32),
    ]


class _CStepInput(ctypes.Structure):
    _fields_ = [
        ("t", ctypes.c_float),
        ("measured", _CBodyState),
        ("left", _CWheelState),
        ("right", _CWheelState),
        ("pose_step", ctypes.c_float),
        ("pose_step_theta", ctypes.c_float),
    ]


class _CTrackRecord(ctypes.Structure):
    _fields_ = [
        ("in_", _CStepInput),
        ("ref", _CRefState),
        ("e_along", ctypes.c_float),
        ("e_cross", ctypes.c_float),
        ("e_theta", ctypes.c_float),
        ("v_trim", ctypes.c_float),
        ("omega_trim", ctypes.c_float),
        ("v_cmd", ctypes.c_float),
        ("omega_cmd", ctypes.c_float),
        ("wheel_left", ctypes.c_float),
        ("wheel_right", ctypes.c_float),
        ("trim_saturated", ctypes.c_int32),
        ("status", ctypes.c_int32),
    ]


class _CStepOutput(ctypes.Structure):
    _fields_ = [
        ("command", _CWheelVelocities),
        ("status", ctypes.c_int32),
        ("record", _CTrackRecord),
    ]


class _CChainTail(ctypes.Structure):
    _fields_ = [("pose", _CPose), ("exit_speed", ctypes.c_float), ("kappa", ctypes.c_float)]


# ---------------------------------------------------------------------------
# dataclass <-> ctypes.Structure conversion helpers.
# ---------------------------------------------------------------------------


def _to_c_profile_limits(p: ProfileLimits) -> _CProfileLimits:
    return _CProfileLimits(p.velocity, p.accel, p.decel, p.jerk)


def _to_c_limits(limits: Limits) -> _CLimits:
    return _CLimits(
        _to_c_profile_limits(limits.linear),
        _to_c_profile_limits(limits.rotational),
        limits.v_wheel_max,
        limits.trim_v_max,
        limits.trim_omega_max,
        limits.wheel_step_max,
        limits.track_k_s,
        limits.track_k_theta,
        limits.track_k_cross,
        limits.min_speed,
    )


def _to_c_pose(p: Pose) -> _CPose:
    return _CPose(p.x, p.y, p.h)


def _from_c_pose(c: _CPose) -> Pose:
    return Pose(c.x, c.y, c.h)


def _to_c_twist(t: Twist) -> _CTwist:
    return _CTwist(t.v_x, t.v_y, t.omega)


def _from_c_twist(c: _CTwist) -> Twist:
    return Twist(c.v_x, c.v_y, c.omega)


def _to_c_goal(g: Goal) -> _CGoal:
    return _CGoal(g.arc_length, g.delta_heading, g.exit_speed)


def _to_c_plan_request(r: PlanRequest) -> _CPlanRequest:
    return _CPlanRequest(_to_c_goal(r.goal), _to_c_pose(r.start), r.entry_speed, r.entry_accel)


def _from_c_wheel_velocities(c: _CWheelVelocities) -> WheelVelocities:
    return WheelVelocities(c.left, c.right)


def _to_c_wheel_state(w: WheelState) -> _CWheelState:
    return _CWheelState(w.position, w.velocity, 1 if w.position_valid else 0,
                         1 if w.velocity_valid else 0)


def _from_c_wheel_state(c: _CWheelState) -> WheelState:
    return WheelState(c.position, c.velocity, bool(c.position_valid), bool(c.velocity_valid))


def _to_c_body_state(b: BodyState) -> _CBodyState:
    return _CBodyState(_to_c_pose(b.pose), _to_c_twist(b.twist))


def _from_c_body_state(c: _CBodyState) -> BodyState:
    return BodyState(_from_c_pose(c.pose), _from_c_twist(c.twist))


def _from_c_ref_state(c: _CRefState) -> RefState:
    return RefState(c.s, c.v, c.a, c.theta, c.omega, c.alpha, c.x, c.y)


def _to_c_step_state(s: StepState) -> _CStepState:
    return _CStepState(s.dwell_start, s.sustain_start, s.last_replan, s.replan_count,
                        1 if s.settling else 0)


def _from_c_step_state(c: _CStepState) -> StepState:
    return StepState(c.dwell_start, c.sustain_start, c.last_replan, c.replan_count,
                      bool(c.settling))


def _to_c_step_input(i: StepInput) -> _CStepInput:
    return _CStepInput(i.t, _to_c_body_state(i.measured), _to_c_wheel_state(i.left),
                        _to_c_wheel_state(i.right), i.pose_step, i.pose_step_theta)


def _from_c_step_input(c: _CStepInput) -> StepInput:
    return StepInput(c.t, _from_c_body_state(c.measured), _from_c_wheel_state(c.left),
                      _from_c_wheel_state(c.right), c.pose_step, c.pose_step_theta)


def _from_c_track_record(c: _CTrackRecord) -> TrackRecord:
    return TrackRecord(
        _from_c_step_input(c.in_), _from_c_ref_state(c.ref), c.e_along, c.e_cross, c.e_theta,
        c.v_trim, c.omega_trim, c.v_cmd, c.omega_cmd, c.wheel_left, c.wheel_right,
        bool(c.trim_saturated), Status(c.status),
    )


def _from_c_step_output(c: _CStepOutput) -> StepOutput:
    return StepOutput(_from_c_wheel_velocities(c.command), Status(c.status),
                       _from_c_track_record(c.record))


def _to_c_chain_tail(t: ChainTail) -> _CChainTail:
    return _CChainTail(_to_c_pose(t.pose), t.exit_speed, t.kappa)


def _from_c_chain_tail(c: _CChainTail) -> ChainTail:
    return ChainTail(_from_c_pose(c.pose), c.exit_speed, c.kappa)


# ---------------------------------------------------------------------------
# Plan -- wraps one opaque Drive::MotionPlan* (drive_api.cpp's "Opaque
# handles"). A context manager; drive_plan_destroy() is called exactly once,
# either by __exit__ or an explicit close() (idempotent, mirrors Sim.close()).
# ---------------------------------------------------------------------------


class Plan:
    def __init__(self, lib: ctypes.CDLL, address: int) -> None:
        self._lib = lib
        self._h: int | None = address

    def __enter__(self) -> "Plan":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def close(self) -> None:
        if self._h:
            self._lib.drive_plan_destroy(ctypes.c_void_p(self._h))
            self._h = None

    def duration(self) -> float:  # [s]
        return float(self._lib.drive_plan_duration(ctypes.c_void_p(self._h)))

    def kappa(self) -> float:  # [1/mm]
        return float(self._lib.drive_plan_kappa(ctypes.c_void_p(self._h)))

    def anchor(self) -> Pose:
        out = _CPose()
        self._lib.drive_plan_anchor(ctypes.c_void_p(self._h), ctypes.byref(out))
        return _from_c_pose(out)

    def goal(self) -> Pose:
        out = _CPose()
        self._lib.drive_plan_goal(ctypes.c_void_p(self._h), ctypes.byref(out))
        return _from_c_pose(out)

    def exit_speed(self) -> float:  # [mm/s]
        return float(self._lib.drive_plan_exit_speed(ctypes.c_void_p(self._h)))

    def effective_ceiling(self) -> float:  # [mm/s] or [rad/s]
        return float(self._lib.drive_plan_effective_ceiling(ctypes.c_void_p(self._h)))

    def is_pivot(self) -> bool:
        return bool(self._lib.drive_plan_is_pivot(ctypes.c_void_p(self._h)))

    def is_velocity_mode(self) -> bool:
        return bool(self._lib.drive_plan_is_velocity_mode(ctypes.c_void_p(self._h)))

    def reference_at(self, elapsed: float) -> RefState:  # [s]
        out = _CRefState()
        self._lib.drive_reference_at(ctypes.c_void_p(self._h), ctypes.c_float(elapsed),
                                      ctypes.byref(out))
        return _from_c_ref_state(out)

    def step(self, step_input: StepInput, state: StepState) -> tuple[StepOutput, StepState]:
        """MotionPlan::step()'s own (const StepInput&, StepState*) contract:
        `state` is NOT mutated in place (Python-side purity convenience --
        see module docstring) -- the resulting StepState is returned
        alongside StepOutput; the caller decides whether to keep it."""
        c_in = _to_c_step_input(step_input)
        c_state = _to_c_step_state(state)
        c_out = _CStepOutput()
        self._lib.drive_step(ctypes.c_void_p(self._h), ctypes.byref(c_in), ctypes.byref(c_state),
                              ctypes.byref(c_out))
        return _from_c_step_output(c_out), _from_c_step_state(c_state)


@dataclass
class PlanResult:
    verdict: Verdict
    plan: Plan | None


# ---------------------------------------------------------------------------
# Drive -- wraps one Drive::Drivetrain* (immutable config). A context
# manager, mirroring firmware.py's Sim class.
# ---------------------------------------------------------------------------


class Drive:
    def __init__(self, limits: Limits, trackwidth: float,
                 lib_path: str | pathlib.Path | None = None) -> None:
        self._lib_path = pathlib.Path(lib_path) if lib_path else _DEFAULT_LIB_PATH
        if not self._lib_path.exists():
            raise FileNotFoundError(
                f"libdrive_host not found at {self._lib_path} -- build it first: just build-drive"
            )
        self._lib = ctypes.CDLL(str(self._lib_path))
        self._setup_types()
        c_limits = _to_c_limits(limits)
        self._h = self._lib.drive_create(ctypes.byref(c_limits), ctypes.c_float(trackwidth))
        if not self._h:
            raise RuntimeError("drive_create() returned NULL")

    def __enter__(self) -> "Drive":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def close(self) -> None:
        """Destroy the underlying Drivetrain. Idempotent -- safe to call
        more than once, mirroring Sim.close()."""
        if self._h is not None:
            self._lib.drive_destroy(self._h)
            self._h = None

    def admit(self, goal: Goal, tail: ChainTail) -> Verdict:
        c_goal = _to_c_goal(goal)
        c_tail = _to_c_chain_tail(tail)
        return Verdict(self._lib.drive_admit(self._h, ctypes.byref(c_goal), ctypes.byref(c_tail)))

    def advance(self, goal: Goal, tail: ChainTail) -> ChainTail:
        c_goal = _to_c_goal(goal)
        c_tail = _to_c_chain_tail(tail)
        c_out = _CChainTail()
        self._lib.drive_advance(self._h, ctypes.byref(c_goal), ctypes.byref(c_tail),
                                 ctypes.byref(c_out))
        return _from_c_chain_tail(c_out)

    def plan(self, request: PlanRequest) -> PlanResult:
        c_req = _to_c_plan_request(request)
        handle = ctypes.c_void_p()
        verdict = self._lib.drive_plan(self._h, ctypes.byref(c_req), ctypes.byref(handle))
        plan_obj = Plan(self._lib, handle.value) if handle.value else None
        return PlanResult(Verdict(verdict), plan_obj)

    def replan(self, plan: Plan, measured: BodyState, elapsed: float) -> PlanResult:
        c_measured = _to_c_body_state(measured)
        handle = ctypes.c_void_p()
        verdict = self._lib.drive_replan(self._h, ctypes.c_void_p(plan._h),
                                          ctypes.byref(c_measured), ctypes.c_float(elapsed),
                                          ctypes.byref(handle))
        plan_obj = Plan(self._lib, handle.value) if handle.value else None
        return PlanResult(Verdict(verdict), plan_obj)

    def plan_velocity(self, target: Twist, deadman: float, current: BodyState) -> PlanResult:
        c_target = _to_c_twist(target)
        c_current = _to_c_body_state(current)
        handle = ctypes.c_void_p()
        verdict = self._lib.drive_plan_velocity(self._h, ctypes.byref(c_target),
                                                 ctypes.c_float(deadman), ctypes.byref(c_current),
                                                 ctypes.byref(handle))
        plan_obj = Plan(self._lib, handle.value) if handle.value else None
        return PlanResult(Verdict(verdict), plan_obj)

    # ------------------------------------------------------------------
    # Internal: argtypes / restype declarations for every drive_* symbol.
    # ------------------------------------------------------------------

    def _setup_types(self) -> None:
        lib = self._lib

        lib.drive_create.argtypes = [ctypes.POINTER(_CLimits), ctypes.c_float]
        lib.drive_create.restype = ctypes.c_void_p

        lib.drive_destroy.argtypes = [ctypes.c_void_p]
        lib.drive_destroy.restype = None

        lib.drive_admit.argtypes = [ctypes.c_void_p, ctypes.POINTER(_CGoal),
                                     ctypes.POINTER(_CChainTail)]
        lib.drive_admit.restype = ctypes.c_int

        lib.drive_advance.argtypes = [ctypes.c_void_p, ctypes.POINTER(_CGoal),
                                       ctypes.POINTER(_CChainTail), ctypes.POINTER(_CChainTail)]
        lib.drive_advance.restype = None

        lib.drive_plan.argtypes = [ctypes.c_void_p, ctypes.POINTER(_CPlanRequest),
                                    ctypes.POINTER(ctypes.c_void_p)]
        lib.drive_plan.restype = ctypes.c_int

        lib.drive_replan.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                      ctypes.POINTER(_CBodyState), ctypes.c_float,
                                      ctypes.POINTER(ctypes.c_void_p)]
        lib.drive_replan.restype = ctypes.c_int

        lib.drive_plan_velocity.argtypes = [ctypes.c_void_p, ctypes.POINTER(_CTwist),
                                             ctypes.c_float, ctypes.POINTER(_CBodyState),
                                             ctypes.POINTER(ctypes.c_void_p)]
        lib.drive_plan_velocity.restype = ctypes.c_int

        lib.drive_plan_destroy.argtypes = [ctypes.c_void_p]
        lib.drive_plan_destroy.restype = None

        lib.drive_plan_duration.argtypes = [ctypes.c_void_p]
        lib.drive_plan_duration.restype = ctypes.c_float

        lib.drive_plan_kappa.argtypes = [ctypes.c_void_p]
        lib.drive_plan_kappa.restype = ctypes.c_float

        lib.drive_plan_anchor.argtypes = [ctypes.c_void_p, ctypes.POINTER(_CPose)]
        lib.drive_plan_anchor.restype = None

        lib.drive_plan_goal.argtypes = [ctypes.c_void_p, ctypes.POINTER(_CPose)]
        lib.drive_plan_goal.restype = None

        lib.drive_plan_exit_speed.argtypes = [ctypes.c_void_p]
        lib.drive_plan_exit_speed.restype = ctypes.c_float

        lib.drive_plan_effective_ceiling.argtypes = [ctypes.c_void_p]
        lib.drive_plan_effective_ceiling.restype = ctypes.c_float

        lib.drive_plan_is_pivot.argtypes = [ctypes.c_void_p]
        lib.drive_plan_is_pivot.restype = ctypes.c_int

        lib.drive_plan_is_velocity_mode.argtypes = [ctypes.c_void_p]
        lib.drive_plan_is_velocity_mode.restype = ctypes.c_int

        lib.drive_reference_at.argtypes = [ctypes.c_void_p, ctypes.c_float,
                                            ctypes.POINTER(_CRefState)]
        lib.drive_reference_at.restype = None

        lib.drive_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(_CStepInput),
                                    ctypes.POINTER(_CStepState), ctypes.POINTER(_CStepOutput)]
        lib.drive_step.restype = None
