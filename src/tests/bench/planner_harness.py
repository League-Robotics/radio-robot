"""planner_harness.py -- Python ctypes harness over libmotionplanner
(motion-planner sketch §7 tier 3). Mirrors RobotState/Move/PlannerLimits/
TickResult field-for-field, runs the same zero-error perfect plant the C++
scenario tests use, and asserts the exactness gates from Python. Run:

    python3 src/tests/bench/planner_harness.py

Build the library first:

    cmake -S src/firm/motion/planner -B src/firm/motion/planner/build
    cmake --build src/firm/motion/planner/build --target motionplanner

stdlib only -- no numpy, no uv needed.

Relocated (128-010) from src/firm/motion/planner/py/ to src/tests/bench/,
joining the bench scripts (hil_drive.py, square_tour_sim.py, etc.) that
import it -- they used to reach across a sibling `py/` directory via a
sys.path hack; now co-located, no path bootstrap needed for the import
itself. ``loadLibrary()`` below still resolves the C++ build directory by
walking up to the repo root, since the CMake build itself still lives at
src/firm/motion/planner/build (unmoved -- it's real C++ source, not a bench
artifact).
"""

import ctypes
import math
import sys
from pathlib import Path


# ---- struct mirrors (field-for-field with the C++ headers) ----

# Mirrors src/firm/types/robot_state.h (Types::RobotState, the real 124
# blackboard header -- the planner's own mirror was deleted at the joint
# checkpoint, 2026-07-26). Field-for-field; the plannerStructSizes guard
# below fails loudly on any drift.
class Time(ctypes.Structure):
    _fields_ = [("cycleStart", ctypes.c_uint32),   # [ms]
                ("cycleBusy", ctypes.c_uint32),    # [us]
                ("cyclePeriod", ctypes.c_uint32)]  # [us]


class Wheel(ctypes.Structure):
    _fields_ = [("position", ctypes.c_float),      # [mm]
                ("velocity", ctypes.c_float),      # [mm/s]
                ("sampleTime", ctypes.c_uint32),   # [ms]
                ("connected", ctypes.c_bool),
                ("positionEpoch", ctypes.c_uint8),
                ("cmdVelocity", ctypes.c_float),   # [mm/s]
                # cmdAccel -- added 130-003 (src/firm/types/robot_state.h);
                # this ctypes mirror missed it at the time (nobody ran this
                # bench script across 130-003/005/007) -- caught and fixed
                # here, 130-009, as a byproduct of exercising the
                # plannerStructSizes() guard this ticket's own PlannerLimits
                # reshape depends on.
                ("cmdAccel", ctypes.c_float)]      # [mm/s^2]


class Otos(ctypes.Structure):
    _fields_ = [("present", ctypes.c_bool),
                ("connected", ctypes.c_bool),
                ("x", ctypes.c_float), ("y", ctypes.c_float),      # [mm]
                ("heading", ctypes.c_float),                       # [rad]
                ("v_x", ctypes.c_float), ("v_y", ctypes.c_float),  # [mm/s]
                ("omega", ctypes.c_float),                         # [rad/s]
                ("sampleTime", ctypes.c_uint32)]                   # [ms]


class Perception(ctypes.Structure):
    _fields_ = [("line", ctypes.c_uint32), ("color", ctypes.c_uint32),
                ("lineFresh", ctypes.c_bool), ("colorFresh", ctypes.c_bool)]


class Pose(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float),      # [mm]
                ("heading", ctypes.c_float),                       # [rad]
                ("v_x", ctypes.c_float), ("v_y", ctypes.c_float),  # [mm/s]
                ("omega", ctypes.c_float)]                         # [rad/s]


class WheelEstimate(ctypes.Structure):
    _fields_ = [("distance", ctypes.c_float),      # [mm]
                ("velocity", ctypes.c_float),      # [mm/s]
                ("basisTime", ctypes.c_uint32),    # [ms]
                ("valid", ctypes.c_bool)]


class BodyEstimate(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float),      # [mm]
                ("heading", ctypes.c_float),                       # [rad]
                ("v_x", ctypes.c_float), ("v_y", ctypes.c_float),  # [mm/s]
                ("omega", ctypes.c_float),                         # [rad/s]
                ("basisTime", ctypes.c_uint32),                    # [ms]
                ("valid", ctypes.c_bool)]


class Innovations(ctypes.Structure):
    _fields_ = [("heading", ctypes.c_float),  # [rad]
                ("omega", ctypes.c_float),    # [rad/s]
                ("valid", ctypes.c_bool)]


class Estimate(ctypes.Structure):
    _fields_ = [("wheelLeft", WheelEstimate),
                ("wheelRight", WheelEstimate),
                ("body", BodyEstimate),
                ("innovations", Innovations)]


class Command(ctypes.Structure):
    _fields_ = [("mode", ctypes.c_uint8),          # Types::Mode
                ("moveActive", ctypes.c_bool),
                ("v_x", ctypes.c_float),           # [mm/s]
                ("omega", ctypes.c_float)]         # [rad/s]


class Health(ctypes.Structure):
    _fields_ = [("i2cSafetyNetCount", ctypes.c_uint32),
                ("commsMalformedCount", ctypes.c_uint32),
                ("commandsDroppedCount", ctypes.c_uint32),
                ("wedgeLatch", ctypes.c_bool),
                ("moveTimeout", ctypes.c_bool),
                ("shapingDisabled", ctypes.c_bool),
                ("positionClamped", ctypes.c_bool),
                # wheelFrozenLeft/wheelFrozenRight/ready -- like Wheel.
                # cmdAccel above, missing from this mirror since whichever
                # ticket added them to robot_state.h; caught and fixed here,
                # 130-009, for the same reason.
                ("wheelFrozenLeft", ctypes.c_bool),
                ("wheelFrozenRight", ctypes.c_bool),
                ("ready", ctypes.c_bool)]


class RobotState(ctypes.Structure):
    _fields_ = [("time", Time),
                ("wheelLeft", Wheel), ("wheelRight", Wheel),
                ("otos", Otos), ("perception", Perception),
                ("pose", Pose), ("estimate", Estimate),
                ("command", Command), ("health", Health)]


# KIND_STOP -- the planned-stop queue entry (Move::Kind::Stop,
# command-ingestion-...-two-stops.md §5); build one via
# lib.plannerPlannedStop(), not by hand-filling a Move.
KIND_TIME, KIND_DISTANCE, KIND_ANGLE, KIND_STOP = 0, 1, 2, 3
VELOCITY_TWIST, VELOCITY_WHEELS = 0, 1


class Move(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint32),
                ("kind", ctypes.c_uint8),
                ("threshold", ctypes.c_float),   # [ms]/[mm]/[rad]
                ("timeout", ctypes.c_float),     # [ms]
                ("velocityKind", ctypes.c_uint8),
                ("v_x", ctypes.c_float),         # [mm/s]
                ("v_y", ctypes.c_float),         # [mm/s] ignored on differential
                ("omega", ctypes.c_float),       # [rad/s]
                ("vLeft", ctypes.c_float),       # [mm/s]
                ("vRight", ctypes.c_float),      # [mm/s]
                # 134-001. What the CALLER asked for, before an
                # ingestion-side rewrite of `threshold` (Core::RobotLoop::
                # handleMove()'s rotation-calibration inversion) turns that
                # into an actuation-sized command. Read only by the
                # Planner's cumulative-baseline ledger; <= 0 means UNSET and
                # falls back to `threshold`, so a harness Move that leaves
                # it at zero behaves exactly as it did before this field
                # existed.
                ("requestedThreshold", ctypes.c_float)]  # [ms]/[mm]/[rad]


# PlannerLimits -- 130-009 reshaped this from 34 flat fields to 18 fields
# under four sub-structs (ceilings/plant/landing/tracking), grouped
# field-for-field the same way src/firm/motion/planner/planner_types.h groups
# the real C++ struct. The 16 fields cut by that ticket (requireSettle/
# settleWindow, the M4 duty-stage gains velKff/velKp/velKi/velIMax/
# velKaff/velIAccelGate/dutyFloor, and the dead planner-side trim gains
# trimKp/trimKi/trimIMax/trimKaff/trimMax -- see planner_types.h's own
# doc comment for why each is gone) have no mirror here any more.
class Ceilings(ctypes.Structure):
    _fields_ = [("vMax", ctypes.c_float),        # [mm/s]
                ("aMax", ctypes.c_float),        # [mm/s^2]
                ("aDecel", ctypes.c_float),      # [mm/s^2]
                ("omegaMax", ctypes.c_float),    # [rad/s]
                ("alphaMax", ctypes.c_float),    # [rad/s^2]
                ("alphaDecel", ctypes.c_float),  # [rad/s^2]
                ("jerkMax", ctypes.c_float),     # [mm/s^3] S-curve
                ("yawJerkMax", ctypes.c_float)]  # [rad/s^3]


class Plant(ctypes.Structure):
    _fields_ = [("trackWidth", ctypes.c_float),      # [mm]
                ("controlPeriod", ctypes.c_float),   # [ms]
                ("actuationDelay", ctypes.c_float),  # [ms]
                ("velocityFilterWeight", ctypes.c_float)]


class Landing(ctypes.Structure):
    _fields_ = [("settleEpsilonLinear", ctypes.c_float),   # [mm]
                ("settleEpsilonAngular", ctypes.c_float),  # [rad]
                ("settleRestVelocity", ctypes.c_float),    # [mm/s]
                ("settleRestOmega", ctypes.c_float),       # [rad/s]
                ("decelPlanFraction", ctypes.c_float),     # [1] decel leeway
                # 134-003 terminal fine-align. alignTol is [rad] (the
                # report's 1.0 deg operating point, converted once in the
                # robot JSON); alignMaxNudges is c_int32, matching the C++
                # side's int32_t so Landing keeps its 4-byte stride.
                ("alignTol", ctypes.c_float),              # [rad] 0 = off
                ("alignMaxNudges", ctypes.c_int32)]        # 0 = off


class Tracking(ctypes.Structure):
    _fields_ = [("headingHoldGain", ctypes.c_float)]  # [1/s]


class PlannerLimits(ctypes.Structure):
    _fields_ = [("ceilings", Ceilings),
                ("plant", Plant),
                ("landing", Landing),
                ("tracking", Tracking)]


# Flat (group, field) path list, in the SAME order as capi.cpp's
# plannerLimitsOffsets() -- what the offset guard below walks to compare
# each leaf field's ABSOLUTE offset against the C++ side. Kept separate
# from the nested ctypes classes above (which is what Python code actually
# sets/reads) purely so the guard has one flat list to zip against the C
# side's flat kOffsets array.
_LIMITS_FIELD_PATHS = [
    ("ceilings", "vMax"), ("ceilings", "aMax"), ("ceilings", "aDecel"),
    ("ceilings", "omegaMax"), ("ceilings", "alphaMax"), ("ceilings", "alphaDecel"),
    ("ceilings", "jerkMax"), ("ceilings", "yawJerkMax"),
    ("plant", "trackWidth"), ("plant", "controlPeriod"),
    ("plant", "actuationDelay"), ("plant", "velocityFilterWeight"),
    ("landing", "settleEpsilonLinear"), ("landing", "settleEpsilonAngular"),
    ("landing", "settleRestVelocity"), ("landing", "settleRestOmega"),
    ("landing", "decelPlanFraction"),
    ("landing", "alignTol"), ("landing", "alignMaxNudges"),
    ("tracking", "headingHoldGain"),
]


# Motion::MovePhase, mirrored for the bench charts' phase shading.
class MovePhase:
    IDLE = 0
    ACCEL = 1
    HOLD = 2
    DECEL = 3
    SETTLE = 4

    NAMES = {0: "idle", 1: "accel", 2: "hold", 3: "decel", 4: "settle"}


class TickResult(ctypes.Structure):
    _fields_ = [("completed", ctypes.c_bool),
                ("moveId", ctypes.c_uint32),
                ("timedOut", ctypes.c_bool),
                ("settled", ctypes.c_bool)]


# ---- library ----

def loadLibrary() -> ctypes.CDLL:
    # 128-010: this file now lives at src/tests/bench/planner_harness.py,
    # four levels below the repo root -- the CMake build directory itself
    # is unmoved (src/firm/motion/planner/build).
    repoRoot = Path(__file__).resolve().parents[3]
    build = repoRoot / "src" / "firm" / "motion" / "planner" / "build"
    for name in ("libmotionplanner.dylib", "libmotionplanner.so"):
        candidate = build / name
        if candidate.exists():
            lib = ctypes.CDLL(str(candidate))
            break
    else:
        sys.exit(f"libmotionplanner not found under {build} -- build it first")

    lib.plannerCreate.restype = ctypes.c_void_p
    lib.plannerCreate.argtypes = [ctypes.POINTER(PlannerLimits)]
    lib.plannerDestroy.argtypes = [ctypes.c_void_p]
    lib.plannerMove.restype = ctypes.c_bool
    lib.plannerMove.argtypes = [ctypes.c_void_p, ctypes.POINTER(Move),
                                ctypes.c_bool]
    lib.plannerEstop.argtypes = [ctypes.c_void_p]
    lib.plannerPlannedStop.restype = ctypes.c_bool
    lib.plannerPlannedStop.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.plannerTick.argtypes = [ctypes.c_void_p, ctypes.POINTER(RobotState),
                                ctypes.POINTER(TickResult)]
    lib.plannerUpdate.argtypes = [ctypes.c_void_p, ctypes.POINTER(RobotState)]
    lib.plannerStructSizes.argtypes = [ctypes.POINTER(ctypes.c_uint32)] * 4
    # 130-007: plannerDuty() (the M4 duty stage's C API, commandedDutyLeft/
    # Right()) is deleted with the rest of the parked duty stage --
    # Motion::WheelPid/Planner::stageDuty() have no live callers left. The
    # binding above it is deleted with it; the two bench scripts that used
    # to read it (hil_drive.py's --duty mode, square_tour_sim.py) are
    # retired/repointed -- see their own module docstrings.
    # 130-005: plannerTrim()'s signature shrank to (profiledLeft,
    # profiledRight, phase) -- Motion::WheelTrim (trim + its integrator) is
    # deleted outright; the wheel-speed controller now lives entirely in
    # Core::DifferentialDrive (drive.h's own header), whose bias/fast-PID observability
    # reaches the wire Telemetry frame instead of this bench-only C API.
    # plannerApplyTrimGains() is deleted with it -- there is no live gains
    # setter left on Motion::Planner to bind.
    lib.plannerTrim.argtypes = ([ctypes.c_void_p] +
                                [ctypes.POINTER(ctypes.c_float)] * 2 +
                                [ctypes.POINTER(ctypes.c_uint8)])

    lib.plannerLimitsOffsets.restype = ctypes.c_uint32
    lib.plannerLimitsOffsets.argtypes = [ctypes.POINTER(ctypes.c_uint32),
                                         ctypes.c_uint32]

    # Layout guard, part 1: C++ sizeof must match the ctypes mirrors.
    sizes = [ctypes.c_uint32() for _ in range(4)]
    lib.plannerStructSizes(*[ctypes.byref(s) for s in sizes])
    expected = [RobotState, Move, PlannerLimits, TickResult]
    for cSize, pyType in zip(sizes, expected):
        assert cSize.value == ctypes.sizeof(pyType), (
            f"{pyType.__name__}: C++ sizeof {cSize.value} != "
            f"ctypes {ctypes.sizeof(pyType)} -- mirror out of date")

    # Layout guard, part 2: per-field OFFSETS for PlannerLimits. Size alone
    # does not catch a field inserted mid-struct on one side and appended
    # on the other -- both totals match while every field after the
    # insertion point reads a different member. That happened (the trim
    # gains, 2026-07-27) and presented as a wildly mistuned controller
    # rather than as a layout error, so check the offsets.
    #
    # 130-009: PlannerLimits is now grouped into four ctypes sub-structs
    # (Ceilings/Plant/Landing/Tracking) -- _LIMITS_FIELD_PATHS walks the
    # flat (group, field) order capi.cpp's kOffsets array uses, and each
    # leaf's ABSOLUTE offset is the group's own offset within PlannerLimits
    # plus the field's offset within the group.
    count = lib.plannerLimitsOffsets(None, 0)
    buffer = (ctypes.c_uint32 * count)()
    lib.plannerLimitsOffsets(buffer, count)
    assert count == len(_LIMITS_FIELD_PATHS), (
        f"PlannerLimits: C++ has {count} fields, ctypes mirror has "
        f"{len(_LIMITS_FIELD_PATHS)} -- mirror out of date")
    for index, (group, name) in enumerate(_LIMITS_FIELD_PATHS):
        groupType = dict(PlannerLimits._fields_)[group]
        groupOffset = getattr(PlannerLimits, group).offset
        fieldOffset = getattr(groupType, name).offset
        pyOffset = groupOffset + fieldOffset
        assert buffer[index] == pyOffset, (
            f"PlannerLimits.{group}.{name}: C++ offset {buffer[index]} != "
            f"ctypes offset {pyOffset} -- field ORDER differs between "
            f"planner_types.h and this mirror")
    return lib


def benchLimits() -> PlannerLimits:
    limits = PlannerLimits()
    limits.ceilings.vMax = 600.0
    limits.ceilings.aMax = 400.0
    limits.ceilings.aDecel = 300.0
    limits.ceilings.omegaMax = 8.0
    limits.ceilings.alphaMax = 12.0
    limits.ceilings.alphaDecel = 10.0
    limits.ceilings.jerkMax = 0.0
    limits.ceilings.yawJerkMax = 0.0
    limits.plant.trackWidth = 100.0
    limits.plant.controlPeriod = 50.0
    limits.plant.actuationDelay = 0.0
    limits.plant.velocityFilterWeight = 1.0
    # settleEpsilonLinear/Angular: ctypes zero-initializes every field
    # regardless of planner_types.h's own default member initializers (a
    # raw ctypes.Structure is never C++-constructed, so those defaults
    # never apply) -- explicit here, matching planner_types.h's own
    # PlannerLimits::Landing defaults (1.0mm/0.005rad), so settleReached()
    # has a real (not zero) tolerance to test against.
    limits.landing.settleEpsilonLinear = 1.0
    limits.landing.settleEpsilonAngular = 0.005
    limits.landing.settleRestVelocity = 5.0
    limits.landing.settleRestOmega = 0.02
    limits.tracking.headingHoldGain = 0.0
    return limits


class PerfectPlant:
    """Zero-error plant: position advances exactly cmdVelocity * dt."""

    def __init__(self) -> None:
        self.positionLeft = 0.0   # [mm]
        self.positionRight = 0.0  # [mm]

    def disturbHeading(self, heading: float, trackWidth: float) -> None:  # [rad] [mm]
        """Jump the heading without moving the body along its path."""
        self.positionLeft -= 0.5 * heading * trackWidth
        self.positionRight += 0.5 * heading * trackWidth

    def step(self, state: RobotState, dt: float, sampleTime: int) -> None:  # [s] [ms]
        self.positionLeft += state.wheelLeft.cmdVelocity * dt
        self.positionRight += state.wheelRight.cmdVelocity * dt
        for wheel, position, velocity in (
                (state.wheelLeft, self.positionLeft,
                 state.wheelLeft.cmdVelocity),
                (state.wheelRight, self.positionRight,
                 state.wheelRight.cmdVelocity)):
            wheel.position = position
            wheel.velocity = velocity
            wheel.sampleTime = sampleTime
            wheel.connected = True


def runDistanceScenario(lib: ctypes.CDLL) -> None:
    limits = benchLimits()
    planner = lib.plannerCreate(ctypes.byref(limits))
    state = RobotState()
    plant = PerfectPlant()
    result = TickResult()
    period = 50  # [ms]

    move = Move(id=1, kind=KIND_DISTANCE, threshold=500.0, timeout=60000.0,
                velocityKind=VELOCITY_TWIST, v_x=150.0)
    assert lib.plannerMove(planner, ctypes.byref(move), False)

    now = 0
    staircase = []  # [mm/s] one commanded velocity per tick
    completedAt = None
    for tick in range(400):
        state.time.cycleStart = now
        lib.plannerTick(planner, ctypes.byref(state), ctypes.byref(result))
        lib.plannerUpdate(planner, ctypes.byref(state))
        staircase.append(state.wheelLeft.cmdVelocity)
        now += period
        plant.step(state, period / 1000.0, now)
        if result.completed:
            completedAt = tick
            assert result.moveId == 1 and not result.timedOut
        if completedAt is not None and tick > completedAt + 5:
            break

    assert completedAt is not None, "move never completed"
    error = abs(plant.positionLeft - 500.0)
    assert error <= 1e-3, f"landing error {error:.6f} mm"
    assert abs(state.wheelLeft.cmdVelocity) < 1e-6, "did not stop"

    ramped = [f"{v:5.1f}" for v in staircase[:8]]
    landing = [f"{v:5.1f}" for v in staircase[completedAt - 4:completedAt + 2]]
    print(f"distance 500 mm @ 150 mm/s: {completedAt + 1} ticks, "
          f"landing error {error * 1000:.3f} um")
    print(f"  ramp-up  [mm/s]: {' '.join(ramped)} ...")
    print(f"  landing  [mm/s]: ... {' '.join(landing)}")
    lib.plannerDestroy(planner)


def runTurnScenario(lib: ctypes.CDLL) -> None:
    limits = benchLimits()
    planner = lib.plannerCreate(ctypes.byref(limits))
    state = RobotState()
    plant = PerfectPlant()
    result = TickResult()
    period = 50  # [ms]
    quarterTurn = math.pi / 2

    move = Move(id=2, kind=KIND_ANGLE, threshold=quarterTurn, timeout=60000.0,
                velocityKind=VELOCITY_TWIST, omega=2.0)
    assert lib.plannerMove(planner, ctypes.byref(move), False)

    now = 0
    done = False
    for _ in range(400):
        state.time.cycleStart = now
        lib.plannerTick(planner, ctypes.byref(state), ctypes.byref(result))
        lib.plannerUpdate(planner, ctypes.byref(state))
        now += period
        plant.step(state, period / 1000.0, now)
        if result.completed:
            done = True
        elif done:
            break
    assert done, "turn never completed"

    heading = (plant.positionRight - plant.positionLeft) / limits.plant.trackWidth
    error = abs(heading - quarterTurn)
    assert error <= 1e-5, f"turn error {error:.8f} rad"
    print(f"turn 90 deg @ 2 rad/s: landing error "
          f"{math.degrees(error) * 3600:.3f} arcsec")
    lib.plannerDestroy(planner)


def runSettleScenario(lib: ctypes.CDLL) -> None:
    """Arrival-confirm on a zero-error plant: TickResult::settled is always
    settleReached(), evaluated truthfully at whichever tick the Move
    actually completes on -- 130-008 deleted the settle-confirm DEFER path
    (PlannerLimits::requireSettle, itself deleted outright by 130-009) that
    used to hold a profile-complete back until settled; there is no longer
    any mechanism that waits for it.

    For a Move landing via a continuous decel ramp (this scenario), the
    `profile-complete` event fires the instant the commanded velocity's
    ZOH lookahead prediction reaches the target -- one tick BEFORE the
    encoder sample confirming that landing speed has actually been
    reached exists (the one-cycle actuation/measurement lag). So
    `settled` is honestly False here: this is the exact "Without
    requireSettle, completion fires at profile-complete, one sample
    BEFORE that reading can exist" case the pre-130-009 version of this
    docstring described -- now the ONLY case, since the field that used
    to select the other one is gone."""
    limits = benchLimits()
    planner = lib.plannerCreate(ctypes.byref(limits))
    state, plant, result = RobotState(), PerfectPlant(), TickResult()
    move = Move(id=3, kind=KIND_DISTANCE, threshold=500.0, timeout=60000.0,
                velocityKind=VELOCITY_TWIST, v_x=150.0)
    assert lib.plannerMove(planner, ctypes.byref(move), False)

    now, completedAt = 0, None
    for tick in range(400):
        state.time.cycleStart = now
        lib.plannerTick(planner, ctypes.byref(state), ctypes.byref(result))
        lib.plannerUpdate(planner, ctypes.byref(state))
        now += 50
        plant.step(state, 0.050, now)
        if result.completed:
            completedAt = tick
            assert not result.settled, (
                "expected settled=False: the confirming zero-velocity sample "
                "lags one tick behind profile-complete on this ramp-landing move")
            assert not result.timedOut
            break
    assert completedAt is not None, "move never completed"
    lib.plannerDestroy(planner)
    print(f"settle-confirm: profile-complete fires, settled honestly False, on "
          f"tick {completedAt + 1}")


def runHeadingHoldScenario(lib: ctypes.CDLL) -> None:
    """Heading hold (M3) drives a disturbance out on the angular axis while
    leaving the distance accounting exactly where it was -- the correction
    is differential, so the mean of the wheel pair is untouched."""
    limits = benchLimits()
    limits.tracking.headingHoldGain = 4.0  # [1/s]
    planner = lib.plannerCreate(ctypes.byref(limits))
    state, plant, result = RobotState(), PerfectPlant(), TickResult()
    move = Move(id=4, kind=KIND_DISTANCE, threshold=500.0, timeout=60000.0,
                velocityKind=VELOCITY_TWIST, v_x=150.0)
    assert lib.plannerMove(planner, ctypes.byref(move), False)

    now, done = 0, False
    for tick in range(400):
        state.time.cycleStart = now
        lib.plannerTick(planner, ctypes.byref(state), ctypes.byref(result))
        lib.plannerUpdate(planner, ctypes.byref(state))
        now += 50
        plant.step(state, 0.050, now)
        if tick == 10:
            plant.disturbHeading(0.20, limits.plant.trackWidth)  # [rad]
        if result.completed:
            done = True
        elif done:
            break
    assert done, "move never completed"

    heading = (plant.positionRight - plant.positionLeft) / limits.plant.trackWidth
    path = 0.5 * (plant.positionLeft + plant.positionRight)
    assert abs(heading) <= 0.01, f"heading not recovered: {heading:.5f} rad"
    assert abs(path - 500.0) <= 1e-3, f"distance no longer exact: {path:.6f} mm"
    print(f"heading hold: 0.200 rad kick -> residual "
          f"{math.degrees(abs(heading)) * 60:.3f} arcmin, distance still "
          f"exact ({abs(path - 500.0) * 1000:.3f} um error)")
    lib.plannerDestroy(planner)


def main() -> None:
    lib = loadLibrary()
    runDistanceScenario(lib)
    runTurnScenario(lib)
    runSettleScenario(lib)
    runHeadingHoldScenario(lib)
    print("planner_harness: all checks passed")


if __name__ == "__main__":
    main()
