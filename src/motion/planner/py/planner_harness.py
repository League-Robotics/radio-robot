"""planner_harness.py -- Python ctypes harness over libmotionplanner
(motion-planner sketch §7 tier 3). Mirrors RobotState/Move/PlannerLimits/
TickResult field-for-field, runs the same zero-error perfect plant the C++
scenario tests use, and asserts the exactness gates from Python. Run:

    python3 src/motion/planner/py/planner_harness.py

Build the library first:

    cmake -S src/motion/planner -B src/motion/planner/build
    cmake --build src/motion/planner/build --target motionplanner

stdlib only -- no numpy, no uv needed.
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
                ("cmdVelocity", ctypes.c_float)]   # [mm/s]


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
                ("wedgeLatch", ctypes.c_bool),
                ("moveTimeout", ctypes.c_bool),
                ("shapingDisabled", ctypes.c_bool),
                ("positionClamped", ctypes.c_bool)]


class RobotState(ctypes.Structure):
    _fields_ = [("time", Time),
                ("wheelLeft", Wheel), ("wheelRight", Wheel),
                ("otos", Otos), ("perception", Perception),
                ("pose", Pose), ("estimate", Estimate),
                ("command", Command), ("health", Health)]


KIND_TIME, KIND_DISTANCE, KIND_ANGLE = 0, 1, 2
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
                ("vRight", ctypes.c_float)]      # [mm/s]


class PlannerLimits(ctypes.Structure):
    _fields_ = [("vMax", ctypes.c_float),            # [mm/s]
                ("aMax", ctypes.c_float),            # [mm/s^2]
                ("aDecel", ctypes.c_float),          # [mm/s^2]
                ("omegaMax", ctypes.c_float),        # [rad/s]
                ("alphaMax", ctypes.c_float),        # [rad/s^2]
                ("alphaDecel", ctypes.c_float),      # [rad/s^2]
                ("trackWidth", ctypes.c_float),      # [mm]
                ("controlPeriod", ctypes.c_float),   # [ms]
                ("actuationDelay", ctypes.c_float),  # [ms]
                ("velocityFilterWeight", ctypes.c_float),
                ("otosStaleness", ctypes.c_uint32),  # [ms]
                ("headingOtosWeight", ctypes.c_float),
                ("requireSettle", ctypes.c_bool),
                ("settleWindow", ctypes.c_float),    # [ms]
                ("headingHoldGain", ctypes.c_float),  # [1/s]
                ("velKff", ctypes.c_float),   # [duty/(mm/s)] M4 duty stage
                ("velKp", ctypes.c_float),    # [duty/(mm/s)]
                ("velKi", ctypes.c_float),    # [duty/(mm/s)/s]
                ("velIMax", ctypes.c_float),  # [duty]
                ("velKaff", ctypes.c_float),  # [duty/(mm/s^2)] accel feedforward
                ("velIAccelGate", ctypes.c_float),  # [mm/s^2] integral ramp gate
                ("jerkMax", ctypes.c_float),     # [mm/s^3] S-curve
                ("yawJerkMax", ctypes.c_float),  # [rad/s^3]
                ("settleRestVelocity", ctypes.c_float),  # [mm/s]
                ("settleRestOmega", ctypes.c_float)]     # [rad/s]


class TickResult(ctypes.Structure):
    _fields_ = [("completed", ctypes.c_bool),
                ("moveId", ctypes.c_uint32),
                ("timedOut", ctypes.c_bool),
                ("settled", ctypes.c_bool)]


# ---- library ----

def loadLibrary() -> ctypes.CDLL:
    build = Path(__file__).resolve().parent.parent / "build"
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
    lib.plannerStop.argtypes = [ctypes.c_void_p]
    lib.plannerTick.argtypes = [ctypes.c_void_p, ctypes.POINTER(RobotState),
                                ctypes.POINTER(TickResult)]
    lib.plannerUpdate.argtypes = [ctypes.c_void_p, ctypes.POINTER(RobotState)]
    lib.plannerStructSizes.argtypes = [ctypes.POINTER(ctypes.c_uint32)] * 4
    lib.plannerDuty.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float)]

    # Layout guard: C++ sizeof must match the ctypes mirrors exactly.
    sizes = [ctypes.c_uint32() for _ in range(4)]
    lib.plannerStructSizes(*[ctypes.byref(s) for s in sizes])
    expected = [RobotState, Move, PlannerLimits, TickResult]
    for cSize, pyType in zip(sizes, expected):
        assert cSize.value == ctypes.sizeof(pyType), (
            f"{pyType.__name__}: C++ sizeof {cSize.value} != "
            f"ctypes {ctypes.sizeof(pyType)} -- mirror out of date")
    return lib


def benchLimits() -> PlannerLimits:
    limits = PlannerLimits()
    limits.vMax = 600.0
    limits.aMax = 400.0
    limits.aDecel = 300.0
    limits.omegaMax = 8.0
    limits.alphaMax = 12.0
    limits.alphaDecel = 10.0
    limits.trackWidth = 100.0
    limits.controlPeriod = 50.0
    limits.actuationDelay = 0.0
    limits.velocityFilterWeight = 1.0
    limits.otosStaleness = 200
    limits.headingOtosWeight = 0.0
    limits.requireSettle = False
    limits.settleWindow = 0.0
    limits.headingHoldGain = 0.0
    limits.velKff = 0.0
    limits.velKp = 0.0
    limits.velKi = 0.0
    limits.velIMax = 0.0
    limits.velKaff = 0.0
    limits.velIAccelGate = 1.0e9
    limits.jerkMax = 0.0
    limits.yawJerkMax = 0.0
    limits.settleRestVelocity = 5.0
    limits.settleRestOmega = 0.02
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

    heading = (plant.positionRight - plant.positionLeft) / limits.trackWidth
    error = abs(heading - quarterTurn)
    assert error <= 1e-5, f"turn error {error:.8f} rad"
    print(f"turn 90 deg @ 2 rad/s: landing error "
          f"{math.degrees(error) * 3600:.3f} arcsec")
    lib.plannerDestroy(planner)


def runSettleScenario(lib: ctypes.CDLL) -> None:
    """Settle-confirm (M1) on a zero-error plant: with requireSettle the
    completion defers at most ONE tick past profile-complete (the sample
    proving v == 0 arrives one cycle after the landing command -- the
    discrete-sensing bound; the settle gate is measured-velocity-only) and
    reports settled=True. Without requireSettle, completion fires at
    profile-complete, one sample BEFORE that reading can exist, so settled
    is honestly False there."""
    ticks = {}
    for requireSettle in (False, True):
        limits = benchLimits()
        limits.requireSettle = requireSettle
        limits.settleWindow = 1000.0  # [ms]
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
                if requireSettle:
                    assert result.settled, \
                        "deferred completion must confirm arrival"
                assert not result.timedOut
                break
        assert completedAt is not None, "move never completed"
        ticks[requireSettle] = completedAt
        lib.plannerDestroy(planner)

    assert ticks[True] - ticks[False] <= 1, (
        f"settle-confirm cost {ticks[True] - ticks[False]} ticks on a "
        f"zero-error plant; it must cost none")
    print(f"settle-confirm: profile-complete and settle-complete both on "
          f"tick {ticks[True] + 1}")


def runHeadingHoldScenario(lib: ctypes.CDLL) -> None:
    """Heading hold (M3) drives a disturbance out on the angular axis while
    leaving the distance accounting exactly where it was -- the correction
    is differential, so the mean of the wheel pair is untouched."""
    limits = benchLimits()
    limits.headingHoldGain = 4.0  # [1/s]
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
            plant.disturbHeading(0.20, limits.trackWidth)  # [rad]
        if result.completed:
            done = True
        elif done:
            break
    assert done, "move never completed"

    heading = (plant.positionRight - plant.positionLeft) / limits.trackWidth
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
