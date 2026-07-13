"""plant.py -- a pure-Python plant model of the LEVEL-2 velocity servo
Drive:: commands wheel setpoints to (ticket 100-006's own AC: "models the
LEVEL-2 velocity servo the drive subsystem outputs setpoints to").

Location choice (documented per the ticket's own "programmer's judgment"
allowance): this lives under tests/_infra/drive/ alongside drive.py/
drive_api.cpp rather than a new tests/sim/drive/ package, because it is pure
Python test INFRASTRUCTURE (no pytest collection happens here -- tests/
_infra/ is outside pyproject.toml's testpaths, mirroring tests/_infra/sim/
firmware.py's own placement) consumed BY the tier-0 tests under
tests/sim/drive/, not a test module itself.

Five independently-configurable fault knobs, conceptually aligned with
tests/_infra/sim/sim_api.cpp's own fault-knob philosophy (motor_lag/
enc_slip/stiction/trackwidth/scrub on the ``Sim`` class) so tier-0 and
tier-1 plant behavior stay comparable:

  motor_lag       -- [ms] first-order lag time constant per wheel (the
                      issue's 120-140ms range is the realistic default).
  stiction        -- [mm/s] commanded-speed deadband: a wheel commanded
                      below this magnitude produces NO motion (mirrors
                      Hal::setSimStiction's own Coulomb-style threshold).
  enc_staleness   -- [ms] sample-and-hold delay applied to the WHOLE
                      "measured" snapshot (pose, twist, wheel state) --
                      encoder-chain-derived quantities (dead-reckoning pose
                      included) share one staleness, physically: they are
                      all downstream of the same encoder tick stream.
  quantization    -- [mm/s] measured wheel-speed/position quantization step
                      (0 = continuous/disabled).
  slip            -- [0,1] fractional wheel slip: the encoder (mounted on
                      the motor shaft) reports the FULL wheel-shaft speed
                      regardless of slip, but the chassis's TRUE ground
                      motion is reduced by (1 - slip) -- the physically
                      accurate direction (a spinning, non-gripping wheel
                      over-reports travel relative to the ground).

Each knob defaults to its "off"/identity value except motor_lag/
enc_staleness, whose issue-realistic defaults (130ms, 80ms) make the
default-constructed Plant() the SAME "realistic" plant the closed-loop
convergence tests use.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from drive import BodyState, Pose, Twist, WheelState


def _wrap_angle(x: float) -> float:  # [rad] wrap to (-pi, pi], same
    # atan2(sin, cos) identity as Drive::wrapAngle (arc_math.h's own class
    # comment traces this identity through four hand-ports in the C++ tree;
    # this is the Python-side fifth).
    return math.atan2(math.sin(x), math.cos(x))


@dataclass
class PlantConfig:
    motor_lag: float = 130.0  # [ms] first-order lag time constant; issue's 120-140ms range
    stiction: float = 0.0  # [mm/s] commanded-speed deadband; 0 = disabled
    enc_staleness: float = 80.0  # [ms] measured-snapshot sample-and-hold delay
    quantization: float = 0.0  # [mm/s] measured wheel-speed/position quantization step; 0 = disabled
    slip: float = 0.0  # [0,1] fractional wheel slip; 0 = full grip
    trackwidth: float = 128.0  # [mm]


class Plant:
    """First-order-lag differential-drive plant. ``step(wheel_left,
    wheel_right, dt)`` advances TRUE state by one tick; ``measured()``
    returns the (possibly stale/quantized/slipped) observation a caller
    would actually see this tick -- the ``BodyState``/left/right
    ``WheelState`` triple ``StepInput`` expects."""

    def __init__(self, config: PlantConfig | None = None, pose: Pose | None = None) -> None:
        self.config = config or PlantConfig()
        self.pose = pose or Pose()  # [mm][mm][rad] TRUE chassis pose
        self._wheel_left = 0.0  # [mm/s] TRUE (post-lag/stiction) wheel-shaft speed
        self._wheel_right = 0.0
        self._enc_left = 0.0  # [mm] TRUE (unslipped) accumulated wheel-shaft travel
        self._enc_right = 0.0
        self._t = 0.0  # [s]
        # (t, pose, twist, wheelLeftVel, wheelRightVel, encLeft, encRight) --
        # a snapshot ring for enc_staleness's sample-and-hold lookup.
        self._history: list[tuple[float, Pose, Twist, float, float, float, float]] = []
        self._record_snapshot()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _record_snapshot(self) -> None:
        v = (self._wheel_left + self._wheel_right) * 0.5
        omega = (self._wheel_right - self._wheel_left) / self.config.trackwidth
        twist = Twist(v_x=v, v_y=0.0, omega=omega)
        self._history.append((
            self._t, Pose(self.pose.x, self.pose.y, self.pose.h), twist,
            self._wheel_left, self._wheel_right, self._enc_left, self._enc_right,
        ))
        # Bound memory: drop entries older than 2x the configured staleness
        # (plus a floor margin), keeping at least one entry.
        horizon = max(0.5, 2.0 * self.config.enc_staleness / 1000.0)
        cutoff = self._t - horizon
        while len(self._history) > 1 and self._history[1][0] < cutoff:
            self._history.pop(0)

    def _quantize(self, value: float) -> float:
        q = self.config.quantization
        if q <= 0.0:
            return value
        return round(value / q) * q

    # ------------------------------------------------------------------
    # Advance
    # ------------------------------------------------------------------

    def step(self, wheel_left_cmd: float, wheel_right_cmd: float, dt: float) -> None:
        """Advance TRUE plant state by ``dt`` seconds given this tick's
        commanded wheel-velocity setpoints. [mm/s] [mm/s] [s]"""
        cfg = self.config

        # Stiction: a commanded magnitude below the threshold produces no
        # motion at all (a deadband on the TARGET the lag filter chases).
        left_target = 0.0 if abs(wheel_left_cmd) < cfg.stiction else wheel_left_cmd
        right_target = 0.0 if abs(wheel_right_cmd) < cfg.stiction else wheel_right_cmd

        if cfg.motor_lag > 0.0:
            tau_s = cfg.motor_lag / 1000.0
            alpha = 1.0 - math.exp(-dt / tau_s)
        else:
            alpha = 1.0
        self._wheel_left += alpha * (left_target - self._wheel_left)
        self._wheel_right += alpha * (right_target - self._wheel_right)

        # TRUE (encoder/wheel-shaft) travel -- unaffected by slip: an
        # encoder is mounted on the motor shaft and reports shaft rotation
        # regardless of whether the wheel is gripping the ground.
        self._enc_left += self._wheel_left * dt
        self._enc_right += self._wheel_right * dt

        # Ground-truth chassis motion -- slip reduces ACTUAL displacement
        # below what the wheel-shaft speed alone would produce.
        grip = 1.0 - cfg.slip
        v = (self._wheel_left + self._wheel_right) * 0.5 * grip
        omega = (self._wheel_right - self._wheel_left) / cfg.trackwidth * grip
        self.pose = Pose(
            x=self.pose.x + v * math.cos(self.pose.h) * dt,
            y=self.pose.y + v * math.sin(self.pose.h) * dt,
            h=_wrap_angle(self.pose.h + omega * dt),
        )
        self._t += dt
        self._record_snapshot()

    # ------------------------------------------------------------------
    # Observation
    # ------------------------------------------------------------------

    def measured(self) -> tuple[BodyState, WheelState, WheelState]:
        """The (BodyState, left WheelState, right WheelState) triple a
        caller would observe THIS tick: a sample-and-hold snapshot from
        ``enc_staleness`` ms ago, with ``quantization`` applied to the
        wheel readings."""
        cfg = self.config
        target_t = self._t - cfg.enc_staleness / 1000.0
        snap = self._history[0]
        for entry in self._history:
            if entry[0] <= target_t:
                snap = entry
            else:
                break
        _, pose, twist, wheel_left, wheel_right, enc_left, enc_right = snap

        left = WheelState(position=self._quantize(enc_left), velocity=self._quantize(wheel_left),
                           position_valid=True, velocity_valid=True)
        right = WheelState(position=self._quantize(enc_right), velocity=self._quantize(wheel_right),
                            position_valid=True, velocity_valid=True)
        body = BodyState(pose=pose, twist=twist)
        return body, left, right
