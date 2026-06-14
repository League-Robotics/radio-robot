"""Abstract base class for robot control."""

from abc import ABC, abstractmethod
from typing import Any, Callable, Generator


class Robot(ABC):
    """Interface for any controllable robot.

    Speeds are in mm/s, distances in mm, angles in degrees.
    """

    @abstractmethod
    def speed(self, left_mms: int, right_mms: int) -> Generator[tuple[int, int], None, None]:
        """Non-blocking PID speed control. Yields (left_mm, right_mm) encoder
        positions as they stream back. Must be consumed (or closed) to keep
        the robot moving — the firmware stops if commands aren't re-sent."""

    @abstractmethod
    def speed_for_time(self, left_mms: int, right_mms: int, ms: int) -> tuple[int, int]:
        """Blocking: drive at speed for a duration. Returns (left_mm, right_mm)."""

    @abstractmethod
    def speed_for_distance(self, left_mms: int, right_mms: int, mm: int) -> tuple[int, int]:
        """Blocking: drive at speed until distance. Returns (left_mm, right_mm)."""

    @abstractmethod
    def stop(self) -> None:
        """Stop all motors immediately."""

    @abstractmethod
    def grip(self, angle: int) -> None:
        """Set gripper servo angle (0=open, 180=closed)."""

    @abstractmethod
    def read_encoders(self) -> tuple[int, int]:
        """Read encoder positions in mm. Returns (left_mm, right_mm)."""

    @abstractmethod
    def zero_encoders(self) -> None:
        """Zero both encoder counters."""

    @abstractmethod
    def send(self, message: str, read_ms: int = 500) -> dict[str, Any]:
        """Send arbitrary command string, return responses."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if the robot connection is active."""

    def go_to(
        self,
        x_mm: int,
        y_mm: int,
        speed_mms: int,
        on_tick: Callable | None = None,
        timeout_s: float = 15.0,
    ) -> tuple[int, int, str]:
        """Blocking go-to (G command). Returns (left_enc_mm, right_enc_mm, outcome).

        ``on_tick`` is called after each update cycle if provided; when
        ``None`` (default) the call blocks until the firmware signals completion.
        ``timeout_s`` caps the total wait time.

        Subclasses that already implement go_to with a different signature must
        override this method directly.
        """
        raise NotImplementedError

    def turn(
        self,
        heading_cdeg: int,
        on_tick: Callable | None = None,
        timeout_s: float = 15.0,
    ) -> str:
        """Rotate to an absolute heading (TURN command). Returns outcome string.

        ``on_tick`` is called after each update cycle if provided.
        ``timeout_s`` caps the total wait time.

        Subclasses that implement turn must override this method.
        """
        raise NotImplementedError
