from wpimath.kinematics import DifferentialDriveKinematics as _WpiDDK
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveWheelSpeeds


class DifferentialDriveKinematics:
    """Wraps wpimath.kinematics.DifferentialDriveKinematics.

    All inputs/outputs use project conventions:
    - distances in mm/s (not m/s)
    - yaw/omega: CW-positive radians (not CCW)
    """

    def __init__(self, trackwidth: float):  # [mm]
        self._tw_m = trackwidth / 1000.0
        self._kinematics = _WpiDDK(self._tw_m)

    def inverse(self, vx: float, omega: float) -> tuple[float, float]:  # [m/s], [rad/s]
        """(vx m/s, omega CW rad/s) -> (v_left mm/s, v_right mm/s)"""
        # WPILib uses CCW-positive omega; negate for our CW convention
        speeds = ChassisSpeeds(vx, 0.0, -omega)
        ws = self._kinematics.toWheelSpeeds(speeds)
        return ws.left * 1000.0, ws.right * 1000.0

    def forward(self, v_left: float, v_right: float) -> tuple[float, float]:  # [mm/s]
        """(v_left mm/s, v_right mm/s) -> (vx m/s, omega CW rad/s)"""
        ws = DifferentialDriveWheelSpeeds(v_left / 1000.0, v_right / 1000.0)
        chassis = self._kinematics.toChassisSpeeds(ws)
        # Negate omega back to CW-positive
        return chassis.vx, -chassis.omega
