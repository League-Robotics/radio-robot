"""WARNING: this module's yaw/omega convention is CW-positive, the OPPOSITE
of the project-wide CCW-positive convention used everywhere else (see
`.claude/rules/coding-standards.md`/`.claude/rules/naming-and-style.md`,
and e.g. `nezha_kinematic.py`'s own CCW-positive world-frame properties).
It negates omega internally at both `inverse()`/`forward()` boundaries to
convert to/from WPILib's own CCW-positive convention -- do not assume this
class's public sign matches any other module's without re-checking.

No live caller today: this module's only importer, `robot/nezha_kinematic.py`,
is itself unreferenced by anything except `robot/__init__.py`'s lazy
re-export and README.md (see `../DESIGN.md`'s `kinematics/` row) -- kept in
place per that row's own recorded orphan status, not because anything calls
it in the current wire-cutover-era code path.
"""

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
