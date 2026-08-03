"""src/tests/bench/rig_drive.py -- HITL turn test: run the REAL src/firm/drive turn
logic (libdrive_host) against the bench rig.

The rig's two motors are the drivetrain's left/right wheels; the encoder
differential (pos_right - pos_left)/trackwidth IS the robot heading (there is no
physical chassis -- the "robot" is a virtual differential drive defined by the
two encoders). The OTOS rides a 360deg servo; for the OTOS-in-the-loop run we
synthesize the "robot rotated" feedback by driving the servo to the
encoder-derived heading (kept slightly ahead) and feeding the OTOS heading back
as the tracker's measured pose.

Because each motor closes its own velocity PID, a +VEL command always makes that
motor's reported position INCREASE -- so sending the drive's right/left wheel
velocities to a fixed (RIGHT_PORT, LEFT_PORT) pair and reading heading back from
the SAME pair is self-consistent negative feedback (loop is stable for either
physical assignment).

Turns terminate on the tracker's measured heading, which degrades to
encoder-only when the OTOS is out of the loop -- so both runs (OTOS out / OTOS
in) always terminate. Run:  uv run python src/tests/bench/rig_drive.py [deg]
"""
from __future__ import annotations

import math
import pathlib
import sys
import time

_REPO = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO / "src" / "tests" / "_infra" / "drive"))
sys.path.insert(0, str(_REPO / "src" / "tests" / "bench"))

from drive import (  # noqa: E402
    BodyState, Drive, Goal, Limits, PlanRequest, Pose, ProfileLimits,
    StepInput, StepState, Status, Twist, Verdict, WheelState,
)
from rig_dev import Rig, SERVO_PIN  # noqa: E402

TRACKWIDTH = 128.0        # [mm] tovez
LEFT_PORT, RIGHT_PORT = 1, 2
# 360deg servo: SERVO command 0..180 maps LINEARLY to physical 0..360deg
# (command 90 = 180deg physical). To place the OTOS at robot heading `theta`,
# command degrees(theta)/2. The turn starts with the servo at command 0 (OTOS
# zeroed there), so the full 0..180 command range spans a complete 0..360deg
# turn -- no range edge until 360deg. (The OTOS heading READING still wraps at
# +/-180deg; the fusion wraps that difference.)
SERVO_START = 0
SERVO_PER_DEG = 0.5   # [servo-command/deg] command = physical_deg / 2


def tovez_limits() -> Limits:
    # Representative tovez drive limits (drive.py docstring canonical set).
    return Limits(
        linear=ProfileLimits(velocity=400.0, accel=800.0, decel=800.0),
        rotational=ProfileLimits(velocity=3.0, accel=15.0, decel=15.0),
        v_wheel_max=600.0, trim_v_max=120.0, trim_omega_max=1.0,
        track_k_s=2.0, track_k_theta=6.0, track_k_cross=1.5e-5, min_speed=20.0)


def gentle_limits() -> Limits:
    # Tuned for the rig's very high-inertia drum/wheel cluster: a slow, gently
    # accelerated pivot and a soft heading gain, so the laggy motors can follow
    # without overshooting into oscillation (the default aggressive profile
    # oscillates -- the wheels keep coasting past the target).
    return Limits(
        linear=ProfileLimits(velocity=200.0, accel=400.0, decel=400.0),
        rotational=ProfileLimits(velocity=0.8, accel=2.0, decel=2.0),
        v_wheel_max=350.0, trim_v_max=60.0, trim_omega_max=0.5,
        track_k_s=1.5, track_k_theta=2.5, track_k_cross=1.5e-5, min_speed=15.0)


def slow_move_limits() -> Limits:
    # Slow linear profile so the line index (256 mm/rev) is sampled reliably by
    # the 240 ms sensor telemetry -- for the secondary-encoder distance check.
    return Limits(
        linear=ProfileLimits(velocity=55.0, accel=120.0, decel=120.0),
        rotational=ProfileLimits(velocity=0.8, accel=2.0, decel=2.0),
        v_wheel_max=120.0, trim_v_max=40.0, trim_omega_max=0.5,
        track_k_s=1.5, track_k_theta=2.5, track_k_cross=1.5e-5, min_speed=15.0)


def _wrap(a: float) -> float:  # [rad] -> (-pi, pi]
    return math.atan2(math.sin(a), math.cos(a))


def servo_for_heading(theta: float) -> int:  # [rad] robot heading -> SERVO command 0..180
    cmd = int(round(math.degrees(theta) * SERVO_PER_DEG))
    return max(0, min(180, cmd))


_TERMINAL = (Status.DONE_STOP, Status.DONE_HANDOFF,
             Status.ABORT_TIMEOUT, Status.ABORT_REPLAN_LIMIT)


DRUM_MM_PER_REV = 256.1   # [mm] motor-1 encoder travel per drum revolution (line-index calibration)


def run_turn(deg: float, otos_in_loop: bool, dt: float = 0.05,
             lead_rad: float = 0.05, timeout: float = 12.0, limits: Limits | None = None,
             fuse_k: float = 0.3, arc_length: float = 0.0):
    """Execute one drive Goal on the rig via the real drive tracker: an in-place
    turn of `deg` degrees and/or a straight `arc_length` mm move. Returns
    (rows, final_status). Each row: t, pos_l/r, vel_l/r, cmd_l/r, heading_enc,
    heading_meas, heading_otos [deg], n0 (line index), status."""
    rig = Rig(settle=3.0)
    rig.stream(80)
    for p in (LEFT_PORT, RIGHT_PORT):
        rig.cmd(f"M {p} RESET"); rig.cmd(f"M {p} PID 1")
    rig.servo(SERVO_START); time.sleep(1.0)
    rig.cmd("ODO SETPOSE 0 0 0"); time.sleep(0.3)
    rig.flush()

    drive = Drive(limits or gentle_limits(), TRACKWIDTH)
    res = drive.plan(PlanRequest(goal=Goal(arc_length=arc_length, delta_heading=math.radians(deg)),
                                 start=Pose()))
    if res.verdict != Verdict.OK:
        rig.close()
        raise RuntimeError(f"plan verdict = {res.verdict.name}")
    plan = res.plan
    state = StepState()

    last_tlm: dict = {}
    last_stlm: dict = {}

    def pump():
        nonlocal last_tlm, last_stlm
        tlm, stlm = rig.read_frames()  # ONE drain -> both (read_tlm/read_stlm would eat each other)
        for d in tlm:
            last_tlm = d
        for d in stlm:
            last_stlm = d

    rows = []
    t_plan = 0.0
    t_wall = 0.0
    status = Status.RUNNING
    wall0 = time.monotonic()
    x_od = y_od = 0.0          # [mm] dead-reckoned position (tracker needs it for moves)
    prev_pl = prev_pr = None
    while t_wall < timeout:
        pump()
        pos_l = last_tlm.get(f"{LEFT_PORT}p"); pos_r = last_tlm.get(f"{RIGHT_PORT}p")
        vel_l = last_tlm.get(f"{LEFT_PORT}v"); vel_r = last_tlm.get(f"{RIGHT_PORT}v")
        if pos_l is None or pos_r is None:
            time.sleep(0.01)
            continue  # wait for first telemetry frame
        heading_enc = (pos_r - pos_l) / TRACKWIDTH
        omega_enc = ((vel_r or 0.0) - (vel_l or 0.0)) / TRACKWIDTH
        # dead-reckon (x, y) from wheel-travel deltas along the current heading
        if prev_pl is not None:
            ds = ((pos_l - prev_pl) + (pos_r - prev_pr)) / 2.0
            x_od += ds * math.cos(heading_enc)
            y_od += ds * math.sin(heading_enc)
        prev_pl, prev_pr = pos_l, pos_r

        if otos_in_loop:
            # Synthesize the "robot rotated" feedback: drive the servo (which
            # carries the OTOS) to the encoder-derived heading, kept slightly
            # ahead so its slew leads rather than lags.
            rig.send(f"SERVO {SERVO_PIN} {servo_for_heading(heading_enc + lead_rad)}")
            oh = last_stlm.get("oh")
            # Complementary fusion, mirroring the firmware EKF: encoder heading
            # is the fast predictor, OTOS a slow absolute correction. Sole-OTOS
            # feedback lags too much (servo + 240ms STLM) and oscillates; a
            # modest blend stays stable AND exercises the OTOS in the loop.
            heading_meas = (heading_enc + fuse_k * _wrap(oh - heading_enc)
                            if oh is not None else heading_enc)
        else:
            heading_meas = heading_enc

        si = StepInput(t=t_plan,
                       measured=BodyState(Pose(x_od, y_od, heading_meas),
                                          Twist(0.0, 0.0, omega_enc)),
                       left=WheelState(pos_l, vel_l or 0.0, True, True),
                       right=WheelState(pos_r, vel_r or 0.0, True, True))
        out, state = plan.step(si, state)
        cmd = out.command
        rig.send(f"M {LEFT_PORT} VEL {cmd.left:.1f}")
        rig.send(f"M {RIGHT_PORT} VEL {cmd.right:.1f}")

        oh = last_stlm.get("oh")
        rows.append({
            "t": t_wall, "pos_l": pos_l, "pos_r": pos_r,
            "vel_l": vel_l, "vel_r": vel_r, "cmd_l": cmd.left, "cmd_r": cmd.right,
            "heading_enc": math.degrees(heading_enc),
            "heading_meas": math.degrees(heading_meas),
            "heading_otos": (math.degrees(oh) if oh is not None else None),
            "x_od": x_od, "y_od": y_od,  # [mm] dead-reckoned position
            "n0": last_stlm.get("n0"),   # line-sensor index channel (secondary encoder)
            "ref_omega": out.record.ref.omega,   # [rad/s] planned yaw rate (accel-limit check)
            "ref_v": out.record.ref.v,           # [mm/s] planned body speed
            "status": int(out.status),
        })
        status = out.status
        if status in _TERMINAL:
            break
        if status == Status.REPLAN_DUE:
            rp = drive.replan(plan, si.measured, t_plan)
            if rp.verdict == Verdict.OK and rp.plan is not None:
                try:
                    plan.close()
                except Exception:
                    pass
                plan = rp.plan
                state = StepState()
                t_plan = 0.0  # new plan re-anchored at current pose

        # pace to dt (pumping telemetry while we wait)
        target = wall0 + t_wall + dt
        while time.monotonic() < target:
            pump()
            time.sleep(0.002)
        t_plan += dt
        t_wall += dt

    for p in (LEFT_PORT, RIGHT_PORT):
        rig.cmd(f"M {p} NEUTRAL")
    rig.servo(SERVO_START)
    rig.cmd("STREAM 0")
    rig.close()
    return rows, status


def _summary(deg, rows, status):
    if not rows:
        print(f"  {deg:.0f} deg: NO DATA")
        return
    final = rows[-1]["heading_enc"]
    err = final - deg
    peak_cmd = max(abs(r["cmd_l"]) for r in rows) if rows else 0
    print(f"  {deg:6.0f} deg turn: status={Status(status).name:12s} "
          f"final_enc_heading={final:+7.2f} deg  err={err:+6.2f} deg  "
          f"n={len(rows)}  peak|wheel_cmd|={peak_cmd:.0f} mm/s")


if __name__ == "__main__":
    d = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    print(f"=== HITL turn {d} deg: OTOS OUT of loop ===")
    rows_out, st_out = run_turn(d, otos_in_loop=False)
    _summary(d, rows_out, st_out)
