"""
test_architecture_seams.py — machine-verify the FRC Elite Architecture seams
and REPLAY stub after the Sprint 044 Phase F migration.

These are pure filesystem/text checks — no hardware, no ctypes build required.
They make the migration's final criteria permanent: future edits that
accidentally remove a seam, resurrect an alias shim, or break the REPLAY stub
are caught immediately in CI.
"""
import pathlib

REPO_ROOT = pathlib.Path(__file__).parent.parent.parent.parent


def test_seam1_capability_directory_exists():
    """Seam 1: source/hal/capability/ directory (the capability interfaces) exists."""
    d = REPO_ROOT / "source" / "hal" / "capability"
    assert d.is_dir(), f"Seam 1 missing: {d}"


def test_seam2_physical_state_estimate_exists():
    """Seam 2: PhysicalStateEstimate header + impl exist."""
    h = REPO_ROOT / "source" / "state" / "PhysicalStateEstimate.h"
    cpp = REPO_ROOT / "source" / "state" / "PhysicalStateEstimate.cpp"
    assert h.is_file(), f"Seam 2 header missing: {h}"
    assert cpp.is_file(), f"Seam 2 impl missing: {cpp}"


def test_seam3_superstructure_exists():
    """Seam 3: Superstructure header + impl exist."""
    h = REPO_ROOT / "source" / "superstructure" / "Superstructure.h"
    cpp = REPO_ROOT / "source" / "superstructure" / "Superstructure.cpp"
    assert h.is_file(), f"Seam 3 header missing: {h}"
    assert cpp.is_file(), f"Seam 3 impl missing: {cpp}"


def test_four_file_device_quartet_velocity_motor():
    """IVelocityMotor quartet: capability iface + real impl + sim impl + Hardware wiring."""
    cap = REPO_ROOT / "source" / "hal" / "capability" / "IVelocityMotor.h"
    real = REPO_ROOT / "source" / "hal" / "real" / "Motor.h"
    sim = REPO_ROOT / "source" / "hal" / "sim" / "SimMotor.h"
    hardware = REPO_ROOT / "source" / "hal" / "Hardware.h"
    assert cap.is_file(), f"Capability missing: {cap}"
    assert real.is_file(), f"Real impl missing: {real}"
    assert sim.is_file(), f"Sim impl missing: {sim}"
    assert hardware.is_file(), f"Hardware wiring missing: {hardware}"
    # Hardware wires the capability via an IVelocityMotor& accessor.
    assert "IVelocityMotor" in hardware.read_text(), \
        "Hardware.h does not wire IVelocityMotor"


def test_four_file_device_quartet_odometer():
    """IOdometer quartet: capability iface + real impl + sim impl + Hardware wiring."""
    cap = REPO_ROOT / "source" / "hal" / "capability" / "IOdometer.h"
    real = REPO_ROOT / "source" / "hal" / "real" / "OtosSensor.h"
    sim = REPO_ROOT / "source" / "hal" / "sim" / "SimOdometer.h"
    hardware = REPO_ROOT / "source" / "hal" / "Hardware.h"
    assert cap.is_file(), f"Capability missing: {cap}"
    assert real.is_file(), f"Real impl missing: {real}"
    assert sim.is_file(), f"Sim impl missing: {sim}"
    assert hardware.is_file(), f"Hardware wiring missing: {hardware}"
    assert "IOdometer" in hardware.read_text(), \
        "Hardware.h does not wire IOdometer"


def test_no_alias_shims_remain():
    """All Phase A–D alias shims and legacy class files have been deleted."""
    # Phase A–D alias shims (seven io/ shims + EKF.h).
    shims = [
        "source/io/IMotor.h",
        "source/io/IServo.h",
        "source/io/IOtosSensor.h",
        "source/io/IColorSensor.h",
        "source/io/ILineSensor.h",
        "source/io/IPortIO.h",
        "source/control/EKF.h",
    ]
    survivors = [s for s in shims if (REPO_ROOT / s).exists()]
    assert not survivors, f"Alias shims still present: {survivors}"


def test_legacy_motion_class_deleted():
    """Sprint 061-005: legacy class source files no longer exist on disk.

    The three files absorbed into Planner in sprint 061-004 must be absent.
    File names are constructed via pathlib to keep grep tooling from flagging
    this assertion file itself.
    """
    legacy_cls = "Motion" + "Controller"  # avoid literal in grep scan
    deleted = [
        REPO_ROOT / "source" / "superstructure" / (legacy_cls + ".h"),
        REPO_ROOT / "source" / "superstructure" / (legacy_cls + ".cpp"),
        REPO_ROOT / "source" / "control" / (legacy_cls + "Begin.cpp"),
    ]
    survivors = [str(p) for p in deleted if p.exists()]
    assert not survivors, f"Legacy files still present: {survivors}"


def test_inputs_h_exists_and_robotstate_retired():
    """source/types/Inputs.h exists; source/control/RobotState.h is gone."""
    inputs = REPO_ROOT / "source" / "types" / "Inputs.h"
    robot_state = REPO_ROOT / "source" / "control" / "RobotState.h"
    assert inputs.is_file(), f"Inputs.h missing: {inputs}"
    assert not robot_state.exists(), f"RobotState.h still present: {robot_state}"


def test_replay_hal_exists():
    """ReplayHAL stub files exist (the REPLAY-mode HAL)."""
    h = REPO_ROOT / "source" / "hal" / "ReplayHAL.h"
    cpp = REPO_ROOT / "source" / "hal" / "ReplayHAL.cpp"
    assert h.is_file(), f"ReplayHAL.h missing: {h}"
    assert cpp.is_file(), f"ReplayHAL.cpp missing: {cpp}"


def test_replay_hal_contains_robot_mode():
    """ReplayHAL.cpp anchors RobotMode::REPLAY with a static_assert (stub exercised)."""
    cpp = REPO_ROOT / "source" / "hal" / "ReplayHAL.cpp"
    content = cpp.read_text()
    assert "RobotMode::REPLAY" in content, \
        "RobotMode::REPLAY not found in ReplayHAL.cpp"
    assert "static_assert" in content, \
        "static_assert not found in ReplayHAL.cpp"


def test_replay_hal_velocity_motor_matches_interface():
    """REPLAY no-op motor overrides match IVelocityMotor (OQ-4: setSpeed, not setSpeed/setOutput mismatch).

    The whole drive-motor tree (IVelocityMotor, Motor, SimMotor, MotorController,
    NoopVelocityMotor) consistently names the command method setSpeed.
    Assert the interface and the shared NoopVelocityMotor stub agree, and that the
    stub uses the `override` keyword so any future signature drift fails to compile.

    NoopVelocityMotor was refactored from ReplayHAL.h into the shared header
    hal/NoopDevices.h in ticket 046-003. ReplayHAL.h includes NoopDevices.h so
    the invariant is preserved; the check now targets the canonical location.
    """
    iface = (REPO_ROOT / "source" / "hal" / "capability" / "IVelocityMotor.h").read_text()
    noop = (REPO_ROOT / "source" / "hal" / "NoopDevices.h").read_text()
    replay = (REPO_ROOT / "source" / "hal" / "ReplayHAL.h").read_text()
    assert "virtual void setSpeed(int8_t pct)" in iface, \
        "IVelocityMotor no longer declares setSpeed(int8_t)"
    assert "setSpeed(int8_t pct) override" in noop, \
        "NoopVelocityMotor (hal/NoopDevices.h) does not override setSpeed(int8_t)"
    assert 'include "hal/NoopDevices.h"' in replay, \
        "ReplayHAL.h no longer includes hal/NoopDevices.h (NoopVelocityMotor moved there in 046-003)"


def test_vendor_baseline_empty():
    """tests/_infra/vendor_baseline.txt is empty (all vendor leaks sealed by T3)."""
    bl = REPO_ROOT / "tests" / "_infra" / "vendor_baseline.txt"
    if bl.exists():
        content = bl.read_text().strip()
        assert content == "", \
            f"vendor_baseline.txt is not empty:\n{content}"
