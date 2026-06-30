"""
test_ekf_dual_source.py — Encoder error injection + dual-source EKF fusion tests
(ticket 058-001).

Three tests:

1. test_encoder_error_injection_only
   Verifies that drive2_api_enable_encoder_sim_model injects meaningful encoder
   error in isolation.  The SimOdometer is initialized (begin_otos) and the
   zero-noise sim model is enabled so OTOS accumulates true position and the EKF
   can lean on it to correct the over-reporting encoder.

2. test_ekf_dual_source_fusion  (headline deliverable — SUC-002)
   Both sensors are simultaneously imperfect and biased in opposite directions:
   the encoder over-reports position (scale error) while OTOS under-reports
   (negative drift).  The EKF blends the two biased sources and the fused
   estimate ends up closer to ground truth than either raw source individually.
   Asserts fused_err < encoder_only_err AND fused_err < optical_only_err,
   with both raw errors > 5 mm so the assertion is non-vacuous.

3. test_otos_bad_encoder_good
   Regression of the 057-005 single-bad-sensor scenario: noisy OTOS without
   OTOS initialization so Drive2's correction path is inactive.  The EKF
   encoder dead-reckoning tracks ground truth while optical_x stays at 0
   (opt_err = gt_x >> 10 mm).  Fused error must be < 20 mm.

Architecture note — EKF sensor weighting:
  The EKF is configured with ekfQxy=800 (high position process noise) and
  ekfROtosXy=50 (low OTOS position measurement noise), making it strongly
  OTOS-primary when OTOS is initialized.  Test 2 exploits this: with encoder
  over-reporting and OTOS under-reporting, the EKF's weighted blend lands
  between the two biased sources and closer to ground truth than either.
  Test 3 keeps OTOS uninitialized so Drive2's OTOS-correction path is inactive,
  and the encoder dead-reckoning alone tracks ground truth.

The encoder shim (drive2_api_enable_encoder_sim_model) and the OTOS
initialization shim (drive2_api_begin_otos) are loaded from the same
libfirmware_host as the existing drive2 tests.  Drive2Ctx and _load_lib are
imported from test_drive2_subsystem (sibling module in tests/simulation/unit/).
"""

from __future__ import annotations

import ctypes
import sys
import pathlib

import pytest

# ---------------------------------------------------------------------------
# Import the shared Drive2Ctx + base _load_lib from the sibling test module.
# ---------------------------------------------------------------------------

_HERE = pathlib.Path(__file__).parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from test_drive2_subsystem import Drive2Ctx, _load_lib as _base_load_lib  # noqa: E402


# ---------------------------------------------------------------------------
# Extended library loader — adds encoder sim model + begin_otos shim
# signatures on top of the base signatures that _base_load_lib configures.
# ---------------------------------------------------------------------------

def _load_lib() -> ctypes.CDLL:
    """Load firmware_host and configure all shim signatures including encoder model."""
    lib = _base_load_lib()

    # drive2_api_enable_encoder_sim_model (ticket 058-001)
    lib.drive2_api_enable_encoder_sim_model.restype  = None
    lib.drive2_api_enable_encoder_sim_model.argtypes = [
        ctypes.c_void_p,   # handle
        ctypes.c_float,    # slip_l
        ctypes.c_float,    # slip_r
        ctypes.c_float,    # scale_err_l
        ctypes.c_float,    # scale_err_r
    ]

    # drive2_api_begin_otos (ticket 058-001) — initialises SimOdometer so
    # Drive2's OTOS-correction path activates.  Mirrors Robot::begin() → otos.begin().
    lib.drive2_api_begin_otos.restype  = None
    lib.drive2_api_begin_otos.argtypes = [ctypes.c_void_p]

    return lib


# ---------------------------------------------------------------------------
# Module-scoped fixture (mirrors dlib in test_drive2_subsystem).
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def dlib(build_lib):  # noqa: ARG001
    """Return a ctypes handle with both base and encoder-sim shim signatures."""
    return _load_lib()


# ---------------------------------------------------------------------------
# Extended Drive2Ctx with encoder sim model and OTOS init helpers.
# ---------------------------------------------------------------------------

class Drive2CtxEx(Drive2Ctx):
    """Drive2Ctx extended with enable_encoder_sim_model and begin_otos."""

    def begin_otos(self) -> "Drive2CtxEx":
        """Initialize the SimOdometer so Drive2's OTOS-correction path is active."""
        self._lib.drive2_api_begin_otos(self._h)
        return self

    def enable_encoder_sim_model(
        self,
        slip_l: float = 0.0,
        slip_r: float = 0.0,
        scale_err_l: float = 0.0,
        scale_err_r: float = 0.0,
    ) -> "Drive2CtxEx":
        self._lib.drive2_api_enable_encoder_sim_model(
            self._h,
            ctypes.c_float(slip_l),
            ctypes.c_float(slip_r),
            ctypes.c_float(scale_err_l),
            ctypes.c_float(scale_err_r),
        )
        return self


# ---------------------------------------------------------------------------
# Test 1: test_encoder_error_injection_only
#
# Configure the encoder to over-report position by 15% (scale error).
# Initialize the OTOS with its zero-noise sim model so it accumulates the
# TRUE plant velocity and provides a perfect position reference to the EKF.
#
# After 60 forward ticks the encoder-only estimate exceeds ground truth by
# ~14 mm (confirming injection is active), while the EKF fused estimate
# corrects towards the clean OTOS reference and ends up within ~1 mm of truth.
# ---------------------------------------------------------------------------

def test_encoder_error_injection_only(dlib):
    """Encoder scale over-report is detected and corrected by clean OTOS via EKF."""
    with Drive2CtxEx(dlib) as d:
        # Initialize OTOS so Drive2 applies optical corrections.
        d.begin_otos()
        # Enable OTOS zero-noise sim model: accumulates true velocity → perfect reference.
        d.enable_otos_sim_model(
            linear_noise_sigma=0.0, yaw_noise_sigma=0.0,
            drift_per_tick_mm=0.0, drift_per_tick_rad=0.0,
            linear_scale_err=0.0, angular_scale_err=0.0,
        )
        # Encoder over-reports by 15% (no slip — pure scale error).
        d.enable_encoder_sim_model(
            slip_l=0.0, slip_r=0.0,
            scale_err_l=0.15, scale_err_r=0.15,
        )

        for _ in range(60):
            d.apply_twist(vx=200.0)
            d.tick(dt_ms=20)

        gt_x    = d.ground_truth_x()
        fused_x = d.fused_x()
        enc_x   = d.encoder_x()

        enc_err   = abs(enc_x   - gt_x)
        fused_err = abs(fused_x - gt_x)

        assert enc_err > 5.0, (
            f"Encoder error injection not working: enc_err={enc_err:.2f} mm <= 5 mm "
            f"(gt={gt_x:.1f}, enc={enc_x:.1f})"
        )
        assert fused_err < enc_err, (
            f"EKF did not improve on raw encoder: fused_err={fused_err:.2f} mm, "
            f"enc_err={enc_err:.2f} mm "
            f"(gt={gt_x:.1f}, fused={fused_x:.1f}, enc={enc_x:.1f})"
        )


# ---------------------------------------------------------------------------
# Test 2: test_ekf_dual_source_fusion  (headline deliverable — SUC-002)
#
# Both sensors are simultaneously imperfect with opposite-direction biases:
#   - Encoder: 15% scale over-report  → encoder estimates too HIGH
#   - OTOS:    -0.15 mm/tick drift   → optical estimates too LOW
#
# The EKF blends the two biased sources and the fused estimate lands between
# them — closer to ground truth than either individual source.
#
# Both raw errors exceed 5 mm (non-trivially wrong) so the assertion is
# meaningful.  Typical values after 75 ticks (1.5 s of forward motion):
#   gt ≈ 144 mm, enc ≈ 166 mm (err ≈ 22), opt ≈ 135 mm (err ≈ 9), fused ≈ 136 mm (err ≈ 8).
# ---------------------------------------------------------------------------

def test_ekf_dual_source_fusion(dlib):
    """EKF fused pose beats BOTH biased encoder (over-report) and biased OTOS (under-report)."""
    with Drive2CtxEx(dlib) as d:
        # Initialize OTOS so Drive2 applies optical corrections.
        d.begin_otos()
        # OTOS: deterministic negative drift (-0.15 mm/tick) → under-reports position.
        # No Gaussian noise so the result is deterministic across runs.
        d.enable_otos_sim_model(
            linear_noise_sigma=0.0,
            yaw_noise_sigma=0.005,
            drift_per_tick_mm=-0.15,
            drift_per_tick_rad=0.0001,
            linear_scale_err=0.01,
            angular_scale_err=0.005,
        )
        # Encoder: 15% scale over-report → over-reports position.
        d.enable_encoder_sim_model(
            slip_l=0.0, slip_r=0.0,
            scale_err_l=0.15, scale_err_r=0.15,
        )

        for _ in range(75):
            d.apply_twist(vx=200.0)
            d.tick(dt_ms=20)

        gt_x    = d.ground_truth_x()
        fused_x = d.fused_x()
        enc_x   = d.encoder_x()
        opt_x   = d.optical_x()

        fused_err = abs(fused_x - gt_x)
        enc_err   = abs(enc_x   - gt_x)
        opt_err   = abs(opt_x   - gt_x)

        # Both raw sensors must be non-trivially wrong so the fusion assertion
        # carries real meaning.
        assert enc_err > 5.0, (
            f"Encoder error not injected: enc_err={enc_err:.2f} mm <= 5 mm "
            f"(gt={gt_x:.1f}, enc={enc_x:.1f}, opt={opt_x:.1f}, fused={fused_x:.1f})"
        )
        assert opt_err > 5.0, (
            f"Optical error not injected: opt_err={opt_err:.2f} mm <= 5 mm "
            f"(gt={gt_x:.1f}, enc={enc_x:.1f}, opt={opt_x:.1f}, fused={fused_x:.1f})"
        )

        # The EKF fused estimate must beat BOTH raw sources.
        assert fused_err < enc_err and fused_err < opt_err, (
            f"EKF fusion did not beat both raw sources: "
            f"fused_err={fused_err:.2f} mm, enc_err={enc_err:.2f} mm, "
            f"opt_err={opt_err:.2f} mm "
            f"(gt={gt_x:.1f}, fused={fused_x:.1f}, enc={enc_x:.1f}, opt={opt_x:.1f})"
        )


# ---------------------------------------------------------------------------
# Test 3: test_otos_bad_encoder_good
#
# Regression of the 057-005 single-bad-sensor scenario.  The SimOdometer is
# NOT initialized (drive2_api_begin_otos not called), so Drive2's OTOS-
# correction path remains inactive.  The EKF encoder dead-reckoning tracks
# ground truth closely; optical_x stays at 0 (opt_err ≈ gt_x >> 10 mm).
#
# This mirrors test_ekf_fusion_beats_noise in test_drive2_subsystem.py and
# verifies that encoder-clean / OTOS-inactive behaviour is unchanged by the
# 058-001 changes.
# ---------------------------------------------------------------------------

def test_otos_bad_encoder_good(dlib):
    """OTOS-inactive / encoder-clean regime: EKF stays within 20 mm of ground truth."""
    with Drive2CtxEx(dlib) as d:
        # OTOS NOT initialized — Drive2 correction path inactive for this test.
        # Same OTOS error parameters as the 057-005 test_ekf_fusion_beats_noise.
        d.enable_otos_sim_model(
            linear_noise_sigma=5.0,
            yaw_noise_sigma=0.02,
            drift_per_tick_mm=0.5,
            drift_per_tick_rad=0.001,
            linear_scale_err=0.03,
            angular_scale_err=0.02,
        )

        for _ in range(50):
            d.apply_twist(vx=200.0)
            d.tick(dt_ms=20)

        gt_x    = d.ground_truth_x()
        fused_x = d.fused_x()
        enc_x   = d.encoder_x()
        opt_x   = d.optical_x()

        fused_err = abs(fused_x - gt_x)
        enc_err   = abs(enc_x   - gt_x)
        opt_err   = abs(opt_x   - gt_x)

        # Fused (= encoder dead-reckoning, since OTOS path is inactive) must
        # be within 20 mm of ground truth.
        assert fused_err < 20.0, (
            f"EKF fused error {fused_err:.1f} mm exceeds 20 mm threshold "
            f"(gt={gt_x:.1f}, fused={fused_x:.1f}, enc={enc_x:.1f}, opt={opt_x:.1f})"
        )
        # At least one raw source must diverge by > 10 mm, proving the test
        # is non-vacuous (opt_x=0 → opt_err = gt_x ≈ 72 mm >> 10 mm).
        assert max(enc_err, opt_err) > 10.0, (
            f"Raw sensors too accurate — noise may not be injected correctly "
            f"(enc_err={enc_err:.1f} mm, opt_err={opt_err:.1f} mm). "
            f"gt={gt_x:.1f}, enc={enc_x:.1f}, opt={opt_x:.1f}"
        )
