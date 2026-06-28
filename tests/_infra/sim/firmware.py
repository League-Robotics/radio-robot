"""
firmware.py — ctypes loader and Sim class for the host simulation library.

Produced by ticket 020-004.  Loads libfirmware_host.dylib (macOS) or
libfirmware_host.so (Linux) and exposes all sim_* C ABI functions through
a Sim context manager.
"""
import ctypes
import os
import pathlib
import sys
import time

_HERE = pathlib.Path(__file__).parent


def _lib_name() -> str:
    return "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"


_DEFAULT_LIB_PATH = _HERE / "build" / _lib_name()

# FIRMWARE_HOST_LIB env var allows overriding the shared library path.
# Used by the coverage harness (tests/_infra/coverage.sh) to load a
# coverage-instrumented build from build_coverage/ without modifying the
# standard build/. Backward-compatible: unset → uses the default build path.
LIB_PATH = pathlib.Path(os.environ.get("FIRMWARE_HOST_LIB", str(_DEFAULT_LIB_PATH)))


class Sim:
    """Context manager wrapping one SimHandle (MockHAL + Robot + CommandProcessor)."""

    def __init__(self) -> None:
        self._lib = ctypes.CDLL(str(LIB_PATH))
        self._setup_types()
        self._h = self._lib.sim_create()
        if not self._h:
            raise RuntimeError("sim_create() returned NULL")
        self._t: int = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def __enter__(self) -> "Sim":
        return self

    def __exit__(self, *_) -> None:
        if self._h:
            self._lib.sim_destroy(self._h)
            self._h = None

    # ------------------------------------------------------------------
    # Time advance
    # ------------------------------------------------------------------

    def tick_for(self, total_ms: int, step_ms: int = 24,
                 real_time: bool = False, speed_factor: float = 1.0) -> None:
        """Advance simulation by total_ms milliseconds in step_ms increments.

        Parameters
        ----------
        total_ms:
            Duration to simulate, in milliseconds.
        step_ms:
            Tick granularity in milliseconds (default 24 ms).
        real_time:
            When True, sleep after each tick to pace execution to wall-clock
            time.  Default False — runs at full CPU speed (CI-safe).
        speed_factor:
            Wall-clock multiplier when real_time=True.  1.0 = real time;
            2.0 = twice as fast; 0.5 = half speed.  Ignored when
            real_time=False.
        """
        end = self._t + total_ms
        while self._t < end:
            dt = min(step_ms, end - self._t)
            self._lib.sim_tick(self._h, ctypes.c_uint32(self._t))
            self._t += dt
            if real_time:
                time.sleep(dt / 1000.0 / speed_factor)

    # ------------------------------------------------------------------
    # Command dispatch
    # ------------------------------------------------------------------

    def send_command(self, line: str) -> str:
        """Send one command line; return the synchronous reply as a decoded string.

        Buffer is 2048 bytes — matches the ReplyStore capacity in sim_api.cpp so
        that multi-line replies (e.g. chunked GET CFG output) are not truncated.
        """
        buf = ctypes.create_string_buffer(2048)
        n = self._lib.sim_command(self._h, line.encode(), buf, 2048)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    def get_async_evts(self) -> str:
        """Return any async EVT replies accumulated since the last send_command call."""
        buf = ctypes.create_string_buffer(2048)
        n = self._lib.sim_get_async_evts(self._h, buf, 2048)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    def drain_reply_store(self) -> None:
        """Discard any replies (TLM, EVTs) accumulated in the reply store.

        Call this between tick_for() and tick_collect_tlm() to avoid
        tick_collect_tlm() picking up TLM frames emitted during tick_for().
        tick_for() uses sim_tick() which accumulates TLM in replyStore but
        does not drain it; tick_collect_tlm() drains it on its first tick,
        inadvertently including stale frames from the tick_for() phase.
        """
        self.get_async_evts()  # sim_get_async_evts resets the store

    # ------------------------------------------------------------------
    # Internal: argtypes / restype declarations
    # ------------------------------------------------------------------

    def _setup_types(self) -> None:
        lib = self._lib

        # sim_create() → void*
        lib.sim_create.argtypes = []
        lib.sim_create.restype = ctypes.c_void_p

        # sim_destroy(void* h)
        lib.sim_destroy.argtypes = [ctypes.c_void_p]
        lib.sim_destroy.restype = None

        # sim_tick(void* h, uint32_t now_ms)
        lib.sim_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        lib.sim_tick.restype = None

        # sim_command(void* h, const char* line, char* out_buf, int out_len) → int
        lib.sim_command.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        lib.sim_command.restype = ctypes.c_int

        # sim_get_enc_l / sim_get_enc_r → float
        lib.sim_get_enc_l.argtypes = [ctypes.c_void_p]
        lib.sim_get_enc_l.restype = ctypes.c_float
        lib.sim_get_enc_r.argtypes = [ctypes.c_void_p]
        lib.sim_get_enc_r.restype = ctypes.c_float

        # sim_get_vel_l / sim_get_vel_r → float
        lib.sim_get_vel_l.argtypes = [ctypes.c_void_p]
        lib.sim_get_vel_l.restype = ctypes.c_float
        lib.sim_get_vel_r.argtypes = [ctypes.c_void_p]
        lib.sim_get_vel_r.restype = ctypes.c_float

        # sim_get_pwm_l / sim_get_pwm_r → float
        lib.sim_get_pwm_l.argtypes = [ctypes.c_void_p]
        lib.sim_get_pwm_l.restype = ctypes.c_float
        lib.sim_get_pwm_r.argtypes = [ctypes.c_void_p]
        lib.sim_get_pwm_r.restype = ctypes.c_float

        # sim_get_pose_x / sim_get_pose_y / sim_get_pose_h → float
        lib.sim_get_pose_x.argtypes = [ctypes.c_void_p]
        lib.sim_get_pose_x.restype = ctypes.c_float
        lib.sim_get_pose_y.argtypes = [ctypes.c_void_p]
        lib.sim_get_pose_y.restype = ctypes.c_float
        lib.sim_get_pose_h.argtypes = [ctypes.c_void_p]
        lib.sim_get_pose_h.restype = ctypes.c_float

        # sim_set_enc_l(void* h, float mm)
        lib.sim_set_enc_l.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_enc_l.restype = None

        # sim_set_enc_r(void* h, float mm)
        lib.sim_set_enc_r.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_enc_r.restype = None

        # sim_set_otos_pose(void* h, float x, float y, float hrad)
        lib.sim_set_otos_pose.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        lib.sim_set_otos_pose.restype = None

        # sim_set_motor_offset(void* h, int side, float factor)
        lib.sim_set_motor_offset.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_float,
        ]
        lib.sim_set_motor_offset.restype = None

        # sim_get_async_evts(void* h, char* evts_buf, int evts_len) → int
        lib.sim_get_async_evts.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        lib.sim_get_async_evts.restype = ctypes.c_int

        # sim_set_motor_slip(void* h, int side, float straight, float turn_extra)
        lib.sim_set_motor_slip.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_float,
            ctypes.c_float,
        ]
        lib.sim_set_motor_slip.restype = None

        # sim_set_otos_fusion(void* h, int on)
        lib.sim_set_otos_fusion.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.sim_set_otos_fusion.restype = None

        # sim_enable_otos_model(void* h)
        lib.sim_enable_otos_model.argtypes = [ctypes.c_void_p]
        lib.sim_enable_otos_model.restype = None

        # sim_set_otos_linear_noise(void* h, float sigma_fraction)
        lib.sim_set_otos_linear_noise.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_otos_linear_noise.restype = None

        # sim_set_otos_yaw_noise(void* h, float sigma_fraction)
        lib.sim_set_otos_yaw_noise.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_otos_yaw_noise.restype = None

        # sim_get_otos_x / _y / _h → float (SimOdometer accumulated odom pose).
        # These read the observation model's reported pose (truth + configured
        # error), used by the 040-005 observation-only isolation matrix.
        lib.sim_get_otos_x.argtypes = [ctypes.c_void_p]
        lib.sim_get_otos_x.restype = ctypes.c_float
        lib.sim_get_otos_y.argtypes = [ctypes.c_void_p]
        lib.sim_get_otos_y.restype = ctypes.c_float
        lib.sim_get_otos_h.argtypes = [ctypes.c_void_p]
        lib.sim_get_otos_h.restype = ctypes.c_float

        # sim_get_exact_pose_x / _y / _h → float
        lib.sim_get_exact_pose_x.argtypes = [ctypes.c_void_p]
        lib.sim_get_exact_pose_x.restype = ctypes.c_float
        lib.sim_get_exact_pose_y.argtypes = [ctypes.c_void_p]
        lib.sim_get_exact_pose_y.restype = ctypes.c_float
        lib.sim_get_exact_pose_h.argtypes = [ctypes.c_void_p]
        lib.sim_get_exact_pose_h.restype = ctypes.c_float

        # ---- 040-003: true-vs-estimate ABI ----
        # sim_get_true_pose_x / _y / _h → float (plant ground truth)
        lib.sim_get_true_pose_x.argtypes = [ctypes.c_void_p]
        lib.sim_get_true_pose_x.restype = ctypes.c_float
        lib.sim_get_true_pose_y.argtypes = [ctypes.c_void_p]
        lib.sim_get_true_pose_y.restype = ctypes.c_float
        lib.sim_get_true_pose_h.argtypes = [ctypes.c_void_p]
        lib.sim_get_true_pose_h.restype = ctypes.c_float

        # sim_get_true_enc_l / _r → float (true unslipped wheel travel)
        lib.sim_get_true_enc_l.argtypes = [ctypes.c_void_p]
        lib.sim_get_true_enc_l.restype = ctypes.c_float
        lib.sim_get_true_enc_r.argtypes = [ctypes.c_void_p]
        lib.sim_get_true_enc_r.restype = ctypes.c_float

        # sim_get_true_vel_l / _r → float (true per-wheel velocity, mm/s)
        lib.sim_get_true_vel_l.argtypes = [ctypes.c_void_p]
        lib.sim_get_true_vel_l.restype = ctypes.c_float
        lib.sim_get_true_vel_r.argtypes = [ctypes.c_void_p]
        lib.sim_get_true_vel_r.restype = ctypes.c_float

        # sim_set_true_pose(void* h, float x, float y, float h_rad)
        lib.sim_set_true_pose.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        lib.sim_set_true_pose.restype = None

        # sim_set_true_wheel_travel(void* h, float enc_l_mm, float enc_r_mm)
        lib.sim_set_true_wheel_travel.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
        ]
        lib.sim_set_true_wheel_travel.restype = None

        # sim_set_true_velocity(void* h, float vel_l_mms, float vel_r_mms)
        lib.sim_set_true_velocity.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
        ]
        lib.sim_set_true_velocity.restype = None

        # sim_get_estimation_error_xy / _h → float (estimate vs. plant truth)
        lib.sim_get_estimation_error_xy.argtypes = [ctypes.c_void_p]
        lib.sim_get_estimation_error_xy.restype = ctypes.c_float
        lib.sim_get_estimation_error_h.argtypes = [ctypes.c_void_p]
        lib.sim_get_estimation_error_h.restype = ctypes.c_float

        # sim_set_perfect(void* h) — reset all observation-model error to no-op
        lib.sim_set_perfect.argtypes = [ctypes.c_void_p]
        lib.sim_set_perfect.restype = None

        # sim_get_ekf_rej_count(void* h) → int  (030-001 N1 diagnostic)
        lib.sim_get_ekf_rej_count.argtypes = [ctypes.c_void_p]
        lib.sim_get_ekf_rej_count.restype = ctypes.c_int

        # N8 sensor-freshness helpers (030-008)
        # sim_init_line_sensor(void* h)
        lib.sim_init_line_sensor.argtypes = [ctypes.c_void_p]
        lib.sim_init_line_sensor.restype = None

        # sim_init_color_sensor(void* h)
        lib.sim_init_color_sensor.argtypes = [ctypes.c_void_p]
        lib.sim_init_color_sensor.restype = None

        # sim_set_line_frozen(void* h, int frozen)
        lib.sim_set_line_frozen.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.sim_set_line_frozen.restype = None

        # sim_set_color_frozen(void* h, int frozen)
        lib.sim_set_color_frozen.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.sim_set_color_frozen.restype = None

        # (045-003) sim_set_line_values(void* h, uint16 l0,l1,l2,l3)
        lib.sim_set_line_values.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint16, ctypes.c_uint16, ctypes.c_uint16, ctypes.c_uint16,
        ]
        lib.sim_set_line_values.restype = None

        # (045-003) sim_set_color_rgbc(void* h, uint16 r,g,b,c)
        lib.sim_set_color_rgbc.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint16, ctypes.c_uint16, ctypes.c_uint16, ctypes.c_uint16,
        ]
        lib.sim_set_color_rgbc.restype = None

        # N9 same-tick OTOS failure helper (030-008)
        # sim_set_otos_read_failure(void* h, int fail)
        lib.sim_set_otos_read_failure.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.sim_set_otos_read_failure.restype = None

        # sim_get_fused_v(void* h) → float
        lib.sim_get_fused_v.argtypes = [ctypes.c_void_p]
        lib.sim_get_fused_v.restype = ctypes.c_float

        # sim_get_fused_omega(void* h) → float
        lib.sim_get_fused_omega.argtypes = [ctypes.c_void_p]
        lib.sim_get_fused_omega.restype = ctypes.c_float

        # sim_set_enc_omega_healthy(void* h, int healthy)
        lib.sim_set_enc_omega_healthy.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.sim_set_enc_omega_healthy.restype = None

        # N11 pose injection helper (030-009)
        # sim_set_pose(void* h, float x, float y, float hrad)
        lib.sim_set_pose.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        lib.sim_set_pose.restype = None

        # N15 EKF P diagonal accessor (030-009)
        # sim_get_ekf_p_diag(void* h, int idx) → float
        lib.sim_get_ekf_p_diag.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.sim_get_ekf_p_diag.restype = ctypes.c_float

        # N2 queue-invariant helper (030-002)
        # sim_get_queue_wired(void* h) → int (1 if queue attached, 0 if not)
        lib.sim_get_queue_wired.argtypes = [ctypes.c_void_p]
        lib.sim_get_queue_wired.restype = ctypes.c_int

        # N7 queue-overflow helpers (030-005)
        # sim_queue_size(void* h) → int (current item count)
        lib.sim_queue_size.argtypes = [ctypes.c_void_p]
        lib.sim_queue_size.restype = ctypes.c_int

        # sim_fill_queue(void* h) → int (number of dummy items pushed)
        lib.sim_fill_queue.argtypes = [ctypes.c_void_p]
        lib.sim_fill_queue.restype = ctypes.c_int

        # sim_command_no_drain(void* h, line, buf, len) → int
        # Like sim_command but skips the two dequeueOne drains.
        lib.sim_command_no_drain.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        lib.sim_command_no_drain.restype = ctypes.c_int

        # D10 telemetry helpers (028-005)
        # sim_get_tlm_bound(void* h) → int (1 if bound, 0 if not)
        lib.sim_get_tlm_bound.argtypes = [ctypes.c_void_p]
        lib.sim_get_tlm_bound.restype = ctypes.c_int

        # sim_set_tlm_bound_radio(void* h, int on) → None (forces radio TLM cap)
        lib.sim_set_tlm_bound_radio.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.sim_set_tlm_bound_radio.restype = None

        # sim_tick_collect_tlm(void* h, start_ms, total_ms, step_ms, buf, len) → int
        lib.sim_tick_collect_tlm.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        lib.sim_tick_collect_tlm.restype = ctypes.c_int

        # Bench OTOS sim hooks (031-002)
        # sim_bench_otos_tick(void* h, float vel_l, float vel_r,
        #                     float trackwidth_mm, uint32_t dt_ms)
        lib.sim_bench_otos_tick.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_uint32,
        ]
        lib.sim_bench_otos_tick.restype = None

        # sim_get_bench_otos_x/y/h → float (noiseless ideal accumulator)
        lib.sim_get_bench_otos_x.argtypes = [ctypes.c_void_p]
        lib.sim_get_bench_otos_x.restype = ctypes.c_float
        lib.sim_get_bench_otos_y.argtypes = [ctypes.c_void_p]
        lib.sim_get_bench_otos_y.restype = ctypes.c_float
        lib.sim_get_bench_otos_h.argtypes = [ctypes.c_void_p]
        lib.sim_get_bench_otos_h.restype = ctypes.c_float

        # sim_get_bench_otos_errored_x/y/h → float (errored accumulator)
        lib.sim_get_bench_otos_errored_x.argtypes = [ctypes.c_void_p]
        lib.sim_get_bench_otos_errored_x.restype = ctypes.c_float
        lib.sim_get_bench_otos_errored_y.argtypes = [ctypes.c_void_p]
        lib.sim_get_bench_otos_errored_y.restype = ctypes.c_float
        lib.sim_get_bench_otos_errored_h.argtypes = [ctypes.c_void_p]
        lib.sim_get_bench_otos_errored_h.restype = ctypes.c_float

        # sim_bench_otos_reset(void* h)
        lib.sim_bench_otos_reset.argtypes = [ctypes.c_void_p]
        lib.sim_bench_otos_reset.restype = None

        # sim_bench_otos_set_noise(void* h, float noise_xy, float noise_h,
        #                          float drift_rad_per_sec)
        lib.sim_bench_otos_set_noise.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        lib.sim_bench_otos_set_noise.restype = None

        # 033-005 wedge-defense sim hooks
        # sim_get_wheel_wedged_l / sim_get_wheel_wedged_r → int (0 or 1)
        lib.sim_get_wheel_wedged_l.argtypes = [ctypes.c_void_p]
        lib.sim_get_wheel_wedged_l.restype = ctypes.c_int
        lib.sim_get_wheel_wedged_r.argtypes = [ctypes.c_void_p]
        lib.sim_get_wheel_wedged_r.restype = ctypes.c_int

        # sim_get_odometry_wedge_active(void* h) → int (0 or 1)
        lib.sim_get_odometry_wedge_active.argtypes = [ctypes.c_void_p]
        lib.sim_get_odometry_wedge_active.restype = ctypes.c_int

        # sim_get_odometry_enc_omega_healthy(void* h) → int (0 or 1)
        lib.sim_get_odometry_enc_omega_healthy.argtypes = [ctypes.c_void_p]
        lib.sim_get_odometry_enc_omega_healthy.restype = ctypes.c_int

        # ---- Three-estimate pose reads (047-002 / 047-005) ----
        # sim_get_enc_pose_x/y/h → float (encoder dead-reckoning, never touched by EKF)
        lib.sim_get_enc_pose_x.argtypes = [ctypes.c_void_p]
        lib.sim_get_enc_pose_x.restype = ctypes.c_float
        lib.sim_get_enc_pose_y.argtypes = [ctypes.c_void_p]
        lib.sim_get_enc_pose_y.restype = ctypes.c_float
        lib.sim_get_enc_pose_h.argtypes = [ctypes.c_void_p]
        lib.sim_get_enc_pose_h.restype = ctypes.c_float

        # sim_get_otos_pose_x/y/h → float (raw optical reading, pre-EKF)
        lib.sim_get_otos_pose_x.argtypes = [ctypes.c_void_p]
        lib.sim_get_otos_pose_x.restype = ctypes.c_float
        lib.sim_get_otos_pose_y.argtypes = [ctypes.c_void_p]
        lib.sim_get_otos_pose_y.restype = ctypes.c_float
        lib.sim_get_otos_pose_h.argtypes = [ctypes.c_void_p]
        lib.sim_get_otos_pose_h.restype = ctypes.c_float

        # sim_get_fused_pose_x/y/h → float (EKF output, same as sim_get_pose_x/y/h)
        lib.sim_get_fused_pose_x.argtypes = [ctypes.c_void_p]
        lib.sim_get_fused_pose_x.restype = ctypes.c_float
        lib.sim_get_fused_pose_y.argtypes = [ctypes.c_void_p]
        lib.sim_get_fused_pose_y.restype = ctypes.c_float
        lib.sim_get_fused_pose_h.argtypes = [ctypes.c_void_p]
        lib.sim_get_fused_pose_h.restype = ctypes.c_float

    # ------------------------------------------------------------------
    # N7 queue-overflow helpers (030-005)
    # ------------------------------------------------------------------

    def queue_size(self) -> int:
        """Return the current number of items in the CommandQueue."""
        return int(self._lib.sim_queue_size(self._h))

    def fill_queue(self) -> int:
        """Fill the CommandQueue to capacity with no-op dummy entries.

        Returns the number of dummy items pushed.  After this call the
        queue is full; any sim_command_no_drain() call that routes through
        dispatchTable() will get ERR full.
        """
        return int(self._lib.sim_fill_queue(self._h))

    def send_command_no_drain(self, line: str) -> str:
        """Send one command WITHOUT draining the queue afterwards.

        Used by overflow tests to see whether dispatchTable() returns
        ERR full when the queue is already full.
        """
        buf = ctypes.create_string_buffer(512)
        n = self._lib.sim_command_no_drain(self._h, line.encode(), buf, 512)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    # ------------------------------------------------------------------
    # N2 queue-invariant helper (030-002)
    # ------------------------------------------------------------------

    def get_queue_wired(self) -> bool:
        """Return True if CommandProcessor has a queue attached.

        The queue is wired in SimHandle's constructor and must remain attached
        for the full session — if this returns False, the Phase-3-style
        move-assign bug has regressed (N2 finding).
        """
        return bool(self._lib.sim_get_queue_wired(self._h))

    # ------------------------------------------------------------------
    # D10 telemetry helpers (028-005)
    # ------------------------------------------------------------------

    def get_tlm_bound(self) -> bool:
        """Return True if the TLM channel is bound (STREAM was issued)."""
        return bool(self._lib.sim_get_tlm_bound(self._h))

    def set_tlm_bound_radio(self, on: bool) -> None:
        """Force the radio TLM-rate-cap flag (telemetryEmit) for testing.

        In sim, runCommsIn never runs so the firmware's channel-type resolution
        is skipped; this lets a test drive the radio cap path directly.
        """
        self._lib.sim_set_tlm_bound_radio(self._h, ctypes.c_int(1 if on else 0))

    def tick_collect_tlm(self, total_ms: int, step_ms: int = 24) -> list[str]:
        """Advance simulation and return a list of TLM frame strings emitted.

        Each entry is one raw TLM line (without trailing newline).
        ``total_ms`` is the window duration; ``step_ms`` is the tick step.
        The sim clock continues from where it was last set by tick_for().
        """
        buf = ctypes.create_string_buffer(65536)
        count = self._lib.sim_tick_collect_tlm(
            self._h,
            ctypes.c_uint32(self._t),
            ctypes.c_uint32(total_ms),
            ctypes.c_uint32(step_ms),
            buf,
            65536,
        )
        self._t += total_ms
        if count <= 0:
            return []
        raw = buf.raw.split(b"\x00")[0].decode(errors="replace")
        lines = [ln for ln in raw.split("\n") if ln.strip()]
        return lines

    # ------------------------------------------------------------------
    # N8 sensor-freshness helpers (030-008)
    # ------------------------------------------------------------------

    def init_line_sensor(self) -> None:
        """Initialize (begin) the MockLineSensor so Robot::lineRead() is active."""
        self._lib.sim_init_line_sensor(self._h)

    def init_color_sensor(self) -> None:
        """Initialize (begin) the MockColorSensor so Robot::colorRead() is active."""
        self._lib.sim_init_color_sensor(self._h)

    def set_line_frozen(self, frozen: bool) -> None:
        """Freeze or unfreeze the MockLineSensor.

        When frozen, readValues() returns false so Robot::lineRead() never
        updates lineVS.lastUpdMs.  After ~2×lagMs the TLM freshness gate
        drops the line= field from TLM frames (N8 fix verification).
        """
        self._lib.sim_set_line_frozen(self._h, ctypes.c_int(1 if frozen else 0))

    def set_color_frozen(self, frozen: bool) -> None:
        """Freeze or unfreeze the MockColorSensor (N8 fix verification)."""
        self._lib.sim_set_color_frozen(self._h, ctypes.c_int(1 if frozen else 0))

    # ------------------------------------------------------------------
    # (045-003) Fixed sensor-value injection (line / color)
    # ------------------------------------------------------------------

    def set_line_values(self, l0: int, l1: int, l2: int, l3: int) -> None:
        """Inject a constant 4-channel line-sensor reading into the SimLineSensor.

        Installs a single-row schedule so the sensor returns (l0,l1,l2,l3) on
        every read.  Call init_line_sensor() first so LineSensor::periodic reads
        it into HardwareState.line[] (where StopCondition::evaluate sees it).
        """
        self._lib.sim_set_line_values(
            self._h,
            ctypes.c_uint16(l0), ctypes.c_uint16(l1),
            ctypes.c_uint16(l2), ctypes.c_uint16(l3),
        )

    def set_color_rgbc(self, r: int, g: int, b: int, c: int) -> None:
        """Inject a constant RGBC color-sensor reading into the SimColorSensor.

        Installs a single-row schedule so the sensor returns (r,g,b,c) on every
        read.  Call init_color_sensor() first so ColorSensor::periodic reads it
        into HardwareState.colorR/G/B/C.
        """
        self._lib.sim_set_color_rgbc(
            self._h,
            ctypes.c_uint16(r), ctypes.c_uint16(g),
            ctypes.c_uint16(b), ctypes.c_uint16(c),
        )

    # ------------------------------------------------------------------
    # N9 same-tick OTOS failure helper (030-008)
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # N11 pose injection helper (030-009)
    # ------------------------------------------------------------------

    def set_pose(self, x: float, y: float, hrad: float) -> None:
        """Directly inject a dead-reckoning pose into state.inputs.

        Used by N11 test to place the robot past a G target so the PURSUE
        backtrack re-gate fires without requiring the robot to physically drive.
        """
        self._lib.sim_set_pose(
            self._h,
            ctypes.c_float(x),
            ctypes.c_float(y),
            ctypes.c_float(hrad),
        )

    # ------------------------------------------------------------------
    # N15 EKF P diagonal accessor (030-009)
    # ------------------------------------------------------------------

    def get_ekf_p_diag(self, idx: int) -> float:
        """Return P[idx][idx] from the EKF covariance matrix.

        Index mapping: 0=x, 1=y, 2=theta, 3=v, 4=omega.
        Returns -1.0 for out-of-range idx.
        """
        return float(self._lib.sim_get_ekf_p_diag(self._h, ctypes.c_int(idx)))

    def set_otos_read_failure(self, fail: bool) -> None:
        """Inject or clear an OTOS read failure.

        When set, MockOtosSensor::readTransformed returns false and emits
        {0,0,0}.  Robot::otosCorrect() must detect this via the return value
        and skip EKF fusion (N9 fix verification).
        """
        self._lib.sim_set_otos_read_failure(self._h, ctypes.c_int(1 if fail else 0))

    def get_fused_v(self) -> float:
        """Return fusedV (EKF body-frame linear speed, mm/s) from state.inputs."""
        return float(self._lib.sim_get_fused_v(self._h))

    def get_fused_omega(self) -> float:
        """Return fusedOmega (EKF yaw rate, rad/s) from state.inputs."""
        return float(self._lib.sim_get_fused_omega(self._h))

    def set_enc_omega_healthy(self, healthy: bool) -> None:
        """Set the encoder-omega health gate (033-003).

        healthy=False simulates a wedged wheel: predict() suppresses the encoder
        yaw-rate observation so a frozen encoder cannot inject phantom omega.
        """
        self._lib.sim_set_enc_omega_healthy(self._h, ctypes.c_int(1 if healthy else 0))

    # ------------------------------------------------------------------
    # Field-profile helpers
    # ------------------------------------------------------------------

    def set_field_profile(self, slip_turn_extra: float = 0.26,
                          fuse_otos: bool = True) -> None:
        """Configure the simulation as a field-profile fixture.

        Sets turn-slip to reproduce encoder over-report on turns (scrub model),
        and optionally enables OTOS EKF fusion.  This is the fixture used for
        sprint-024 motion-bounding regression tests.

        Args:
            slip_turn_extra: Fractional encoder over-report during turns
                             (positive = encoder reads MORE arc than body rotates,
                             matching real scrub; 0.26 ≈ field-measured value).
                             Negated before passing to sim_set_motor_slip because
                             the MockMotor formula is ``enc = vel * (1 - slip)`` —
                             negative slip produces over-report (sprint 024-006).
            fuse_otos:       Whether to enable OTOS→EKF correction each tick.
        """
        # Negate: positive slip_turn_extra → negative raw slip → encoder over-reports.
        # MockMotor tick: enc = vel * (1 - slip); slip < 0 → enc > vel (over-report).
        self._lib.sim_set_motor_slip(self._h, ctypes.c_int(2),
                                     ctypes.c_float(0.0),
                                     ctypes.c_float(-slip_turn_extra))
        if fuse_otos:
            self._lib.sim_enable_otos_model(self._h)
            self._lib.sim_set_otos_fusion(self._h, ctypes.c_int(1))

    # ------------------------------------------------------------------
    # 040-003: true-vs-estimate ABI (WorldView / PhysicsWorld truth)
    # ------------------------------------------------------------------

    def get_true_pose(self) -> tuple[float, float, float]:
        """Return (x_mm, y_mm, h_rad) from PhysicsWorld ground truth.

        This is the plant truth (the consolidated oracle that replaced
        ExactPoseTracker), NOT the firmware's fused/dead-reckoned estimate.
        """
        x = float(self._lib.sim_get_true_pose_x(self._h))
        y = float(self._lib.sim_get_true_pose_y(self._h))
        h = float(self._lib.sim_get_true_pose_h(self._h))
        return (x, y, h)

    def get_true_wheel_travel(self) -> tuple[float, float]:
        """Return (enc_l_mm, enc_r_mm) true unslipped wheel travel from the plant."""
        l = float(self._lib.sim_get_true_enc_l(self._h))
        r = float(self._lib.sim_get_true_enc_r(self._h))
        return (l, r)

    def get_true_velocity(self) -> tuple[float, float]:
        """Return (vel_l_mms, vel_r_mms) true per-wheel velocity (mm/s) from the plant."""
        l = float(self._lib.sim_get_true_vel_l(self._h))
        r = float(self._lib.sim_get_true_vel_r(self._h))
        return (l, r)

    def set_true_pose(self, x: float, y: float, h_rad: float) -> None:
        """Set ground-truth pose directly (bypasses physics integration).

        The next sim_tick does NOT overwrite it unless that tick integrates the
        actuator path (non-zero PWM with dt>0).
        """
        self._lib.sim_set_true_pose(
            self._h,
            ctypes.c_float(x),
            ctypes.c_float(y),
            ctypes.c_float(h_rad),
        )

    def set_true_wheel_travel(self, enc_l_mm: float, enc_r_mm: float) -> None:
        """Set true wheel-travel accumulators directly (plant ground truth).

        Touches ONLY the true accumulators — for pure plant-truth isolation
        tests.  Use set_enc / sim_set_enc_l/r to flow an injected encoder value
        through the reported path and state.inputs as well.
        """
        self._lib.sim_set_true_wheel_travel(
            self._h,
            ctypes.c_float(enc_l_mm),
            ctypes.c_float(enc_r_mm),
        )

    def set_true_velocity(self, vel_l_mms: float, vel_r_mms: float) -> None:
        """Set true per-wheel velocity directly (plant ground truth, mm/s)."""
        self._lib.sim_set_true_velocity(
            self._h,
            ctypes.c_float(vel_l_mms),
            ctypes.c_float(vel_r_mms),
        )

    def estimation_error(self) -> tuple[float, float]:
        """Return (xy_mm, h_rad) estimation error: firmware estimate vs. plant truth.

        xy_mm is the Euclidean distance between the plant true pose and the
        firmware's fused/dead-reckoned pose (state.inputs.poseX/Y).  h_rad is the
        heading error wrapped to [-pi, pi].  Both are 0.0 when the robot has not
        moved (estimate == truth == origin).
        """
        xy = float(self._lib.sim_get_estimation_error_xy(self._h))
        h = float(self._lib.sim_get_estimation_error_h(self._h))
        return (xy, h)

    def set_perfect(self) -> None:
        """Reset all observation-model error layers to no-op (perfect sensors).

        Clears every Sim* model's freeze/dropout/read-failure/lift state and
        zeros all noise/drift/slip — restoring the fresh-construction
        "sensor is PERFECT" invariant.
        """
        self._lib.sim_set_perfect(self._h)

    # ------------------------------------------------------------------
    # OTOS observation model (SimOdometer) — 040-005 isolation matrix
    # ------------------------------------------------------------------

    def enable_otos_model(self) -> None:
        """Enable the SimOdometer sim-model integrator (accumulated odom pose).

        After this call sim_get_otos_* return the integrated OTOS reading
        (true per-tick velocity arc ± configured noise) rather than 0.
        """
        self._lib.sim_enable_otos_model(self._h)

    def set_otos_fusion(self, on: bool) -> None:
        """Enable/disable firmware OTOS→EKF correction inside sim_tick().

        Also marks the SimOdometer initialised (begin()) so otosCorrect()
        does not early-return on its is_initialized() guard.
        """
        self._lib.sim_set_otos_fusion(self._h, ctypes.c_int(1 if on else 0))

    def set_otos_pose(self, x: float, y: float, h_rad: float) -> None:
        """Inject an OTOS pose reading into the SimOdometer (back-compat path).

        The injected pose is returned by SimOdometer::readTransformed() on the
        next otosCorrect() call.  Used to feed a deliberately bad measurement to
        the EKF Mahalanobis gate (estimator-only isolation test).
        """
        self._lib.sim_set_otos_pose(
            self._h,
            ctypes.c_float(x),
            ctypes.c_float(y),
            ctypes.c_float(h_rad),
        )

    def set_otos_linear_noise(self, sigma_fraction: float) -> None:
        """Set the SimOdometer linear-position noise sigma (fraction of arc)."""
        self._lib.sim_set_otos_linear_noise(
            self._h, ctypes.c_float(sigma_fraction))

    def get_otos_pose(self) -> tuple[float, float, float]:
        """Return (x_mm, y_mm, h_rad) from the SimOdometer accumulated odom pose.

        This is the OTOS observation model's output (plant truth ± configured
        sensor error), NOT the firmware's fused EKF estimate.  Zero until the
        sim-model integrator is enabled via enable_otos_model().
        """
        x = float(self._lib.sim_get_otos_x(self._h))
        y = float(self._lib.sim_get_otos_y(self._h))
        h = float(self._lib.sim_get_otos_h(self._h))
        return (x, y, h)

    def get_ekf_rej_count(self) -> int:
        """Return the cumulative EKF gate rejection count (all channels)."""
        return int(self._lib.sim_get_ekf_rej_count(self._h))

    def get_pose(self) -> tuple[float, float, float]:
        """Return (x_mm, y_mm, h_rad) from the firmware's fused/DR estimate.

        This is state.inputs.poseX/Y/Hrad (what Odometry writes), i.e. the
        ESTIMATE — contrast with get_true_pose() (plant ground truth).
        """
        x = float(self._lib.sim_get_pose_x(self._h))
        y = float(self._lib.sim_get_pose_y(self._h))
        h = float(self._lib.sim_get_pose_h(self._h))
        return (x, y, h)

    # ------------------------------------------------------------------
    # Bench OTOS sim hooks (031-002)
    # ------------------------------------------------------------------

    def bench_otos_tick(self, vel_l: float, vel_r: float,
                        trackwidth_mm: float, dt_ms: int) -> None:
        """Manually tick the BenchOtosSensor with explicit velocities.

        In firmware this is driven by Robot::benchOtosTick() via NezhaHAL.
        In the host sim NezhaHAL is excluded, so tests drive the sensor
        directly to verify the integrator.

        Args:
            vel_l:          Left wheel commanded velocity, mm/s.
            vel_r:          Right wheel commanded velocity, mm/s.
            trackwidth_mm:  Wheel-to-wheel track width, mm.
            dt_ms:          Elapsed time for this step, ms.
        """
        self._lib.sim_bench_otos_tick(
            self._h,
            ctypes.c_float(vel_l),
            ctypes.c_float(vel_r),
            ctypes.c_float(trackwidth_mm),
            ctypes.c_uint32(dt_ms),
        )

    def get_bench_otos_ideal(self) -> tuple:
        """Return (x, y, h) from the noiseless ideal accumulator."""
        x = float(self._lib.sim_get_bench_otos_x(self._h))
        y = float(self._lib.sim_get_bench_otos_y(self._h))
        h = float(self._lib.sim_get_bench_otos_h(self._h))
        return (x, y, h)

    def get_bench_otos_errored(self) -> tuple:
        """Return (x, y, h) from the errored accumulator."""
        x = float(self._lib.sim_get_bench_otos_errored_x(self._h))
        y = float(self._lib.sim_get_bench_otos_errored_y(self._h))
        h = float(self._lib.sim_get_bench_otos_errored_h(self._h))
        return (x, y, h)

    def bench_otos_reset(self) -> None:
        """Zero both BenchOtosSensor accumulators."""
        self._lib.sim_bench_otos_reset(self._h)

    # ------------------------------------------------------------------
    # 033-005 wedge-defense sim hooks
    # ------------------------------------------------------------------

    def get_wheel_wedged_l(self) -> bool:
        """Return True if the left wheel wedge latch is set (033-005e).

        The latch fires when the left encoder has been identical for
        kWedgeThreshold consecutive commanded ticks (after the first move).
        Resets when the encoder changes again.
        """
        return bool(self._lib.sim_get_wheel_wedged_l(self._h))

    def get_wheel_wedged_r(self) -> bool:
        """Return True if the right wheel wedge latch is set (033-005e)."""
        return bool(self._lib.sim_get_wheel_wedged_r(self._h))

    def get_odometry_wedge_active(self) -> bool:
        """Return True when Odometry::_wedgeActive is set (dTheta suppressed)."""
        return bool(self._lib.sim_get_odometry_wedge_active(self._h))

    def get_odometry_enc_omega_healthy(self) -> bool:
        """Return True when the encoder-omega health gate is enabled (033-003/005)."""
        return bool(self._lib.sim_get_odometry_enc_omega_healthy(self._h))

    def bench_otos_set_noise(self, noise_xy: float = 0.0,
                              noise_h: float = 0.0,
                              drift_rad_per_sec: float = 0.0) -> None:
        """Set BenchOtosSensor error model parameters.

        Args:
            noise_xy:         Per-tick linear noise sigma (fraction of arc dist).
            noise_h:          Per-tick yaw noise sigma (fraction of heading change).
            drift_rad_per_sec: Slow additive yaw drift, rad/s.
        """
        self._lib.sim_bench_otos_set_noise(
            self._h,
            ctypes.c_float(noise_xy),
            ctypes.c_float(noise_h),
            ctypes.c_float(drift_rad_per_sec),
        )

    # ------------------------------------------------------------------
    # Three-estimate pose accessors (047-002 / 047-005)
    # ------------------------------------------------------------------

    def get_enc_pose(self) -> tuple[float, float, float]:
        """Return (x_mm, y_mm, h_rad) from the encoder dead-reckoning accumulator.

        This path is NEVER overwritten by EKF fusion — it is the pure
        encoder-only dead-reckoned estimate (state.actual.encoder.pose).
        """
        x = float(self._lib.sim_get_enc_pose_x(self._h))
        y = float(self._lib.sim_get_enc_pose_y(self._h))
        h = float(self._lib.sim_get_enc_pose_h(self._h))
        return (x, y, h)

    def get_optical_pose(self) -> tuple[float, float, float]:
        """Return (x_mm, y_mm, h_rad) from the raw OTOS optical estimate.

        This is the last value written by otosCorrect() before the EKF
        correction step (state.actual.optical.pose).
        """
        x = float(self._lib.sim_get_otos_pose_x(self._h))
        y = float(self._lib.sim_get_otos_pose_y(self._h))
        h = float(self._lib.sim_get_otos_pose_h(self._h))
        return (x, y, h)

    def get_fused_pose(self) -> tuple[float, float, float]:
        """Return (x_mm, y_mm, h_rad) from the EKF fused estimate.

        This is the EKF output (state.actual.fused.pose) — the authoritative
        belief that consumers such as GoTo use.  Equivalent to get_pose().
        """
        x = float(self._lib.sim_get_fused_pose_x(self._h))
        y = float(self._lib.sim_get_fused_pose_y(self._h))
        h = float(self._lib.sim_get_fused_pose_h(self._h))
        return (x, y, h)
