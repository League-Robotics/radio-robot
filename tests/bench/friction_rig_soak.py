#!/usr/bin/env python3
"""friction_rig_soak.py — sprint 078's hardware acceptance soak (ticket 005).

Exercises the sprint's write-path armor (`Hal::Motor`'s zero-dwell reversal,
output deadband, guarded resets, motion-qualified wedge reporting —
`source/hal/capability/motor.h`) against the real coupled friction rig
(brick ports 3/4 by default — two motors whose wheels are in FRICTION
contact, per `pid_hold_speed.py`'s docstring; friction only transmits load
when BOTH motors are spinning). Follows `dev_exercise.py`'s conventions:
widen `DEV WD` at session start, restore + `DEV STOP` in a `finally` block,
retry-tolerant `dev_send()` for the known USB/radio drop-rate, CSV +
transcript output per run (`tests/bench/out/`, gitignored per this
project's existing bench-artifact convention).

Three phases, run in one session:

1. **Hot-flip soak, A/B bracketed** (SUC-001/SUC-004): `--test-port`
   (default 3) alternates commanded duty +/-`--duty` (default 40%, the
   30-50% band the ticket specifies) while `--load-port` (default 4) holds
   a constant duty the whole time to keep the friction contact engaged.
   Two arms:
     - **control** (`dwell=0`) — the architecture's own definition of
       "legacy": `Hal::Motor::armoredWrite()`'s dwell branch is skipped
       entirely at `dwell=0`, so a detected reversal falls straight to an
       immediate `writeRawDuty()` call — reproducing sprint-077's shipped
       behavior byte-for-byte (see the code comment on `armoredWrite()`).
       Note this is NOT a raw, unclamped H-bridge slam: the pre-existing
       `|deltaPWM|` slew cap (064-002, `source/hal/nezha/motor_slew.h`,
       default 25 counts/write) still ramps ANY write regardless of dwell
       setting, so `dwell=0` tests "sprint-077's already-slew-mitigated
       legacy", not the fully raw pre-064-002 flip the original wedgelab
       campaign characterized. A clean control-arm run is therefore
       ambiguous between "these motor units are immune" and "the
       independent slew cap alone already provides enough protection at
       this duty" — the results section below must not conflate the two.
     - **treatment** (`dwell=100`, the ship default) — the sprint's new
       armor, exercised through the same flip schedule.
   Latches are detected from `wsus=` (motion-qualified wedge-suspect),
   NEVER the raw `wedged=` flag (an idle `wedged=1` between flips is
   benign per SUC-003 — see `docs/protocol-v2.md` §16). On a detected
   `wsus` transition (0 -> 1), the script recovers in-band: neutralizes
   both rig motors, waits for genuine rest, issues `RESET` (expects the
   hard path — `hrc=` increments), confirms recovery, then resumes the
   flip schedule (the recovery itself is not counted as one of the
   `--flips` commanded flips).

2. **Mid-motion / at-rest RESET guard check** (SUC-002): drives
   `--test-port`, immediately issues `RESET` while it is moving, and
   asserts the SOFT path fired (`src=` +1, `hrc=` unchanged, `pos=` ~0
   right after). Then neutralizes, waits for genuine standstill
   (`kRestVelocity`/`kRestTicksRequired`, `source/hal/capability/
   motor.h`), issues `RESET` again, and asserts the HARD path fired
   (`hrc=` +1).

3. Every session ends with `DEV STOP` (and the watchdog window restored),
   in a `finally` block — on a clean run, an assertion failure, an
   unhandled exception, or Ctrl-C.

Usage:
    uv run python tests/bench/friction_rig_soak.py
    uv run python tests/bench/friction_rig_soak.py --port /dev/cu.usbmodem2121102 \\
        --test-port 3 --load-port 4 --duty 40 --flips 150
    uv run python tests/bench/friction_rig_soak.py --skip-soak   # reset-guard check only
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, ParsedResponse, parse_response

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
SESSION_WATCHDOG_WINDOW = 5000    # [ms] widened for the whole (long) session
BOOT_WATCHDOG_WINDOW = 1000       # [ms] firmware default -- restored on exit
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
_OUT_DIR = _REPO_ROOT / "tests" / "bench" / "out"

DEFAULT_SHIP_DWELL = 100.0      # [ms] Hal::Motor::kDefaultReversalDwell
DEFAULT_SHIP_DEADBAND = 0.03    # [-1,1] Hal::Motor::kDefaultOutputDeadband


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default {DEFAULT_PORT})")
    p.add_argument("--test-port", type=int, default=3, choices=(1, 2, 3, 4),
                   help="Motor port whose duty is flipped +/- (default 3)")
    p.add_argument("--load-port", type=int, default=4, choices=(1, 2, 3, 4),
                   help="Motor port held at a constant duty to keep the "
                        "friction contact engaged (default 4)")
    p.add_argument("--duty", type=float, default=40.0,
                   help="Flip amplitude, percent -100..100 (default 40, "
                        "within the ticket's 30-50%% band)")
    p.add_argument("--load-duty", type=float, default=30.0,
                   help="Constant --load-port duty, percent (default 30)")
    p.add_argument("--flips", type=int, default=150,
                   help="Commanded sign flips per arm (default 150, matching "
                        "the wedgelab campaign's soak precedent; >=100 required "
                        "for acceptance)")
    p.add_argument("--settle-time", type=float, default=0.8,
                   help="Seconds to hold each flip before polling STATE (default 0.8)")
    p.add_argument("--deadband", type=float, default=DEFAULT_SHIP_DEADBAND,
                   help=f"output_deadband applied in BOTH arms (default {DEFAULT_SHIP_DEADBAND:g}, ship default)")
    p.add_argument("--control-dwell", type=float, default=0.0,
                   help="reversal_dwell for the control arm (default 0 -- explicit legacy)")
    p.add_argument("--treatment-dwell", type=float, default=DEFAULT_SHIP_DWELL,
                   help=f"reversal_dwell for the treatment arm (default {DEFAULT_SHIP_DWELL:g}, ship default)")
    p.add_argument("--rest-settle-time", type=float, default=2.0,
                   help="Seconds to wait for genuine standstill before an at-rest RESET (default 2.0)")
    p.add_argument("--mid-motion-settle", type=float, default=1.5,
                   help="Seconds to let the motor spin up before the mid-motion RESET check (default 1.5)")
    p.add_argument("--pos-zero-tolerance", type=float, default=10.0,
                   help="Acceptable |pos| right after the AT-REST RESET, deg (default 10 "
                        "-- the motor is genuinely stopped by this point, so this can be tight)")
    p.add_argument("--pos-drift-slack", type=float, default=30.0,
                   help="Slack, deg, added to (elapsed-since-RESET * pre-reset velocity) "
                        "for the MID-MOTION RESET's timing-robust 'rebased to ~0' check "
                        "(default 30 -- see run_reset_guard_check()'s comment)")
    p.add_argument("--skip-soak", action="store_true", help="Skip the hot-flip A/B soak")
    p.add_argument("--skip-reset-guard", action="store_true", help="Skip the RESET guard check")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Small helpers (same shape as dev_exercise.py's / pid_hold_speed.py's --
# kept local; these scripts are standalone CLI tools, not a shared library).
# ---------------------------------------------------------------------------

def dev_send(proto: NezhaProtocol, cmd: str, timeout: int = 500,  # [ms]
            retries: int = 6) -> ParsedResponse | None:
    """Send one DEV command, retrying on a totally silent reply.

    See dev_exercise.py's dev_send() docstring for the measured USB/radio
    burst-drop-rate rationale this mirrors. Safe to retry unconditionally for
    a pure query (STATE) or an idempotent absolute-value write (DUTY/NEUTRAL/
    CFG/WD/STOP) -- resending an unacknowledged one just re-applies the same
    value. `RESET` is the one exception found during this ticket's bench
    pass: it is NOT idempotent at the counter/position level -- each
    ACCEPTED `RESET` dispatches a real soft-rebaseline-or-hard-reset on the
    next tick (increments `src=`/`hrc=` and rebases `pos=` off whatever the
    motor's position is at that moment), so if the command lands but only
    its `OK` reply is dropped, blindly resending it double-fires the reset
    (observed live: one dev_send() call consumed 8 corr-ids across silent
    internal retries and left `src=` incremented by 2, not 1, with `pos=`
    reading a full ~60 mm off "immediately after reset" -- the second,
    retried RESET actually landed a few hundred ms later while the motor
    kept spinning). Call sites that assert an exact hrc=/src= delta pass
    `retries=1` for this reason; a `None` result there means "retry the
    whole check", not "safe to resend".
    """
    for attempt in range(retries):
        resp = proto.send(cmd, timeout)
        for raw in resp.get("responses", []):
            r = parse_response(raw)
            if r is not None and r.tag in ("OK", "ERR"):
                return r
        if attempt < retries - 1:
            time.sleep(0.1)
    return None


def _fmt(r: ParsedResponse | None) -> str:
    if r is None:
        return "(no reply)"
    return r.raw


def read_motor_state(proto: NezhaProtocol, port: int) -> dict | None:
    """DEV M <port> STATE, parsed into the eight documented fields.

    Returns None on a dropped reply or an unparsable field -- callers must
    treat that as "no sample" (recorded as a blank CSV row), never as "wedge
    cleared"/"wedge present".
    """
    st = dev_send(proto, f"DEV M {port} STATE")
    if st is None or st.tag != "OK":
        return None
    try:
        return {
            "pos": float(st.kv["pos"]),
            "vel": float(st.kv["vel"]),
            "applied": float(st.kv["applied"]),
            "wedged": int(st.kv["wedged"]),
            "wsus": int(st.kv["wsus"]),
            "hrc": int(st.kv["hrc"]),
            "src": int(st.kv["src"]),
        }
    except (KeyError, ValueError):
        return None


class Transcript:
    """Tees every logged line to stdout AND to a session transcript file."""

    def __init__(self, path: pathlib.Path) -> None:
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(path, "w")

    def log(self, msg: str = "") -> None:
        print(msg)
        self._fh.write(msg + "\n")
        self._fh.flush()

    def close(self) -> None:
        self._fh.close()


# ---------------------------------------------------------------------------
# Phase 1: hot-flip soak (one arm).
# ---------------------------------------------------------------------------

def run_hot_flip_arm(proto: NezhaProtocol, tx: Transcript, args: argparse.Namespace,
                      label: str, dwell: float, csv_path: pathlib.Path) -> dict:
    cfg_resp = dev_send(proto, f"DEV M {args.test_port} CFG dwell={dwell} deadband={args.deadband}")
    tx.log(f"    CFG dwell={dwell:g} deadband={args.deadband:g} -> {_fmt(cfg_resp)}")

    dev_send(proto, f"DEV M {args.load_port} DUTY {args.load_duty}")
    # Let both motors reach speed before the flip schedule starts -- friction
    # coupling only transmits load when BOTH are spinning (pid_hold_speed.py).
    time.sleep(2.0)

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["t", "arm", "flip_i", "commanded_duty", "pos", "vel",
                      "applied", "wedged", "wsus", "hrc", "src", "event"])

    t0 = time.monotonic()
    baseline = read_motor_state(proto, args.test_port)
    prev_wsus = baseline["wsus"] if baseline is not None else 0

    episodes: list[dict] = []
    flips_done = 0
    i = 0
    try:
        while flips_done < args.flips:
            duty = args.duty if i % 2 == 0 else -args.duty
            dev_send(proto, f"DEV M {args.test_port} DUTY {duty}")
            time.sleep(args.settle_time)
            st = read_motor_state(proto, args.test_port)
            row_t = time.monotonic() - t0

            if st is None:
                tx.log(f"    [{label}] flip {i:3d} duty={duty:+.0f}  (STATE poll failed -- no reply)")
                writer.writerow([f"{row_t:.3f}", label, i, duty, "", "", "", "", "", "", "", "poll_failed"])
                i += 1
                flips_done += 1
                continue

            event = ""
            tx.log(f"    [{label}] flip {i:3d} duty={duty:+.0f}  pos={st['pos']:8.1f}  vel={st['vel']:7.1f}"
                   f"  applied={st['applied']:+.2f}  wedged={st['wedged']}  wsus={st['wsus']}"
                   f"  hrc={st['hrc']}  src={st['src']}")

            if st["wsus"] == 1 and prev_wsus == 0:
                event = "latch_detected"
                episodes.append({"flip": i, "pos_at_latch": st["pos"]})
                tx.log(f"    *** [{label}] LATCH DETECTED at flip {i} (wsus 0->1) -- recovering ***")
                writer.writerow([f"{row_t:.3f}", label, i, duty, st["pos"], st["vel"],
                                  st["applied"], st["wedged"], st["wsus"], st["hrc"], st["src"], event])

                dev_send(proto, f"DEV M {args.test_port} NEUTRAL B")
                dev_send(proto, f"DEV M {args.load_port} NEUTRAL B")
                time.sleep(args.rest_settle_time)
                reset_resp = dev_send(proto, f"DEV M {args.test_port} RESET", retries=1)
                time.sleep(0.3)
                st_after = read_motor_state(proto, args.test_port)
                recovered = (st_after is not None
                             and st_after["hrc"] == st["hrc"] + 1
                             and abs(st_after["pos"]) <= args.pos_zero_tolerance)
                tx.log(f"    recovery: RESET->{_fmt(reset_resp)}  post-state={st_after}"
                       f"  recovered={'YES' if recovered else 'NO'}")
                episodes[-1]["recovered"] = recovered

                dev_send(proto, f"DEV M {args.load_port} DUTY {args.load_duty}")
                time.sleep(1.0)
                prev_wsus = 0
                i += 1
                flips_done += 1
                continue

            writer.writerow([f"{row_t:.3f}", label, i, duty, st["pos"], st["vel"],
                              st["applied"], st["wedged"], st["wsus"], st["hrc"], st["src"], event])
            prev_wsus = st["wsus"]
            i += 1
            flips_done += 1
    finally:
        dev_send(proto, f"DEV M {args.test_port} NEUTRAL B")
        dev_send(proto, f"DEV M {args.load_port} NEUTRAL B")
        csv_file.close()

    return {"label": label, "dwell": dwell, "flips": flips_done,
            "episodes": episodes, "csv": str(csv_path)}


# ---------------------------------------------------------------------------
# Phase 2: mid-motion / at-rest RESET guard check.
# ---------------------------------------------------------------------------

def run_reset_guard_check(proto: NezhaProtocol, tx: Transcript, args: argparse.Namespace,
                           csv_path: pathlib.Path) -> dict:
    dev_send(proto, f"DEV M {args.test_port} CFG dwell={args.treatment_dwell} deadband={args.deadband}")

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["t", "phase", "step", "pos", "vel", "applied", "hrc", "src"])
    t0 = time.monotonic()

    def snapshot(phase: str, step: str) -> dict | None:
        st = read_motor_state(proto, args.test_port)
        row_t = time.monotonic() - t0
        if st is not None:
            writer.writerow([f"{row_t:.3f}", phase, step, st["pos"], st["vel"],
                              st["applied"], st["hrc"], st["src"]])
            tx.log(f"    [{phase}/{step}] pos={st['pos']:.1f}  vel={st['vel']:.1f}"
                   f"  applied={st['applied']:+.2f}  hrc={st['hrc']}  src={st['src']}")
        else:
            tx.log(f"    [{phase}/{step}] STATE poll failed -- no reply")
        return st

    try:
        # --- Mid-motion RESET: expect the SOFT path (src +1, hrc unchanged). ---
        dev_send(proto, f"DEV M {args.test_port} DUTY {args.duty}")
        time.sleep(args.mid_motion_settle)
        mid_baseline = snapshot("mid_motion", "baseline")
        t_reset_sent = time.monotonic()
        reset_resp = dev_send(proto, f"DEV M {args.test_port} RESET", retries=1)
        tx.log(f"    RESET (mid-motion) -> {_fmt(reset_resp)}")
        time.sleep(0.3)
        mid_after = snapshot("mid_motion", "post_reset")
        # Deliberately do NOT neutralize before this snapshot: NEUTRAL is a
        # coast (not a real brake -- docs/protocol-v2.md §16's "Nezha has no
        # distinct brake register"), so stopping the motor first just adds
        # an uncontrolled coast-down drift on top of this bench's already-
        # documented USB CDC burst-drop-rate (both would otherwise inflate
        # "how far did pos= travel between the reset and the poll" by an
        # amount that has nothing to do with whether the rebaseline itself
        # worked). Instead, compare pos= against the drift a STILL-SPINNING
        # motor would rack up over the ACTUAL measured elapsed time since
        # RESET was dispatched (elapsed * the pre-reset velocity, plus
        # slack) -- this is timing-robust (scales with whatever the
        # transport's real round-trip turned out to be this call) and still
        # sharply distinguishes "rebased to ~0, then grew only by the
        # elapsed drift" from "no rebase happened at all" (which would read
        # close to `mid_baseline['pos']` plus that same drift -- hundreds of
        # mm bigger).
        elapsed = time.monotonic() - t_reset_sent
        expected_max_pos = (abs(mid_baseline["vel"]) * elapsed + args.pos_drift_slack
                             if mid_baseline is not None else None)
        if expected_max_pos is not None:
            tx.log(f"    elapsed since RESET dispatch: {elapsed:.2f}s"
                   f"  expected_max_|pos|={expected_max_pos:.1f}"
                   f" (vel_baseline={mid_baseline['vel']:.1f} + slack {args.pos_drift_slack:g})")
        mid_ok = (mid_baseline is not None and mid_after is not None
                  and expected_max_pos is not None
                  and mid_after["src"] == mid_baseline["src"] + 1
                  and mid_after["hrc"] == mid_baseline["hrc"]
                  and abs(mid_after["pos"]) <= expected_max_pos)
        tx.log(f"    mid-motion RESET soft-path check: {'PASS' if mid_ok else 'FAIL'}")

        # --- At-rest RESET: expect the HARD path (hrc +1). ---
        dev_send(proto, f"DEV M {args.test_port} NEUTRAL B")
        time.sleep(args.rest_settle_time)
        rest_baseline = snapshot("at_rest", "baseline")
        reset_resp2 = dev_send(proto, f"DEV M {args.test_port} RESET", retries=1)
        tx.log(f"    RESET (at-rest) -> {_fmt(reset_resp2)}")
        time.sleep(0.3)
        rest_after = snapshot("at_rest", "post_reset")
        rest_ok = (rest_baseline is not None and rest_after is not None
                   and rest_after["hrc"] == rest_baseline["hrc"] + 1
                   and abs(rest_after["pos"]) <= args.pos_zero_tolerance)
        tx.log(f"    at-rest RESET hard-path check: {'PASS' if rest_ok else 'FAIL'}")
    finally:
        dev_send(proto, f"DEV M {args.test_port} NEUTRAL B")
        csv_file.close()

    return {
        "mid_motion": {"baseline": mid_baseline, "after": mid_after, "pass": mid_ok},
        "at_rest": {"baseline": rest_baseline, "after": rest_after, "pass": rest_ok},
        "csv": str(csv_path),
    }


# ---------------------------------------------------------------------------
# Main.
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse_args()
    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    tx = Transcript(_OUT_DIR / "friction_rig_soak.transcript.log")
    tx.log(f"friction_rig_soak.py -- port={args.port}  test_port={args.test_port}  load_port={args.load_port}")
    tx.log(f"  duty={args.duty:g}%  load_duty={args.load_duty:g}%  flips/arm={args.flips}"
           f"  control_dwell={args.control_dwell:g}ms  treatment_dwell={args.treatment_dwell:g}ms"
           f"  deadband={args.deadband:g}")

    conn = SerialConnection(port=args.port)
    proto: NezhaProtocol | None = None
    control_result: dict | None = None
    treatment_result: dict | None = None
    reset_result: dict | None = None
    overall_pass = False

    try:
        info = conn.connect()
        if "error" in info:
            tx.log(f"ERROR: connect failed: {info['error']}")
            return 2
        tx.log(f"  connected: mode={info.get('mode')}")
        proto = NezhaProtocol(conn)

        wd = dev_send(proto, f"DEV WD {SESSION_WATCHDOG_WINDOW}")
        tx.log(f"  DEV WD {SESSION_WATCHDOG_WINDOW} -> {_fmt(wd)}")

        if not args.skip_soak:
            tx.log("\n=== CONTROL ARM (dwell=0, explicit legacy) ===")
            control_result = run_hot_flip_arm(proto, tx, args, "control", args.control_dwell,
                                               _OUT_DIR / "friction_rig_soak_control.csv")
            tx.log(f"  control arm: {len(control_result['episodes'])} latch episode(s)"
                   f" over {control_result['flips']} flips")

            tx.log(f"\n=== TREATMENT ARM (dwell={args.treatment_dwell:g}, armor default) ===")
            treatment_result = run_hot_flip_arm(proto, tx, args, "treatment", args.treatment_dwell,
                                                 _OUT_DIR / "friction_rig_soak_treatment.csv")
            tx.log(f"  treatment arm: {len(treatment_result['episodes'])} latch episode(s)"
                   f" over {treatment_result['flips']} flips")

        if not args.skip_reset_guard:
            tx.log("\n=== RESET-GUARD CHECK ===")
            reset_result = run_reset_guard_check(proto, tx, args,
                                                  _OUT_DIR / "friction_rig_soak_reset_guard.csv")

        treatment_clean = (args.skip_soak or (treatment_result is not None
                            and len(treatment_result["episodes"]) == 0
                            and treatment_result["flips"] >= 100))
        reset_ok = (args.skip_reset_guard or (reset_result is not None
                    and reset_result["mid_motion"]["pass"] and reset_result["at_rest"]["pass"]))
        overall_pass = treatment_clean and reset_ok

        tx.log("\n=== SUMMARY ===")
        tx.log(f"  motors used: test_port={args.test_port}  load_port={args.load_port}"
               f"  (physical unit history/susceptibility unknown prior to this session)")
        if control_result is not None:
            tx.log(f"  control   (dwell=0)                : {len(control_result['episodes'])} latch(es)"
                   f" / {control_result['flips']} flips   csv={control_result['csv']}")
        if treatment_result is not None:
            tx.log(f"  treatment (dwell={args.treatment_dwell:g})              : {len(treatment_result['episodes'])} latch(es)"
                   f" / {treatment_result['flips']} flips   csv={treatment_result['csv']}")
        if reset_result is not None:
            tx.log(f"  reset-guard mid-motion (soft path) : {'PASS' if reset_result['mid_motion']['pass'] else 'FAIL'}")
            tx.log(f"  reset-guard at-rest    (hard path)  : {'PASS' if reset_result['at_rest']['pass'] else 'FAIL'}")
            tx.log(f"  reset-guard csv: {reset_result['csv']}")
        if control_result is not None and len(control_result["episodes"]) == 0:
            tx.log("  CAVEAT: the control arm reproduced 0 latches. This is NOT proof the armor is what")
            tx.log("  prevents the trigger -- (a) susceptibility is motor-unit- and state-dependent")
            tx.log("  (docs/knowledge/2026-07-04-encoder-wedge.md: fresh motors were immune at every dose")
            tx.log("  in the wedgelab campaign), these specific units' history/hot-state is unknown, and")
            tx.log("  (b) the pre-existing +/-25 ΔPWM/write slew cap (064-002) ramps EVERY write regardless")
            tx.log("  of dwell, in BOTH arms, so a clean control run does not by itself isolate the new")
            tx.log("  dwell mechanism's own contribution from the slew cap's.")
        tx.log(f"\n  OVERALL: {'PASS' if overall_pass else 'FAIL'}")

    except KeyboardInterrupt:
        tx.log("\n  interrupted -- stopping motors...")
        overall_pass = False
    finally:
        if proto is not None:
            try:
                stop_resp = dev_send(proto, "DEV STOP")
                tx.log(f"\n  DEV STOP -> {_fmt(stop_resp)}")
            except Exception as exc:
                tx.log(f"  WARN: DEV STOP failed during cleanup: {exc}")
            try:
                wd_resp = dev_send(proto, f"DEV WD {BOOT_WATCHDOG_WINDOW}")
                tx.log(f"  DEV WD {BOOT_WATCHDOG_WINDOW} (restore) -> {_fmt(wd_resp)}")
            except Exception as exc:
                tx.log(f"  WARN: DEV WD restore failed during cleanup: {exc}")
        if conn.is_open:
            conn.disconnect()
        tx.close()

    return 0 if overall_pass else 1


if __name__ == "__main__":
    sys.exit(main())
