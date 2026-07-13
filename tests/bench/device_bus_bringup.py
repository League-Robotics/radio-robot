#!/usr/bin/env python3
"""device_bus_bringup.py — DB-009 HITL bench-gate script for the DeviceBus
bring-up firmware image (`source/devices/bringup_main.cpp`,
`codal.devicebus.json`).

DEFERRED THIS SESSION: this script is written and syntax/import-verified but
NOT run against real hardware in DB-009's own implementation pass — the bench
robot currently runs the verified sprint-099 firmware, and flashing this
bring-up image would displace it (hardware-serial with sprint 100's
cutover). It is ready to run as soon as a coordinated hardware session flashes
`codal.devicebus.json` (`cp codal.devicebus.json codal.json && uv run python3
build.py --fw-only`, then restore `codal.json` afterward — see this ticket's
own report for the exact sequence) and `mbdeploy deploy`s the resulting
`MICROBIT.hex`.

Talks to bringup_main.cpp's OWN bespoke DEV grammar (see that file's header
comment for the full command list) over `robot_radio.io.serial_conn.
SerialConnection(mode="direct")` directly — NOT `NezhaProtocol`/
`parse_response()` (host/robot_radio/robot/protocol.py), which assume the
FULL v2 wire protocol (`PROTO_VERSION` handshake, `ID`/`VER` banner shape,
the DEV family text_channel.cpp implements) that this minimal bring-up image
does not carry. `SerialConnection.send()` already appends the `#<corr_id>`
suffix bringup_main.cpp's own parser strips-and-echoes, so no protocol.py
dependency is needed at all — `parse_kv()` below is this script's own tiny
reply parser, matching bringup_main.cpp's own grammar exactly.

Implements the issue's ("clasi/issues/device-bus-fiber-owned-self-contained-
device-subsystem.md", "Bench gates") five DB-009-scoped gates (gate 6,
098/099 motion non-regression, is explicitly out of scope for this bring-up
image — issue: "cutover-time, out of scope here"):

  1. dual-per-motorId encoder-request pipelining probe — drives both motors,
     watches `M <port> STATE`'s `glitch=`/`wedged=` fields for corruption and
     confirms both wheels actually moved.
  2. `fiber_sleep(4)` actual-latency distribution — approximated from the
     motor ring's own published stamps (`M <port> RING <age>`'s `t=` field,
     one publish per fiber cycle). This is a PROXY, not an isolated
     measurement of the 4 ms settle sleep alone: bringup_main.cpp's minimal
     DEV grammar (deliberately scoped to "stage targets / toggle PID / read
     handles / dump rings" — device-bus-tickets.md's DB-009 description) has
     no dedicated settle-latency instrumentation verb, so the best a
     black-box serial client can observe is the WHOLE cycle period (settle
     sleep + collect + perceive + publish + pace sleep) via consecutive ring
     timestamps. Reported honestly as "cycle-period distribution," not
     re-labeled as the settle sleep itself.
  3. reversal-stress armor re-verification — alternates +/- VEL on one motor
     and confirms no persistent wedge latch and bounded glitch growth.
  4. serial health — a burst-ping loss-rate probe. bringup_main.cpp's DEV
     parser is TEXT-only (no `*B<base64>` binary plane at all — that is a
     legacy-stack-only feature, source/commands/binary_channel.cpp,
     explicitly excluded from this bring-up image's minimal scope per DB-009's
     own ticket description: "uses only Devices:: + CODAL"), so the project's
     usual binary-vs-text same-boot discriminator has nothing to compare
     against here. This gate instead measures the text plane's own
     burst-reply loss rate as the "no worse than the per-transaction
     IRQ-guard baseline" health signal the ticket's acceptance criteria asks
     for.
  5. flash/RAM footprint — reads `arm-none-eabi-size` on the built ELF
     directly; needs no serial connection at all (runs even with
     `--flash-only`, or first, before any hardware gate). RAM is reported but
     never gates (this target's RAM sits ~98% full BY DESIGN — the project's
     own `codal-ram-always-near-full` finding); only flash overflow (which
     would already fail the link step before this script ever runs) is a
     genuine failure.

Safety (`.claude/rules/hardware-bench-testing.md`): the `finally` block
always sends `STOP` (bringup_main.cpp's own lightweight safety verb — both
motors to `Neutral::Coast` without tearing down the fiber) and disconnects,
on a clean run, an assertion failure, an exception, or Ctrl-C. Motors must
never be left running.

Usage (once flashed on the bench):
    uv run python tests/bench/device_bus_bringup.py
    uv run python tests/bench/device_bus_bringup.py --port /dev/cu.usbmodem2121102
    uv run python tests/bench/device_bus_bringup.py --flash-only   # gate 5 only, no robot needed
"""

from __future__ import annotations

import argparse
import statistics
import subprocess
import sys
import time
from pathlib import Path

from robot_radio.io.serial_conn import SerialConnection

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
DEFAULT_ELF = "build/MICROBIT"

# nRF52833 usable budget this project's own DB-009 verification build
# reported (codal-nrf52's linker map: 364 KB FLASH region; 122816 B usable
# RAM after BOOTLOADER/SETTINGS/UICR/NOINIT reservations). Informational
# percentage denominator only — see flash_ram_gate_pass()'s own docstring
# for why RAM never gates.
FLASH_BUDGET_BYTES = 364 * 1024
RAM_BUDGET_BYTES = 122816


# ---------------------------------------------------------------------------
# Pure helpers — no I/O, no hardware. Unit-tested directly by
# tests/unit/test_device_bus_bringup_bench.py.
# ---------------------------------------------------------------------------

def parse_kv(line: str) -> dict | None:
    """Parse one bringup_main.cpp reply line: ``<TAG> [tok...] [#<id>]``.

    ``TAG`` is ``OK`` or ``ERR`` (bringup_main.cpp's ReplyBuilder always
    starts a reply with one of these two words — its own header comment's
    grammar table). Trailing ``key=value`` tokens populate ``kv``; any other
    bare token (e.g. PING's ``pong``) is collected into ``tokens``. A
    trailing ``#<digits>`` correlation-id suffix (bringup_main.cpp's own
    ``stripCorrId()``/echo — matches ``SerialConnection.send()``'s own
    ``f"{message} #{corr_id}"`` framing) is split into ``corr_id`` and
    excluded from both. Returns ``None`` if `line` doesn't start with a
    recognized tag (e.g. an empty string, a relay comment line the reader
    thread should have already dropped, or garbage).
    """
    if not line:
        return None
    parts = line.strip().split()
    if not parts:
        return None
    tag = parts[0]
    if tag not in ("OK", "ERR"):
        return None
    rest = parts[1:]
    corr_id = None
    if rest and rest[-1].startswith("#"):
        corr_id = rest[-1][1:]
        rest = rest[:-1]
    kv: dict[str, str] = {}
    tokens: list[str] = []
    for tok in rest:
        if "=" in tok:
            k, _, v = tok.partition("=")
            kv[k] = v
        else:
            tokens.append(tok)
    return {"tag": tag, "tokens": tokens, "kv": kv, "corr_id": corr_id, "raw": line}


def kv_float(parsed: dict | None, key: str) -> float | None:
    if parsed is None or key not in parsed["kv"]:
        return None
    try:
        return float(parsed["kv"][key])
    except ValueError:
        return None


def kv_int(parsed: dict | None, key: str) -> int | None:
    if parsed is None or key not in parsed["kv"]:
        return None
    try:
        return int(parsed["kv"][key])
    except ValueError:
        return None


def compute_deltas(stamps: list[int]) -> list[int]:
    """Consecutive positive deltas [us] from a NEWEST-FIRST list of ring
    stamps (e.g. ``M <port> RING`` queried for age=0..4 in that order, the
    order this script's gate_cycle_latency() queries in). Fewer than two
    valid stamps yields ``[]``.
    """
    return [stamps[i] - stamps[i + 1] for i in range(len(stamps) - 1)]


def summarize_deltas(deltas: list[int]) -> dict | None:
    """min/max/mean/stdev [us] of `deltas`, or ``None`` if empty."""
    if not deltas:
        return None
    return {
        "n": len(deltas),
        "min_us": min(deltas),
        "max_us": max(deltas),
        "mean_us": statistics.fmean(deltas),
        "stdev_us": statistics.pstdev(deltas) if len(deltas) > 1 else 0.0,
    }


def loss_rate(sent: int, received: int) -> float:
    """Fraction of `sent` requests that did NOT receive a reply. 0.0 if
    `sent` is 0 (nothing attempted, nothing lost)."""
    if sent <= 0:
        return 0.0
    return 1.0 - (received / sent)


def pipelining_gate_pass(
    glitch1_before: int | None, glitch1_after: int | None,
    glitch2_before: int | None, glitch2_after: int | None,
    pos_delta1: float | None, pos_delta2: float | None,
    wedge_seen: bool,
    min_pos_delta: float = 5.0,  # [mm] — "both wheels actually moved"
    max_glitch_growth: int = 2,
) -> bool:
    """Gate 1 pass/fail: no wedge during the probe, bounded encGlitchCount
    growth on BOTH motors (a corrupted/mispaired pipelined request would show
    up as rejected-sample growth — nezha_motor.cpp's own outlier-rejection
    gate), and both wheels moved at least `min_pos_delta`."""
    if wedge_seen:
        return False
    for before, after in ((glitch1_before, glitch1_after), (glitch2_before, glitch2_after)):
        if before is None or after is None:
            return False
        if (after - before) > max_glitch_growth:
            return False
    for delta in (pos_delta1, pos_delta2):
        if delta is None or delta < min_pos_delta:
            return False
    return True


def reversal_gate_pass(
    wedge_seen_during: bool,
    wedged_after: int | None,
    glitch_before: int | None,
    glitch_after: int | None,
    max_glitch_growth: int = 3,
) -> bool:
    """Gate 3 pass/fail: no persistent wedge latch left behind (`wedged_after
    == 0`; a TRANSIENT latch during a hard reversal is expected armor
    behavior — see the project's own encoder-wedge-boundary-latch finding —
    so `wedge_seen_during` is recorded but NOT itself a failure), and bounded
    glitch growth across the whole reversal-stress sequence."""
    if wedged_after != 0:
        return False
    if glitch_before is None or glitch_after is None:
        return False
    return (glitch_after - glitch_before) <= max_glitch_growth


def loss_gate_pass(rate: float, max_loss: float = 0.05) -> bool:
    """Gate 4 pass/fail: burst reply-loss rate at or below `max_loss`."""
    return rate <= max_loss


def flash_ram_gate_pass(
    flash_bytes: int, ram_bytes: int,
    flash_budget: int = FLASH_BUDGET_BYTES,
    ram_budget: int = RAM_BUDGET_BYTES,
) -> bool:
    """Gate 5 pass/fail: flash within budget. RAM is reported (see the
    caller) but never gates — this target runs ~98% RAM-full BY DESIGN
    (project's own codal-ram-always-near-full finding); only flash overflow
    is a real limit, and it would already fail the link step long before
    this script runs, so this check is a headroom report, not a novel gate.
    `ram_budget` is accepted only so callers can format a percentage.
    """
    del ram_bytes, ram_budget  # informational only — see docstring
    return flash_bytes <= flash_budget


# ---------------------------------------------------------------------------
# Result — accumulates (name, passed, detail) checks, mirrors
# tests/bench/dev_exercise.py's own Result class.
# ---------------------------------------------------------------------------

class Result:
    def __init__(self) -> None:
        self.checks: list[tuple[str, bool, str]] = []

    def record(self, name: str, passed: bool, detail: str = "") -> None:
        self.checks.append((name, passed, detail))
        mark = "PASS" if passed else "FAIL"
        suffix = f"  ({detail})" if detail else ""
        print(f"  [{mark}] {name}{suffix}")

    def summary(self) -> bool:
        total = len(self.checks)
        passed = sum(1 for _, ok, _ in self.checks if ok)
        print(f"\n{passed}/{total} checks passed")
        for name, ok, detail in self.checks:
            if not ok:
                print(f"  FAILED: {name}" + (f" — {detail}" if detail else ""))
        return passed == total


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def dev_send(conn: SerialConnection, cmd: str, timeout: int = 500,  # [ms]
              retries: int = 5) -> dict | None:
    """Send one bringup_main.cpp DEV-grammar line, return the first parsed
    OK/ERR reply, or ``None`` on total silence after `retries` attempts.

    Retries on silence for the same reason tests/bench/dev_exercise.py's own
    ``dev_send()`` does (its own docstring): this bench's direct-USB link
    intermittently drops replies under load, independent of firmware
    processing time. Every command this script sends is either a pure query
    or an idempotent absolute-value write, so a blind resend on silence is
    always safe.
    """
    for attempt in range(retries):
        resp = conn.send(cmd, timeout)
        for raw in resp.get("responses", []):
            parsed = parse_kv(raw)
            if parsed is not None:
                return parsed
        if attempt < retries - 1:
            time.sleep(0.1)
    return None


# ---------------------------------------------------------------------------
# Gates
# ---------------------------------------------------------------------------

def gate_pipelining(conn: SerialConnection, result: Result, args: argparse.Namespace) -> None:
    print("\n[gate 1] dual-per-motorId encoder-request pipelining probe")
    s1_0 = dev_send(conn, "M 1 STATE")
    s2_0 = dev_send(conn, "M 2 STATE")
    glitch1_0, glitch2_0 = kv_int(s1_0, "glitch"), kv_int(s2_0, "glitch")
    pos1_0, pos2_0 = kv_float(s1_0, "pos"), kv_float(s2_0, "pos")

    dev_send(conn, f"M 1 VEL {args.pipelining_vel}")
    dev_send(conn, f"M 2 VEL {args.pipelining_vel}")

    wedge_seen = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.pipelining_duration:
        s1 = dev_send(conn, "M 1 STATE")
        s2 = dev_send(conn, "M 2 STATE")
        if kv_int(s1, "wedged") == 1 or kv_int(s2, "wedged") == 1:
            wedge_seen = True
        time.sleep(0.05)

    dev_send(conn, "M 1 NEUTRAL C")
    dev_send(conn, "M 2 NEUTRAL C")
    time.sleep(0.2)

    s1_1 = dev_send(conn, "M 1 STATE")
    s2_1 = dev_send(conn, "M 2 STATE")
    glitch1_1, glitch2_1 = kv_int(s1_1, "glitch"), kv_int(s2_1, "glitch")
    pos1_1, pos2_1 = kv_float(s1_1, "pos"), kv_float(s2_1, "pos")
    delta1 = None if pos1_0 is None or pos1_1 is None else abs(pos1_1 - pos1_0)
    delta2 = None if pos2_0 is None or pos2_1 is None else abs(pos2_1 - pos2_0)

    passed = pipelining_gate_pass(
        glitch1_0, glitch1_1, glitch2_0, glitch2_1, delta1, delta2, wedge_seen,
        min_pos_delta=args.pipelining_min_delta,
    )
    result.record(
        "gate 1: pipelining probe (no wedge, bounded glitch growth, both wheels moved)",
        passed,
        f"glitch1 {glitch1_0}->{glitch1_1} glitch2 {glitch2_0}->{glitch2_1} "
        f"delta1={delta1} delta2={delta2} wedge_seen={wedge_seen}",
    )


def gate_cycle_latency(conn: SerialConnection, result: Result, args: argparse.Namespace) -> None:
    print("\n[gate 2] fiber_sleep(4)/cycle-period distribution (ring-stamp deltas)")
    dev_send(conn, f"M {args.motor} VEL {args.pipelining_vel}")
    time.sleep(0.3)  # let a few cycles' worth of ring history accumulate

    stamps: list[int] = []
    for age in range(5):
        s = dev_send(conn, f"M {args.motor} RING {age}")
        if kv_int(s, "valid") == 1:
            t = kv_int(s, "t")
            if t is not None:
                stamps.append(t)

    dev_send(conn, f"M {args.motor} NEUTRAL C")

    deltas = compute_deltas(stamps)
    stats = summarize_deltas(deltas)
    print(f"  ring stamps (us, newest-first): {stamps}")
    print(f"  cycle-period deltas (us): {deltas}")
    print(f"  stats: {stats}")

    passed = stats is not None and stats["min_us"] > 0
    result.record(
        "gate 2: cycle-period distribution captured "
        "(proxy for fiber_sleep(4) settle latency — see this script's own module "
        "docstring for why this is a whole-cycle proxy, not an isolated measurement)",
        passed,
        str(stats),
    )


def gate_reversal_stress(conn: SerialConnection, result: Result, args: argparse.Namespace) -> None:
    print("\n[gate 3] reversal-stress armor re-verification")
    s0 = dev_send(conn, f"M {args.motor} STATE")
    glitch0 = kv_int(s0, "glitch")

    wedge_seen_during = False
    for i in range(args.reversal_reps):
        vel = args.reversal_vel if i % 2 == 0 else -args.reversal_vel
        dev_send(conn, f"M {args.motor} VEL {vel}")
        time.sleep(args.reversal_dwell)
        st = dev_send(conn, f"M {args.motor} STATE")
        if kv_int(st, "wedged") == 1:
            wedge_seen_during = True

    dev_send(conn, f"M {args.motor} NEUTRAL C")
    time.sleep(0.3)  # let the armor's own reversal dwell/rest-tracking settle

    s1 = dev_send(conn, f"M {args.motor} STATE")
    glitch1 = kv_int(s1, "glitch")
    wedged_after = kv_int(s1, "wedged")

    passed = reversal_gate_pass(
        wedge_seen_during, wedged_after, glitch0, glitch1,
        max_glitch_growth=args.reversal_max_glitch_growth,
    )
    result.record(
        "gate 3: reversal-stress (no persistent wedge, bounded glitch growth)",
        passed,
        f"wedge_seen_during={wedge_seen_during} wedged_after={wedged_after} "
        f"glitch {glitch0}->{glitch1}",
    )


def gate_serial_health(conn: SerialConnection, result: Result, args: argparse.Namespace) -> None:
    print("\n[gate 4] serial health — burst-ping loss-rate probe")
    sent = 0
    received = 0
    for _ in range(args.serial_burst):
        sent += 1
        resp = conn.send("PING", args.serial_timeout, stop_token="OK pong")
        if any(parse_kv(line) is not None for line in resp.get("responses", [])):
            received += 1
    rate = loss_rate(sent, received)
    passed = loss_gate_pass(rate, max_loss=args.serial_max_loss)
    result.record(
        f"gate 4: serial burst-loss ({sent} pings, text plane only — no binary "
        "plane on this bring-up image, see this script's own module docstring)",
        passed,
        f"received={received} loss_rate={rate:.1%} threshold={args.serial_max_loss:.1%}",
    )


def gate_flash_ram(result: Result, args: argparse.Namespace) -> None:
    print("\n[gate 5] flash/RAM footprint")
    elf = Path(args.elf)
    if not elf.exists():
        result.record("gate 5: flash/RAM footprint", False,
                        f"ELF not found: {elf} (build the bring-up image first)")
        return
    try:
        out = subprocess.run(["arm-none-eabi-size", str(elf)],
                               capture_output=True, text=True, check=True).stdout
    except Exception as exc:  # noqa: BLE001 — report, don't crash the bench run
        result.record("gate 5: flash/RAM footprint", False,
                        f"arm-none-eabi-size failed: {exc}")
        return

    lines = out.strip().splitlines()
    if len(lines) < 2:
        result.record("gate 5: flash/RAM footprint", False,
                        f"unexpected `size` output: {out!r}")
        return

    text_b, data_b, bss_b = (int(x) for x in lines[1].split()[:3])
    flash_bytes = text_b + data_b
    ram_bytes = data_b + bss_b
    passed = flash_ram_gate_pass(flash_bytes, ram_bytes)
    result.record(
        "gate 5: flash/RAM footprint",
        passed,
        f"flash={flash_bytes}B ({flash_bytes / FLASH_BUDGET_BYTES:.1%} of "
        f"{FLASH_BUDGET_BYTES}B budget)  ram={ram_bytes}B "
        f"({ram_bytes / RAM_BUDGET_BYTES:.1%} of {RAM_BUDGET_BYTES}B — expected "
        "near-full by design, informational only, never gates)",
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default {DEFAULT_PORT})")
    p.add_argument("--motor", type=int, default=1, choices=(1, 2),
                    help="Motor port for gates 2/3 (default 1)")
    p.add_argument("--elf", default=DEFAULT_ELF,
                    help=f"Bring-up image ELF for gate 5 (default {DEFAULT_ELF})")

    p.add_argument("--pipelining-vel", type=float, default=150.0,
                    help="Gate 1/2 VEL target, mm/s (default 150)")
    p.add_argument("--pipelining-duration", type=float, default=2.0,
                    help="Gate 1 drive duration, seconds (default 2)")
    p.add_argument("--pipelining-min-delta", type=float, default=5.0,
                    help="Gate 1 minimum |position delta| to count as moved, mm (default 5)")

    p.add_argument("--reversal-vel", type=float, default=150.0,
                    help="Gate 3 alternating VEL magnitude, mm/s (default 150)")
    p.add_argument("--reversal-reps", type=int, default=10,
                    help="Gate 3 number of +/- reversals (default 10)")
    p.add_argument("--reversal-dwell", type=float, default=0.4,
                    help="Gate 3 seconds held per reversal (default 0.4)")
    p.add_argument("--reversal-max-glitch-growth", type=int, default=3,
                    help="Gate 3 max acceptable encGlitchCount growth (default 3)")

    p.add_argument("--serial-burst", type=int, default=60,
                    help="Gate 4 number of back-to-back PINGs (default 60, matching the "
                          "project's own binary-vs-text-same-boot-loss-discriminator burst size)")
    p.add_argument("--serial-timeout", type=int, default=300,
                    help="Gate 4 per-PING timeout, ms (default 300)")
    p.add_argument("--serial-max-loss", type=float, default=0.05,
                    help="Gate 4 max acceptable loss rate, fraction (default 0.05)")

    p.add_argument("--skip-pipelining", action="store_true", help="Skip gate 1")
    p.add_argument("--skip-latency", action="store_true", help="Skip gate 2")
    p.add_argument("--skip-reversal", action="store_true", help="Skip gate 3")
    p.add_argument("--skip-serial", action="store_true", help="Skip gate 4")
    p.add_argument("--skip-flash-gate", action="store_true", help="Skip gate 5")
    p.add_argument("--flash-only", action="store_true",
                    help="Run ONLY gate 5 (no serial connection needed/attempted)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    result = Result()

    if not args.skip_flash_gate:
        gate_flash_ram(result, args)

    if args.flash_only:
        return 0 if result.summary() else 1

    print(f"\n  port: {args.port}   motor: {args.motor}")
    conn = SerialConnection(port=args.port, mode="direct")

    try:
        info = conn.connect()
        if "error" in info:
            print(f"ERROR: connect failed: {info['error']}")
            return 2
        print(f"  connected: mode={info.get('mode')}")

        if not args.skip_pipelining:
            gate_pipelining(conn, result, args)
        if not args.skip_latency:
            gate_cycle_latency(conn, result, args)
        if not args.skip_reversal:
            gate_reversal_stress(conn, result, args)
        if not args.skip_serial:
            gate_serial_health(conn, result, args)

    except KeyboardInterrupt:
        print("\n  interrupted — stopping motors...")
    finally:
        # HITL safety (.claude/rules/hardware-bench-testing.md): motors must
        # never be left running, regardless of which gate ran, failed, or
        # threw. STOP is bringup_main.cpp's own lightweight safety verb.
        if conn.is_open:
            try:
                dev_send(conn, "STOP")
            except Exception as exc:  # noqa: BLE001 — best-effort cleanup
                print(f"  WARN: STOP failed during cleanup: {exc}")
            conn.disconnect()

    return 0 if result.summary() else 1


if __name__ == "__main__":
    sys.exit(main())
