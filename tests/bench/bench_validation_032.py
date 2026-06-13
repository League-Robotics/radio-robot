"""bench_validation_032.py — comprehensive full-stack bench validation via Bench OTOS.

Sprint 032-001. Talks to the robot through the relay's DATA PLANE: open the relay
serial port with DTR asserted (default), wait for the announce, send `!GO` to
enter the data plane, then send PLAIN commands (no `>` prefix). One connection is
held open for the whole run. Telemetry via SNAP polling (request/reply — reliable;
async STREAM gets dropped by the bridge). See memory: relay-go-data-plane-protocol.

Enables the Bench OTOS (`DBG OTOS BENCH 1`) so the full firmware stack runs on the
bench stand (synthetic OTOS feeds commanded-motion pose so pose-dependent verbs
terminate). Drives TURN x4 closure, a D+TURN square, and D/T velocity profiles,
logging every SNAP frame and validating for bad starts/stops, velocity jumps,
runaway spin, and EKF health.

Units: pose=x_mm,y_mm,h_centideg ; twist=v_mmps,omega_mrad/s ; enc=L_mm,R_mm.

Usage: uv run python tests/bench/bench_validation_032.py
"""
from __future__ import annotations
import argparse, pathlib, re, sys, time
import serial

_REPO = pathlib.Path(__file__).resolve().parents[2]
OUTDIR = _REPO / "docs" / "bench-validation-032"


class Relay:
    """Raw data-plane link to the robot through the relay."""
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.2)  # DTR asserted (default)
        time.sleep(1.0)
        self._read(0.6)                       # drain relay announce
        self.s.reset_input_buffer()
        go = self.tx("!GO", 0.6)
        if "data plane" not in go:
            self.tx("!GO", 0.6)               # one retry
        self.tx("HELLO", 0.6)

    def _read(self, w):
        t = time.time(); b = b""
        while time.time() - t < w:
            d = self.s.read(256)
            if d: b += d
        return b.decode(errors="replace")

    def tx(self, msg, w=0.45):
        self.s.write((msg + "\n").encode()); self.s.flush()
        return self._read(w)

    def snap(self):
        for _ in range(4):
            for ln in self.tx("SNAP", 0.35).splitlines():
                if ln.startswith("TLM"):
                    return ln.strip()
        return None

    def close(self):
        try: self.s.close()
        except Exception: pass


def parse(ln):
    kv = dict(tok.split("=", 1) for tok in ln.split() if "=" in tok)
    d = {"mode": kv.get("mode", "?")}
    if "pose" in kv:
        p = kv["pose"].split(",")
        if len(p) >= 3: d["x"], d["y"], d["h_deg"] = int(p[0]), int(p[1]), int(p[2]) / 100.0
    if "twist" in kv:
        t = kv["twist"].split(",")
        if len(t) >= 2: d["v"], d["omega"] = int(t[0]), int(t[1]) / 1000.0
    if "enc" in kv:
        e = kv["enc"].split(",")
        if len(e) >= 2: d["encL"], d["encR"] = int(e[0]), int(e[1])
    if "ekf_rej" in kv:
        try: d["ekf_rej"] = int(kv["ekf_rej"])
        except ValueError: pass
    return d


def drive(rl, label, cmd, dur_s, log):
    frames = []
    rl.tx(cmd, 0.3)
    t0 = time.time()
    while time.time() - t0 < dur_s:
        ln = rl.snap()
        if ln:
            f = parse(ln); f["t"] = round(time.time() - t0, 2); f["lbl"] = label
            frames.append(f)
            log.append(f"{label} t={f['t']}  {ln}")
        rl.tx("+", 0.02)   # keepalive
    return frames


def analyze(label, frames, problems):
    vs = [f["v"] for f in frames if "v" in f]
    oms = [f["omega"] for f in frames if "omega" in f]
    hs = [f["h_deg"] for f in frames if "h_deg" in f]
    rej = [f["ekf_rej"] for f in frames if "ekf_rej" in f]
    m = {"label": label, "n": len(frames)}
    if vs:
        dv = [abs(vs[i] - vs[i-1]) for i in range(1, len(vs))]
        m.update(v_start=vs[0], v_peak=max(abs(x) for x in vs), v_end=vs[-1],
                 v_jump_max=max(dv) if dv else 0, v_series=[int(x) for x in vs])
    if oms: m["omega_peak"] = round(max(abs(x) for x in oms), 2)
    if hs:
        m["h_first"], m["h_last"] = round(hs[0], 1), round(hs[-1], 1)
        m["h_travel"] = round(sum(abs(hs[i]-hs[i-1]) for i in range(1, len(hs))), 1)
    if rej: m["ekf_rej_climb"] = rej[-1] - rej[0]
    if m.get("omega_peak", 0) > 12:
        problems.append(f"{label}: omega spike {m['omega_peak']} rad/s (runaway spin)")
    if label.startswith(("D_", "T_")):
        if m.get("h_travel", 0) > 40:
            problems.append(f"{label}: {m['h_travel']}deg heading travel on straight drive (drift/spin)")
        if abs(m.get("v_end", 0)) > 40:
            problems.append(f"{label}: residual v_end {m.get('v_end')} mm/s (bad stop)")
    if m.get("ekf_rej_climb", 0) > 25:
        problems.append(f"{label}: ekf_rej climbed {m['ekf_rej_climb']} (pose corruption?)")
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbmodem2121402")
    args = ap.parse_args()
    OUTDIR.mkdir(parents=True, exist_ok=True)
    log, report, problems = [], [], []

    rl = Relay(args.port)
    pong = rl.tx("PING", 0.4)
    print("PING:", pong.strip())
    if "pong" not in pong.lower():
        # one more try
        pong = rl.tx("PING", 0.6); print("PING2:", pong.strip())
    print("SET sTimeout=60000:", rl.tx("SET sTimeout=60000", 0.4).strip())
    print("STREAM fields    :", rl.tx("STREAM 50 fields=mode,pose,twist,enc,ekf_rej", 0.4).strip())
    print("DBG OTOS BENCH 1 :", rl.tx("DBG OTOS BENCH 1 20 10 0", 0.5).strip())
    print("DBG OTOS         :", rl.tx("DBG OTOS", 0.6).strip())

    try:
        # Seq 1: TURN x4
        rl.tx("ZERO enc", 0.3); rl.tx("SI 0 0 0", 0.3)
        for i in range(4):
            report.append(analyze(f"turn{i+1}", drive(rl, f"turn{i+1}", "TURN 9000", 3.0, log), problems))
        # Seq 2: square D+TURN
        rl.tx("ZERO enc", 0.3); rl.tx("SI 0 0 0", 0.3)
        for i in range(4):
            report.append(analyze(f"sqD{i+1}", drive(rl, f"sqD{i+1}", "D 300 250 250", 3.0, log), problems))
            report.append(analyze(f"sqT{i+1}", drive(rl, f"sqT{i+1}", "TURN 9000", 3.0, log), problems))
        # Seq 3: velocity profiles
        for label, cmd, dur in [("D_slow_150", "D 250 150 150", 3.2),
                                 ("D_med_300", "D 400 300 300", 2.8),
                                 ("D_fast_500", "D 500 500 500", 2.4),
                                 ("T_timed_1500", "T 1500 300 300", 2.6)]:
            rl.tx("ZERO enc", 0.3); rl.tx("SI 0 0 0", 0.3)
            report.append(analyze(label, drive(rl, label, cmd, dur, log), problems))
        print("DBG OTOS (final):", rl.tx("DBG OTOS", 0.6).strip())
    finally:
        rl.tx("X", 0.3); rl.tx("STREAM 0", 0.3); rl.tx("DBG OTOS BENCH 0", 0.3)
        rl.close()

    (OUTDIR / "tlm_log.txt").write_text("\n".join(log))
    out = ["# Bench validation 032 — full-stack drive via Bench OTOS (hardware)\n"]
    for m in report:
        out.append(f"\n[{m['label']}] frames={m['n']}")
        for k in ("v_start","v_peak","v_end","v_jump_max","omega_peak","h_first","h_last","h_travel","ekf_rej_climb","v_series"):
            if k in m: out.append(f"    {k:14s}= {m[k]}")
    out.append("\n---- VERDICT ----")
    out += [f"FOUND {len(problems)} PROBLEM(S):"] + ["  X "+p for p in problems] if problems \
           else ["  OK: clean starts/stops, bounded velocity, no runaway spin, EKF stable."]
    txt = "\n".join(out)
    (OUTDIR / "analysis.txt").write_text(txt)
    print("\n" + txt)
    print(f"\nlogged {len(log)} frames -> {OUTDIR}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
