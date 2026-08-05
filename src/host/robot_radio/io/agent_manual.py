"""The rogo agent manual — printed verbatim by ``rogo --agent``.

Same convention as ``mbdeploy --agent``: a self-contained Markdown manual
written for AI coding agents (and power users) driving the tool
non-interactively. ``rogo --help`` gives the short usage; THIS document is
the complete reference, with particular depth on the rogo daemon
(``rogo serve``) and its socket protocol.

Keep this text in lockstep with the code it describes: the verb table
mirrors ``robot_radio.io.repl._VERBS``/``_HELP``, the daemon protocol
mirrors ``robot_radio.io.server``, and the client recipes mirror
``robot_radio.io.client.RogoClient``. A unit test
(``src/tests/unit/test_rogo_agent_manual.py``) pins the load-bearing
sections so a rename breaks a test instead of silently staling the manual.
"""
from __future__ import annotations

MANUAL = r"""# rogo — Agent Manual

Written for AI coding agents (and power users) driving `rogo`
non-interactively. `rogo --help` / `rogo <subcommand> --help` give the
short usage; this document is the complete reference. It covers the full
command surface, the repl verb grammar, and — most importantly — the rogo
DAEMON (`rogo serve`), which is how multiple programs share one robot.

---

## 1. What rogo is, and the one rule about serial ports

`rogo` drives a protocol-v5 robot (docs/protocol-v5.md): four cleartext
verbs (`HELLO`/`PING`/`ID`/`VER`), a binary command plane (bounded `Move`s,
`WHEELS` teleop, `CONFIG` push/read-back, planned `STOP`, `ESTOP`), and an
always-on binary telemetry push.

**The one rule: on macOS, CLOSING a robot's serial port RESETS the robot**
(HUPCL drops DTR on last close). Every one-shot invocation that opens and
closes the port reboots the MCU, restarts its clock, and wipes RAM-only
live config. This is why the daemon exists — for any session with more
than one command, run `rogo serve` and let it hold the port open.

Ports move on every re-enumeration and more than one robot may share the
hub. Discover with `uv run mbdeploy list` (see `mbdeploy --agent`) and pass
the port for THIS session only, e.g. `rogo --port /dev/cu.usbmodemXXXX serve`.

---

## 2. Command surface

```
rogo [--port PORT] [-v] [--agent] <subcommand> ...
```

| Subcommand | What it does |
|---|---|
| `serve` | **The daemon.** Open the serial connection ONCE, hold it, serve the repl grammar to any number of local programs over TCP. `--listen HOST:PORT` (default `$ROGO_ADDR`, else `127.0.0.1:7646`), `--record FILE.jsonl` for daemon-lifetime telemetry capture. |
| `repl` (aliases `run`, `exec`) | Run repl commands: argument list, piped stdin, or interactive prompt. `--connect [ADDR]` routes through a running daemon instead of opening the serial port. `--record FILE.jsonl` logs telemetry. |
| `ports` | List serial ports. |
| `hello` | One-shot probe (HELLO banner). NOTE: one-shot = port close = robot reset on exit. |
| others (`drive`, `turn`, `enc`, ...) | Legacy one-shot commands. Each opens and closes the port (= resets the robot on exit). Prefer the daemon + repl. |

`rogo --agent` prints this manual and exits.

---

## 3. The daemon: `rogo serve`

```
uv run rogo --port /dev/cu.usbmodemXXXX serve          # 127.0.0.1:7646
uv run rogo serve --listen :7700                       # another TCP port
```

One serial connection, held for the daemon's whole lifetime. All wire I/O
happens on a single executor thread; client handlers only parse and queue.
On Ctrl-C, SIGTERM, or a client's `shutdown` verb, the daemon halts the
robot (`halt_now`/ESTOP) BEFORE the port ever closes.

### 3.1 Socket protocol

Requests are newline-delimited, two equivalent shapes:

* plain text — exactly a repl command line: `drive 200`
* JSON — `{"id": "req-7", "cmd": "drive 200"}` (the reply echoes `id`,
  so a pipelining client can correlate)

Every reply is one JSON line:

| Event | Shape |
|---|---|
| on connect | `{"type": "hello", "server": "rogo", "client_id": N, "serial_port": ..., "mode": ...}` |
| per request | `{"type": "result", "id": ..., "ok": bool, "error": str\|null, "output": ["  human line", ...]}` |
| telemetry (after `sub tlm`) | `{"type": "tlm", "frame": {...}}` — the frame is `TLMFrame` as JSON (`enc`, `vel`, `pose`, `active`, `acks`, ..., plus `t_recv`) |
| daemon stopping | `{"type": "shutdown"}` |

### 3.2 Server-local verbs (handled in the daemon, never reach the robot)

| Verb | Effect |
|---|---|
| `sub tlm [N]` | Stream every Nth telemetry frame to THIS client (`sub tlm 5` ≈ 5x decimation). Fresh-booted firmware idles telemetry-quiet — send `tlm on` once after subscribing. |
| `unsub tlm` | Stop the stream. |
| `status` | Daemon status: held serial port, mode, uptime, client count, queue depth (rides the result's `status` key). |
| `quit` / `exit` | Drop THIS client. The serial port stays open; other clients unaffected. |
| `shutdown` | Halt the robot, then stop the daemon and close the port. |

### 3.3 The panic path

`estop` (or `halt`) from ANY client jumps the daemon's command queue and
aborts an in-progress completion wait — another client's long `drive` can
never delay a halt by more than one loop iteration. The verb then VERIFIES
the halt: it watches telemetry for the active flag (flags bit 2) to clear
and re-issues up to 3x if it does not (a single unverified estop write can
be lost with the wheels still turning — measured; see
.claude/rules/playfield-testing.md).

`stop` is NOT a halt: it is the PLANNED stop, an ordinary planner-queue
entry that waits behind the in-flight Move (measured on hardware: sent
mid-move, the robot drove the FULL commanded distance first). Scripts that
mean "stop now" must send `estop`.

---

## 4. Repl verb grammar (same grammar direct or through the daemon)

```
drive <mm> [speed] [nowait]  straight Move, DISTANCE stop  [mm] [mm/s]
turn <deg> [speed] [nowait]  in-place Move, ANGLE stop, +=CCW  [deg] [deg/s]
twist <v_x> <omega> <ms>     bounded body twist, TIME stop  [mm/s] [rad/s] [ms]
wheels <l> <r> <ms>          teleop wheel pair (bypasses planner)  [mm/s] [ms]
stop                         PLANNED stop -- queues behind the active move
estop | halt                 halt NOW + verify the active flag cleared
ping | id | ver | hello      cleartext liveness/identity round trip
config <k>=<v> ...           push config fields (SET vocabulary, e.g. pid.ki=6)
get <group>                  read a config group back WITH provenance
                             (drive, motors, geometry, wheelcontrol, planner,
                             plannershaper, otos, estimator)
raw <arm> [f=v ...]          build an envelope arm directly
                             (move|wheels|stop|estop|config)
enc | pose | otos | vel      print the latest telemetry field
twistfb | line | color       print the latest telemetry field
tlm [on|off|now]             dump freshest frame as JSON / switch TLM mode
sleep <ms> | wait <ms>       idle (keeps recording/streaming telemetry)
record <file> | record off   toggle telemetry->JSONL recording
```

Notes that save agents time:

* `drive`/`turn` wait for the Move's COMPLETION ack (`move_id=... DONE`)
  unless you append `nowait`. Every Move carries a firmware `timeout`
  backstop (3x expected duration, >= 2s).
* `get <group>` reports provenance: `BAKED` (compiled-in), `LIVE` (a wire
  push landed this power cycle), `PERSISTED` (restored from flash). This is
  how you prove a config push actually landed — an ack alone is not
  evidence.
* Fresh boot = telemetry quiet. `tlm on` first, or `enc`/`pose` will
  report no frames.
* The radio channel is a robot-JSON field (`connection.radio_channel`,
  bake-only — reflash to change). No wire verb can retune it.

---

## 5. Client recipes

### 5.1 Python (`RogoClient`) — the way other programs should drive the robot

```python
from robot_radio.io.client import RogoClient

with RogoClient() as robot:                 # $ROGO_ADDR or 127.0.0.1:7646
    robot.cmd("tlm on")
    robot.cmd("drive 200")                   # blocks until the Move completes
    robot.cmd("turn 90 nowait")              # returns after the enqueue ack
    print(robot.cmd("enc")["output"])        # ['  enc [mm] (L,R): (412, 408)']
    print(robot.status())                    # daemon status dict
    robot.estop()                            # panic path -- jumps the queue

with RogoClient() as robot:                  # telemetry stream
    robot.subscribe_tlm(decimate=5)          # every 5th frame
    robot.cmd("tlm on")
    for frame in robot.frames(duration=2.0):
        print(frame["enc"], frame["vel"])
```

`cmd()` raises `RogoDaemonError` if the daemon is unreachable or goes away.
One `RogoClient` per thread; the daemon happily takes many connections.

### 5.2 Interactive / scripted repl through the daemon

```
uv run rogo repl --connect                       # interactive prompt
uv run rogo repl "ping; drive 200; estop" --connect
cat run.rogo | uv run rogo repl --connect
```

Argparse quirk: `--connect` takes an optional ADDR, so put the command
string BEFORE `--connect` (as above) or pass the address explicitly
(`--connect 127.0.0.1:7646 "ping"`). Ctrl-C in connected mode sends an
`estop` through the daemon before exiting.

### 5.3 Anything else (netcat)

```
printf 'status\nping\nquit\n' | nc 127.0.0.1 7646
```

---

## 6. Exit codes and failure modes

* `rogo repl` exits 0 on a clean run, 1 on a connection error.
* `RogoDaemonError: no rogo daemon at ...` — start one with `rogo serve`.
* A daemon whose serial device vanishes (unplug) stops executing commands;
  results carry `"error": "connection error: ..."`. Restart it after the
  device returns (`mbdeploy list` for the fresh port -- it moves).
* Two daemons cannot share one serial port; the second open fails.
"""
