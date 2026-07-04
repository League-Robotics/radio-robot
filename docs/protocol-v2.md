# Protocol v2 Wire Specification

Version 2 of the Nezha firmware command/telemetry protocol.
Hard break from v1 — no backward compatibility.

---

## 1. Overview

Protocol v2 is a line-oriented text protocol: one message equals one
`\n`-terminated line.  Tokens are whitespace-delimited.  `key=value`
pairs carry named parameters.  Only the first token (the verb) is
upper-cased; all remaining tokens, keys, and values preserve the case
as sent.  The protocol identifier is `proto=2`.

v1 commands (`K*`, `ENC`, `SO`, `SSE`, `SSO`, `SSC`, `SSL`, `HELLO`,
`DEVICE:`, and packed sign-prefix motion verbs) are removed.  Any
unrecognised verb returns `ERR unknown`.

Transport: the RadioRelay operates in RAW250 mode (247-byte MTU,
`[SEQ][FLAGS][LEN]` fragment framing).  Fragmentation and reassembly
are handled transparently by the firmware HAL (`Radio.cpp`) and the
relay.  The serial path (115 200 baud) accepts full lines directly.
The application layer sees only complete, NUL-terminated lines — it is
not aware of transport framing.

---

## 2. Grammar

```
message   ::= verb [token…] ['#' corr_id] '\n'
verb      ::= UPPER-CASE-WORD         ; upper-cased by firmware on receive
token     ::= positional | key_value
positional ::= non-whitespace-string  ; no '=' character (or '=' not at start)
key_value  ::= key '=' value
key       ::= non-whitespace-string   ; no '='
value     ::= non-whitespace-string   ; may be empty
corr_id   ::= one-or-more decimal digits
```

Rules:
- Leading and trailing whitespace is stripped from the line.
- Tokens are split on any run of space or tab characters.
- The verb token is upper-cased in place; all other tokens preserve case.
- A trailing `#<digits>` token (decimal digits only, no letters) is
  extracted as the *correlation id* and is not counted as a positional
  argument.  The firmware echoes it in every synchronous response for
  that command.  Async `EVT done T/D/G` and `EVT safety_stop` events
  also echo the `#id` when the originating drive command carried one;
  bare events (no originating id) carry no `#id`.
- A `key=value` token with an empty key (starts with `=`) is rejected
  with `ERR badarg missing key`.
- A `key=value` token with an empty value (ends with `=`) is valid; the
  value is the empty string.

---

## 3. Response Taxonomy

Every response line begins with one of six tags.

| Tag   | Meaning                          | Example                                      |
|-------|----------------------------------|----------------------------------------------|
| `OK`  | Command accepted / result        | `OK pong t=12345`                            |
| `ERR` | Rejected                         | `ERR badarg missing key`                     |
| `EVT` | Async event (unsolicited)        | `EVT done T`                                 |
| `TLM` | Telemetry frame                  | `TLM t=12345 mode=S enc=1024,1019`           |
| `CFG` | Config dump (response to GET)    | `CFG ml=0.487 mr=0.481 tw=120`               |
| `ID`  | Identity / capabilities          | `ID model=Nezha2 name=GUTOV serial=… proto=2` |

**OK format:**

```
OK <verb> [<body>] [#<corr_id>]
```

**ERR format:**

```
ERR <code> [<detail>] [#<corr_id>]
```

**EVT format:**

```
EVT <name> [<body>] [#<corr_id>]
```

`#<corr_id>` is present on `EVT done T/D/G` and `EVT safety_stop` only when
the originating T/D/G command carried a `#id`.  Uncorrelated drives produce
bare events with no `#id`.

---

## 4. Error Codes

| Code      | Meaning                                                      |
|-----------|--------------------------------------------------------------|
| `unknown` | Verb not recognised; detail is the verb that was sent        |
| `badarg`  | Wrong number or type of positional arguments                 |
| `badkey`  | Unknown `key=value` key; detail is the offending key name    |
| `nodev`   | Hardware device not present; detail is the command verb      |
| `range`   | Numeric argument outside the allowed range; detail is param  |
| `unsupported` | Mode rejected by `capabilities()` (e.g. `DEV M <n> VOLT` on a Nezha motor); detail is the mode keyword — §16 |

---

## 5. `#id` Correlation

A command may carry a trailing `#<digits>` token:

```
PING #7
T 200 200 1000 #42
GET ml pid.kp #9
```

The firmware strips the `#id` token from the argument list and echoes
it in every *synchronous* response for that command:

```
OK pong t=12345 #7
OK drive l=200 r=200 ms=1000 #42
CFG ml=0.487 pid.kp=300.000 #9
```

Rules:
- The id must consist of decimal digits only (no letters or other chars).
- If the last token begins with `#` but contains non-digit characters it
  is treated as a positional argument, not a correlation id.
- `EVT done T/D/G` and `EVT safety_stop` echo the `#id` of the originating
  T, D, or G command when that command carried one.  S-mode watchdog
  (`EVT safety_stop`) echoes the `#id` of the S command that established
  the session, if any.  Commands with no `#id` produce bare events.
- Multiple `ERR` lines from a single `SET` command (one per bad key) each
  carry the correlation id.

---

## 6. Liveness / Identity

### PING

```
PING [#id]
→ OK pong t=<robot_ms> [#id]
```

`t` is the robot clock in milliseconds since boot (`uBit.systemTime()`).
This is the time-synchronisation probe — see §9.

Example:

```
PING
OK pong t=12345

PING #3
OK pong t=12347 #3
```

### ECHO

```
ECHO <payload…> [#id]
→ OK echo <payload…> [#id]
```

The payload is everything after the `ECHO` token, with original
whitespace and case preserved (extracted from the raw line).  The
trailing `#id` token, if present, is stripped from the echoed payload.

Used to verify fragmentation+reassembly for large messages over the
relay; the payload may be up to ~490 bytes (512-byte buffer, minus
`ECHO ` prefix and `OK echo ` reply prefix).

Example:

```
ECHO hello world
OK echo hello world

ECHO the-quick-brown-fox #1
OK echo the-quick-brown-fox #1
```

### ID

```
ID [#id]
→ ID model=Nezha2 name=<name> serial=<serial> fw=<ver> proto=2 caps=<caps> [#id]
```

The response tag is `ID`, not `OK`.  Fields:

| Field    | Value                                                             |
|----------|-------------------------------------------------------------------|
| `model`  | Always `Nezha2`                                                   |
| `name`   | micro:bit friendly name (5-character CODAL name)                  |
| `serial` | Hardware serial number (decimal `uint32_t`)                        |
| `fw`     | Firmware version string (e.g. `0.20260602.6`)                     |
| `proto`  | Protocol version; always `2` for v2 firmware                      |
| `caps`   | Comma-separated list of detected subsystems (see below)           |

`caps` values: `otos`, `line`, `color`, `gripper`, `portio`.  `portio`
is always present.  Others are included only when hardware was detected
at boot.

Example:

```
ID
ID model=Nezha2 name=GUTOV serial=1234567 fw=0.20260602.6 proto=2 caps=otos,line,color,gripper,portio
```

### VER

```
VER [#id]
→ OK ver fw=<ver> proto=2 [#id]
```

Lightweight version query; does not probe hardware.

Example:

```
VER
OK ver fw=0.20260602.6 proto=2
```

### HELP

```
HELP [#id]
→ OK help PING ECHO ID VER HELP SET GET STREAM SNAP S T D G STOP GRIP ZERO OI OZ OR OP OV OL OA P PA [#id]
```

Returns a space-separated list of all implemented verbs in a single `OK
help` response.

---

## 7. Config: `SET` / `GET`

### GET

```
GET [<key>…] [#id]
→ CFG <key>=<value>… [#id]
```

With no arguments, dumps all registered keys.  With one or more key
names, returns only those keys.  For each unknown key a separate `ERR
badkey <key>` is emitted (does not prevent the CFG line from being sent
for valid keys).

Examples:

```
GET
CFG ml=0.487 mr=0.481 kff=0.150 klf=1.000 klb=1.000 krf=1.000 krb=1.000 adjThr=0.500 adjGain=0.050 tw=120 pid.kp=300.000 pid.ki=0.000 pid.kd=0.000 pid.max=30.000 distScale=0.940 turnScale=1.070 minSpeed=50 sTimeout=500 tick=20 tlmPeriod=0

GET ml pid.kp
CFG ml=0.487 pid.kp=300.000

GET ml #9
CFG ml=0.487 #9

GET badkey
ERR badkey badkey
```

### GET VEL — Velocity Readout

```
GET VEL [#id]
→ OK get vel=<vL>:<srcL>,<vR>:<srcR> [#id]
```

Returns the per-wheel measured velocity in mm/s (integer) and the velocity
source flag for each wheel.  Source flags:

- `C` — chip velocity from register 0x47 (`Motor::readSpeed`), corrected to
  mm/s via `(raw / 10.0) * wheelTravelCalib * sign`.
- `E` — encoder-delta fallback (used when chip I2C read fails or the chip
  reading fails the 2× implausibility gate).

`vL` and `vR` are the last values computed by `MotorController::tick()`.
They reflect the velocity at the most recent tick, not at command time.
If `tick()` has never run (e.g., before any drive command), both values
are 0.

Used for bench confirmation of the `readSpeed` unit factor and PID tuning.
To bench-confirm: drive at a steady speed (`S 200 200`), then poll `GET VEL`
and compare chip vs. encoder-derived velocity.  If chip velocity is ~10× the
encoder velocity, the `/10` (tenths) interpretation is correct.  If they
match, the raw register is whole degrees/s — change `kUnitFactor` in
`Motor.cpp` from `10.0` to `1.0`.

Example:

```
GET VEL
OK get vel=198:C,201:C

GET VEL #5
OK get vel=0:E,0:E #5
```

In the first example, both wheels read chip velocity (~198–201 mm/s forward).
In the second (motors stopped, chip read failed), both fall back to encoder-delta.

### SET

```
SET <key>=<value>… [#id]
→ OK set <applied-key>=<value>… [#id]
   [ERR badkey <key> [#id]]…
   [ERR badval <key>=<value> [#id]]
```

Applies all valid keys atomically to the live config.  The entire SET is
all-or-nothing: if any key is unknown, non-numeric, or out of range, no keys
are applied and the live config is unchanged.

- Unknown key → `ERR badkey <key>`
- Non-numeric or empty value → `ERR badval <key>` (parse failure)
- Out-of-range or invariant violation → `ERR badval <key>=<value>` (first
  failing key shown with its candidate value)

Validated invariants (sprint 028-004):

| Invariant                        | Failure consequence                      |
|----------------------------------|------------------------------------------|
| `tw > 0`                         | Division by zero in odometry arc/heading |
| `ctrlPeriod > 0`                 | Scheduler sleep wraps to huge uint32     |
| `vWheelMax > steerHeadroom`      | Saturation ceiling goes negative         |
| `rotSlip` in [0.5, 1.0]          | Nonsensical arc estimates break odometry |
| `safetyMargin > 0`               | D's runaway safety net would fire on any negative-going noise (at 0) or be meaningless (negative) — sprint 072 |
| `stallConfirm >= 0`               | D's stall-confirm debounce would fire instantly on the first tick inside `distArriveTol`, even while still moving — sprint 072 |

If all keys parse and validate, `cfg = candidate` is applied atomically and
`OK set <applied>` is emitted.  Changing any of `pid.kp`, `pid.ki`, `pid.kd`,
or `pid.max` calls `MotorController::updatePidGains()` after the commit.

Examples:

```
SET ml=0.487 mr=0.481
OK set ml=0.487 mr=0.481

SET tw=0
ERR badval tw=0

SET tw=abc
ERR badval tw

SET pid.kp=1.5 tw=0
ERR badval tw=0

SET bad=1
ERR badkey bad

SET
ERR badarg no key=value pairs
```

### Named Key Table

All 22 registered config keys, their types, defaults, and the v1
equivalents they replace.

| Key         | Type        | Wire format | Default  | Meaning                                 | v1 equiv  |
|-------------|-------------|-------------|----------|-----------------------------------------|-----------|
| `ml`        | float       | `%.3f`      | `0.487`  | mm per degree of rotation, left wheel   | `KCL`     |
| `mr`        | float       | `%.3f`      | `0.481`  | mm per degree of rotation, right wheel  | `KCR`     |
| `kff`       | float       | `%.3f`      | `0.150`  | Feed-forward gain                       | `KFF`     |
| `klf`       | float       | `%.3f`      | `1.000`  | Left-forward motor scale factor         | `KLF`     |
| `klb`       | float       | `%.3f`      | `1.000`  | Left-backward motor scale factor        | `KLB`     |
| `krf`       | float       | `%.3f`      | `1.000`  | Right-forward motor scale factor        | `KRF`     |
| `krb`       | float       | `%.3f`      | `1.000`  | Right-backward motor scale factor       | `KRB`     |
| `adjThr`    | float       | `%.3f`      | `0.500`  | Slower-wheel adjustment threshold       | —         |
| `adjGain`   | float       | `%.3f`      | `0.050`  | Slower-wheel adjustment gain            | —         |
| `tw`        | float-as-int| `%d`        | `120`    | Track width in mm                       | `KAT`     |
| `pid.kp`    | float       | `%.3f`      | `300.000`| Ratio PID proportional gain             | `KCP`     |
| `pid.ki`    | float       | `%.3f`      | `0.000`  | Ratio PID integral gain                 | `KCI`     |
| `pid.kd`    | float       | `%.3f`      | `0.000`  | Ratio PID derivative gain               | `KCD`     |
| `pid.max`   | float       | `%.3f`      | `30.000` | Ratio PID output clamp                  | `KCM`     |
| `distScale` | float       | `%.3f`      | `0.940`  | Distance command scale factor           | `KDS`     |
| `turnScale` | float       | `%.3f`      | `1.070`  | Turn command scale factor               | `KTS`     |
| `minSpeed`  | int32       | `%d`        | `50`     | Minimum drive speed (mm/s)              | `KMS`     |
| `sTimeout`  | int32       | `%d`        | `500`    | Streaming watchdog timeout (ms)         | `KST`     |
| `tick`      | int32       | `%d`        | `20`     | Main-loop tick period (ms)              | `KTK`     |
| `tlmPeriod` | int32       | `%d`        | `0`      | TLM streaming period (ms); 0 = off      | —         |
| `ekfQxy`    | float       | `%.3f`      | `200.000`| EKF process noise: position (mm²/s)     | —         |
| `ekfQtheta` | float       | `%.3f`      | `0.500`  | EKF process noise: heading (rad²/s)     | —         |
| `ekfQv`     | float       | `%.3f`      | `5000.000`| EKF process noise: body speed (mm²/s³) | —         |
| `ekfQomega` | float       | `%.3f`      | `1.000`  | EKF process noise: yaw rate (rad²/s³)   | —         |
| `ekfROtosXy`| float       | `%.3f`      | `50.000` | EKF OTOS measurement noise: position (mm²) | —      |
| `ekfROtosV` | float       | `%.3f`      | `200.000`| EKF OTOS measurement noise: body speed (mm²/s²) | — |
| `ekfREncV`  | float       | `%.3f`      | `100.000`| EKF encoder measurement noise: body speed (mm²/s²) | — |

(Sprint 069-001: these seven rows close 067's Open Question 5 -- a live
`SET` routes through `Drive::configure()`'s `setNoise()` push, which
updates EKF fusion noise WITHOUT resetting fused pose/covariance. This
table has pre-existing drift from several long-landed keys, e.g. `vel.kP`,
`ekfRHead` itself -- not backfilled here, out of scope per ticket
068-001's Open Question 1 precedent.)

Type `float-as-int`: stored internally as `float`, read/written on the
wire as a decimal integer (no fractional part).  `SET tw=121` writes
`121.0f`; `GET tw` returns `121`.

Value conventions:
- All distances are integer millimetres; no implicit scaling, no `×10`
  multipliers.
- Float keys use three decimal places on output (`%.3f`).
- Integer and float-as-int keys use `%d` on output.
- `SET` accepts float text for float keys (`strtof` with end-pointer
  validation) and integer text for int and float-as-int keys (`strtol`
  base-10 with end-pointer validation).  Trailing non-numeric characters or
  empty values are rejected with `ERR badval <key>`.

---

## 8. Telemetry: `TLM` Frame

### STREAM

```
STREAM <ms> [#id]
→ OK stream period=<ms> [#id]

STREAM fields=<field>,… [#id]
→ OK stream fields=<field>,… [#id]
```

`STREAM <ms>` sets the periodic telemetry interval in milliseconds.
`ms=0` disables streaming.  The minimum enforced period is 20 ms;
smaller positive values are clamped to 20.  The `OK` reply echoes the
*clamped* period (e.g. `STREAM 10` → `OK stream period=20`).

`STREAM fields=<csv>` sets the field subscription bitmask.  The value
is a comma-separated list of field names (`enc`, `pose`, `vel`, `line`,
`color`).  Any unrecognised name is silently ignored.  An empty or
all-unrecognised list resets the mask to `TLM_FIELD_ALL` (all fields).

**Channel binding (D10, firmware 028-005).** The TLM stream is bound to
the communication channel (serial or radio) that issued the most recent
`STREAM <ms>` command.  Subsequent commands arriving on a *different*
channel do not redirect the stream.  This is intentional: a radio drive
command during an active serial TLM session must not silently steal the
serial stream.

*Implication:* a session that issues drive commands via radio without
first issuing `STREAM` on serial will *not* receive serial TLM output.
Issue `STREAM <ms>` on the channel that should receive TLM before
driving.

**Idle-rate (D10, firmware 028-005).** The stream continues even when the
robot is stopped (idle > 400 ms).  When idle, the effective emit period
is `max(period_ms, 500 ms)` so the host can distinguish "robot idle"
from "serial dropped."  A gap exceeding 600 ms (500 ms idle period plus
one loop tick) indicates a true loss.

Examples:

```
STREAM 100
OK stream period=100

STREAM 10
OK stream period=20

STREAM 0
OK stream period=0

STREAM fields=enc,pose
OK stream fields=enc,pose

STREAM fields=enc,pose,line
OK stream fields=enc,pose,line
```

### SNAP

```
SNAP [#id]
→ TLM t=<ms> mode=<char> seq=<n> … (raw TLM frame)
```

Returns one TLM frame synchronously.  The frame is emitted directly as
a `TLM` line (not wrapped in `OK`).  SNAP and STREAM share the same
`_tlmSeq` counter, so the `seq=` field is consistent across both paths
(see *TLM Frame Format* below).

### TLM Frame Format

```
TLM t=<ms> mode=<char> seq=<n> [enc=<l>,<r>] [pose=<x>,<y>,<h>] [encpose=<x>,<y>,<h>] [vel=<vl>,<vr>] [line=<g1>,<g2>,<g3>,<g4>] [color=<r>,<g>,<b>,<c>]
```

Fields are emitted in the order shown; fields whose subscription bit is
clear, or whose hardware is absent, are omitted. (This list has drifted
behind a few fields shipped in later sprints — `wedge=`, `twist=`,
`ekf_rej=` — that are not yet documented here; see the sprint-068
architecture update, Open Question 1. `encpose=` is current as of Sprint
068; `otos=`'s semantics and `otos_health=` (new) are current as of Sprint
074-004 — see the notes below the table.)

| Field      | Format                      | Units / notes                                                          |
|------------|-----------------------------|------------------------------------------------------------------------|
| `t`        | `%lu` (unsigned long)       | Robot clock in ms at sensor-sample time                                |
| `mode`     | single character            | `I`=idle, `S`=streaming (`S`/`VW`), `T`=timed, `D`=distance, `G`=go-to |
| `seq`      | `%u` (uint16, wraps at 65535) | D10 sequence counter — shared by STREAM and SNAP (firmware 028-005+). Absent on older firmware. Use `tlm_drop_rate(frames)` to detect loss. |
| `enc`      | `%d,%d`                     | Left and right encoder accumulated distance in mm                      |
| `pose`     | `%d,%d,%d`                  | x mm, y mm, heading in centi-degrees                                   |
| `encpose`  | `%d,%d,%d`                  | Encoder-only dead-reckoned world pose (x mm, y mm, heading in centi-degrees) — integrated from wheel-encoder deltas only, independent of `pose=`'s EKF fusion and any OTOS input. Gated by `TLM_FIELD_ENCPOSE`; on by default. No freshness gate (updates every control tick). Sprint 068. |
| `vel`      | `%d,%d`                     | Left and right actual velocity in mm/s                                 |
| `line`     | `%u,%u,%u,%u`               | Four greyscale channels (raw ADC counts)                               |
| `color`    | `%u,%u,%u,%u`               | R, G, B, clear channels (raw ADC counts)                               |

**Timestamp discipline.** `t=` is captured at the start of sensor
reading (before `snprintf`), not at line-send time.  This ensures the
translated host time reflects when the measurements were taken, not the
variable send latency.

**Pose source.** When the OTOS sensor is present and detected at boot,
`pose=` values come from `OtosSensor::getPositionRaw()`.  Otherwise
they come from the dead-reckoning odometry integrator.

**`otos=` field semantics (074-004).** `otos=<x>,<y>,<h>` (bit
`TLM_FIELD_OTOS = 0x40`) is the raw, most-recently-**successfully-read**
pose from whichever odometer is currently active (real chip, or the bench
sensor when `DBG OTOS BENCH 1` is on) — independent of whether that reading
was admitted into EKF fusion. It does **not** go stale, freeze, or change
meaning when the fusion gate blocks a reading (`Drive::_otosFusionBlocked`
true): `otos=` keeps reporting the live raw reads throughout a block, and
`otos_health=` below is the field that tells a host fusion is currently
blocked. `otos=` is gated by the N8 freshness rule (absent if
`now - last_upd > 2 * lag`, same as `line=`/`color=`); a **read failure**
clears that freshness envelope the same tick it occurs
(`Drive::tickUpdate()` STEP 5, `RobotTelemetry.cpp`), so a persistent read
failure makes `otos=` disappear from TLM rather than repeating the
last-good value forever (regression-tested,
`tests/simulation/unit/test_otos_health_tlm.py`).

**`otos_health=` field (Sprint 074-004).** `otos_health=<status>,<blocked>`
(bit `TLM_FIELD_OTOS_HEALTH = 0x200`, on by default) is the OTOS
fusion-gate's health state, added to close a diagnosability gap: before
this field, the only wire-visible symptom of a fusion block was an
indirectly-inferred one (a climbing `ekf_rej=` counter alongside a
suspiciously static `otos=`). `<status>` is the raw OTOS chip STATUS byte
(`%u`, 0 = clean); `<blocked>` is `Drive::_otosFusionBlocked` (`0`/`1`).
Unlike every other sensor field in this table, `otos_health=` is emitted
**unconditionally** once its bit is set — no freshness/staleness gate —
matching `wedge=`'s existing precedent (Sprint 064-004): the health field
must stay visible precisely when `otos=` itself is going stale or the gate
is blocked, so a host can tell the two conditions apart without guessing.
See the sprint-074 architecture update ("OTOS fusion recovery and health
visibility"), Design Rationale Decision 4, for the full design rationale.

**`vel=` field.** The field bitmask bit is `TLM_FIELD_VEL = 0x04`.  The
field is populated from `MotorController::getActualVelocity()` (landed in
Sprint 010).  Values reflect the last `tick()` measurement; see `GET VEL`
for per-wheel source flags (`C` = chip, `E` = encoder-delta).

**`seq=` field (D10, firmware 028-005+).** A monotonically increasing
`uint16` counter shared by all TLM frames (STREAM and SNAP).  Wraps at
65 535.  The host can compute the drop rate with
`tlm_drop_rate(frames)` from `robot_radio.robot.protocol`.  Frames from
pre-028-005 firmware omit this field; `TLMFrame.seq` is `None`.

Example:

```
TLM t=12345 mode=S seq=0 enc=1024,1019 pose=350,-12,1780 encpose=349,-11,1779 vel=198,201 line=120,340,330,118 color=21,30,18,80
TLM t=12395 mode=S seq=1 enc=1068,1063 pose=352,-12,1780 encpose=351,-11,1779 vel=200,200
TLM t=12895 mode=I seq=2 enc=1068,1063 pose=352,-12,1780 encpose=351,-11,1779 vel=0,0
```

---

## 9. Time Synchronisation

The robot runs a free-running clock (`uBit.systemTime()`, milliseconds
since boot).  The robot clock is never set from the host; setting it
would corrupt odometry `dt` computations and is unnecessary.

Instead, the host estimates the *offset* (and optionally the *skew*)
between host-monotonic time and robot time, then translates robot `t=`
timestamps into host time for event correlation.

### Algorithm (NTP-style min-RTT filtering)

For each PING exchange the host records:
- `T0` — host monotonic time (ms) immediately *before* sending `PING`
- `T1` — host monotonic time (ms) immediately *after* receiving the reply
- `t_r` — robot clock stamp (ms) from `OK pong t=<t_r>`

Assuming a roughly symmetric link delay:

```
offset_ms = (T0 + T1) / 2 − t_r
```

Fire N PINGs (default 5) and keep the sample with the smallest RTT
(= T1 − T0).  The minimum-RTT sample has the least relay/queuing
jitter.  Accuracy is bounded by approximately half the minimum RTT.

### Skew Compensation

After accumulating samples spanning at least 1 ms of robot time, the
host fits a linear model by ordinary least squares:

```
host_mid ≈ a · t_robot + b
```

`a` is the skew factor (ideally ≈ 1.0); `b` is the intercept.  The
`to_host_time()` function uses this model when available, falling back
to the offset-only estimate.  Re-sync (new `ping_burst()`) is
recommended every 30–60 s to track micro:bit crystal drift (~tens of
ppm, a few ms/min).

### Host-side API (`ClockSync`)

```python
cs = ClockSync()
cs.ping_burst(lambda cmd: proto.ping_and_raw(cmd))  # fire 5 PINGs
host_ms = cs.to_host_time(tlm_frame.t)              # translate robot timestamp
```

`ClockSync.stale(max_age_s=60.0)` returns `True` if no burst has been
recorded within `max_age_s` seconds.

---

## 10. Motion Commands

Motion commands are asynchronous: the firmware returns `OK …` immediately
(acknowledging the command parameters) and later sends an `EVT done …`
or `EVT safety_stop` when the drive ends.

When the originating T, D, or G command carried a `#id`, that id is echoed
on the asynchronous completion event.  Commands issued without a `#id`
produce bare events with no `#id`.

### EVT Completion Events

| Event                  | Emitted when                                          |
|------------------------|-------------------------------------------------------|
| `EVT done T [#id]`     | Timed drive elapsed                                   |
| `EVT done D [#id]`     | Distance drive target reached (or 5-second timeout)   |
| `EVT done G [#id]`     | Go-to arc completed within `arriveTol` mm             |
| `EVT safety_stop [#id]`| S/VW watchdog expired (no `S` or `VW` command within `sTimeout` ms) |

`[#id]` is present only when the originating command carried one.  Example:

```
T 200 200 1000 #12
OK drive l=200 r=200 ms=1000 #12
… (later) …
EVT done T #12
```

Bare form (no corr id):

```
T 200 200 1000
OK drive l=200 r=200 ms=1000
EVT done T
```

**`reason=` field (sprint 052+).** Every `EVT done …` and `EVT safety_stop`
line carries a trailing `reason=<token>` field indicating why the motion ended.
The field follows any `#<id>` token:

| Reason token  | Fired by                                                                  |
|---------------|---------------------------------------------------------------------------|
| `time`        | Time stop (`stop=t:` or T/D built-in time stop)                           |
| `dist`        | Distance stop (`stop=d:` or D built-in distance stop)                     |
| `rot`         | Rotation stop (`stop=rot:`)                                               |
| `heading`     | Heading stop (`stop=heading:`)                                           |
| `pos`         | Position stop (G/GOTO arrival)                                            |
| `line`        | Line-any stop (`stop=line:`)                                              |
| `color`       | Color-match stop (`stop=color:`)                                          |
| `<channel>`   | Sensor stop (`stop=sensor:<ch>:`) — token is the channel name (e.g. `line0`) |
| `watchdog`    | Safety watchdog expired (`EVT safety_stop reason=watchdog`)               |
| `runaway`     | D runaway safety net tripped (`EVT safety_stop reason=runaway`, sprint 072) |
| `arrive`      | D stalled short of target, forced complete (`EVT done D reason=arrive`, sprint 072) |

The `reason=` token is additive: existing hosts that match on the verb
(`EVT done T`) continue to work unchanged. `runaway` (sprint 072) is an
additive new VALUE on the existing `EVT safety_stop` label — hosts that
already recognize `EVT safety_stop` from the keepalive-watchdog path need no
changes; the base label is identical, only the `reason=` value differs
(`watchdog` vs `runaway`). `arrive` (sprint 072) is likewise an additive new
VALUE on the existing `EVT done D` label — hosts that only check for
`EVT done D` (ignoring `reason=`) see no behavior change.

Examples:

```
EVT done T #12 reason=time
EVT done D reason=dist
EVT done D reason=arrive
EVT safety_stop reason=watchdog
EVT safety_stop reason=runaway
```

### stop= Clauses

Any open-loop motion command (`VW`, `S`, `R`, `T`, `D`, `TURN`) may carry one
or more `stop=<kind>:<args>` clauses as `key=value` pairs.  Each clause adds a
stop condition that fires when its condition is satisfied; conditions are
OR-combined.  Up to 4 `stop=` clauses are accepted per command
(`kMaxStopConds = 4`).

| Clause                              | Fires when                                                        |
|-------------------------------------|-------------------------------------------------------------------|
| `stop=t:<ms>`                       | Duration ≥ ms milliseconds                                        |
| `stop=d:<mm>`                       | Average encoder travel ≥ mm millimetres                           |
| `stop=line:<ge\|le>:<thr>`          | Any of line[0..3] satisfies the threshold                         |
| `stop=sensor:<ch>:<ge\|le>:<thr>`   | Named channel satisfies the threshold                             |
| `stop=color:<h>:<s>:<v>:<dist>`     | HSV colour distance from target ≤ dist                            |
| `stop=heading:<cdeg>:<eps_cdeg>`    | Heading within eps of target (centi-degrees)                      |
| `stop=rot:<arc_mm>`                 | Per-wheel encoder arc ≥ arc_mm                                    |

Channel names for `stop=sensor:`: `line0`..`line3`, `colorR`..`colorC`,
`analogIn0`..`analogIn3`.

`sensor=<ch>:<op>:<thr>` is accepted as a back-compat alias for
`stop=sensor:<ch>:<op>:<thr>`.

`T` and `D` retain their positional time/distance arguments AND may carry
additional `stop=` clauses (OR-combined with the built-in stop):

```
T 200 200 1000 stop=sensor:line0:ge:512
D 200 200 300 stop=t:5000
VW 200 0 stop=d:300 stop=t:5000
S 200 200 stop=line:ge:512
TURN 9000 stop=sensor:line0:ge:512
R 200 500 stop=d:400
```

### S — Streaming (Watchdog) Drive

```
S <l> <r> [#id]
→ OK drive l=<l> r=<r> [#id]
```

Sets left and right wheel velocities (mm/s) and resets the streaming
watchdog.  If no `S` command arrives within `sTimeout` ms (default 500),
the firmware stops the motors and emits `EVT safety_stop reason=watchdog`.

Velocity range: −1000 … +1000 mm/s per wheel.  Values outside this
range return `ERR range l` or `ERR range r`.

Example:

```
S 200 150
OK drive l=200 r=150

S -100 100
OK drive l=-100 r=100
```

### T — Timed Drive

```
T <l> <r> <ms> [stop=<kind>:<args>]… [#id]
→ OK drive l=<l> r=<r> ms=<ms> [#id]
  … (later, asynchronously) …
  EVT done T [#id] reason=<token>
```

Drives at the given speeds for `ms` milliseconds (1 … 30 000).  Optional
`stop=` clauses may be appended; each fires an early stop (OR-combined with
the built-in time stop).

Velocity range: −1000 … +1000 mm/s.  Duration range: 1 … 30 000 ms.

Example:

```
T 200 200 1000
OK drive l=200 r=200 ms=1000
EVT done T reason=time

T 200 200 1000 #12
OK drive l=200 r=200 ms=1000 #12
EVT done T #12 reason=time

T 200 200 5000 stop=sensor:line0:ge:512
OK drive l=200 r=200 ms=5000
… (stops when line0 ≥ 512 or 5 s elapses) …
EVT done T reason=line0
```

### D — Distance Drive

```
D <l> <r> <mm> [stop=<kind>:<args>]… [#id]
→ OK drive l=<l> r=<r> mm=<mm> [#id]
  … (later, asynchronously) …
  EVT done D [#id] reason=<token>
```

Drives at the given speeds until the average encoder travel **in the
commanded direction** reaches `mm` millimetres (1 … 10 000), or until a
generous (2× nominal + 2 s) timeout fires.  A reverse-commanded drive
(negative `l`/`r`) completes on that same magnitude of BACKWARD travel; a
wrong-direction encoder reading (e.g. a forward-commanded `D` that instead
travels backward) does **not** satisfy the distance stop from that
wrong-direction travel (sprint 072 — previously this compared the absolute
value of the travel, so a runaway wrong-direction drive could self-report a
false completion). Optional `stop=` clauses may be appended; each fires an
early stop (OR-combined with the built-in distance stop) — only 1 slot
remains free out of `kMaxStopConds`'s 4 (the built-in DISTANCE/TIME/
SAFETY_MARGIN trio occupies the other 3, sprint 072); a second `stop=`
clause overflows and the drive is cancelled with `ERR stopoverflow`.

Velocity range: −1000 … +1000 mm/s.  Distance range: 1 … 10 000 mm.

**Runaway safety net (sprint 072).** If signed travel goes more than
`safetyMargin` mm (default 50; `SET`-able) NEGATIVE relative to the
commanded direction — the robot demonstrably moving the wrong way — the
firmware forces an immediate hard stop and emits `EVT safety_stop
reason=runaway` instead of the configured `EVT done D`, within one control
tick of crossing the margin (far faster than the timeout above):

```
D 200 200 500
OK drive l=200 r=200 mm=500
… (encoders instead run backward past the margin) …
EVT safety_stop reason=runaway
```

**Terminal-completion guarantee (sprint 072).** The decel profile's commanded
speed is floored at `minWheelSpeed` once it enters the final approach zone
(instead of asymptotically approaching zero AT the target), so a real motor's
stiction/deadband is less likely to stall the drive right at the finish.  As
a backstop independent of that floor being high enough for a given robot:
if the remaining distance sits within `distArriveTol` mm (default 5;
`SET`-able) of the target and stops shrinking for `stallConfirm` ms (default
300; `SET`-able), the drive completes now — `EVT done D reason=arrive` —
instead of leaving the robot stalled short until the TIME net fires.  This
trades a small, bounded, known under-travel (up to `distArriveTol`) for
eliminating an unbounded stall/reversal/thrash failure mode; a drive that
reaches its target via the normal strict crossing is unaffected
(`reason=dist`, unchanged):

```
D 200 200 500
OK drive l=200 r=200 mm=500
… (motor stiction stalls the drive 1-3 mm short of target) …
EVT done D reason=arrive
```

Example:

```
D 200 200 300
OK drive l=200 r=200 mm=300
EVT done D reason=dist

D 200 200 300 #5
OK drive l=200 r=200 mm=300 #5
EVT done D #5 reason=dist

D 200 200 500 stop=t:3000
OK drive l=200 r=200 mm=500
… (stops at 500 mm or 3 s, whichever comes first) …
EVT done D reason=dist
```

### G — Go-To (relative XY)

```
G <x> <y> <speed> [#id]
→ OK goto x=<x> y=<y> speed=<speed> [#id]
  … (later, asynchronously) …
  EVT done G [#id]
```

Navigate to the relative XY point `(x, y)` (mm) at the given
`speed` (mm/s).  The coordinate system is robot-relative: +x is forward,
+y is left.  Heading is in centi-degrees.

The firmware optionally pre-rotates in place when the bearing angle to the
target exceeds `turnGate` degrees (default 35), then pursues the target and
completes when within `arriveTol` mm (default 25) of the goal.  The
`G` verb is unambiguously go-to; the gripper is controlled by `GRIP`.

Coordinate range: −10 000 … +10 000 mm per axis.
Speed range: 1 … 1 000 mm/s.

Example:

```
G 300 0 200
OK goto x=300 y=0 speed=200
EVT done G

G 300 0 200 #7
OK goto x=300 y=0 speed=200 #7
EVT done G #7
```

### VW — Body-Twist Velocity Drive (Watchdogged)

```
VW <v> <omega_mrads> [stop=<kind>:<args>]… [#id]
→ OK vw v=<v> omega=<omega_mrads> [#id]
```

Sets a body-twist velocity: `v` is the forward speed in mm/s and
`omega_mrads` is the yaw rate in **milli-radians per second** (integer).
Positive `omega` is CCW (left turn).

The firmware converts `(v, ω)` to individual wheel speeds via
`BodyKinematics::inverse()`, applies `saturate()`, and enters `VELOCITY`
mode.  If no `VW` (or `S`) command arrives within `sTimeout` ms the motors
stop and `EVT safety_stop [#id] reason=watchdog` is emitted (with the `#id`
from the last `VW` command, if one was supplied).

Optional `stop=` clauses may be appended; the first clause that fires ends
the drive and emits `EVT done VW [#id] reason=<token>`.

The TLM `mode=` field uses `S` for both `S` and `VW` commands.

Ranges:
- `v` — −1 000 … +1 000 mm/s.  Out of range → `ERR range v`.
- `omega_mrads` — −3 142 … +3 142 mrad/s (≈ ±π rad/s).  Out of range →
  `ERR range omega`.
- Too few arguments → `ERR badarg`.

Example:

```
VW 200 0
OK vw v=200 omega=0

VW 0 500
OK vw v=0 omega=500

VW 200 300 #7
OK vw v=200 omega=300 #7
… (watchdog fires with no subsequent VW) …
EVT safety_stop #7 reason=watchdog

VW 200 0 stop=d:300 stop=t:5000
OK vw v=200 omega=0
… (stops at 300 mm or 5 s) …
EVT done VW reason=dist
```

### RF — Radio Channel

```
RF              → OK rf chan=<n> group=10        (query)
RF <n> [#id]    → OK rf chan=<n> group=10 [#id]  (set + persist)
```

Gets or sets the radio **channel** (frequency band).  The radio **group is
always 10** and cannot be changed.  Channel range is `0 … 35`; out of range
returns `ERR range chan`.  The channel renders as a single base-36 character
on the LED matrix (`0`-`9` then `A`-`Z`, so channel 10 = `A`).

The channel is **persisted** in the micro:bit's flash key-value store, so it
survives power cycles.  On boot the firmware loads the stored channel (default
`0`), **flashes the channel character** then the heart, and starts the radio
on it.

**Setting the channel over the radio drops the link.**  When `RF <n>` re-tunes,
the relay is still on the old channel and can no longer hear the robot.  The
`OK` reply is sent on the *old* channel before re-tuning (so you do see it), but
all subsequent traffic is on the new channel.  Change the channel either:

- over **USB serial** (`RF <n>` on the direct port), or
- with the **on-board buttons at boot**: hold `A`+`B` together while powering
  on to enter edit mode — release, then the channel character stays on the LED
  while you press `A` (−1) / `B` (+1); after ~5 s with no input it saves,
  flashes a checkmark, then shows the heart.

Example:

```
RF
OK rf chan=0 group=10

RF 7
OK rf chan=7 group=10
… (robot is now on channel 7; relay must also move to channel 7) …
```

### STOP

```
STOP [#id]
→ OK stop [#id]
```

Stops motors immediately.  Clears any active drive mode.  No `EVT` is
emitted.

Example:

```
STOP
OK stop
```

### GRIP — Gripper Control

```
GRIP <deg> [#id]   → OK grip deg=<deg> [#id]   (set angle)
GRIP       [#id]   → OK grip deg=<deg> [#id]   (query current angle)
```

Sets the gripper servo to the given angle (0 … 180 degrees).  With no
argument, returns the last commanded angle (0 at power-on).

Range: 0 … 180 degrees.

Example:

```
GRIP 90
OK grip deg=90

GRIP
OK grip deg=90
```

### ZERO — Zero Encoders / Odometry

```
ZERO enc          [#id]  → OK zero enc [#id]
ZERO pose         [#id]  → OK zero pose [#id]
ZERO enc pose     [#id]  → OK zero enc pose [#id]
```

`enc` resets the encoder accumulators (calls
`MotorController::resetEncoderAccumulators()`).  `pose` resets the
odometry integrator to `(0, 0, 0)` (calls `Odometry::zero()`).  Both
may be specified in one command.

At least one of `enc` or `pose` must be present; otherwise `ERR badarg`.

Example:

```
ZERO enc
OK zero enc

ZERO enc pose
OK zero enc pose
```

---

## 11. OTOS / Port I/O Commands

### OI — OTOS Init

```
OI [#id]
→ OK oi [#id]
   ERR nodev oi [#id]   (if OTOS not detected at boot)
```

Calls `OtosSensor::init()` to re-initialise OTOS signal processing.

### OZ — OTOS Zero Position

```
OZ [#id]
→ OK oz [#id]
   ERR nodev oz [#id]
```

Zeroes the OTOS world-frame position to the current location
(`setPositionRaw(0, 0, 0)`).

### OR — OTOS Reset Tracking

```
OR [#id]
→ OK or [#id]
   ERR nodev or [#id]
```

Resets OTOS Kalman filter state (`resetTracking()`).

### OP — OTOS Read Position

```
OP [#id]
→ OK pos x=<x> y=<y> h=<h> [#id]
   ERR nodev op [#id]
```

Returns the current OTOS world-frame position.  `x` and `y` are in
mm; `h` is the heading in centi-degrees (integer).

Example:

```
OP
OK pos x=350 y=-12 h=1780
```

### OV — OTOS Set Position

```
OV <x> <y> <h> [#id]
→ OK setpos x=<x> y=<y> h=<h> [#id]
   ERR nodev ov [#id]
```

Sets the OTOS world-frame position (`setPositionRaw(x, y, h)`).

Example:

```
OV 0 0 0
OK setpos x=0 y=0 h=0
```

### OL — OTOS Linear Scalar

```
OL <val> [#id]   → OK linear scalar=<val> [#id]   (set)
OL       [#id]   → OK linear scalar=<val> [#id]   (read)
   ERR nodev ol [#id]
```

Gets or sets the OTOS linear scalar calibration register (`int8_t`).

### OA — OTOS Angular Scalar

```
OA <val> [#id]   → OK angular scalar=<val> [#id]   (set)
OA       [#id]   → OK angular scalar=<val> [#id]   (read)
   ERR nodev oa [#id]
```

Gets or sets the OTOS angular scalar calibration register (`int8_t`).

### P — Digital Port Read / Write

```
P <port>        [#id]  → OK port p=<port> v=<val> [#id]   (read)
P <port> <val>  [#id]  → OK port p=<port> v=<val> [#id]   (write)
```

`port` is 1–4 (J-port number).  On read, `v` is 0 or 1.  On write,
any non-zero `val` is treated as 1.

Examples:

```
P 1
OK port p=1 v=0

P 2 1
OK port p=2 v=1
```

### PA — Analog Port Read / Write

```
PA <port>        [#id]  → OK aport p=<port> v=<val> [#id]   (read)
PA <port> <val>  [#id]  → OK aport p=<port> v=<val> [#id]   (write)
```

`port` is 1–4.  `val` is in the range 0–1023 (10-bit ADC or PWM).
A write `val` outside 0–1023 returns `ERR range val`.

Examples:

```
PA 3
OK aport p=3 v=512

PA 3 256
OK aport p=3 v=256
```

---

## 12. Buffer and Framing Note

The firmware uses a 512-byte line buffer (`buf[512]` in `main.cpp`,
`REASM_MAX = 512` in `Radio.h`).  The maximum message size is therefore
511 bytes (one byte for the NUL terminator).

A full `GET` dump of all 22 keys is approximately 290 bytes — well
within this limit.  Large `SET` commands or long `ECHO` payloads are
similarly accommodated.

The 20 ms minimum telemetry period (`STREAM`) prevents buffer overrun
from rapid TLM frames.

The RAW250 radio transport uses a 247-byte MTU per fragment; the HAL
transparently fragments and reassembles messages of up to `REASM_MAX`
bytes.  The protocol layer is not aware of fragmentation.

---

## 13. Verification Examples

### ECHO Round-Trip

```
ECHO the-quick-brown-fox-jumped-over-the-lazy-dog
OK echo the-quick-brown-fox-jumped-over-the-lazy-dog
```

A ~200-byte payload tests reassembly in both directions over the relay.

### GET Dump (all keys)

```
GET
CFG ml=0.487 mr=0.481 kff=0.150 klf=1.000 klb=1.000 krf=1.000 krb=1.000 adjThr=0.500 adjGain=0.050 tw=120 pid.kp=300.000 pid.ki=0.000 pid.kd=0.000 pid.max=30.000 distScale=0.940 turnScale=1.070 minSpeed=50 sTimeout=500 tick=20 tlmPeriod=0
```

### SET and Verify

```
SET ml=0.490 mr=0.485
OK set ml=0.490 mr=0.485

GET ml mr
CFG ml=0.490 mr=0.485
```

### TLM Frame

```
SNAP
OK snap
TLM t=12345 mode=I enc=0,0 pose=0,0,0 line=120,340,330,118 color=21,30,18,80
```

### Motion End-to-End

Without correlation id:

```
D 200 200 300
OK drive l=200 r=200 mm=300
TLM t=12400 mode=D enc=45,44 pose=45,0,0
TLM t=12420 mode=D enc=89,88 pose=89,0,0
EVT done D reason=dist
```

With correlation id (host can match completion to originating request):

```
D 200 200 300 #5
OK drive l=200 r=200 mm=300 #5
TLM t=12400 mode=D enc=45,44 pose=45,0,0
EVT done D #5 reason=dist
```

With `stop=` early-exit clause:

```
T 200 200 1000 #12 stop=sensor:line0:ge:512
OK drive l=200 r=200 ms=1000 #12
… (line0 crosses 512 before 1 s elapses) …
EVT done T #12 reason=line0
```

### Clock-Sync Alignment

After a `ping_burst()` of 5 samples:

```python
cs = ClockSync()
cs.ping_burst(lambda cmd: proto._conn.send(cmd, read_ms=500)
              .get("responses", [""])[0])

# Translate a TLM timestamp to host time:
host_ms = cs.to_host_time(tlm_frame.t)

# Check staleness:
if cs.stale(max_age_s=60.0):
    cs.ping_burst(...)
```

Accuracy: within approximately half the minimum RTT (typically a few ms
over the half-duplex relay).

---

## 14. Debug Commands (`DBG …`)

Debug commands are diagnostic-only and always reply on the serial port
(`ForceReply::SERIAL`), regardless of which channel the command arrived on.
They are registered by `DebugCommands` and require `CMD_ACCESS_HARDWARE`
where they modify hardware routing.

### DBG OTOS BENCH — Enable / Disable Bench OTOS Sensor

```
DBG OTOS BENCH <0|1> [noiseXY=<f>] [noiseH=<f>] [drift=<f>]
```

- `1` — Enable bench mode: redirect the active OTOS pointer to `BenchOtosSensor`,
  which synthesizes pose from commanded wheel velocity instead of reading the
  real optical sensor.  Useful when the robot is on a stand and the floor sensor
  sees no motion.
- `0` — Disable bench mode: restore the real `OtosSensor`.
- Optional KV args (applied when `1`):
  - `noiseXY=<f>` — linear noise sigma (fraction of arc distance per tick;
    default 0.02 = 2%)
  - `noiseH=<f>` — yaw noise sigma (fraction of heading change per tick;
    default 0.01 = 1%)
  - `drift=<f>` — slow additive yaw drift in rad/s (default 0.0)
- Flag: `CMD_ACCESS_HARDWARE`.

Reply:

```
OK dbg otos bench=<0|1>
```

Examples:

```
DBG OTOS BENCH 1
OK dbg otos bench=1

DBG OTOS BENCH 0
OK dbg otos bench=0

DBG OTOS BENCH 1 noiseXY=0.02 noiseH=0.01 drift=0.0001
OK dbg otos bench=1
```

### DBG OTOS — Query Ideal / Errored / Fused Pose

```
DBG OTOS
```

No arguments.  Query the three-way pose comparison for bench session analysis:

- `ideal` — noiseless accumulator from `BenchOtosSensor` (ground truth of
  commanded motion; always `0,0,0` when bench mode is off).
- `otos` — errored accumulator from `BenchOtosSensor` (Gaussian noise + drift
  applied; this is what `readTransformed()` returned and what the EKF fused).
- `fused` — EKF-fused pose from `state.inputs.otosX/Y/H` (written by
  `Robot::otosCorrect()` each control tick).
- `err` — component-wise `ideal − otos` (noise accumulated so far).

Reply (one pose line, then OK):

```
ideal=<x>,<y>,<h> otos=<x>,<y>,<h> fused=<x>,<y>,<h> err=<dx>,<dy>,<dh>
OK dbg otos
```

Pose units: `x`, `y` in mm; `h` in radians (4 decimal places).

Example:

```
DBG OTOS
ideal=245.3,0.0,0.0000 otos=242.1,1.2,-0.0031 fused=243.7,0.6,-0.0015 err=3.2,-1.2,0.0031
OK dbg otos
```

---

## 15. Sim-Only: `SIMSET` / `SIMGET`

**These verbs exist ONLY in sim / `HOST_BUILD` binaries.** They are
registered by the optional `SimCommands` Commandable
(`source/commands/SimCommands.{h,cpp}`), which the ARM firmware target never
constructs and never links (069-003; see architecture-update.md Design
Rationale Decision 1). On real hardware, `SIMSET`/`SIMGET` are unrecognised
verbs and return `ERR unknown`, exactly like any other unregistered command
— no different from typing a typo'd verb.

`SIMSET`/`SIMGET` give the simulator's plant/error parameters
(`PhysicsWorld`, `SimHardware`) the same runtime-settable, wire-native
mechanism `SET`/`GET` gives `RobotConfig` — grammar mirrors §7 exactly.

### SIMGET

```
SIMGET [<key>…] [#id]
→ SIMCFG <key>=<value>… [#id]
```

With no arguments, dumps all registered sim keys. With one or more key
names, returns only those keys. For each unknown key a separate `ERR
badkey <key>` is emitted (does not prevent the SIMCFG line from being sent
for valid keys). A bare-dump `SIMGET` chunks its reply across multiple
`SIMCFG` lines if the full dump would exceed ~200 content bytes (mirrors
GET's CFG chunking, §7).

Examples:

```
SIMGET
SIMCFG bodyRotScrub=1.000 bodyLinScrub=1.000 trackwidthMm=128.000 motorOffsetL=1.000 motorOffsetR=1.000

SIMGET bodyRotScrub
SIMCFG bodyRotScrub=1.000

SIMGET badkey
ERR badkey badkey
```

### SIMSET

```
SIMSET <key>=<value>… [#id]
→ OK simset <applied-key>=<value>… [#id]
   [ERR badkey <key> [#id]]…
   [ERR badval <key> [#id]]…
```

Applies all valid keys atomically to the live plant. The entire SIMSET is
all-or-nothing: if any key is unknown or its value fails to parse as a
float, NO keys are applied and the plant is unchanged.

- Unknown key → `ERR badkey <key>`
- Non-numeric or empty value → `ERR badval <key>` (parse failure only —
  unlike `SET`, `SIMSET` keys have no cross-field invariant checks in this
  ticket's registry)

Examples:

```
SIMSET bodyRotScrub=0.92
OK simset bodyRotScrub=0.92

SIMSET bodyRotScrub=0.5 notARealKey=1.0
ERR badkey notARealKey

SIMSET trackwidthMm=abc
ERR badval trackwidthMm

SIMSET motorOffsetL=0.95 motorOffsetR=1.05
OK simset motorOffsetL=0.95 motorOffsetR=1.05
```

### Named Key Table

`kSimRegistry[]` (`source/commands/SimCommands.cpp`) dispatches through
named setter/getter FUNCTION POINTERS over `SimHardware&`, not `offsetof` —
`PhysicsWorld` is an encapsulated class with invariants, not a POD struct
(see architecture-update.md Design Rationale Decision 3). Rows 1–5 are
ticket 003's first batch; rows 6–17 (069-004) surface six per-wheel
encoder-report-error knobs (`PhysicsWorld`) and six OTOS-error knobs
(`SimOdometer`) that were already fully implemented but write-only, reachable
(if at all) only through legacy per-field ctypes test hooks.

| Key                 | Type  | Wire format | Default   | Meaning                                          |
|---------------------|-------|-------------|-----------|---------------------------------------------------|
| `bodyRotScrub`      | float | `%.3f`      | `1.000`   | Independent plant body-rotational scrub (069-002); combines multiplicatively with the legacy `_rotationalSlip`/`setSlip()` channel |
| `bodyLinScrub`      | float | `%.3f`      | `1.000`   | Independent plant body-linear scrub (069-002)     |
| `trackwidthMm`      | float | `%.3f`      | `128.000` | Robot trackwidth (mm); forwards to `SimHardware::setTrackwidth()` |
| `motorOffsetL`      | float | `%.3f`      | `1.000`   | Left-wheel plant offset factor (`PhysicsWorld::setOffsetFactor(0, f)`) |
| `motorOffsetR`      | float | `%.3f`      | `1.000`   | Right-wheel plant offset factor (`PhysicsWorld::setOffsetFactor(1, f)`) |
| `encScaleErrL`      | float | `%.3f`      | `0.000`   | Left-wheel REPORTED-encoder fractional scale error (`PhysicsWorld::setEncoderScaleError(0, err)`); true accumulator/chassis pose unaffected |
| `encScaleErrR`      | float | `%.3f`      | `0.000`   | Right-wheel REPORTED-encoder fractional scale error (`PhysicsWorld::setEncoderScaleError(1, err)`) |
| `encSlipL`          | float | `%.3f`      | `0.000`   | Left-wheel REPORTED-encoder under-report fraction (`PhysicsWorld::setEncoderSlip(0, fraction)`) |
| `encSlipR`          | float | `%.3f`      | `0.000`   | Right-wheel REPORTED-encoder under-report fraction (`PhysicsWorld::setEncoderSlip(1, fraction)`) |
| `encNoiseL`         | float | `%.3f`      | `0.000`   | Left-wheel REPORTED-encoder Gaussian noise sigma, mm (`PhysicsWorld::setEncoderNoise(0, sigma)`) |
| `encNoiseR`         | float | `%.3f`      | `0.000`   | Right-wheel REPORTED-encoder Gaussian noise sigma, mm (`PhysicsWorld::setEncoderNoise(1, sigma)`) |
| `otosLinScaleErr`   | float | `%.3f`      | `0.000`   | OTOS linear fractional scale error (`SimOdometer::setLinearScaleError()`) |
| `otosAngScaleErr`   | float | `%.3f`      | `0.000`   | OTOS angular fractional scale error (`SimOdometer::setAngularScaleError()`) |
| `otosLinNoise`      | float | `%.3f`      | `0.000`   | OTOS linear noise sigma (`SimOdometer::setLinearNoiseSigma()`) |
| `otosYawNoise`      | float | `%.3f`      | `0.000`   | OTOS yaw noise sigma (`SimOdometer::setYawNoiseSigma()`) |
| `otosLinDriftMmS`   | float | `%.3f`      | `0.000`   | OTOS linear drift, mm/second (wire unit). Converted to/from `SimOdometer`'s internal PER-TICK `_linearDriftPerTick` using `RobotConfig::controlPeriod`: `per_tick = per_second * (controlPeriod / 1000.0f)`, and the inverse on `SIMGET`. |
| `otosYawDriftDegS`  | float | `%.3f`      | `0.000`   | OTOS yaw drift, degrees/second (wire unit). Converted to/from `SimOdometer`'s internal PER-TICK `_yawDriftPerTick` (radians) using BOTH the same time-domain formula as `otosLinDriftMmS` AND a deg↔rad conversion. |

---

## 16. Development Commands (`DEV …`, dev builds only)

**These verbs exist ONLY when the firmware is built with `ROBOT_DEV_BUILD`
set** (`codal.json`'s `"config"` object — force-included into every
translation unit as a preprocessor `#define`, the same mechanism
`MICROBIT_BLE_ENABLED` already uses; see `source/commands/dev_commands.h`).
Sprint 077's `source/` tree sets `ROBOT_DEV_BUILD=1` — there is no
production motion firmware yet, so this dev-bench build IS the only build.
A future production firmware flips the define to `0` and `DEV` disappears
(`ERR unknown`), exactly like `SIMSET`/`SIMGET` disappear on real hardware
(§15) — same idea, different gating mechanism (`#if` here vs. a CMake
source-file exclusion there).

`DEV` drives the HAL directly for bench bring-up: individual motors by
**port** (1..4 — matching how `NezhaHal` instantiates one `NezhaMotor` per
port; never an L/R role name) and, through a bound port pair, the
`Drivetrain` subsystem. Every `DEV` handler that changes a motor or
drivetrain's commanded state builds a `msg::MotorCommand`/
`msg::DrivetrainCommand` and dispatches it through `apply()` — the full
message plane, capability validation included — rather than calling a
primitive setter directly. Replies use the standard taxonomy exclusively
(§3): `OK`/`ERR`, `EVT dev_watchdog` for the one asynchronous event this
family emits. No new reply tag is introduced.

### Authority: `DEV M` vs. `DEV DT`

This firmware runs only the dev loop — there is no planner to fight — so
there is exactly one authority conflict: a single motor commanded directly
by `DEV M` vs. the same motor being driven by the `Drivetrain` under
`DEV DT`. Rule:

- Any `DEV M <n>` verb that actually changes the motor's commanded state
  (`DUTY`, `VEL`, `POS`, `VOLT`, `NEUTRAL`, `RESET`) drops drivetrain
  authority — but only when the command is **accepted**; a capability
  rejection (`VOLT` on Nezha) never touched the motor and so never steals
  authority.
- Any `DEV DT` verb that commands the drivetrain (`VW`, `WHEELS`, `NEUTRAL`)
  (re)activates drivetrain authority.
- `DEV DT PORTS`, `DEV M <n> STATE`, `DEV M <n> CAPS`, `DEV DT STATE` are
  queries/bindings and never change authority.
- `DEV STOP` and `DEV DT STOP` always drop authority (see below).

### Port binding: `DEV DT PORTS`

`DEV DT PORTS <left> <right>` selects which two motor ports the
`Drivetrain` treats as its wheel pair. Default at boot: `1 2` (the robot's
normal drive pair). The coupled PID/governor bench rig uses ports `3 4`
(two motors with mechanically linked shafts — running one loads the
other). The binding **persists** across `DEV STOP` and a serial-silence
watchdog neutral event; it resets only on reboot.

### `DEV M <n> …` — Single-Motor Control

```
DEV M <n> DUTY <duty>        [%, -100..100]   → OK DEV M <n> applied=<duty/100>
DEV M <n> VEL <velocity>     [mm/s] signed    → OK DEV M <n> vel=<velocity>
DEV M <n> POS <position>     [deg]            → OK DEV M <n> pos=<position>
DEV M <n> VOLT <voltage>     [V]              → ERR unsupported volt (Nezha has no voltage mode)
DEV M <n> NEUTRAL <B|C>                        → OK DEV M <n> neutral=<B|C>
DEV M <n> RESET                                → OK DEV M <n> reset=1
DEV M <n> STATE                                → OK DEV M <n> pos=.. vel=.. applied=.. wedged=.. conn=..
DEV M <n> CAPS                                  → OK DEV M <n> duty=.. volt=.. vel=.. pos=.. enc=..
DEV M <n> CFG k=v ...                          → OK DEV M <n> <applied k=v ...>
```

`<n>` is the motor port, `1..4`.

- `DUTY <duty>` — open-loop duty cycle as a percentage (`-100..100`);
  converted to the `[-1, 1]` fraction `Motor::setDutyCycle()` takes before
  `apply()`. The `applied=` field echoes that fraction, not a hardware
  readback (the physical write happens on the next `tick()`).
- `VEL <velocity>` — `mm/s`, signed. Closed by the embedded velocity PID
  inside `NezhaMotor::tick()` (gains from `DEV M <n> CFG`); `VEL` alone does
  not guarantee motion if `kp`/`kff` are still at their zero boot default —
  see `CFG` below.
- `POS <position>` — `deg`. Onboard absolute-angle move via the Nezha's
  `0x5D` register; no PID involved.
- `VOLT <voltage>` — always `ERR unsupported volt` on a Nezha motor
  (`capabilities().voltage == false`). This is `Motor::apply()`'s capability
  gate firing, not a special case in the `DEV` handler — the same code path
  a future voltage-capable leaf would accept through.
- `NEUTRAL <B|C>` — `B` = brake, `C` = coast. Nezha has one physical stop
  path (duty 0 via `0x60`); both letters currently produce the same
  hardware action but are accepted and echoed distinctly for forward
  compatibility.
- `RESET` — zeroes the encoder (`MotorCommand.reset_position`); always
  accepted (not capability-gated).
- `STATE` — one line, always all five fields even when a leaf lacks an
  encoder (unset fields report `0`/`false`, never a blank).
- `CAPS` — `1`/`0` per capability flag (`duty`, `volt`, `vel`, `pos`,
  `enc` = `has_encoder`).
- `CFG k=v ...` — a **delta**, not a full replace: only the named keys
  change; every other field of the motor's current `msg::MotorConfig`
  (already applied, or the bench-placeholder boot default) is preserved.
  Recognized keys:

  | Key              | `MotorConfig` field         | Wire format |
  |------------------|------------------------------|-------------|
  | `kp`             | `vel_gains.kp`               | `%.3f`      |
  | `ki`             | `vel_gains.ki`               | `%.3f`      |
  | `kff`            | `vel_gains.kff`               | `%.3f`      |
  | `i_max`          | `vel_gains.i_max`             | `%.3f`      |
  | `kaw`            | `vel_gains.kaw`               | `%.3f`      |
  | `slew`           | `slew_rate`                   | `%.1f`      |
  | `min_duty`       | `min_duty`                    | `%.3f`      |
  | `travel_calib`   | `travel_calib`                | `%.4f`      |
  | `fwd_sign`       | `fwd_sign`                     | `%d`        |
  | `vel_filt_alpha` | `vel_filt_alpha`               | `%.3f`      |

  An unrecognized key emits `ERR badkey <key>` (does not prevent the
  other, valid keys in the same command from applying — mirrors `SET`'s
  per-key error reporting, §7). The final `OK` line lists only the keys
  that actually applied.

Examples:

```
DEV M 1 DUTY 30
OK DEV M 1 applied=0.30

DEV M 1 STATE
OK DEV M 1 pos=177.8 vel=0.0 applied=0.30 wedged=0 conn=1

DEV M 1 VOLT 3
ERR unsupported volt

DEV M 1 RESET
OK DEV M 1 reset=1

DEV M 1 CAPS
OK DEV M 1 duty=1 volt=0 vel=1 pos=1 enc=1

DEV M 3 CFG kp=0.8 slew=400
OK DEV M 3 kp=0.800 slew=400.0
```

### `DEV DT …` — Drivetrain Control

```
DEV DT PORTS <left> <right>          → OK DEV DT ports=<left>,<right>
DEV DT VW <v_x> <v_y> <omega>        [mm/s mm/s rad/s] → OK DEV DT vx=.. vy=.. omega=..
DEV DT WHEELS <left> <right>         [mm/s] → OK DEV DT left=.. right=..
DEV DT NEUTRAL <B|C>                  → OK DEV DT neutral=<B|C>
DEV DT STATE                          → OK DEV DT active=<0|1> ports=<l>,<r> vel=<vL>,<vR>
DEV DT STOP                           → OK DEV DT STOP
DEV DT CFG k=v ...                     → OK DEV DT <applied k=v ...>
```

- `PORTS <left> <right>` — see "Port binding" above. Also refreshes the
  Drivetrain's cached capability read of the newly-bound pair
  (`DrivetrainCapabilities.onboard_position`).
- `VW <v_x> <v_y> <omega>` — a body twist in `mm/s`, `mm/s`, `rad/s`.
  `v_y` is accepted on the wire but ignored: this Drivetrain is
  differential-only this sprint (`capabilities().holonomic == false`), so
  only `v_x`/`omega` reach `BodyKinematics::inverse()`. Ratio-governed (see
  below).
- `WHEELS <left> <right>` — direct per-wheel velocity targets (`mm/s`,
  signed), bypassing kinematics. Also ratio-governed. This is the arm the
  coupled bench rig's curve tests drive (ports `3 4`).
- Both `VW` and `WHEELS` pass through the ratio (sync) governor
  (`Drivetrain::governRatio()`): if one wheel is bogged down relative to
  its own commanded target, BOTH wheel targets are scaled down together so
  the commanded left/right ratio (curvature) is held rather than letting
  the healthy wheel run away. Observable by hand-loading one wheel while
  polling `DEV DT STATE`.
- `NEUTRAL <B|C>` — same B/C semantics as `DEV M`'s `NEUTRAL`, applied to
  the whole Drivetrain.
- `STATE` — `vel=` reports the current commanded (pre-governor) per-wheel
  targets, `active=` is the authority flag (§ above), `ports=` is the
  current `PORTS` binding.
- `STOP` — the drivetrain-scoped stop: neutrals the Drivetrain AND its
  currently-bound pair, and drops drivetrain authority — but leaves any
  OTHER motor (independently under `DEV M` control) untouched. Contrast
  with the global `DEV STOP` below.
- `CFG k=v ...` — a **delta**, not a full replace, against the single
  shared `msg::DrivetrainConfig` (one Drivetrain instance, not per bound
  pair — unlike `DEV M <n> CFG`, this is not indexed by port). Added in
  077-007 to close a gap found in this ticket's HITL bench pass: `sync_gain`
  (the ratio governor's gain) boots at `0` (governor OFF) with no other way
  to change it short of a reflash. Never capability-gated, never changes
  authority. Recognized keys:

  | Key           | `DrivetrainConfig` field | Wire format |
  |---------------|--------------------------|-------------|
  | `sync_gain`   | `sync_gain`              | `%.3f`      |
  | `trackwidth`  | `trackwidth`             | `%.1f`      |

  An unrecognized key emits `ERR badkey <key>` (mirrors `DEV M <n> CFG`'s
  per-key error reporting); the final `OK` line lists only the keys that
  actually applied.

Examples:

```
DEV DT PORTS 3 4
OK DEV DT ports=3,4

DEV DT VW 150 0 0
OK DEV DT vx=150.0 vy=0.0 omega=0.000

DEV DT STATE
OK DEV DT active=1 ports=3,4 vel=150.0,150.0

DEV DT WHEELS 80 40
OK DEV DT left=80.0 right=40.0

DEV DT CFG sync_gain=0.8
OK DEV DT sync_gain=0.800

DEV DT STOP
OK DEV DT STOP
```

### `DEV STATE` / `DEV STOP` — Everything, at Once

```
DEV STATE   → one OK line per motor (ports 1..4, DEV M <n> STATE's shape)
              + one OK DEV DT line (DEV DT STATE's shape) -- 5 lines total
DEV STOP    → all four motors neutral, drivetrain idle, authority dropped
              → OK DEV STOP
```

`DEV STATE` is a pure query (no authority change); `DEV STOP` is the same
"neutralize everything" action the serial-silence watchdog fires on
expiry (see below) — both go through the identical code path so there is
exactly one "make it safe" implementation to audit.

Example:

```
DEV STATE
OK DEV M 1 pos=177.8 vel=0.0 applied=0.00 wedged=0 conn=1
OK DEV M 2 pos=0.0 vel=0.0 applied=0.00 wedged=1 conn=1
OK DEV M 3 pos=0.0 vel=0.0 applied=0.00 wedged=1 conn=1
OK DEV M 4 pos=0.0 vel=0.0 applied=0.00 wedged=1 conn=1
OK DEV DT active=0 ports=1,2 vel=0.0,0.0

DEV STOP
OK DEV STOP
```

### `DEV WD <window>` — Serial-Silence Watchdog Window

```
DEV WD <window>   [ms, 50..60000] → OK DEV WD window=<window>
```

Sets the serial-silence watchdog's window at runtime. Default at boot:
`1000` ms.

### Serial-Silence Watchdog — Non-Negotiable

Every `DEV`/liveness command **line** that arrives on either comms channel
(serial or radio) resets a wall-clock timer — regardless of the line's
content or whether it parsed to a known verb. If no line arrives within
the current window, the firmware:

1. Commands **all four** motors to neutral (`Neutral::BRAKE`), regardless
   of which family (`DEV M` or `DEV DT`) was last authoritative.
2. Idles the Drivetrain and drops drivetrain authority.
3. Emits `EVT dev_watchdog` (no body, no `#id` — this is not tied to any
   single originating command) on the serial channel.

This is deliberately **not** the legacy motion-watchdog flag mechanism
(only certain command kinds reset that one) — a silent host is a silent
host whether it stopped sending `PING` or `DEV DT VW`. This exists even
though this is a bench-only build with no planner to fight: the robot's
runaway history (`.claude/rules/hardware-bench-testing.md` and prior
incident notes) makes it a hard requirement, not a nice-to-have.

Example:

```
DEV M 1 VEL 100
OK DEV M 1 vel=100.0
… (host goes silent for > the current window) …
EVT dev_watchdog
DEV M 1 STATE
OK DEV M 1 pos=412.0 vel=0.0 applied=0.00 wedged=0 conn=1
```

---

## Appendix: Removed v1 Commands

The following v1 command vocabulary is removed in v2.  Any of these
verbs returns `ERR unknown <verb>`.

| Removed verb / prefix    | v1 meaning                             |
|--------------------------|----------------------------------------|
| `K*` (e.g. `KCP`, `KCL`) | Per-constant calibration set/get       |
| `ENC`                    | Encoder query                          |
| `SO`                     | Sensor output (legacy)                 |
| `SSE`, `SSO`, `SSC`, `SSL` | Streaming sensor commands             |
| `HELLO`                  | Legacy identity / handshake            |
| `DEVICE:`                | Legacy device prefix format            |
| `X`                      | Legacy stop/reset                      |
| Packed motion (`S+200-150`) | Sign-prefix packed speed arguments  |
