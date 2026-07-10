# Protocol v2 Wire Specification

> **SUPERSEDED by [`docs/protocol-v3.md`](protocol-v3.md).** Sprint 097
> ("Protocol v3 Sprint 3: host completion and text retirement") replaced
> the firmware's text command plane described below with a schema-driven
> binary envelope plane (`*B<base64(protobuf)>`), a 3-verb text safety
> rump (`STOP`/`PING`/`HELLO`), and a host-side `rogo proxy` PTY bridge
> for legacy text clients. Most of §6–§10 and §13 below now describe
> verbs that are **no longer live on the firmware text plane** — this
> file is kept for history, not deleted, but `docs/protocol-v3.md` is the
> current wire reference. §11 (OTOS/Port I/O) and §16 (Development
> Commands) still describe those two families' text grammar accurately,
> but that grammar has been off the wire (unregistered) since before
> sprint 097 started — see protocol-v3.md §8.

Version 2 of the Nezha firmware command/telemetry protocol.
Hard break from v1 — no backward compatibility.

---

## 1. Overview

Protocol v2 is a line-oriented text protocol: one message equals one
`\n`-terminated line.  Tokens are whitespace-delimited.  `key=value`
pairs carry named parameters.  Only the first token (the verb) is
upper-cased; all remaining tokens, keys, and values preserve the case
as sent.  The protocol identifier is `proto=2`.

v1 commands (`K*`, `ENC`, `SO`, `SSE`, `SSO`, `SSC`, `SSL`, and packed
sign-prefix motion verbs) are removed.  Any unrecognised verb returns
`ERR unknown`.  `HELLO` and the `DEVICE:` boot announcement (also v1
vocabulary) are re-added under v2, in a new `NEZHA2`/`robot` wire format —
see §6.

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

### DEVICE: Boot Announcement

```
DEVICE:NEZHA2:robot:<name>:<serial>
```

Emitted once, unsolicited, as the **first line out on both the serial
link and the radio link**, immediately after comms bring-up — before the
main loop starts, before anything else is sent.  Colon-delimited, no
`#id` correlation (nothing requested it).  Mirrors the microbit-radio-relay
protocol's own boot announcement (`DEVICE:RADIOBRIDGE:relay:<name>:<serial>`,
see <https://robots.jointheleague.org/subsystems/microbit-radio-relay/protocol/>
§3.4) so host tooling that already classifies devices off field 1/field 2
works unchanged.

| Field      | Value                                                          |
|------------|-----------------------------------------------------------------|
| 1          | Literal `NEZHA2` (model — matches `ID`'s `model=` field)        |
| 2          | Literal `robot` (role — the relay's own boot banner uses `relay`) |
| `<name>`   | micro:bit friendly name — same source as `ID`'s `name=` field    |
| `<serial>` | Hardware serial number, decimal `uint32_t` — same source as `ID`'s `serial=` field |

Radio is a fire-and-forget broadcast with no link-up handshake: the boot
radio banner only reaches a host if a relay is already listening on the
configured channel when the robot boots.  A missed boot radio banner is
not a failure — the serial banner reaches a directly-connected host
reliably, and `HELLO` (below) is the reliable way to re-request the
banner over either channel, e.g. once a relay attaches after boot.

Example (serial, at boot):

```
DEVICE:NEZHA2:robot:GUTOV:1234567
```

### HELLO

```
HELLO [#id]
→ DEVICE:NEZHA2:robot:<name>:<serial>
```

Re-emits the identical banner described above, on whichever channel
`HELLO` arrived on (serial → serial reply, radio → radio reply) — the
reliable, on-demand way to (re)request the robot's identity banner.  The
response tag is `DEVICE:...`, not `OK`: like `ID`, this is its own reply
taxonomy rather than an `OK`/`ERR` wrapper, and (also like the boot
banner) it carries no `#id` echo even when the request included one.

Example:

```
HELLO
DEVICE:NEZHA2:robot:GUTOV:1234567
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

All 22 registered config keys (plus five added since, see the sprint notes
below the table), their types, defaults, and the v1 equivalents they
replace. **Status** marks which keys `source/`'s `SET`/`GET`
(`source/commands/config_commands.cpp`, sprint 084 ticket 006) actually
implements as of this sprint — see architecture-update.md (084) Decision 2
for the full key-by-key rationale. `current` rows are live in `source/`
today; `superseded`/`not carried forward` rows are `source_old`-only —
`SET`/`GET` against them in `source/` returns `ERR badkey`, identical wire
behavior to any never-existed key. This is not a case of a key being
forgotten; each disposition below was a deliberate sprint-084 decision.

| Key             | Type        | Wire format | Default  | Meaning                                 | v1 equiv  | Status (`source/`) |
|-----------------|-------------|-------------|----------|------------------------------------------|-----------|---------------------|
| `ml`            | float       | `%.3f`      | `0.487`  | mm per degree of rotation, left wheel   | `KCL`     | current (084-006) — `MotorConfig.travel_calib`, bound-pair left |
| `mr`            | float       | `%.3f`      | `0.481`  | mm per degree of rotation, right wheel  | `KCR`     | current (084-006) — `MotorConfig.travel_calib`, bound-pair right |
| `kff`           | float       | `%.3f`      | `0.150`  | Feed-forward gain                       | `KFF`     | superseded (084-006 Decision 2) — folded into `pid.kff` below |
| `klf`           | float       | `%.3f`      | `1.000`  | Left-forward motor scale factor         | `KLF`     | superseded (084-006 Decision 2) — no per-direction scale concept in the new motor model |
| `klb`           | float       | `%.3f`      | `1.000`  | Left-backward motor scale factor        | `KLB`     | superseded (084-006 Decision 2) — see `klf` |
| `krf`           | float       | `%.3f`      | `1.000`  | Right-forward motor scale factor        | `KRF`     | superseded (084-006 Decision 2) — see `klf` |
| `krb`           | float       | `%.3f`      | `1.000`  | Right-backward motor scale factor       | `KRB`     | superseded (084-006 Decision 2) — see `klf` |
| `adjThr`        | float       | `%.3f`      | `0.500`  | Slower-wheel adjustment threshold       | —         | superseded (084-006 Decision 2) — replaced by `DrivetrainConfig.sync_gain` (`DEV DT CFG sync_gain=`) |
| `adjGain`       | float       | `%.3f`      | `0.050`  | Slower-wheel adjustment gain            | —         | superseded (084-006 Decision 2) — see `adjThr` |
| `tw`            | float-as-int| `%d`        | `120`    | Track width in mm                       | `KAT`     | current (084-006) — `DrivetrainConfig.trackwidth` |
| `pid.kp`        | float       | `%.3f`      | `300.000`| Ratio PID proportional gain             | `KCP`     | current (084-006) — both bound motors' `MotorConfig.vel_gains.kp` |
| `pid.ki`        | float       | `%.3f`      | `0.000`  | Ratio PID integral gain                 | `KCI`     | current (084-006) — both bound motors' `MotorConfig.vel_gains.ki` |
| `pid.kd`        | float       | `%.3f`      | `0.000`  | Ratio PID derivative gain               | `KCD`     | superseded (084-006 Decision 2) — no `kd` term in the new `Gains{kp,ki,kff,i_max,kaw}` shape |
| `pid.max`       | float       | `%.3f`      | `30.000` | Ratio PID output clamp                  | `KCM`     | superseded (084-006 Decision 2) — replaced by `pid.iMax` (the `Gains.i_max` integrator clamp) below |
| `pid.kff`       | float       | `%.3f`      | `0.004`  | Velocity-loop feed-forward gain         | —         | **new, current (084-006)** — both bound motors' `MotorConfig.vel_gains.kff`; supersedes standalone `kff` above |
| `pid.iMax`      | float       | `%.3f`      | `0.300`  | Velocity-loop integrator clamp          | —         | **new, current (084-006)** — both bound motors' `MotorConfig.vel_gains.i_max`; supersedes `pid.max` above |
| `pid.kaw`       | float       | `%.3f`      | `0.000`  | Velocity-loop back-calc anti-windup gain | —        | **new, current (084-006)** — both bound motors' `MotorConfig.vel_gains.kaw` |
| `rotSlip`       | float       | `%.3f`      | `0.000`  | Rotational-slip correction factor (0 = unset -> 1.0) | — | **new, current (084-006)** — `DrivetrainConfig.rotational_slip`; not implemented in `source/` before this ticket |
| `distScale`     | float       | `%.3f`      | `0.940`  | Distance command scale factor           | `KDS`     | superseded (084-006 Decision 2) — no fudge factor needed against correctly-modeled `BodyKinematics` |
| `turnScale`     | float       | `%.3f`      | `1.070`  | Turn command scale factor               | `KTS`     | superseded (084-006 Decision 2) — see `distScale` |
| `minSpeed`      | int32       | `%d`        | `50`     | Minimum drive speed (mm/s)              | `KMS`     | current (084-006) — `PlannerConfig.min_speed` (float-as-int wire encoding; the new-tree field is a `float`) |
| `sTimeout`      | int32       | `%d`        | `500`    | Streaming watchdog timeout (ms)         | `KST`     | current (084-006) — ticket 002's `StreamingDriveWatchdog` window (plain field, no message) |
| `tick`          | int32       | `%d`        | `20`     | Main-loop tick period (ms)              | `KTK`     | superseded (084-006 Decision 2) — loop cadence is structural (sprint 079), not a runtime knob |
| `tlmPeriod`     | int32       | `%d`        | `0`      | TLM streaming period (ms); 0 = off      | —         | superseded (084-006 Decision 2) — redundant with the `STREAM <ms>` verb itself (082) |
| `ekfQxy`        | float       | `%.3f`      | `200.000`| EKF process noise: position (mm²/s)     | —         | current (084-006) — `DrivetrainConfig.ekf_q_xy` |
| `ekfQtheta`     | float       | `%.3f`      | `0.500`  | EKF process noise: heading (rad²/s)     | —         | current (084-006) — `DrivetrainConfig.ekf_q_theta` |
| `ekfQv`         | float       | `%.3f`      | `5000.000`| EKF process noise: body speed (mm²/s³) | —         | not carried forward (084-006) — `DrivetrainConfig.ekf_q_v` exists but is outside this sprint's approved key table |
| `ekfQomega`     | float       | `%.3f`      | `1.000`  | EKF process noise: yaw rate (rad²/s³)   | —         | not carried forward (084-006) — see `ekfQv` |
| `ekfROtosXy`    | float       | `%.3f`      | `50.000` | EKF OTOS measurement noise: position (mm²) | —      | current (084-006) — `DrivetrainConfig.ekf_r_otos_xy` |
| `ekfROtosTheta` | float       | `%.3f`      | `0.000`  | EKF OTOS measurement noise: heading (rad²) | —      | **new, current (084-006)** — `DrivetrainConfig.ekf_r_otos_theta`; closes 082 Decision 4's deferred item (`source_old` called this field's key `ekfRHead`, itself pre-existing drift never backfilled into this table — the new tree uses the field-matching spelling instead) |
| `ekfROtosV`     | float       | `%.3f`      | `200.000`| EKF OTOS measurement noise: body speed (mm²/s²) | — | not carried forward (084-006) — see `ekfQv` |
| `ekfREncV`      | float       | `%.3f`      | `100.000`| EKF encoder measurement noise: body speed (mm²/s²) | — | not carried forward (084-006) — see `ekfQv` |

(Sprint 069-001: the `ekfQxy`/`ekfQtheta`/`ekfQv`/`ekfQomega`/`ekfROtosXy`/
`ekfROtosV`/`ekfREncV` rows closed 067's Open Question 5 in `source_old` --
a live `SET` routed through `Drive::configure()`'s `setNoise()` push, which
updated EKF fusion noise WITHOUT resetting fused pose/covariance. `source/`'s
sprint-084 `SET`/`GET` does not preserve that non-resetting behavior --
`PoseEstimator::configure()` (the `source/` equivalent) calls
`EkfTiny::init()` unconditionally, which DOES re-zero the fused
pose/covariance on every drivetrain-scoped `SET` (`tw`/`rotSlip`/`ekf*`) --
see architecture-update.md (084) Decision 2 and `config_commands.h`'s file
header for this known, deliberate consequence. This table has pre-existing
drift from several long-landed `source_old` keys, e.g. `vel.kP`, `ekfRHead`
itself -- not backfilled here, out of scope per ticket 068-001's Open
Question 1 precedent.)

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

> **Sprint 082 note — minimal subset in `source/`.** This section documents
> the OLD tree's (`source_old/`) full `STREAM`/`SNAP`/`TLM` richness, kept
> here as the field-syntax/wire-format reference. As of sprint 082 (ticket
> 004), the new `source/` tree implements only a deliberately minimal
> subset (architecture-update.md Decision 5):
>
> - **No `STREAM fields=<csv>` subscription.** Every frame always carries
>   the full fixed field set below (`enc= vel= pose= encpose= otos= twist=`);
>   there is no second, smaller field set to select between yet.
> - **`otosconn=<0|1>` (sprint 092, ticket 002) is a field this section's
>   legacy table below does NOT document** -- it is new to `source/`, not
>   carried over from `source_old/`. Emitted as a standalone token
>   immediately after `otos=`, sharing that field's own omission gate (both
>   present or both absent together, never one without the other): `1` iff
>   `Hal::Odometer::connected()` was true this pass (a real device detected
>   and answering), `0` otherwise (matches `Hal::NullOdometer` and a
>   `Hal::OtosOdometer` that never detected its chip's product ID at
>   `begin()`). Added as a diagnostic for the frozen-fused-pose
>   investigation (`clasi/issues/poseestimator-fused-pose-frozen-on-
>   hardware.md`) -- see that ticket's completion notes: no existing wire
>   verb told a bench session whether a real OTOS chip was ever detected,
>   as opposed to `otos=` merely holding its all-zero boot-default because
>   no odometer ever wrote it.
> - **No D10 idle-rate refinement.** The periodic emission period is
>   exactly `periodMs` (clamped to the 20ms floor) at all times — it does
>   NOT relax to `max(period, 500ms)` when idle.
> - **No channel-rebinding nuance** beyond "the channel that most recently
>   issued `STREAM` is the bound recipient" — described below under
>   *Channel binding*.
> - `mode=` implements the full `I`/`S`/`T`/`D`/`G` vocabulary as of sprint
>   084 (ticket 005) — see the *`mode=` field verb-sharing* note below the
>   table for exactly which verb produces which character, including the
>   deliberate `TURN`/`RT`/bounded-`R` → `T` collapse (architecture-
>   update.md (084) Decision 6).
> - `line=`/`color=`/`ekf_rej=`/`otos_health=`/`wedge=` do not exist in
>   `source/` yet (no line/color sensor leaves, no EKF rejection counters,
>   no OTOS health/wedge detector wiring this sprint).
>
> Do not assume the new tree already has the old tree's full richness
> documented below just because this section is unchanged — check
> `source/telemetry/tlm_frame.{h,cpp}` and `source/commands/
> binary_channel.{h,cpp}`'s `tickTelemetry()` (097-011: relocated from
> `telemetry_commands.{h,cpp}`) for what actually ships.

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

**`mode=` field verb-sharing (084-005).** In `source/` (as opposed to
`source_old/`, where this table's characters originate), `mode=` is derived
from exactly one source — `Subsystems::Planner::state().mode` — and is `I`
if and only if `Planner::hasActiveCommand()` is false. Each character below
is shared by every verb family listed, not just the one it is named after:

| Wire char | `Planner` state              | Verbs that produce it                                  |
|-----------|-------------------------------|----------------------------------------------------------|
| `I`       | no active `Planner` command   | boot; after any `EVT done`/`safety_stop`; after `STOP`   |
| `S`       | `DriveMode::STREAMING`         | `S`, `VW`, a bare `R` (no `stop=` clause)                |
| `T`       | `DriveMode::TIMED`             | `T`, an `R` with a `stop=` clause, `TURN`, `RT`          |
| `D`       | `DriveMode::DISTANCE`          | `D`                                                       |
| `G`       | `DriveMode::GO_TO`             | `G`                                                       |

`TURN`/`RT`/a `stop=`-bearing `R` sharing `T` with a plain timed drive is a
**deliberate, approved scope decision** (architecture-update.md (084)
Decision 6), not an oversight: `msg::DriveMode` has no dedicated `TURN`/
`ROTATE` value (mirroring `source_old`'s own internal collapse of
`STREAM`/`TIMED`/`ARC` into a single `Goal::VELOCITY`), and no present
consumer — including TestGUI's tour-completion logic, which only needs
`mode=I` at idle — needs to distinguish "turning" from "driving a bounded
straight" over the wire. A future sprint may revisit this (Decision 6's own
Open Question 2) if that ever changes; it is not revisited here.

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
| `EVT done R [#id]`     | Arc drive ended via a `stop=` clause (a bare `R` runs open-ended until `STOP`, which emits no event) |
| `EVT done TURN [#id]`  | Absolute-heading turn reached the target within `eps` (or a `stop=` clause fired) |
| `EVT done RT [#id]`    | Relative turn reached the target per-wheel encoder arc (or a `stop=` clause fired) |
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

Any open-loop motion command (`VW`, `S`, `R`, `T`, `D`, `TURN`, `RT`) may
carry one or more `stop=<kind>:<args>` clauses as `key=value` pairs.  Each
clause adds a stop condition that fires when its condition is satisfied;
conditions are OR-combined.  Up to 4 `stop=` clauses are accepted per
command (`kMaxStopConds = 4`) — `TURN` and `RT` each reserve one of those 4
slots for their own built-in stop (`heading` / `rot` respectively), so up to
3 additional `stop=` clauses are accepted on those two; clauses beyond the
available slots are silently dropped, not an error.

| Clause                              | Fires when                                                        |
|-------------------------------------|-------------------------------------------------------------------|
| `stop=t:<ms>`                       | Duration ≥ ms milliseconds                                        |
| `stop=d:<mm>`                       | Average encoder travel ≥ mm millimetres                           |
| `stop=line:<ge\|le>:<thr>`          | Any of line[0..3] satisfies the threshold                         |
| `stop=sensor:<ch>:<ge\|le>:<thr>`   | Named channel satisfies the threshold                             |
| `stop=color:<h>:<s>:<v>:<dist>`     | HSV colour distance from target ≤ dist                            |
| `stop=heading:<cdeg>:<eps_cdeg>`    | Heading within eps of target (centi-degrees; delta from the drive's own starting heading, not an absolute compass heading) |
| `stop=rot:<arc_mm>`                 | Per-wheel encoder arc ≥ arc_mm                                    |

Channel names for `stop=sensor:`: `line0`..`line3`, `colorR`..`colorC`,
`analogIn0`..`analogIn3`.

`sensor=<ch>:<op>:<thr>` is accepted as a back-compat alias for
`stop=sensor:<ch>:<op>:<thr>`.

**`sensor`/`color`/`line` — recognized, not yet supported (sprint 084).**
The greenfield-rebuilt `source/` tree's motion executor (`Subsystems::
Planner`, sprint 084 ticket 001) implements `t`/`d`/`heading`/`rot` in full.
`stop=sensor:...`, `stop=color:...`, `stop=line:...`, and the `sensor=...`
back-compat alias are all recognized syntactically — the wire parser matches
their kind prefix and does not fall through to a generic `ERR unknown`/`ERR
badarg missing key`-class failure meant for genuinely malformed input — but
every one of them is rejected with `ERR badarg`, since no line or color
sensor `Hal` leaf exists yet in that tree. A future sprint that lands the
corresponding sensor leaf can implement these three without any wire-shape
change (sprint 084 architecture-update.md, Decision 4).

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

**`sTimeout` is a live, production watchdog (sprint 084).** It is a
separate timer from `DEV WD`'s bench-only serial-silence watchdog (which
resets on *any* command, on *any* channel, regardless of content):
`sTimeout` is fed *only* by `S`, and only matters while a streaming
(`S`-driven) goal is the one actually active — conflating the two would
defeat the point of either. It is not yet `SET`/`GET`-able (still a fixed
500&nbsp;ms default) — sprint 084 ticket 006 wires it into the top-level
config-registry surface alongside the rest of §7's key table.

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

### R — Arc Drive (constant curvature, open-loop)

```
R <speed> <radius> [stop=<kind>:<args>]… [#id]
→ OK arc speed=<speed> radius=<radius> [#id]
  … (only if a stop= clause fires) …
  EVT done R [#id] reason=<token>
```

Drives a constant-curvature arc: `speed` is the forward body speed (mm/s)
and `radius` is the arc radius (mm).  The firmware computes the yaw rate as
`omega = speed / radius` (0 when `radius` is 0, i.e. a straight line) and
enters `VELOCITY` mode — open-loop, matching `S`'s family: **`R` has no
built-in stop of its own.**  A bare `R` (no `stop=` clause) runs open-ended
until `STOP` (which emits no event, same as `S`/`VW`); optional `stop=`
clauses may be appended, and the first one that fires ends the drive and
emits `EVT done R [#id] reason=<token>`.

Positive `radius` is a CCW (left) arc; negative `radius` is a CW (right)
arc; `radius = 0` degenerates to a straight-line body-velocity command
(same effect as `VW <speed> 0`).

Ranges:
- `speed` — −1 000 … +1 000 mm/s.  Out of range → `ERR range speed`.
- `radius` — −10 000 … +10 000 mm.  Out of range → `ERR range radius`.

Example:

```
R 200 500
OK arc speed=200 radius=500

R 200 500 stop=d:400
OK arc speed=200 radius=500
… (stops after 400 mm of average encoder travel) …
EVT done R reason=dist

R 200 500 #9
OK arc speed=200 radius=500 #9
… (runs open-ended; a later STOP halts it with no EVT) …
```

### TURN — Absolute-Heading Turn-in-Place (closed-loop, fused heading)

```
TURN <heading> [eps=<cdeg>] [stop=<kind>:<args>]… [#id]
→ OK turn heading=<heading> eps=<eps> [#id]
  … (later, asynchronously) …
  EVT done TURN [#id] reason=<token>
```

Rotates in place to the **absolute** heading `heading` (centi-degrees,
compass-style: 0 is the heading at boot/last `ZERO pose`/`SI`).  The
firmware reads the current fused pose heading (`PoseEstimator::fusedPose()`)
at command time, resolves the shortest-path signed turn direction, and spins
at a fixed rate until the fused heading is within `eps` (centi-degrees,
default 300 = 3°) of the target — a **`heading` stop condition**, the one
built-in stop this verb always carries.  Optional `stop=` clauses may be
appended (up to 3 more, since one of the 4 available slots is reserved for
the built-in `heading` stop); the first condition that fires ends the turn
and emits `EVT done TURN [#id] reason=<token>`.

Ranges:
- `heading` — −18 000 … +18 000 cdeg (±180°).  Out of range →
  `ERR range heading`.
- `eps` — 10 … 1 800 cdeg (0.1° … 18°), default 300.  Out of range →
  `ERR range eps`.

Example:

```
TURN 9000
OK turn heading=9000 eps=300
EVT done TURN reason=heading

TURN -9000 eps=100
OK turn heading=-9000 eps=100
EVT done TURN reason=heading

TURN 9000 stop=t:2000
OK turn heading=9000 eps=300
… (stops at 90° or 2 s, whichever comes first) …
EVT done TURN reason=time
```

### RT — Relative Turn-in-Place (closed-loop, encoder arc)

```
RT <relAngle> [stop=<kind>:<args>]… [#id]
→ OK rt rot=<relAngle> [#id]
  … (later, asynchronously) …
  EVT done RT [#id] reason=<token>
```

Rotates in place by the **relative** angle `relAngle` (centi-degrees;
positive is CCW/left, negative is CW/right) from the robot's current
heading.  Unlike `TURN`, `RT` closes the loop against the **per-wheel
encoder arc** (a `rot` stop condition — the geometry-verified arc for the
requested angle, independent of the fused pose/OTOS), the one built-in stop
this verb always carries.  Optional `stop=` clauses may be appended (up to 3
more, same 4-slot budget as `TURN`); the first condition that fires ends the
turn and emits `EVT done RT [#id] reason=<token>`.

Range: `relAngle` — −180 000 … +180 000 cdeg (±1 800°, up to 5 full turns).
Out of range → `ERR range relAngle`.

Example:

```
RT 9000
OK rt rot=9000
EVT done RT reason=rot

RT -9000
OK rt rot=-9000
EVT done RT reason=rot

RT 9000 stop=t:500
OK rt rot=9000
… (stops at 90° of arc or 500 ms, whichever comes first) …
EVT done RT reason=time
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

### SI — Set World Pose

```
SI <x> <y> <h> [#id]   → OK setpose x=<x> y=<y> h=<h> [#id]
```

Re-anchors the robot's **believed** world pose — the pose motion verbs
(`G`/`TURN`/`RT`) steer against — to `(x, y, h)` without moving the robot
itself.  Establishes the onboard pose from an external fix (e.g. a
downward-facing playfield camera), so a subsequent `G`/`D`/`TURN` drives in
the correct world frame.

- `x`, `y` — position, mm.
- `h` — heading, centi-degrees.

All three arguments are plain integers with no range check (values are cast
internally; an absurd input is the caller's mistake, not a wire error). Too
few arguments → `ERR badarg`.

`SI` re-anchors the encoder-only dead-reckoning reading (`encpose=`), the
EKF's fused belief (`pose=`) — see `TLM`'s field list (§8) — **and** (as of
sprint 084 ticket 008) the active `Hal::Odometer`'s own world-frame reading
(`OV`, §11), issuing all three in the SAME wire dispatch. `source/`'s
`handleSI` (`source/commands/text_channel.cpp`, formerly
`pose_commands.cpp`) calls
`PoseEstimator::setPose()` and then, if `hardware->odometer()` is non-null,
also stages an `OdometerCommand::SET_POSE` matching the same `(x, y, h)` —
mirroring `source_old`'s own two-call `handleSI` (`PoseEstimator` reset +
`hal.otos().setWorldPose()`). Because the very next fusion pass therefore
reads an odometer sample that already agrees with the freshly-set anchor,
the EKF update's residual is zero and `pose=` reads back **exactly**
`x`,`y`,`h` too — a separate `OV` fix is no longer needed to avoid the
partial-correction-back-toward-the-old-frame hazard earlier drafts of this
section described (see `tests/sim/unit/test_pose_commands.py`'s
`test_si_reanchors_both_encpose_and_the_fused_pose_exactly` and
`tests/sim/unit/test_config_pose_set_otos_surface.py`'s
`test_si_teleports_fused_pose_confirmed_via_snap_and_through_otos_op`, both
of which read the post-`SI` pose back through `OP` too).
`Subsystems::NezhaHardware::odometer()` is null (no real OTOS driver this
program — see `clasi/issues/nezha-hardware-otos-driver-for-new-source-tree.md`),
so on hardware `SI`'s odometer re-anchor step is a no-op, unchanged from its
pre-008 behavior there.

`SI` does not itself cancel an active drive: a `G`/`TURN` in progress keeps
pursuing its goal using the newly-anchored pose on its very next tick,
which may produce a visible course correction rather than a smooth
continuation.

Example:

```
SI 1230 450 2700
OK setpose x=1230 y=450 h=2700
```

### ZERO — Zero Encoders / Odometry

```
ZERO enc          [#id]  → OK zero enc [#id]
ZERO pose         [#id]  → OK zero pose [#id]
ZERO enc pose     [#id]  → OK zero enc pose [#id]
```

> **Sprint 084 note — `enc` only in `source/`.** This section documents the
> full `source_old/` grammar (all three forms). As of sprint 084 (ticket
> 007), `source/`'s `ZERO` (`source/commands/text_channel.cpp`'s, formerly
> `pose_commands.cpp`'s, `parseZero`) implements only the `enc` sub-verb — a deliberate scope
> decision (see that file's own doc comment), not an oversight. `ZERO`
> (bare), `ZERO pose`, and `ZERO enc pose` all return `ERR badarg` in
> `source/` today; only the exact literal `ZERO enc` succeeds. A future
> sprint may add `pose` without any wire-shape change.

`enc` resets the encoder accumulators (calls
`MotorController::resetEncoderAccumulators()`).  `pose` resets the
odometry integrator to `(0, 0, 0)` (calls `Odometry::zero()`).  Both
may be specified in one command.

At least one of `enc` or `pose` must be present; otherwise `ERR badarg`.

`enc`'s effect additionally resets `PoseEstimator`'s own encoder-delta
baseline in the same call, so the next tick's dead-reckoning delta is
computed against the freshly-zeroed encoders rather than a stale
pre-zero baseline (which would otherwise fabricate a phantom jump).

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

## 15. Sim parameters and telemetry — ctypes-only (no `SIMSET` / `SIMGET`)

There is **no `SIMSET`/`SIMGET` wire command family, and no sim-specific `TLM`
field.** Simulator plant and error-model parameters, ground-truth reads, and
sim-only telemetry are reachable **exclusively** through the host simulation
library's direct ctypes ABI (`tests/_infra/sim/sim_api.cpp`'s `sim_*`
functions) — never over the wire. The protocol a physical robot speaks and the
protocol a simulator speaks are therefore identical, and a test harness can
never accidentally teach the real robot to answer a sim-only verb (sprint 081
architecture decision).

Earlier drafts of this section documented a `SIMSET`/`SIMGET` verb family backed
by a `source/commands/SimCommands.{h,cpp}`. That command surface does **not**
exist in the current `source/` tree and has been removed — it contradicted the
decision above.

---

## 16. Development Commands (`DEV …`, dev builds only)

**These verbs exist ONLY when the firmware is built with `ROBOT_DEV_BUILD`
set** (`codal.json`'s `"config"` object — force-included into every
translation unit as a preprocessor `#define`, the same mechanism
`MICROBIT_BLE_ENABLED` already uses; see `source/commands/text_channel.h`
Section 3, formerly `dev_commands.h`).
Sprint 077's `source/` tree sets `ROBOT_DEV_BUILD=1` — there is no
production motion firmware yet, so this dev-bench build IS the only build.
A future production firmware flips the define to `0` and `DEV` disappears
(`ERR unknown`) — the verbs simply become unrecognised, no different from any
other unregistered command.

`DEV` drives the HAL for bench bring-up: individual motors by **port**
(1..4 — matching how `NezhaHal` instantiates one `NezhaMotor` per port;
never an L/R role name) and, through a bound port pair, the `Drivetrain`
subsystem. Every `DEV` handler that changes a motor or drivetrain's
commanded state builds a `msg::MotorCommand`/`msg::DrivetrainCommand`,
pre-validates it against the target's `capabilities()`
(`Hal::motorCommandAllowed()` for motor commands — the same capability gate
`Motor::apply()` itself uses), and **stages** it into `DevLoopState`'s
per-consumer outbox (`hasHalCommand`/`hasDrivetrainCommand`) rather than
calling `apply()` directly — `main.cpp`'s loop is the sole caller of
`Hal::NezhaHal::apply()`/`Subsystems::Drivetrain::apply()` for anything
DEV-sourced, draining the outbox once per pass (sprint 079,
architecture-update.md's "The processor is a pure transformer"). `OK` still
means parsed + validated + delivered-for-staging — the same wire behavior as
before this reshape (pre-validating against the same capability gate makes
the guarantee before staging instead of after applying, with no observable
difference at the wire). Replies use the standard taxonomy exclusively
(§3): `OK`/`ERR`, `EVT dev_watchdog` for the one asynchronous event this
family emits. No new reply tag is introduced.

### Authority: `DEV M` vs. `DEV DT`

This firmware runs only the dev loop — there is no planner to fight — so
there is exactly one authority conflict: a single motor commanded directly
by `DEV M` vs. that **same** motor being driven by the `Drivetrain` under
`DEV DT`. Rule (refined in 077-007 — see the note below):

- Any `DEV M <n>` verb that actually changes the motor's commanded state
  (`DUTY`, `VEL`, `POS`, `VOLT`, `NEUTRAL`, `RESET`) drops drivetrain
  authority **only when `<n>` is one of the Drivetrain's currently-bound
  `PORTS`** (the port it is actually driving) — but only when the command is
  also **accepted**; a capability rejection (`VOLT` on Nezha) never touched
  the motor and so never steals authority. A `DEV M <n>` on a port the
  Drivetrain is NOT bound to (e.g. an independent load motor used by a bench
  test — see the coupled-rig section below) is unrelated to the Drivetrain
  and leaves its authority/`active` state untouched.
- Any `DEV DT` verb that commands the drivetrain (`VW`, `WHEELS`, `NEUTRAL`)
  (re)activates drivetrain authority.
- `DEV DT PORTS`, `DEV M <n> STATE`, `DEV M <n> CAPS`, `DEV DT STATE` are
  queries/bindings and never change authority.
- `DEV STOP` and `DEV DT STOP` always drop authority (see below).

077-007 found and fixed the pre-existing behavior, which unconditionally
dropped drivetrain authority on ANY accepted `DEV M` motion verb regardless
of port: this silently killed the governor mid-test whenever a bench script
drove an independent load motor (e.g. `DEV M 4 DUTY ...`) while the
Drivetrain was bound to a different pair (`DEV DT PORTS 2 3`) — exactly the
coupled-rig test pattern where one bound wheel is friction-loaded by a
separate, unbound motor. `isBoundPort()` in `text_channel.cpp` (formerly
`dev_commands.cpp`) is the fix.

### Port binding: `DEV DT PORTS`

`DEV DT PORTS <left> <right>` selects which two motor ports the
`Drivetrain` treats as its wheel pair. Default at boot: `1 2` (the robot's
normal drive pair). The coupled PID/governor bench rig uses ports `3 4`
(two motors with mechanically linked shafts — running one loads the
other). The binding **persists** across `DEV STOP` and a serial-silence
watchdog neutral event; it resets only on reboot.

Sprint 079: the binding is now backed by `DrivetrainConfig.left_port`/
`right_port` (read via `Subsystems::Drivetrain::ports()`) rather than a
`DevLoopState` field — a config-plane command like any other `CFG` key,
per architecture-update.md's "Config-plane vs. command-plane". The wire
text is unchanged: `DEV DT PORTS <left> <right>` → `OK DEV DT
ports=<left>,<right>`.

### Poll-schedule membership: `DEV M <n> CFG polled=<bool>`

Sprint 091: `NezhaHardware`'s I2C flip-flop sequencer only samples/dispatches
ports in a **configured poll-set** — `msg::MotorConfig.polled`, established
once at boot (`true` for the drive-pair ports, `false` for every other port
— never robot-JSON-configurable, a firmware-scheduling fact) and mutable
thereafter ONLY through `DEV M <n> CFG polled=<bool>`, one more key on the
existing `DEV M <n> CFG` verb (below). This replaced an earlier,
command-derived scheme where simply addressing a port with any `DEV M`
verb silently and permanently added it to the round-robin for the rest of
the session, with no way back short of reboot.

- `polled=true` / `polled=1` sets it; `polled=false` / `polled=0` (or any
  other token) clears it — the same lenient, no-strict-validation
  convention every other `CFG` key already applies (a malformed token
  silently becomes the zero value, never an `ERR`).
- **This is the door the coupled PID/governor bench rig needs.**
  `tests/bench/pid_hold_speed.py` drives ports 3/4 standalone (no
  `DEV DT` at all); `tests/bench/ratio_governor_curve.py`'s primary
  protocol binds the `Drivetrain` to ports 2/3 and drives port 4
  standalone. Since only the boot drive pair (`1`/`2`) starts polled and
  `DEV DT PORTS` does **not** auto-follow the poll-set (a rebind alone does
  not opt the newly-bound pair in), every non-default port either script
  touches needs its own `DEV M <n> CFG polled=true` line in its setup
  preamble, alongside its existing `DEV WD 3000` line — both scripts do
  this.
- Unpolled ↔ never sampled/dispatched: on **real hardware**, a motor
  whose port is not in the poll-set never gets its embedded PID closed or
  its encoder read (`NezhaMotor::tick()` never runs for it) — see the
  `ERR nodev` behavior below, which exists precisely so a bench operator
  gets an immediate, loud signal instead of a command that "succeeds" but
  silently never converges.

### `DEV M <n> …` — Single-Motor Control

```
DEV M <n> DUTY <duty>        [%, -100..100]   → OK DEV M <n> applied=<duty/100>
DEV M <n> VEL <velocity>     [mm/s] signed    → OK DEV M <n> vel=<velocity>
DEV M <n> POS <position>     [deg]            → OK DEV M <n> pos=<position>
DEV M <n> VOLT <voltage>     [V]              → ERR unsupported volt (Nezha has no voltage mode)
DEV M <n> NEUTRAL <B|C>                        → OK DEV M <n> neutral=<B|C>
DEV M <n> RESET                                → OK DEV M <n> reset=1
DEV M <n> STATE                                → OK DEV M <n> pos=.. vel=.. applied=.. wedged=.. wsus=.. hrc=.. src=.. conn=..
DEV M <n> CAPS                                  → OK DEV M <n> duty=.. volt=.. vel=.. pos=.. enc=..
DEV M <n> CFG k=v ...                          → OK DEV M <n> <applied k=v ...>
```

`<n>` is the motor port, `1..4`.

`DUTY`/`VEL`/`POS` are additionally gated on **poll-schedule membership**
(sprint 091, see above): if `<n>` is not in `NezhaHardware`'s configured
poll-set (`bb.motorConfig[<n>-1].polled == false` — the default for every
port except the boot drive pair, `1`/`2`), the verb is rejected `ERR nodev
<mode>` (`<mode>` is `duty`/`vel`/`pos`) BEFORE anything is posted —
mirrors `OI`/`OZ`/`OR`/`OV`'s device-presence convention (§11). Extend the
poll set explicitly with `DEV M <n> CFG polled=true` first. `NEUTRAL`/
`RESET`/`STATE`/`CAPS`/`CFG` are never gated by poll membership.

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

  **Sim note (081-003):** `Hal::SimMotor` (`source/hal/sim/sim_motor.{h,cpp}`)
  reports `capabilities().position == false` — the simulated plant has no
  onboard absolute-angle move to model, unlike a real Nezha's `0x5D`
  register. `DEV M <n> POS` therefore answers `ERR unsupported` in a sim
  build, exactly like `VOLT` does on real hardware (the identical
  `Motor::apply()` capability gate firing, not a sim-specific special
  case). `DUTY`/`VEL`/`NEUTRAL`/`RESET`/`STATE`/`CAPS`/`CFG` are all
  supported identically to a real Nezha motor. This is a capability
  divergence to be aware of when writing a test that is meant to run
  against both a bench robot and the sim, not a wire-format change — no new
  verb, reply field, or `SET`/`GET`/`CFG` key is introduced by the sim — the
  simulator adds no wire surface at all (sim parameters are ctypes-only; see
  §15).
- `VOLT <voltage>` — always `ERR unsupported volt` on a Nezha motor
  (`capabilities().voltage == false`). This is `Motor::apply()`'s capability
  gate firing, not a special case in the `DEV` handler — the same code path
  a future voltage-capable leaf would accept through.
- `NEUTRAL <B|C>` — `B` = brake, `C` = coast. Nezha has one physical stop
  path (duty 0 via `0x60`); both letters currently produce the same
  hardware action but are accepted and echoed distinctly for forward
  compatibility.
- `RESET` — stages a reset (`MotorCommand.reset_position`); always accepted
  (not capability-gated). The `OK reset=1` reply reports **acceptance, not
  completion-kind**: the hard-reset-vs-soft-rebaseline decision is made at
  the top of the *next* `tick()` (`Motor::processResetIfPending()`), based
  on whether the motor has been at a verified standstill for several
  consecutive ticks —
  - **verified standstill** (several consecutive at-rest ticks observed) →
    a **hard reset**: an atomic hardware re-prime burst that zeroes the
    encoder. Increments `hrc=`.
  - **not at rest** (the motor is still moving when the reset is
    dispatched) → a **soft rebaseline**: a software-only rebaseline that
    issues no bus transaction. Increments `src=`.

  Either way, `position()` reads ~0 on the very next `STATE` poll — the
  wire contract is the same regardless of which internal path fired. The
  `RESET` reply itself never reveals which kind ran; only a subsequent
  `STATE` poll's `hrc=`/`src=` deltas do.
- `STATE` — one line, always all eight fields even when a leaf lacks an
  encoder (unset fields report `0`/`false`, never a blank).
  - `wedged=` — the **raw, unconditional** stuck-encoder latch: several
    consecutive identical position reads, with no gating on whether the
    motor is currently commanded to move. It latches (and stays latched)
    even for an **idle** motor sitting at rest — that is benign and
    expected, not a fault. Bench operators reading logs should not treat a
    lone `wedged=1` on an otherwise-idle motor as an alarm.
  - `wsus=` ("wedge-suspect") — the same identical-reads test, but
    additionally qualified on `|applied|` exceeding the `deadband`
    threshold — i.e. the motor is genuinely being commanded to move *right
    now* and the encoder isn't responding. This is the field to alarm on;
    `wedged=` alone is not sufficient evidence of a real stuck-motor fault.
  - `hrc=` / `src=` — cumulative hard-reset / soft-rebaseline counters (see
    `RESET` above). Monotonically increasing for the life of the process;
    never reset by `STATE` itself.
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
  | `dwell`          | `reversal_dwell`               | `%.1f`      |
  | `deadband`       | `output_deadband`              | `%.3f`      |
  | `polled`         | `polled`                       | `0`/`1`     |

  An unrecognized key emits `ERR badkey <key>` (does not prevent the
  other, valid keys in the same command from applying — mirrors `SET`'s
  per-key error reporting, §7). The final `OK` line lists only the keys
  that actually applied.

  `dwell` and `deadband` are `optional` fields (not the `slew`-style
  "`<= 0` means unconfigured" sentinel): an explicit `0` is a valid,
  meaningful configuration for each, distinct from "never configured."
  Never sending either key leaves the motor on its ship-safe defaults
  (`100` ms dwell, `0.03` deadband).

  - `dwell` — `[ms]`. How long the write path holds the output at
    commanded-zero after detecting a commanded sign change (reversal),
    before forwarding the new-direction duty. `0` is the explicit
    legacy/A-B-test configuration: reversals fall straight through to an
    immediate write with no dwell, reproducing pre-sprint-078 behavior.
    Default when unset: `100` ms.
  - `deadband` — `[-1,1]` fraction of the write-path's output duty. `|duty|`
    below this threshold writes `0` immediately (never dwelt, never
    clamped) instead of the commanded value; it also gates the
    motion-qualified `wsus=` wedge-suspect signal (`STATE` above). Default
    when unset: `0.03`.

    **`deadband` is NOT the same knob as `min_duty`, despite both being
    "deadband-shaped."** They gate different layers, at different points
    in the pipeline, over different quantities:
    - `min_duty` gates the **velocity PID's integrator-freeze threshold**,
      tested against `|target|` in **mm/s** (a velocity magnitude) —
      despite its name, it is not an output-duty gate at all.
    - `deadband` gates the **write path's output duty fraction**, tested
      against `|duty|` in the **`[-1, 1]`** unitless fraction range, after
      the PID (or any other mode) has already produced a duty to write.

    Tuning one has no effect on the other; a bench operator adjusting
    "the deadband" via `DEV M <n> CFG` must pick the correct key for the
    layer they actually mean to affect.

  - `polled` — I2C flip-flop poll-schedule membership (see "Poll-schedule
    membership" above). Accepts `true`/`1` (sets it) or anything else,
    including `false`/`0` (clears it) — the same lenient, no-strict-
    validation convention every other key on this table already applies.
    Echoed back as `0`/`1` in the `OK` ack. Not persisted anywhere but the
    live config — a reboot resets every port to its boot-baked value (the
    drive pair `true`, everything else `false`).

Examples:

```
DEV M 1 DUTY 30
OK DEV M 1 applied=0.30

DEV M 1 STATE
OK DEV M 1 pos=177.8 vel=0.0 applied=0.30 wedged=0 wsus=0 hrc=0 src=0 conn=1

DEV M 1 VOLT 3
ERR unsupported volt

DEV M 1 RESET
OK DEV M 1 reset=1

DEV M 1 CAPS
OK DEV M 1 duty=1 volt=0 vel=1 pos=1 enc=1

DEV M 3 CFG kp=0.8 slew=400
OK DEV M 3 kp=0.800 slew=400.0

DEV M 1 CFG dwell=0
OK DEV M 1 dwell=0.0

DEV M 4 DUTY 30
ERR nodev duty

DEV M 4 CFG polled=true
OK DEV M 4 polled=1

DEV M 4 DUTY 30
OK DEV M 4 applied=0.30
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
OK DEV M 1 pos=177.8 vel=0.0 applied=0.00 wedged=0 wsus=0 hrc=0 src=0 conn=1
OK DEV M 2 pos=0.0 vel=0.0 applied=0.00 wedged=1 wsus=0 hrc=0 src=1 conn=1
OK DEV M 3 pos=0.0 vel=0.0 applied=0.00 wedged=1 wsus=0 hrc=0 src=0 conn=1
OK DEV M 4 pos=0.0 vel=0.0 applied=0.00 wedged=1 wsus=0 hrc=0 src=0 conn=1
OK DEV DT active=0 ports=1,2 vel=0.0,0.0

DEV STOP
OK DEV STOP
```

Motors 2-4 above illustrate the `wedged=`-vs-`wsus=` distinction directly: all
three show the raw `wedged=1` latch (several consecutive identical position
reads), but since none is currently being commanded to move (`applied=0.00`,
below the `deadband` threshold), `wsus=0` on every one — a benign, at-rest
latch, not a fault. Motor 2's `src=1` shows a soft rebaseline having fired
while it was not at a verified standstill.

### `DEV WD <window>` — Serial-Silence Watchdog Window

```
DEV WD <window>   [ms, 50..60000] → OK DEV WD window=<window>
```

Sets the serial-silence watchdog's window at runtime. Default at boot:
`1000` ms.

### Serial-Silence Watchdog — Non-Negotiable

Every `DEV`/liveness command line that arrives on either comms channel
(serial or radio) resets a wall-clock timer — regardless of the line's
content or whether it parsed to a known verb. This feed/reset behavior is
unconditional and runs every pass, whether or not anything is currently
driving.

**The FIRE action is gated on motors actually being commanded to run**
(sprint 091 ticket 003): if no line arrives within the current window AND
at least one motor is currently under a non-neutral commanded mode (the
Drivetrain's own bound-pair output, or any individual port driven
standalone, e.g. via `DEV M <n> VEL`/`DUTY`/`POS`), the firmware:

1. Commands **all four** motors to neutral (`Neutral::BRAKE`), regardless
   of which family (`DEV M` or `DEV DT`) was last authoritative.
2. Idles the Drivetrain and drops drivetrain authority.
3. Emits `EVT dev_watchdog` (no body, no `#id` — this is not tied to any
   single originating command) on the serial channel.

If the window expires while the robot is genuinely idle (no motor under a
non-neutral commanded mode), none of the above happens — no neutralize, no
`EVT dev_watchdog` — since there is no runaway to prevent. This gate
covers the standalone-bench-motor case too, not just Drivetrain-bound
driving: a bound-port `DEV M` motion verb steals Drivetrain authority
(putting it into standby) the moment it lands, so the fire condition also
checks each motor's own commanded state, not the Drivetrain's alone.

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
OK DEV M 1 pos=412.0 vel=0.0 applied=0.00 wedged=0 wsus=0 hrc=0 src=0 conn=1
```

---

## Appendix: Removed v1 Commands

The following v1 command vocabulary is removed in v2.  Any of these
verbs returns `ERR unknown <verb>`.  (`HELLO` and `DEVICE:` were removed
under v1->v2 too, but are re-added — see §6.)

| Removed verb / prefix    | v1 meaning                             |
|--------------------------|----------------------------------------|
| `K*` (e.g. `KCP`, `KCL`) | Per-constant calibration set/get       |
| `ENC`                    | Encoder query                          |
| `SO`                     | Sensor output (legacy)                 |
| `SSE`, `SSO`, `SSC`, `SSL` | Streaming sensor commands             |
| `X`                      | Legacy stop/reset                      |
| Packed motion (`S+200-150`) | Sign-prefix packed speed arguments  |
