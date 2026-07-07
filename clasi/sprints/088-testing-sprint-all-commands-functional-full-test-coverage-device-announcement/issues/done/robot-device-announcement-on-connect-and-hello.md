---
status: done
sprint: 088
tickets:
- 088-005
---

# Robot device announcement: emit `DEVICE:NEZHA2:robot:<name>:<serial>` first on serial + radio connect, and on `HELLO`

## Context

The microbit-radio-relay protocol §3.4
(<https://robots.jointheleague.org/subsystems/microbit-radio-relay/protocol/>)
defines a device-announcement banner. The relay emits
`DEVICE:RADIOBRIDGE:relay:<deviceName>:<serialNumber>` on boot (entering the
command plane) and re-emits it on demand when the host sends `HELLO`. Fields:
`DEVICE:` prefix, device-type/model (field 1), role descriptor (field 2), CODAL
friendly name (field 3), nRF serial number (field 4), colon-delimited.

The **current** firmware (`source/`) emits **no** connect-time banner. `HELLO`
and `DEVICE:` are explicitly-removed v1 verbs under protocol v2
([docs/protocol-v2.md:16-18](docs/protocol-v2.md#L16-L18),
[:2103-2104](docs/protocol-v2.md#L2103-L2104)). The only identity surface is the
request/response `ID` verb ([system_commands.cpp:102-125](source/commands/system_commands.cpp#L102-L125)),
which must be asked for and is a different format.

The retired firmware did exactly this feature over serial: it emitted
`DEVICE:NEZHA2:robot:<name>:<serial>` once at boot
([source_old/commands/SystemCommands.cpp:84-94](source_old/commands/SystemCommands.cpp#L84-L94),
boot emit [source_old/main.cpp:231-232](source_old/main.cpp#L231-L232)) and had
a `HELLO` verb to re-trigger it. **The Python host still expects and parses this
line** — three parsers
([serial_conn.py:156-174](host/robot_radio/io/serial_conn.py#L156-L174) &
[:448-457](host/robot_radio/io/serial_conn.py#L448-L457),
[connection.py:97-114](host/robot_radio/robot/connection.py#L97-L114) &
[:297-298](host/robot_radio/robot/connection.py#L297-L298),
[testgui/transport.py:153-184](host/robot_radio/testgui/transport.py#L153-L184))
classify robot-vs-relay off field 1, and cached announcements live in
[config/devices.json](config/devices.json) as
`DEVICE:NEZHA2:robot:tovez:2314287040` / `...:togov:...`. Design intent for an
`Announcer` is already sketched in
[docs/architecture.md:153-155](docs/architecture.md#L153-L155). So this is
"re-introduce the boot banner the host already consumes, and extend it to
radio," not greenfield.

## Desired behavior

1. **On connect (boot), first line out on BOTH channels.** As part of
   comm-channel bring-up, emit `DEVICE:NEZHA2:robot:<deviceName>:<serialNumber>`
   as the first line on serial **and** on radio, before the main loop begins.
   - `<deviceName>` = `microbit_friendly_name()`, `<serialNumber>` =
     `microbit_serial_number()` (decimal `uint32_t`) — the exact pair the live
     `ID` handler already uses
     ([system_commands.cpp:110-111](source/commands/system_commands.cpp#L110-L111)).
   - Natural hook: the single bring-up point for both channels,
     `Communicator::begin()`
     ([communicator.cpp:28-32](source/subsystems/communicator.cpp#L28-L32)),
     using the existing `sendSerial()` / `sendRadio()` primitives
     ([:89-91](source/subsystems/communicator.cpp#L89-L91)). It runs once from
     [main.cpp:104-106](source/main.cpp#L104-L106) before the first loop pass.
2. **On `HELLO`, re-emit the same banner.** Re-add a `HELLO` verb (removed in v2)
   whose handler re-emits `DEVICE:NEZHA2:robot:<name>:<serial>`. Register it
   alongside `PING`/`VER`/`HELP`/`ECHO`/`ID`
   ([system_commands.cpp:129-137](source/commands/system_commands.cpp#L129-L137)).

## Wire format (stakeholder decision, 2026-07-07)

`DEVICE:NEZHA2:robot:<deviceName>:<serialNumber>` — field 1 = model `NEZHA2`
(matches the `ID` verb, `config/devices.json`, and the old firmware exactly),
field 2 = role `robot` (mirrors the relay's `relay`). This was chosen over
`DEVICE:ROBOT:...` specifically so the host's existing caches and classifier
keep working unchanged. The user's original "device type ROBOT instead of
RADIOBRIDGE" intent is satisfied by the `robot` role token; keeping `NEZHA2` in
field 1 preserves host compatibility.

## Open questions / notes for whoever picks this up

- **HELLO re-announce scope.** Which channel(s) does `HELLO` re-announce on — the
  channel the `HELLO` arrived on, or both? Recommend re-announcing on the
  arriving channel so a host querying over serial gets it on serial and over
  radio gets it on radio. (Old firmware re-announced serial-only.) Confirm at
  planning.
- **Radio banner semantics.** Radio `send()` is a fire-and-forget broadcast on
  channel/group 10 with no link-up handshake — the boot radio banner only
  reaches a host if a relay is already listening. The serial banner reaches a
  directly-connected host reliably. `HELLO`-triggered re-announce is the reliable
  path over radio. Call this asymmetry out; don't treat a missed boot radio
  banner as a failure.
- **Docs.** Update [docs/protocol-v2.md](docs/protocol-v2.md) to re-add `HELLO`
  and document the boot announcement (both are currently listed as removed), and
  reconcile with the `Announcer` design note at
  [docs/architecture.md:153-155](docs/architecture.md#L153-L155).
- Out of scope: the pre-existing `ID` model-token casing mismatch (`model=NEZHA2`
  in code vs `model=Nezha2` in the doc) and the missing `caps=` field on `ID` —
  note but don't fix here.

## Acceptance

- Firmware emits `DEVICE:NEZHA2:robot:<name>:<serial>` as the **first** serial
  line and **first** radio line at boot.
- Sending `HELLO` re-emits the same banner.
- The host (`serial_conn` / `connection`) discovers and classifies the robot as
  a direct/robot device from the banner (boot banner on serial, and/or the
  `HELLO`-triggered banner over the relay path).
- **HITL bench (on the stand):** the banner appears on the serial link at
  connect; over the radio/relay path a `HELLO` re-request returns the banner.
