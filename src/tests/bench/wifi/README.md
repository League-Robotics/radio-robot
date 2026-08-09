# Planet X WiFi module (Ai-WB2-12F) — bench bring-up kit

Bring-up of the ELECFREAKS Planet X WiFi module (Ai-Thinker **Ai-WB2-12F**,
BL602, 2.4 GHz only) as a candidate alternative radio link for the robot.
First bring-up 2026-08-08 on `gopiv`, module in a Nezha RJ11 port.
**Verified end to end: AP join, UDP and TCP sockets to a host on the LAN,
bidirectional data, transparent passthrough streaming.**

**Operating reference:** [docs/wifi-module-sockets.md](../../../../docs/knowledge/2026-08-08-wifi-module-sockets.md) — the module's AT dialect, UDP/TCP socket recipes, passthrough mode, and example code, with verified-on-hardware markings.

## What the module is

- Speaks the **ESP-AT command dialect** over UART, 115200 8N1 (Ai-Thinker
  "Combo AT" firmware: `AT version:1.4.1`, `release_bl_iot_sdk_1.6.36`,
  `Bin V4.18_P1.5.1`). `AT+CWJAP`, `AT+CIPSTART`/`AT+CIPSEND`, `AT+CWLAP`,
  `AT+CIPMODE=1` passthrough, `AT+PING`, `AT+MQTT*`, `AT+HTTPCLIENT` all
  present. Vendor reference: elecfreaks/pxt-PlanetX `wifi.ts`.
- RJ11 port → micro:bit pins (micro:bit TX/RX): **J1=P8/P1, J2=P12/P2,
  J3=P14/P13, J4=P16/P15** (from `wifi.ts` `initWIFI()`).

## Measured results (2026-08-08, gopiv + Busboom Mesh)

| test | result |
|---|---|
| AP join (`AT+CWJAP`) | joins in ~2 s, `WIFI GOT IP`, DHCP 192.168.1.222 |
| ICMP Mac→module | 1.8–6.5 ms |
| UDP module↔Mac (AT mode) | round trip verified; 19/20 datagrams delivered |
| TCP module↔Mac | connect + round trip verified |
| Passthrough upstream stream | 8×254 B chunks, **2032/2032 bytes byte-perfect**, 0.45 s |
| Passthrough UDP RTT (4 B payload) | **30/30, min 41 / median 49.5 / p90 57 ms** |

The ~45 ms gap between ICMP RTT and passthrough RTT is the AT firmware's
passthrough packetization timer (serial bytes are batched until
buffer-full-or-timeout before each datagram). Small packets pay the timer;
bulk streams don't. A robot link wanting low latency should either tune
packet timing, use AT-mode `CIPSEND` framing, or accept ~25 ms one-way.

## Gotchas (all hit during bring-up)

- **The SSID is `Busboom Mesh` — SPACE, not underscore.** `+CWJAP:3`
  means "AP not found" and it means it literally.
- The module **saves the last AP + lease** and auto-rejoins at boot;
  `AT+CIPSTA?` can show a stale-looking IP that is in fact its lease.
  Verify connectivity with `AT+PING`, not `CIPSTA?`.
- `AT+CWLAP` scan results can arrive 10+ s late and the module sometimes
  wedges after a scan — a power-cycle (or gopiv port-close reset cycle)
  recovers it.
- `SEND FAIL` on a socket that read back correct (`AT+CIPSTATUS`) peer
  params = module not actually associated to WiFi. Join state first.
- **The module→host reply path drops ~5–10% of chars, and it is NOT the
  module or UARTE1**: the loss rate is identical at 57600 (baud-invariant,
  measured via the bridge's live baud switch), which matches sprint 123's
  measured ~5–11% outbound corruption on the DAPLink CDC leg
  (nRF→KL27→USB). Commands in (host→module) are near-lossless; replies out
  are holey. AT-mode parsing must be loss-tolerant; passthrough payload
  integrity module-ward is unaffected (2032/2032). A production link needs
  framing+CRC over the CDC leg — same answer the robot protocol already
  uses. (`CONFIG_SERIAL_DMA_BUFFER_SIZE` 32→128 was tried first and
  changed nothing; NB a codal.json config change without `--clean`
  produces a silently corrupt image — boot-dead board, no banner.)

## The atbridge firmware

[`atbridge_main.cpp`](atbridge_main.cpp) is a standalone CODAL app: a
transparent USB↔module-UART pipe (UARTE0 = USB via `uBit.serial`, UARTE1 =
RJ11 pins). At boot it auto-probes J1–J4 for an `AT`→`OK` responder,
announces `ATBRIDGE: module on Jn` over USB, then pipes bytes verbatim.
Every host port CLOSE resets the board (DTR) → it re-probes on each script
run; opening does NOT reset.

Build (in a scratch worktree — do not point the main tree's codal.json away
from src/firm):

1. `git worktree add --detach <scratch>/wt-wifi HEAD`, symlink the four
   `src/libraries/codal-*` clones and the `AprilTags` sibling in.
2. Copy `atbridge_main.cpp` to `<wt>/src/atbridge/main.cpp`; set
   `codal.json` `"application": "src/atbridge"`.
3. Guard the unconditional motion-library glob in the root CMakeLists
   (`list(APPEND SOURCE_FILES ${MOTION_SOURCE_FILES})` → wrap in
   `if("${CODAL_APP_SOURCE_DIR}" MATCHES "firm")`).
4. `uv run python3 build.py --fw-only` → flash `MICROBIT.hex` by UID.

## Fixed IP from the robot config (2026-08-08)

`connection.wifi_ip` in the robot JSON (schema source:
`src/protos/robot_config.proto`, host-only Connection group) records the
robot's reserved WiFi address — the bench DNS resolves each robot's name
to it (gopiv=192.168.4.10, tovez=192.168.4.11). Absent/empty means "no
WiFi module" and every consumer no-ops.

`wifi_setup.py --robot <name> --port <bridge-port>` applies it: re-enables
DHCP (idempotent from any prior state), majority-reads gateway+netmask
from `AT+CIPSTA?` over the lossy serial (a value must win 3+ sightings
with a 2+ lead — "255.255.255.0" corrupts to the *repeatable*
"25.255.255.0", so seen-twice is NOT proof; masks are also structurally
validated), applies `AT+CIPSTA="<config-ip>","<gw>","<mask>"`, verifies by
readback, then pings from the host as end-to-end truth.

**Open item (2026-08-08): the .4.x reservations are unroutable from the
bench LAN** — the module joins Busboom Mesh and DHCPs onto
192.168.1.0/24 (gw 192.168.1.1), so a static 192.168.4.10 there answers
nobody; the Mac has no route to 192.168.4.0/24 (`traceroute`: no route to
host, nothing at 192.168.4.1). Needs a network-side decision: route/VLAN
for 192.168.4.0/24, or move the DNS reservations into the pool the mesh
actually serves.

## Host-side scripts

- `at_drive.py PORT [--wait S] CMD...` — send AT commands through the
  bridge (`raw:` prefix = no CRLF, `sleep:N` = pause). Loss-tolerant eyes.
- `udp_echo_server.py [port=5005]`, `tcp_echo_server.py [port=5007]` —
  logging echo peers.
- `wifi_rtt_passthrough.py PORT PEER N` — passthrough-mode RTT
  (time-to-first-echo-byte; robust to serial char loss).
- `wifi_latency.py` — AT-mode RTT + passthrough stream test (AT-mode
  matcher is flaky over the lossy bridge; trust the server logs).

Smoke sequence: start `udp_echo_server.py`, then
`at_drive.py <port> 'AT+CWJAP="Busboom Mesh","<pw>"' 'AT+CIPSTART="UDP","<mac-ip>",5005,5006,0' 'AT+CIPSEND=5' 'raw:hello' 'sleep:2'`
and watch the server log.
