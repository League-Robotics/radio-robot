---
date: 2026-08-08
tags: [wifi, ai-wb2-12f, planetx, udp, tcp, sockets, esp-at, radio-alternative, bench]
related-tickets: []
---

# WiFi Module Sockets — Ai-WB2-12F (Planet X) Operating Reference

How to command the ELECFREAKS Planet X WiFi module to open **UDP and TCP
sockets** and move data through them. Everything marked ✅ was exercised on
real hardware 2026-08-08 (module on `gopiv`, J1, via the `atbridge` pipe —
see [src/tests/bench/wifi/README.md](../../src/tests/bench/wifi/README.md));
anything not so marked is from the ESP-AT dialect docs and still needs a
bench pass.

## 1. The device

| | |
|---|---|
| Module | Ai-Thinker **Ai-WB2-12F** (Bouffalo BL602, RISC-V, **2.4 GHz only**) |
| Firmware | Ai-Thinker "Combo AT" — ESP-AT-compatible dialect (✅ `AT+GMR`: AT 1.4.1, `release_bl_iot_sdk_1.6.36`, Bin V4.18_P1.5.1) |
| UART | 115200 8N1 at power-on; CRLF-terminated commands |
| Wiring | Nezha RJ11 ports, micro:bit TX/RX: **J1=P8/P1, J2=P12/P2, J3=P14/P13, J4=P16/P15** |

Command grammar is classic AT: send `AT+VERB=args\r\n`, read reply lines,
terminated by `OK` or `ERROR`. The module also emits **unsolicited** lines
at any time — `WIFI CONNECTED`, `WIFI GOT IP`, `WIFI DISCONNECT`,
`+IPD,...` (inbound socket data), `CONNECT`/`CLOSED` (TCP state) — so a
driver must treat the RX stream as event-driven, never strictly
request/reply. Echo is on by default; `ATE0` ✅ turns it off.

The module **persists its last AP join and auto-reconnects at boot** ✅.
It also keeps a host-commanded `AT+UART_CUR` baud across micro:bit resets
(it only reverts on its own power cycle) ✅ — probe at both bauds if it
goes quiet.

## 2. Getting on the network

```
ATE0                                   # echo off
AT+CWMODE=1                            # station mode
AT+CWJAP="Busboom Mesh","<password>"   # join -- SSID has a SPACE, not _
  → WIFI CONNECTED / WIFI GOT IP      (~2 s)                          ✅
AT+CWJAP?                              # readback: SSID, BSSID, RSSI    ✅
AT+CIPSTA?                             # ip / gateway / netmask         ✅
AT+PING="192.168.1.40"                 # the ONLY trustworthy liveness  ✅
```

Join failures come back as `+CWJAP:<code>` then `ERROR` — code 3 = AP not
found ✅ (that is how the underscore-vs-space SSID typo presents), 1 =
timeout, 2 = wrong password, 4 = connect failed.

**Trap (cost an hour):** `AT+CIPSTA?` shows the *saved* lease even when
the module is not associated ✅ — a plausible-looking IP is not proof of a
connection. `AT+PING` is; a `SEND FAIL` on a socket whose `AT+CIPSTATUS`
readback is perfect means "not actually on WiFi" ✅.

Static IP (the fixed-IP scheme — address from the robot JSON's
`connection.wifi_ip`, everything else from DHCP):

```
AT+CWDHCP=1,1                          # (re)enable DHCP, get the net shape ✅
AT+CIPSTA?                             # read gateway + netmask             ✅
AT+CIPSTA="192.168.4.10","192.168.1.1","255.255.255.0"                    # ✅
```

`src/tests/bench/wifi/wifi_setup.py` automates exactly that sequence.

## 3. UDP sockets ✅

One connection slot (single-link mode; `AT+CIPMUX=1` multi-link exists in
the dialect, untested here). Open, send, receive:

```
AT+CIPSTART="UDP","192.168.1.40",5005,5006,0
  → CONNECT
  → OK
```

Args: peer IP, peer port, **local port** (the robot's own; peers reply to
it), mode `0` = peer fixed at CIPSTART time. (Mode `2` = "peer becomes
whoever sent last" — the right shape for a robot that serves whichever
host speaks; in the dialect docs, untested here.)

Send a datagram — length-prompted, binary-safe:

```
AT+CIPSEND=12         → OK, then a bare ">" prompt
<exactly 12 raw bytes, no terminator>
  → Recv 12 bytes
  → SEND OK
```

Receive side is unsolicited, one line per datagram:

```
+IPD,17:echo:hello-radio3
```

`17` is the payload byte count; the payload follows the colon verbatim
(binary included — count bytes, don't scan for delimiters).

Verified round trip: robot→Mac datagram logged by a Python UDP listener,
Mac→robot echo delivered as `+IPD` ✅. 19/20 delivery in the ping series ✅.
Close (either socket type): `AT+CIPCLOSE` → `CLOSED` ✅. A second
`CIPSTART` while one is open returns `ERROR` — close first ✅.

## 4. TCP sockets ✅

Identical surface, minus the local-port/mode args:

```
AT+CIPSTART="TCP","192.168.1.40",5007
  → CONNECT / OK                       (server saw the accept)          ✅
AT+CIPSEND=10  →  ">"  →  10 bytes  →  SEND OK                          ✅
  → +IPD,15:echo:tcp-test-1            (stream data in, same framing)   ✅
```

A dropped connection announces itself with an unsolicited `CLOSED`. TCP
gives you ordering and retransmit for free; the cost is head-of-line
blocking — for robot command traffic where stale packets are worthless,
UDP is usually the right default, with TCP attractive for bulk/log
streaming.

## 5. Transparent passthrough — the streaming mode ✅

This is the mode a robot link would actually run: after setup the UART IS
the socket, no per-send handshake, binary-clean both directions.

```
AT+CIPSTART="UDP",...   (or TCP)
AT+CIPMODE=1
AT+CIPSEND              # no length arg
  → ">"                 # from here: raw pipe
```

Everything written to the UART goes out the socket; everything received
comes back raw (no `+IPD` framing). Measured ✅: 8×254 B chunks arrived at
the peer **byte-perfect (2032/2032)**; small-payload round trips ran
**30/30, median 49.5 ms** against a 2–6 ms ICMP floor — the gap is the
module's packetization timer (serial bytes are batched briefly before
each datagram), so tiny writes pay ~25 ms one-way while bulk writes don't.

Exit: ≥1 s of silence, then `+++` (no CRLF), then ≥1 s ✅, then
`AT+CIPMODE=0`. In UDP-mode-0 the datagram boundaries you write are NOT
preserved — the timer decides packet edges; if message framing matters,
frame in-band (protocol v5's COBS+CRC+0x0A does this already).

## 6. Failure modes seen on the bench

| symptom | actual meaning |
|---|---|
| `SEND FAIL`, socket params read back fine | not associated to WiFi ✅ |
| `+CWJAP:3` / `ERROR` on join | SSID literally not visible (typo/5 GHz-only/band) ✅ |
| `AT+CWLAP` slow, results 10+ s late, module wedged after | known quirk; power-cycle the module (or reset cycle) recovers ✅ |
| module silent at 115200 after working earlier | it kept a commanded `AT+UART_CUR` baud across the micro:bit reset ✅ |
| `ERROR` on `CIPSTART` | a link is already open — `AT+CIPCLOSE` first ✅ |

## 6b. Fixed-IP routing findings (2026-08-09, all ✅ on the bench)

The fixed-IP scheme (192.168.4.10 for gopiv) came alive 2026-08-09 after a
morning of one-way traffic. The network is one flat 192.168.0.0/21 L2; the
final working combination and the traps, in order discovered:

- **The mesh's DHCP scope serves netmask 255.255.255.0 on the /21
  network.** A client honoring that lease cannot reply to hosts outside
  its /24 slice (its gateway may even sit off-subnet). The module needed
  the /21 (`255.255.248.0`) applied explicitly. Symptom of the mismatch:
  inbound packets arrive (visible as `+IPD` through the bridge) while
  every module-originated packet — UDP replies, TCP SYN-ACK, ICMP — dies.
  NOTE for `wifi_setup.py`: it takes netmask from DHCP by design, so until
  the DHCP scope's mask option is corrected to /21, it will re-apply the
  broken /24.
- **The AP tracks client IP bindings (anti-spoof/quarantine behavior).**
  A static source IP with no matching DHCP binding was silently dropped
  AP-side even with a correct netmask, and the block persisted until the
  association was bounced. Working recipe: DHCP reservation for the
  module's MAC (`AT+CIPSTAMAC?` → e.g. gopiv's b4:0e:cf:af:1b:09) at the
  reserved address, THEN `AT+RST` so the module rejoins and the AP
  re-learns the binding. After that, static and leased address agree and
  everything flows.
- **`AT+CIPSTA` persists across module power cycles** (survived a battery
  swap and multiple `AT+RST`s) — a fixed IP set once stays set. `AT+CWDHCP=1,1`
  reverts to leased operation.
- **The module DOES answer ICMP echo** — ping 3/3 at 3.8–62 ms once the
  path was clean (the wide spread is WiFi power-save wake latency). Earlier
  "module never answers ping" observations were the AP filtering, not the
  firmware; and the 2026-08-08 "ping works" observation was a different
  device squatting the stale address. Ping is a valid liveness probe ONLY
  once the binding/association state is known-good — `AT+PING` from the
  module side remains the more diagnostic direction.
- **Server shapes verified**: `AT+CIPMUX=1` + `AT+CIPSERVER=1,7` accepts
  inbound TCP (link events `0,CONNECT`/`0,CLOSED`, data as
  `+IPD,<link>,<len>:`), and `AT+CIPSTART=4,"UDP","0.0.0.0",0,7,2`
  (wildcard peer, mode 2) receives on port 7 from any host and retargets
  replies to the last sender — `AT+CIPSTATUS` shows the peer swap live.
  Echo replies on a TCP link race the peer's close: a one-shot
  `echo x | nc` sees its connection close before the AT-driven echo lands;
  an interactive session sees echoes fine.

## 7. Example code

Host-side (through the atbridge pipe — this exact flow ran ✅; loss-aware
variants live in `src/tests/bench/wifi/`):

```python
import serial, time

ser = serial.Serial("/dev/cu.usbmodem2121302", 115200, timeout=0.1)

def at(cmd, wait=1.5):
    ser.write(cmd.encode() + b"\r\n")
    time.sleep(wait)
    return ser.read(8192).decode("utf-8", "replace")

at('ATE0')
at('AT+CWJAP="Busboom Mesh","<password>"', wait=8)   # skip if auto-rejoined
at('AT+CIPSTART="UDP","192.168.1.40",5005,5006,0')   # peer, peerport, localport
at('AT+CIPSEND=5')                                    # wait for ">"
ser.write(b"hello")                                   # exactly 5 raw bytes
print(at('', wait=1))                                 # SEND OK, then +IPD echoes
```

Firmware-side sketch for the future `WifiTransport` (design shape only —
NOT yet built; the UARTE1 bring-up half is exactly what `atbridge` proved):

```cpp
// Owns the module on UARTE1 (RJ11 pins), presents App::Comms a byte pipe.
NRF52Serial wifi(uBit.io.P8, uBit.io.P1, NRF_UARTE1);  // J1
wifi.setBaudrate(115200);

// boot: join using Config::kWifiSsid/kWifiIp (bake path TBD), then
//   AT+CWDHCP=1,1 / read CIPSTA? / AT+CIPSTA=<Config::kWifiIp>,gw,mask
//   AT+CIPSTART="UDP",<host>,<port>,<localport>,2   // 2: follow last peer
//   AT+CIPMODE=1 + AT+CIPSEND                       // passthrough
// after that: send() writes raw frames, RX events feed processMessage() --
// protocol v5's COBS+CRC framing rides through unchanged.
```

Mac-side peers are ordinary sockets — `udp_echo_server.py` /
`tcp_echo_server.py` in the bench kit are complete working examples.

## 8. Verb quick-reference (all ✅ on this module unless noted)

`AT` `ATE0/1` `AT+GMR` `AT+RST` (untested) `AT+CWMODE=1` `AT+CWJAP`
`AT+CWJAP?` `AT+CWLAP` `AT+CWDHCP=1,1` `AT+CIPSTA?` `AT+CIPSTA=ip,gw,mask`
`AT+UART_CUR=baud,8,1,0,0` `AT+PING="host"` `AT+CIPSTART="UDP"|"TCP",...`
`AT+CIPSTATUS` `AT+CIPSEND=n` `AT+CIPSEND` (passthrough) `AT+CIPMODE=0/1`
`+++` `AT+CIPCLOSE` — plus `AT+MQTT*`/`AT+HTTPCLIENT` in the vendor lib,
untouched here.
