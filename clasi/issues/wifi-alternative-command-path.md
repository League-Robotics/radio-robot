---
status: pending
---

# WiFi (Ai-WB2-12F) as an alternative command path to the robot

The Planet X WiFi module (Ai-Thinker Ai-WB2-12F, BL602, 2.4 GHz) was
bench-verified end to end on `gopiv` 2026-08-08 — AP join, UDP and TCP
sockets in both directions, transparent passthrough streaming byte-perfect
(2032/2032 B), 30/30 UDP round trips at median 49.5 ms against a 2–6 ms
ICMP floor (the gap is the module's packetization timer on small writes;
bulk streams don't pay it). Operating reference with hardware-verified
markings: `docs/knowledge/2026-08-08-wifi-module-sockets.md`; bring-up
kit, findings, and the atbridge probe firmware: `src/tests/bench/wifi/`.

Proposal: integrate the module as an alternative transport for commanding
the robot, alongside USB serial and the radio relay.

## Scope

1. **Firmware `WifiTransport` leaf** alongside `App::SerialTransport` /
   `App::RadioTransport`: owns UARTE1 on the RJ11 pins, boots the module
   (join AP, apply the configured fixed IP, open a UDP socket, enter
   passthrough), and feeds `App::Comms` the same protocol-v5 framed bytes
   the radio does. Notably, the lossy leg measured during bring-up
   (~5–10 % char drops, DAPLink CDC outbound) is NOT in this path — the
   nRF↔module UART ran clean — and COBS+CRC framing already covers the
   wire.
2. **Config completion.** `connection.wifi_ip` landed 2026-08-08
   (robot_config.proto + regenerated schema/pydantic + gopiv/tovez JSONs,
   currently uncommitted). A production boot additionally needs the
   SSID/credentials decision — bake into config vs rely on the module's
   own saved AP — per `.claude/rules/configuration-discipline.md`.
3. **Host-side UDP connection class** in `robot_radio.io`, so
   `NezhaProtocol` / `rogo` can target `<robot-ip>:<port>` instead of a
   serial device.
4. **Network prerequisite (stakeholder-owned).** The bench-DNS
   reservations (gopiv=192.168.4.10, tovez=192.168.4.11) sit on
   192.168.4.0/24, which nothing currently routes — the mesh DHCPs
   clients onto 192.168.1.0/24. Either route/VLAN the .4 subnet or move
   the reservations into the pool the mesh serves.
5. **Acceptance = the standing hardware gate**: `move_twist` over UDP on
   the stand with climbing encoders, and `estop()` over WiFi verified.

## Design questions for the sprint

- UDP vs TCP for the command plane. UDP favored — stale command packets
  are worthless, and the measured 19/20 delivery is exactly what protocol
  v5's ack-ring semantics already handle. TCP remains attractive for
  bulk/log streaming.
- Passthrough (`AT+CIPMODE=1`) vs AT-mode `CIPSEND` framing — passthrough
  is the streaming-native shape but does not preserve datagram boundaries
  (in-band COBS/0x0A framing covers this); AT mode preserves boundaries
  at a per-send handshake cost.
- UDP mode 2 ("peer = whoever sent last", untested in bring-up) as the
  server shape for multi-host use, and how telemetry fans out when more
  than one listener wants it.
