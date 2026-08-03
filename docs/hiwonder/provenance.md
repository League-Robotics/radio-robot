# HiWonder 4-Channel Encoder Motor Driver — Source Provenance

Which claims in [the transfer package](hiwonder-motor-board.md) rest
on which sources. Compiled 2026-08-02.

## Sources

**S1 — Vendor documentation (Hiwonder's own).**
"Development Tutorial (Arduino)", Black Mecanum-Wheel Chassis v1.0,
docs.hiwonder.com
(`https://docs.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/3._Arduino_checked.html`,
fetched 2026-08-02). Contains Hiwonder's own Arduino sample code with
their register constants and code comments. The board's product/learning
page (`https://www.hiwonder.com.cn/store/learn/142.html`) is a landing
shell whose technical content lives in a Google Drive package; the
"2. Software" part of that package (downloaded by the stakeholder,
2026-08-02) contains only environment-setup tooling, no protocol
documentation. There is **no vendor datasheet with a formal register
map** that we have found — the tutorial code IS the vendor's protocol
documentation.

**S2 — PX4 HiwonderEMM driver** (third-party, BSD-3-Clause; copy in
[`px4-hiwonder-emm/`](px4-hiwonder-emm/) with license header intact,
from `PX4/PX4-Autopilot` `src/drivers/hiwonder_emm`, fetched
2026-08-02). Independent implementation; corroborates registers and
formats but is not vendor authority.

**S3 — Our bench measurements** on the vogop rig (JGB37-520 motors,
2026-08-02), datasets in `hwspike/out/`.

## Corroboration matrix

| claim | S1 vendor | S2 PX4 | S3 measured |
|---|---|---|---|
| I2C address 0x34 | ✔ | ✔ | ✔ (board answers) |
| battery/ADC at reg 0 | ✔ (`ADC_BAT_ADDR 0`) | ✔ (u16 LE) | ✔ |
| ADC unit is millivolts | — | — (raw, scaled downstream) | ✔ 8053 ≈ 2S pack |
| motor type reg 20/0x14 | ✔ | ✔ | ✔ (accepted) |
| type 3 = JGB, magnetic encoder, 44 pulses/rev, ratio 131 default | ✔ | ✔ (named JGB37-520-12V-110RPM) | our motors differ (JGB37-520 pair) |
| encoder polarity reg 21/0x15, values 0/1 | ✔ | ✔ | ✔ (0 works) |
| PWM reg 31/0x1F, open loop, range −100..100 | ✔ | — (unused) | ✔ (127 out of range, barely turned) |
| speed reg 51/0x33, closed loop | ✔ | ✔ | ✔ |
| speed unit: pulses per 10 ms | ✔ (verbatim comment) | — | ✔ (57 p/10ms at saturation) |
| speed usable range ~±50 | ✔ ("typically around ±50"; demo caps at 30, recommends [−50,50]) | ✗ PX4 maps [−128,127] — no range guard | ✔ saturation at ~57 |
| one 4-byte frame commands all channels | ✔ | ✔ | ✔ |
| encoder totals reg 60/0x3C, 4× int32 | ✔ | ✔ (LE on ARM) | ✔ |
| totals auto-increment on multi-byte read | implied by sample code | ✔ (16-byte read) | ✔ |
| totals reset by writing zeros | — | — | not exercised (we never reset) |
| PWM is non-latching, needs continuous send | vendor comment (earlier research) | — | ✔ decays to stop <1 s command-silent at speed |
| speed (0x33) latches while moving; single-write setpoint changes hold | — | — | ✔ full speed thru 2 s total silence; 3127→1564 on one write |
| from-rest start needs a write burst (~200 ms) | — | — | ✔ 1 write never, 2–5 flaky, 20+ reliable |
| no watchdog; speed latches forever | — | — | ✔ (observed across session) |
| encoder registers update every **9.56 ms** | — (unit implies 10 ms) | — | ✔ measured two ways |
| driver chip YX-4055AM, DC 3–12 V per channel | ✔ | — | — |
| board supply 6.4–8.4 V | product-page spec (not re-fetched) | — | ✔ dead at 2.9 V, fine 6.7–8.4 V |
| coast on 0, brake needs reverse ~0.30 | — | — | ✔ |
| spin-up τ ≈ 145 ms; lead taus 150/150 | — | — | ✔ |
| ~0.8 ms bus cost at 400 kHz | — | ✔ (PX4 also runs 400 kHz) | ✔ |

## Items needing attention

1. ~~"PWM is non-latching" unsourced~~ **RESOLVED 2026-08-02 on the
   bench**: PWM at speed decays to stopped within 1 s of command
   silence; speed (0x33) latches absolutely while moving but needs a
   write burst (~200 ms) to start from rest. Full protocol in the
   driver doc §1.3–1.4 and the `L` verb in the characterization
   firmware.
2. **Encoder reset (write zeros to 0x3C)** appears in community drivers
   but not in S1/S2; unexercised by us and unused by design (never-reset
   convention). Treat as unverified.
3. **9.56 ms tick** is ours alone and contradicts the naive reading of
   the vendor's "per 10 ms" unit. It is the transfer package's most
   load-bearing uncorroborated number; the measurement method (two
   independent estimates agreeing) is documented there.
