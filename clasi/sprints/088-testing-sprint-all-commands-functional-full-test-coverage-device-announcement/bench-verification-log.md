# Sprint 088 — On-stand bench verification log (ticket 088-009)

**Date:** 2026-07-07 (autonomous overnight sprint run)
**Firmware:** `v0.20260707.5` (bench build, `BENCH_OTOS_ENABLED`), clean-built via
`just build-clean` and flashed with `mbdeploy deploy robot --hex MICROBIT.hex`.
**Robot device:** ROLE=`NEZHA2`, name=`tovez`, serial `2314287040`, UID
`9906360200052820a8fdb5e413abb276...`, `/dev/cu.usbmodem2121102` — confirmed via
`mbdeploy list` ROLE column (the relay, ROLE=`RADIOBRIDGE` on `...2121402`, was
NOT touched; mbdeploy auto-refuses flashing a relay).
**Tool:** `tests/bench/motion_command_verify.py` (committed; correlation-robust via
`SerialConnection` + `NezhaProtocol`).
**Transport:** direct USB serial (auto-detected `mode=direct`).

## Result — 10/10 automated checks PASS (final run)

| Check | Result | Evidence |
|---|---|---|
| Device announcement (088-005) | ✅ | `connect()` classified `mode=direct` — the host sent HELLO, received `DEVICE:NEZHA2:robot:tovez:2314287040`, and classified it off field 1. Raw HELLO returned that banner verbatim in direct-serial probes. |
| PING | ✅ | `OK pong t=…` |
| VER (088-001) | ✅ | `OK ver fw=0.20260707.5 proto=2` (confirms the final firmware is flashed; VER was never broken) |
| ID | ✅ | `ID model=NEZHA2 name=tovez serial=2314287040 fw=0.20260707.5 proto=2` |
| HELP full verb list (088-003) | ✅ | `OK help PING VER HELP ECHO ID HELLO DEV M DEV DT DEV STATE DEV STOP DEV WD STREAM SNAP S T D R TURN RT G STOP GET SET SI ZERO OI OZ OR OP OV OL OA` — the full registered table, not the old hardcoded 5 verbs |
| SET/GET config | ✅ | `SET tw=111` → `OK set tw=111`; `GET tw` → `CFG tw=111` within ~1 tick (087 two-plane Configurator applies on the next pass). `distScale` correctly `ERR badkey` (not a registered key). |
| **D** distance-drive | ✅ | `OK drive l=150 r=150 mm=120`; encoders **port1 +140, port2 +128** (both drive) |
| **T** timed-drive | ✅ | `OK drive l=150 r=150 ms=900`; encoders **port1 +205, port2 +196** (both drive) |
| **S** streaming-drive | ✅ | `OK drive l=150 r=150`; encoders **port1 +112, port2 +113** (both drive) |
| **RT** relative-turn (spin) | ✅ | `OK rt rot=45` + `EVT done RT`; encoders **port1 −24, port2 +30** (opposite signs — correct spin) |
| Watchdog safety | ✅ | `DEV WD 5000` widened for runs; `STOP` + `DEV STOP` + `DEV WD 1000` restored in `finally` |

Motion verbs drive the wheels and the encoders respond correctly: **straight
commands (D/T/S) move both ports the same sign; the spin command (RT) moves them
opposite.** This satisfies the stakeholder's hard bar ("all motion/config
commands should function, test by looking at the encoders").

## Notes / caveats

- **`fwd_sign` (088-002) absolute direction is not encoder-observable on a stand.**
  The encoder path scales by `fwd_sign` (`NezhaMotor`), so a "forward" command
  reads positive encoders with EITHER polarity. This tool proves the verbs
  function and that straight/spin encoder *relationships* are correct; the
  wheel-direction fix (port 2 = −1, from the old working firmware's proven
  `fwdSignL=-1`/chip-M2=port-2 mapping) makes both wheels physically drive
  forward — **please eyeball one straight `D 150 150 120` to confirm both wheels
  roll the same way** (a one-look human check the stand can't automate).
- **`S` can show a net-negative encoder delta** on some runs: the velocity loop
  briefly reverse-spins at an abrupt STOP (the separate, already-tracked
  terminal-overshoot issue — `clasi/issues/rt-open-loop-overshoot-under-synchronous-update.md`).
  Adding a settle before the final encoder read gives the clean numbers above; the
  verb drives the wheels regardless.

## Stand-limited (smoke/dispatch-only on the stand — need real motion)

- **`TURN` and `G`** are closed-loop on FUSED pose/heading. On the stand the OTOS
  sees no translation, so these verbs cannot converge/complete meaningfully — they
  dispatch (`OK`) but full validation needs the playfield (real motion). Not
  encoder-verified here by design.
- **`R`** (open-loop arc) dispatches `OK arc …`; a curvature check likewise wants
  real motion / a longer run than a stand pass cleanly gives.
- **OTOS absolute position (`OP`/`OR`/`OV`), `SI` camera pose-inject** — need real
  translation / a camera; dispatch-only on the stand.

## Deferred (not a stand limit — a follow-up)

- **Radio-relay round-trip** was NOT exercised this pass — verification was over
  direct USB serial only. The relay path (`!GO` data plane) works on the stand;
  recommend a quick relay pass with `tests/bench/motion_command_verify.py --port
  /dev/cu.usbmodem2121402` (relay) when convenient. The announcement/HELLO design
  specifically supports the relay rediscovery path.

## Pre-bench sim gate

`uv run python -m pytest tests/sim` was green (260 passed, 4 xfailed) at the last
firmware ticket (088-004) before this bench pass; re-confirmed at sprint close.
