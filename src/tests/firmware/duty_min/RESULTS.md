# duty_min: minimum-duty probe results (2026-07-27, ~00:30-00:50)

Standalone firmware (no robot loop): per wheel, duty 1..14% forward, one
0x60 write (byte-identical to the vendor blocks AND the main firmware),
3 s holds, encoder delta read after. Modes: Q = zero bus traffic during
the hold; B = brick-addressed encoder polling every ~47 ms; O =
other-device traffic (OTOS burst + line read every ~47 ms), no brick
traffic.

## Findings

1. TRUE dead zone is ~1-6% duty, per wheel, and WANDERS with usage
   state: right wheel dead below 4% (run 1), below 6% (run 2), then
   moving at 1% (run 3, after minutes of exercise). Left moved at 1% in
   every run. Matches the stakeholder's vendor-blocks rig (1-5%).
2. Bus traffic is EXONERATED: neither brick-addressed polling (B) nor
   other-device traffic (O) raises the threshold; O costs only ~10-15%
   speed at fixed duty.
3. The main-firmware sweeps' 10-16% "dead zone" is therefore NOT a
   firmware-path effect and NOT plant physics at that magnitude. Prime
   suspect: the sweep's own criterion -- ALL THREE 500 ms cold-start
   repeats must move >1 mm (a strict AND over short trials of a sticky,
   state-dependent process). Follow-up: re-measure through the main
   firmware with 3 s holds and a per-trial criterion.
4. Above the threshold the duty->speed relation is smooth from ~1%
   (e.g. quiet right: 1%=1027, 4%=2144, 8%=3439, 14%=5412 tenths-deg
   per 3 s -- offset+linear, no cliff).

## Implications (pending stakeholder direction)

- Crawl pulse 0.20 is oversized; ~0.08 would clear the observed worst
  case, and crawl may reduce to a narrow sub-5% helper.
- The open-loop low end deserves re-mapping with longer holds.
