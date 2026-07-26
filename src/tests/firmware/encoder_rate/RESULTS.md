# Encoder refresh characterization — results moved

The authoritative write-up lives in the design docs:
[docs/design/encoder-refresh-characterization.md](../../../../docs/design/encoder-refresh-characterization.md)
(stakeholder direction 2026-07-26: results belong in `docs/`, code
references them).

One-line summary: the "~80 ms register refresh" theory is FALSE — the
register is live at ≤ 16 ms; the historical staleness was
interposed-traffic sample invalidation under the pre-118 loop schedule.

Raw captures from the three measurement runs are the `capture*.log`
files beside this note; the firmware and analysis script in this
directory reproduce the measurement.
