"""replay.py -- the tier-0 replay harness (ticket 100-006 AC): given a
recorded ``TrackRecord.in_`` sequence (from ANY tier -- sim, bench, field),
replay it through ``Plan.step()`` and reproduce the recorded output
bit-exact.

This is the mechanism every higher-tier ticket (100-007/010/011/012) relies
on to isolate a defect to ``source/drive/`` itself vs. the adapter/plant: a
suspicious wheel command captured on the bench (or in the sim/tier-1 tier)
carries a ``TrackRecord`` with the exact ``StepInput`` that produced it;
feeding that SAME input sequence through ``Plan.step()`` here, starting from
a fresh ``StepState()`` (a plan's ``StepState`` always starts default at
segment start -- motion_plan.h's own contract), reproduces the recorded
output bit-exact if and only if ``source/drive/`` itself is not where the
discrepancy comes from.

``replay_track_records()`` does not care WHICH tier produced the
``TrackRecord`` sequence -- only that each record's own ``.in_`` field is
the exact ``StepInput`` that tier fed to ``step()``, in order.
"""
from __future__ import annotations

from drive import Plan, StepOutput, StepState, TrackRecord


def replay_track_records(plan: Plan, records: list[TrackRecord]) -> list[StepOutput]:
    """Replay ``records`` (in order) through ``plan.step()``, starting from a
    fresh ``StepState()``. Returns the resulting ``StepOutput`` sequence."""
    state = StepState()
    outputs: list[StepOutput] = []
    for record in records:
        out, state = plan.step(record.in_, state)
        outputs.append(out)
    return outputs
