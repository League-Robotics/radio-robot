---
status: pending
---

# Reorganize TestGUI layout for better use of space

## Description

The TestGUI's control area and sim-errors panel waste vertical space and are
hard to scan. Reorganize as follows:

### Top control rows

1. **Pull-downs on one line.** Put the three combo boxes — transport, robot,
   and import — on a single row.
2. **Session buttons on one line, with icons.** Put Connect, Disconnect,
   Record, Pause, and Stop on a single row. Add icons to each button so they
   are distinguishable at a glance (e.g. plug/unplug for connect/disconnect,
   record dot, pause bars, stop square).

### Sim errors panel

3. The sim-error parameters currently take up two columns, which makes the
   panel tall and hard to read. Rework it:
   - Move the OTOS error group (and likely geometry and actuation too) into a
     left-side column.
   - Tighten the numeric spin-box widths — with more compact number boxes the
     panel should fit in **three columns**.

## Acceptance sketch

- Transport / robot / import selectors share one row.
- Connect / Disconnect / Record / Pause / Stop share one row and carry
  distinguishing icons.
- Sim errors panel is laid out in three columns with compact numeric fields;
  OTOS (and geometry/actuation) errors live in the left column.
- No functional changes — layout and iconography only.
